//! This was built to run on the S3Tiny

#![no_std]
#![no_main]
#![feature(
    asm_experimental_arch,
    async_fn_in_trait,
    const_mut_refs,
    custom_test_frameworks,
    exclusive_range_pattern,
    impl_trait_projections,
    return_position_impl_trait_in_trait,
    type_alias_impl_trait
)]

extern crate alloc;

use alloc::format;
use core::cell::RefCell;

use adafruit_seesaw_async::{
    prelude::EncoderModule, SeesawDevice, SeesawDeviceInit, SeesawDriver,
};
use defmt::Debug2Format;
use defmt_macros::Format;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex},
    mutex::Mutex,
    pubsub::{PubSubChannel, WaitResult},
};
use embassy_time::{Delay, Instant};
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    primitives::{Primitive, PrimitiveStyleBuilder},
    text::{Baseline, Text},
};
use embedded_graphics_core::{
    pixelcolor::BinaryColor, prelude::Point, primitives::Rectangle, Drawable,
};
use embedded_hal_async::digital::Wait;
use esp32s3_hal::{
    self as hal, clock::ClockControl, embassy, interrupt, peripherals::Peripherals,
    prelude::*, system::SystemParts, Rmt, IO,
};
use esp_hal_common::{
    gpio::{Gpio7, Unknown},
    Uart,
};
use esp_hal_procmacros::main;
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
use esp_println::logger::init_logger_from_env;
use futures::{pin_mut, prelude::*};
use peripheral_controller::{
    init_heap,
    peripherals::{new_i2c, new_uart_bus, PeripheralI2cLink},
    shared_bus::DualModeI2cDevice,
    stepper::{
        motor_constants::NEMA8_S20STH30_0604A_CONSTANTS,
        ramp_generator::RampGenerator,
        tune::tune_driver,
        uart::{Tmc2209UartConnection, UART_BAUD_RATE},
    },
    uart::bus::UartDevice,
    ui::input::ano_rotary_encoder::AnoRotaryEncoder,
};
use profont::PROFONT_10_POINT;
use rgb::RGB8;
use smart_leds::brightness;
use smart_leds_trait::SmartLedsWrite;
use ssd1306::{
    prelude::{Brightness, DisplayConfig},
    rotation::DisplayRotation,
    size::DisplaySize128x64,
    I2CDisplayInterface, Ssd1306,
};
use static_cell::make_static;
use tmc2209::reg::VACTUAL;

#[main]
async fn main(spawner: Spawner) {
    init_heap();
    init_logger_from_env();

    defmt::info!("Init!");

    let peripherals = Peripherals::take();
    let system: SystemParts = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();

    embassy::init(
        &clocks,
        hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    hal::interrupt::enable(
        hal::peripherals::Interrupt::GPIO,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    {
        io.pins.gpio17.into_push_pull_output().set_high().unwrap();
        let mut led = <smartLedAdapter!(0, 1)>::new(
            rmt.channel0,
            io.pins.gpio18.into_push_pull_output(),
        );
        led.write(brightness(
            core::iter::once(RGB8::new(0x33, 0x66, 0x00)),
            90,
        ))
        .unwrap();
    }

    let i2c0 = new_i2c(
        peripherals.I2C0,
        io.pins.gpio8.into_floating_input(),
        io.pins.gpio9.into_floating_input(),
        &clocks,
    );
    let i2c0_bus: &'static mut Mutex<
        CriticalSectionRawMutex,
        RefCell<esp_hal_common::i2c::I2C<'static, esp_hal_common::peripherals::I2C0>>,
    > = make_static!({
        Mutex::<
            CriticalSectionRawMutex,
            RefCell<esp_hal_common::i2c::I2C<'static, esp_hal_common::peripherals::I2C0>>,
        >::new(RefCell::new(i2c0))
    });

    let uart2 = new_uart_bus(
        peripherals.UART2,
        io.pins.gpio21.into_push_pull_output(),
        io.pins.gpio5.into_floating_input(),
        UART_BAUD_RATE,
        &clocks,
    );

    let velocity_command_channel: &'static mut VelocityCommandChannel =
        make_static!(VelocityCommandChannel::new());
    let velocity_state_channel: &'static mut VelocityStateChannel =
        make_static!(VelocityStateChannel::new());

    spawner
        .spawn(capture_input(
            velocity_command_channel,
            DualModeI2cDevice::new(i2c0_bus),
            io.pins.gpio7,
        ))
        .unwrap();

    spawner
        .spawn(drive_steppers(
            velocity_command_channel,
            velocity_state_channel,
            uart2,
        ))
        .unwrap();

    spawner
        .spawn(render_display(
            velocity_command_channel,
            velocity_state_channel,
            DualModeI2cDevice::new(i2c0_bus),
        ))
        .unwrap();
}

type VelocityCommandChannel = PubSubChannel<CriticalSectionRawMutex, VelocityMsg, 3, 10, 3>;
type VelocityStateChannel = VelocityCommandChannel;
#[derive(Clone, Format)]
struct VelocityMsg {
    degrees_per_second: i32,
}

#[embassy_executor::task]
async fn capture_input(
    velocity_command_channel: &'static VelocityCommandChannel,
    encoder_link: PeripheralI2cLink<hal::peripherals::I2C0>,
    interrupt_pin: Gpio7<Unknown>,
) {
    defmt::info!("Initializing RotaryEncoder device");
    let rotary_encoder = AnoRotaryEncoder::new_with_default_addr(
        adafruit_seesaw_async::SeesawDriver::new(encoder_link, embassy_time::Delay),
    );
    let rotary_encoder = match rotary_encoder.init().await {
        Ok(encoder) => {
            defmt::info!("Rotary encoder initialized");
            encoder
        }
        Err(e) => {
            defmt::error!("Rotary encoder failed to initialize, {}", Debug2Format(&e));
            panic!("nooo!");
        }
    };
    let interrupt_pin = interrupt_pin.into_floating_input();

    let command_pub = velocity_command_channel.publisher().unwrap();

    let input_events = construct_input_event_stream(rotary_encoder, interrupt_pin).await;
    pin_mut!(input_events);

    let mut current_target = 0;
    loop {
        let event = match input_events.next().await {
            Some(e) => e,
            None => break,
        };

        match event {
            InputEvent::RotaryPositionChange { delta, .. } => {
                current_target += -1 * delta * 10;
            }
            InputEvent::SelectReleased => {
                defmt::info!("Zeroing velocity");
                current_target = 0;
            }
            InputEvent::LeftReleased => {
                defmt::info!("Reducing by 100");
                current_target += 100;
            }
            InputEvent::RightReleased => {
                defmt::info!("Increasing by 100");
                current_target -= 100;
            }

            _ => {}
        }

        defmt::info!(
            "Rotary change: Setting target velocity to {}°/s",
            current_target
        );
        command_pub.publish_immediate(VelocityMsg {
            degrees_per_second: current_target,
        });
    }
}

#[allow(unused)]
enum InputEvent {
    RotaryPositionChange { position: i32, delta: i32 },
    SelectPressed,
    SelectReleased,
    UpPressed,
    UpReleased,
    RightPressed,
    RightReleased,
    DownPressed,
    DownReleased,
    LeftPressed,
    LeftReleased,
}

async fn construct_input_event_stream(
    mut enc: AnoRotaryEncoder<
        SeesawDriver<
            DualModeI2cDevice<
                'static,
                CriticalSectionRawMutex,
                esp_hal_common::i2c::I2C<'_, esp_hal_common::peripherals::I2C0>,
            >,
            Delay,
        >,
    >,
    interrupt_pin: impl Wait,
) -> impl Stream<Item = InputEvent> {
    let position = enc.position().await.unwrap();
    let buttons = enc.button_states().await.unwrap();

    stream::unfold(
        (enc, interrupt_pin, position, buttons),
        |(mut enc, mut interrupt_pin, last_position, last_buttons)| async move {
            interrupt_pin.wait_for_low().await.ok();

            let position = enc.position().await.unwrap();
            let buttons = enc.button_states().await.unwrap();

            let mut events = alloc::vec![];

            if position != last_position {
                let delta = position - last_position;
                events.push(InputEvent::RotaryPositionChange { position, delta });
            }
            if buttons.select_pressed != last_buttons.select_pressed {
                events.push(if buttons.select_pressed {
                    InputEvent::SelectPressed
                } else {
                    InputEvent::SelectReleased
                });
            }
            if buttons.up_pressed != last_buttons.up_pressed {
                events.push(if buttons.up_pressed {
                    InputEvent::UpPressed
                } else {
                    InputEvent::UpReleased
                });
            }
            if buttons.right_pressed != last_buttons.right_pressed {
                events.push(if buttons.right_pressed {
                    InputEvent::RightPressed
                } else {
                    InputEvent::RightReleased
                });
            }
            if buttons.down_pressed != last_buttons.down_pressed {
                events.push(if buttons.down_pressed {
                    InputEvent::DownPressed
                } else {
                    InputEvent::DownReleased
                });
            }
            if buttons.left_pressed != last_buttons.left_pressed {
                events.push(if buttons.left_pressed {
                    InputEvent::LeftPressed
                } else {
                    InputEvent::LeftReleased
                });
            }

            Some((
                stream::iter(events),
                (enc, interrupt_pin, position, buttons),
            ))
        },
    )
    .flatten()
}

#[embassy_executor::task]
async fn drive_steppers(
    velocity_command_channel: &'static VelocityCommandChannel,
    velocity_state_channel: &'static VelocityStateChannel,
    uart2: Uart<'static, hal::peripherals::UART2>,
) {
    let uart2_bus: &'static mut Mutex<
        CriticalSectionRawMutex,
        Uart<'static, hal::peripherals::UART2>,
    > = make_static!({ Mutex::<CriticalSectionRawMutex, _>::new(uart2) });

    // Connect to the pan stepper driver, and tune it for its attached motor
    let mut pan_driver = Tmc2209UartConnection::connect(UartDevice::new(uart2_bus), 0x00)
        .await
        .unwrap();
    tune_driver(&mut pan_driver, NEMA8_S20STH30_0604A_CONSTANTS).await;

    let mut vactual = VACTUAL::default();
    let mut ramp_generator = RampGenerator::new(1.8, 256, 32);
    let mut command_sub = velocity_command_channel.subscriber().unwrap();
    let state_pub = velocity_state_channel.publisher().unwrap();

    loop {
        let (new_velocity, new_vactual) = ramp_generator.next().await;
        if new_vactual != vactual.get() {
            vactual.set(new_vactual);
            pan_driver.write_register(vactual).await.unwrap();
            state_pub.publish_immediate(VelocityMsg {
                degrees_per_second: new_velocity,
            })
        } else {
            match command_sub.next_message().await {
                WaitResult::Lagged(missed_count) => {
                    defmt::warn!("Stepper task missed {} velocity commands", missed_count);
                }
                WaitResult::Message(VelocityMsg { degrees_per_second }) => {
                    defmt::info!("Target velocity: {}", degrees_per_second);
                    ramp_generator.set_target_speed(degrees_per_second);
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn render_display(
    velocity_command_channel: &'static VelocityCommandChannel,
    velocity_state_channel: &'static VelocityStateChannel,
    display_link: PeripheralI2cLink<hal::peripherals::I2C0>,
) {
    defmt::info!("Initializing display device");

    let mut display = Ssd1306::new(
        I2CDisplayInterface::new_alternate_address(display_link),
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    display.init().await.unwrap();
    display.set_brightness(Brightness::BRIGHTEST).await.unwrap();
    display.clear_buffer();

    let data_text_style = MonoTextStyleBuilder::new()
        .font(&PROFONT_10_POINT)
        .text_color(BinaryColor::On)
        .build();
    let clear_style = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::Off)
        .stroke_width(0)
        .build();

    let char_h = data_text_style.font.character_size.height as i32;
    let display_w = display.dimensions().0 as i32;

    // Construct a stream of "events" that we know how to render
    let events =
        construct_display_event_stream(velocity_command_channel, velocity_state_channel);
    pin_mut!(events);

    loop {
        let event = match events.next().await {
            Some(e) => e,
            None => break,
        };

        let ((x0, y0), (x1, y1), label) = match event {
            DisplayEvent::CurrentVelocity { degrees_per_second } => {
                let y = 0;
                (
                    (0, y),
                    (display_w, y + char_h),
                    format!("C: {}°/s", -degrees_per_second),
                )
            }
            DisplayEvent::TargetVelocity { degrees_per_second } => {
                let y = char_h + 1;
                (
                    (0, y),
                    (display_w, y + char_h),
                    format!("T: {}°/s", -degrees_per_second),
                )
            }
        };

        // Clear region
        let ts = Instant::now();
        Rectangle::with_corners(Point::new(x0, y0), Point::new(x1, y1))
            .into_styled(clear_style)
            .draw(&mut display)
            .unwrap();
        // Draw text
        Text::with_baseline(&label, Point::new(x0, y0), data_text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        // Flush screen (note this will only flush the modified region)
        display.flush().await.unwrap();
        defmt::info!("Render complete in {}ms", (Instant::now() - ts).as_millis());
    }
}

enum DisplayEvent {
    TargetVelocity { degrees_per_second: i32 },
    CurrentVelocity { degrees_per_second: i32 },
}

fn construct_display_event_stream(
    velocity_command_channel: &'static VelocityCommandChannel,
    velocity_state_channel: &'static VelocityStateChannel,
) -> impl Stream<Item = DisplayEvent> {
    let command_sub_stream =
        channel_to_stream(velocity_command_channel).map(|v| DisplayEvent::TargetVelocity {
            degrees_per_second: v.degrees_per_second,
        });
    let state_sub_stream =
        channel_to_stream(velocity_state_channel).map(|v| DisplayEvent::CurrentVelocity {
            degrees_per_second: v.degrees_per_second,
        });

    futures::stream::select(command_sub_stream, state_sub_stream)
}

pub fn channel_to_stream<
    M: RawMutex,
    T: Clone,
    const CAP: usize,
    const SUBS: usize,
    const PUBS: usize,
>(
    channel: &'static PubSubChannel<M, T, CAP, SUBS, PUBS>,
) -> impl Stream<Item = T> {
    stream::unfold(channel.subscriber().unwrap(), |mut sub| async {
        // TODO(shyndman): Maybe add optional logging on lagged streams?
        Some((sub.next_message_pure().await, sub))
    })
}
