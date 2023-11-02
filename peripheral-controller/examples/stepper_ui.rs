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
    prelude::{EncoderModule, GpioModule},
    SeesawDevice, SeesawDeviceInit,
};
use defmt::Debug2Format;
use defmt_macros::Format;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    mutex::Mutex,
    pubsub::{PubSubChannel, WaitResult},
};
use embassy_time::Instant;
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    text::{Baseline, Text},
};
use embedded_graphics_core::{pixelcolor::BinaryColor, prelude::Point, Drawable};
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
use profont::PROFONT_14_POINT;
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
    let mut rotary_encoder = match rotary_encoder.init().await {
        Ok(encoder) => {
            defmt::info!("Rotary encoder initialized");
            encoder
        }
        Err(e) => {
            defmt::error!("Rotary encoder failed to initialize, {}", Debug2Format(&e));
            panic!("nooo!");
        }
    };

    let command_pub = velocity_command_channel.publisher().unwrap();
    let mut last_position: i32 = 0;
    let mut interrupt_pin = interrupt_pin.into_floating_input();
    loop {
        interrupt_pin.wait_for_low().await.ok();

        let press_states = rotary_encoder.button_states().await.unwrap();
        let position = rotary_encoder.position().await.unwrap();

        rotary_encoder.consume_interrupt_state().await.unwrap();
        rotary_encoder.reset_interrupts().await.unwrap();

        defmt::debug!("Position: {}, Buttons: {}", position, press_states);
        if last_position != position {
            let target_velocity = -1 * position * 10;
            defmt::info!(
                "Rotary change: Setting target velocity to {}°/s",
                target_velocity
            );
            command_pub.publish_immediate(VelocityMsg {
                degrees_per_second: target_velocity,
            });
            last_position = position;
        }
    }
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
    let mut ramp_generator = RampGenerator::new(1.8, 256, 8);
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

    let data_text_style = MonoTextStyleBuilder::new()
        .font(&PROFONT_14_POINT)
        .text_color(BinaryColor::On)
        .build();

    let mut target_velocity: i32 = 0;
    let mut current_velocity: i32 = 0;
    let mut command_sub = velocity_command_channel.subscriber().unwrap();
    let mut state_sub = velocity_state_channel.subscriber().unwrap();
    loop {
        display.clear_buffer();

        let ts = Instant::now();
        Text::with_baseline(
            &format!("C: {current_velocity}°/s"),
            Point::new(0, 0),
            data_text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_baseline(
            &format!("T: {target_velocity}°/s"),
            Point::new(0, data_text_style.font.character_size.height as i32 + 2),
            data_text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();
        defmt::debug!("Text::draw() in {}ms", (Instant::now() - ts).as_millis());

        let ts = Instant::now();
        display.flush().await.unwrap();
        defmt::debug!("display.flush() in {}ms", (Instant::now() - ts).as_millis());

        let command_future = command_sub.next_message_pure();
        let state_future = state_sub.next_message_pure();
        pin_mut!(command_future);
        pin_mut!(state_future);

        match future::select(command_future, state_future).await {
            future::Either::Left((VelocityMsg { degrees_per_second }, _)) => {
                target_velocity = -1 * degrees_per_second
            }
            future::Either::Right((VelocityMsg { degrees_per_second }, _)) => {
                current_velocity = -1 * degrees_per_second
            }
        }
    }
}
