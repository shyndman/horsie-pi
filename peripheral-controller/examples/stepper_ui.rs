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
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker, Timer};
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
use esp_hal_common::gpio::{Gpio7, Unknown};
use esp_hal_procmacros::main;
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
use esp_println::logger::init_logger_from_env;
use peripheral_controller::{
    init_heap,
    peripherals::{new_i2c, PeripheralI2cLink},
    shared_bus::DualModeI2cDevice,
    user_input::ano_rotary_encoder::AnoRotaryEncoder,
};
use profont::PROFONT_12_POINT;
use rand::{rngs::SmallRng, RngCore, SeedableRng};
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
        esp32s3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    esp32s3_hal::interrupt::enable(
        esp32s3_hal::peripherals::Interrupt::GPIO,
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

    spawner
        .spawn(capture_input(
            DualModeI2cDevice::new(i2c0_bus),
            io.pins.gpio7,
        ))
        .unwrap();

    spawner
        .spawn(render_display(DualModeI2cDevice::new(i2c0_bus)))
        .unwrap();

    loop {
        Timer::after(Duration::from_secs(30)).await;
    }
}

#[embassy_executor::task]
async fn capture_input(
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

    let mut interrupt_pin = interrupt_pin.into_floating_input();
    loop {
        interrupt_pin.wait_for_low().await.ok();

        let press_states = rotary_encoder.button_states().await.unwrap();
        let position = rotary_encoder.position().await.unwrap();
        defmt::info!("Position: {}, Buttons: {}", position, press_states);

        rotary_encoder.consume_interrupt_state().await.unwrap();
        rotary_encoder.reset_interrupts().await.unwrap();
    }
}

#[embassy_executor::task]
async fn render_display(display_link: PeripheralI2cLink<hal::peripherals::I2C0>) {
    defmt::info!("Initializing display device");

    let mut bigger_display = Ssd1306::new(
        I2CDisplayInterface::new_alternate_address(display_link),
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    bigger_display.init().await.unwrap();
    bigger_display
        .set_brightness(Brightness::BRIGHTEST)
        .await
        .unwrap();

    let data_text_style = MonoTextStyleBuilder::new()
        .font(&PROFONT_12_POINT)
        .text_color(BinaryColor::On)
        .build();

    let mut rng = SmallRng::seed_from_u64(0xdeadbeef);

    let mut ticker = Ticker::every(Duration::from_hz(4));
    loop {
        bigger_display.clear_buffer();

        let usteps = rng.next_u32() / 40_000 + 1000;

        Text::with_baseline(
            &format!("{usteps}µHz"),
            Point::new(0, 13),
            data_text_style,
            Baseline::Top,
        )
        .draw(&mut bigger_display)
        .unwrap();

        bigger_display.flush().await.unwrap();
        ticker.next().await;
    }
}
