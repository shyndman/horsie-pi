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

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32s3_hal::{
    self,
    clock::ClockControl,
    embassy, interrupt,
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    system::SystemParts,
    uart, Rmt, Uart, IO,
};
use esp_hal_procmacros::main;
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
use esp_println::logger::init_logger_from_env;
use peripheral_controller::{
    init_heap,
    stepper::{
        motor_constants::NEMA8_S20STH30_0604A_CONSTANTS,
        tune::tune_driver,
        uart::{Tmc2209UartConnection, UART_BAUD_RATE},
    },
};
use rgb::RGB8;
use smart_leds::brightness;
use smart_leds_trait::SmartLedsWrite;
use tmc2209::reg::VACTUAL;

#[main]
async fn main(spawner: Spawner) {
    init_heap();
    init_logger_from_env();

    defmt::info!("Init!");

    let peripherals = Peripherals::take();
    let system: SystemParts = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();

    embassy::init(
        &clocks,
        esp32s3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    Timer::after(Duration::from_secs(1)).await;

    let mut neopixel_power_pin = io.pins.gpio17.into_push_pull_output();
    neopixel_power_pin.set_high().unwrap();
    let neopixel_data_pin = io.pins.gpio18.into_push_pull_output();
    let mut led = <smartLedAdapter!(0, 1)>::new(rmt.channel0, neopixel_data_pin);
    led.write(brightness(
        core::iter::once(RGB8::new(0xFF, 0x00, 0xFF)),
        90,
    ))
    .unwrap();

    let uart2 = Uart::new_with_config(
        peripherals.UART2,
        uart::config::Config {
            baudrate: UART_BAUD_RATE,
            ..uart::config::Config::default()
        },
        Some(uart::TxRxPins::new_tx_rx(
            io.pins.gpio21.into_push_pull_output(),
            io.pins.gpio5.into_floating_input(),
        )),
        &clocks,
    );
    let reg = esp32s3_hal::peripherals::UART2::register_block();
    reg.conf0.modify(|_, w| {
        w.rxfifo_rst().set_bit();
        w
    });
    reg.conf0.modify(|_, w| {
        w.rxfifo_rst().clear_bit();
        w
    });
    reg.conf0.modify(|_, w| {
        w.txfifo_rst().set_bit();
        w
    });
    reg.conf0.modify(|_, w| {
        w.txfifo_rst().clear_bit();
        w
    });

    interrupt::enable(Interrupt::UART2, interrupt::Priority::Priority1).unwrap();
    uart2.reset_rx_fifo_full_interrupt();
    spawner.spawn(configure_stepper_drivers(uart2)).unwrap();

    loop {
        Timer::after(Duration::from_secs(30)).await;
    }
}

#[embassy_executor::task]
async fn configure_stepper_drivers(mut uart: Uart<'static, esp32s3_hal::peripherals::UART2>) {
    defmt::info!("Configuring stepper driver");
    Timer::after(Duration::from_secs(1)).await;

    let mut pan_driver = Tmc2209UartConnection::connect(&mut uart, 0x00).await;
    defmt::info!("Connected to pan driver");

    defmt::info!("Tuning pan driver");
    tune_driver(&mut pan_driver, NEMA8_S20STH30_0604A_CONSTANTS, &mut uart).await;
    defmt::info!("Tuned!");

    let tstep = pan_driver
        .read_register::<tmc2209::reg::TSTEP, esp32s3_hal::peripherals::UART2>(&mut uart)
        .await
        .unwrap();
    defmt::info!("TSTEP is {}", tstep.0);

    let mut vactual = VACTUAL::default();
    vactual.0 = 4000;
    pan_driver.write_register(&mut uart, vactual).await.unwrap();
}
