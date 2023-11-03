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
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};
use esp32s3_hal::{
    self, clock::ClockControl, embassy, peripherals::Peripherals, prelude::*,
    system::SystemParts, Rmt, Uart, IO,
};
use esp_hal_procmacros::main;
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
use esp_println::logger::init_logger_from_env;
use peripheral_controller::{
    init_heap,
    peripherals::new_uart_bus,
    stepper::{
        motor_constants::NEMA8_S20STH30_0604A_CONSTANTS,
        ramp_generator::RampGenerator,
        tune::tune_driver,
        uart::{Tmc2209UartConnection, UART_BAUD_RATE},
    },
    uart::bus::UartDevice,
};
use rgb::RGB8;
use smart_leds::brightness;
use smart_leds_trait::SmartLedsWrite;
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
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();

    embassy::init(
        &clocks,
        esp32s3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    Timer::after(Duration::from_secs(1)).await;

    {
        io.pins.gpio17.into_push_pull_output().set_high().unwrap();
        let mut led = <smartLedAdapter!(0, 1)>::new(
            rmt.channel0,
            io.pins.gpio18.into_push_pull_output(),
        );
        led.write(brightness(
            core::iter::once(RGB8::new(0xFF, 0x00, 0xFF)),
            90,
        ))
        .unwrap();
    }

    let uart2 = new_uart_bus(
        peripherals.UART2,
        io.pins.gpio21.into_push_pull_output(),
        io.pins.gpio5.into_floating_input(),
        UART_BAUD_RATE,
        &clocks,
    );

    spawner.spawn(configure_stepper_drivers(uart2)).unwrap();

    loop {
        Timer::after(Duration::from_secs(30)).await;
    }
}

#[embassy_executor::task]
async fn configure_stepper_drivers(uart2: Uart<'static, esp32s3_hal::peripherals::UART2>) {
    defmt::info!("Configuring stepper driver");
    Timer::after(Duration::from_secs(1)).await;

    let uart2_bus: &'static mut Mutex<
        CriticalSectionRawMutex,
        Uart<'static, esp32s3_hal::peripherals::UART2>,
    > = make_static!({ Mutex::<CriticalSectionRawMutex, _>::new(uart2) });

    let mut pan_driver = Tmc2209UartConnection::connect(UartDevice::new(uart2_bus), 0x00)
        .await
        .unwrap();
    defmt::info!("Connected to pan driver");

    defmt::info!("Tuning pan driver");
    let start_ts = Instant::now();
    tune_driver(&mut pan_driver, NEMA8_S20STH30_0604A_CONSTANTS)
        .await
        .unwrap();
    defmt::info!("Tuned in {}ms", (Instant::now() - start_ts).as_millis());
    let tstep = pan_driver
        .read_register::<tmc2209::reg::TSTEP>()
        .await
        .unwrap();
    defmt::info!("TSTEP is {}", tstep.0);

    let mut vactual = VACTUAL::default();

    let mut ramp_generator = RampGenerator::new(1.8, 256, 8);
    ramp_generator.set_target_speed(360 * 32);
    loop {
        let (_new_velocity, new_vactual) = ramp_generator.next().await;
        if new_vactual != vactual.get() {
            vactual.set(new_vactual);
            pan_driver.write_register(vactual).await.unwrap();
        } else {
            Timer::after(Duration::from_secs(4)).await;
            ramp_generator.set_target_speed(0);
        }
    }
}
