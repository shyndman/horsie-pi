#![no_std]
#![no_main]
#![feature(
    async_fn_in_trait,
    custom_test_frameworks,
    exclusive_range_pattern,
    impl_trait_projections,
    type_alias_impl_trait
)]
#![test_runner(peripheral_controller::run_tests)]

extern crate alloc;

use defmt::Debug2Format;
use defmt_macros::unwrap;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};
use esp32s3_hal::{
    self,
    clock::ClockControl,
    embassy, entry,
    gpio::{AnyPin, Gpio18, Output, PushPull},
    i2c::I2C,
    interrupt,
    mcpwm::{PeripheralClockConfig, MCPWM},
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    system::SystemParts,
    uart, Rmt, Uart, IO,
};
use esp_hal_procmacros::main;
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
use esp_println::logger::init_logger_from_env;
use hp_embedded_drivers::{husb238_async::Husb238, ina260_async::Ina260};
use peripheral_controller::{
    init_heap,
    ui::rgb_button::RgbButton,
};
use smart_leds::{brightness, colors};
use smart_leds_trait::*;
use static_cell::make_static;

use crate as _;

#[embassy_executor::task]
async fn run_color_wheel(
    rmt: Rmt<'static>,
    mut power_pin: AnyPin<Output<PushPull>>,
    data_pin: AnyPin<Output<PushPull>>,
    mut power_button: RgbButton<'static>,
) {
    fn rgb_color_wheel(i: u8) -> RGB8 {
        match i % 255 {
            i @ 0..85 => RGB8::new(255 - i * 3, 0, i * 3),
            mut i @ 85..170 => {
                i -= 85;
                RGB8::new(0, i * 3, 255 - i * 3)
            }
            mut i @ _ => {
                i -= 170;
                RGB8::new(i * 3, 255 - i * 3, 0)
            }
        }
    }

    // let mut power_pin = power_pin.into() as Output<Unknown>;

    unwrap!(power_pin.set_high());
    let data_pin: Gpio18<Output<PushPull>> = data_pin.try_into().unwrap();

    let mut led = <smartLedAdapter!(0, 1)>::new(rmt.channel0, data_pin);

    let mut color_index = 0u8;
    loop {
        let color = rgb_color_wheel(color_index);
        led.write(brightness(core::iter::once(color), 60)).unwrap();
        power_button.set_color(color);

        color_index = color_index.wrapping_add(1);
        Timer::after(Duration::from_millis(15)).await;
    }
}

#[embassy_executor::task]
async fn run_host_communication(mut uart: Uart<'static, esp32s3_hal::peripherals::UART1>) {
    loop {
        match embedded_io_async::Write::write_all(&mut uart, "hello hello\n".as_bytes()).await
        {
            Ok(_) => defmt::debug!("uart1 write successful"),
            Err(e) => defmt::error!("uart1 write failure, {}", e),
        }
        Timer::after(Duration::from_secs(2)).await;
    }
}

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
    let uart1 = Uart::new_with_config(
        peripherals.UART1,
        uart::config::Config {
            baudrate: 115_200,
            ..uart::config::Config::default()
        },
        Some(uart::TxRxPins::new_tx_rx(io.pins.gpio7, io.pins.gpio6)),
        &clocks,
    );
    let i2c0 = I2C::new(
        peripherals.I2C0,
        io.pins.gpio2,
        io.pins.gpio1,
        100u32.kHz(),
        &clocks,
    );
    interrupt::enable(Interrupt::I2C_EXT0, interrupt::Priority::Priority1).unwrap();

    embassy::init(
        &clocks,
        esp32s3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    let peripheral_clock_config =
        PeripheralClockConfig::with_frequency(&clocks, 40u32.MHz()).unwrap();
    let mut power_button = RgbButton::new(
        io.pins.gpio40,
        io.pins.gpio41,
        io.pins.gpio42,
        MCPWM::new(
            peripherals.MCPWM0,
            PeripheralClockConfig::with_frequency(&clocks, 40u32.MHz()).unwrap(),
        ),
        io.pins.gpio39.into_push_pull_output().degrade(),
        peripheral_clock_config,
    );
    power_button.set_color(colors::CHARTREUSE);

    spawner
        .spawn(run_color_wheel(
            rmt,
            io.pins.gpio17.into_push_pull_output().degrade(),
            io.pins.gpio18.into_push_pull_output().degrade(),
            power_button,
        ))
        .unwrap();

    spawner.spawn(run_host_communication(uart1)).unwrap();

    spawner.spawn(read_power_metrics(i2c0)).unwrap();

    loop {
        let uptime_seconds = Instant::now().as_millis() as f32 / 1000.0;

        defmt::debug!("Up {} seconds", uptime_seconds);
        Timer::after(Duration::from_secs(10)).await;
    }
}

#[embassy_executor::task]
async fn read_power_metrics(i2c0: I2C<'static, esp32s3_hal::peripherals::I2C0>) {
    Timer::after(Duration::from_secs(2)).await;

    let i2c0_bus: &'static mut Mutex<
        CriticalSectionRawMutex,
        I2C<'static, esp32s3_hal::peripherals::I2C0>,
    > = make_static!({ Mutex::<CriticalSectionRawMutex, _>::new(i2c0) });

    defmt::info!("Attempting to communicate with INA260");
    let mut ina260 = match Ina260::new(I2cDevice::new(i2c0_bus)).await {
        Ok(device) => {
            defmt::info!("Established contact");
            device
        }
        Err(e) => {
            defmt::error!("ERROR, {}", Debug2Format(&e));
            return;
        }
    };

    defmt::info!("Attempting to communicate with HUSB238");
    let husb238 = match Husb238::new(I2cDevice::new(i2c0_bus)).await {
        Ok(device) => {
            defmt::info!("Established contact");
            device
        }
        Err(e) => {
            defmt::error!("ERROR, {}", Debug2Format(&e));
            return;
        }
    };

    loop {
        let m_volts = ina260.read_bus_voltage().await.unwrap();
        let m_amps = ina260.read_current().await.unwrap();
        let m_watts = ina260.read_power().await.unwrap();

        defmt::info!("{}mV {}mA {}mW", m_volts, m_amps, m_watts);

        Timer::after(Duration::from_secs(4)).await;
    }
}
