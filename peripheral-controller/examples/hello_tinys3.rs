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
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    system::SystemParts,
    uart, Rmt, Uart, IO,
};
use esp_hal_procmacros::main;
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
use esp_println::logger::init_logger_from_env;
use hp_embedded_drivers::ina260_async::Ina260;
use peripheral_controller::{self as _, init_heap};
use smart_leds::brightness;
use smart_leds_trait::*;
use static_cell::make_static;

#[embassy_executor::task]
async fn run_color_wheel(
    rmt: Rmt<'static>,
    mut power_pin: AnyPin<Output<PushPull>>,
    data_pin: AnyPin<Output<PushPull>>,
) {
    fn rgb_color_wheel(mut i: u8) -> RGB8 {
        i %= 255;

        if i < 85 {
            RGB8::new(255 - i * 3, 0, i * 3)
        } else if i < 170 {
            i -= 85;
            RGB8::new(0, i * 3, 255 - i * 3)
        } else {
            i -= 170;
            RGB8::new(i * 3, 255 - i * 3, 0)
        }
    }

    unwrap!(power_pin.set_high());
    let data_pin: Gpio18<Output<PushPull>> = data_pin.try_into().unwrap();

    let mut led = <smartLedAdapter!(0, 1)>::new(rmt.channel0, data_pin);

    let mut color_index = 0u8;
    loop {
        let color = rgb_color_wheel(color_index);
        unwrap!(led.write(brightness(core::iter::once(color), 90)));

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
        io.pins.gpio43,
        io.pins.gpio44,
        100u32.kHz(),
        &clocks,
    );
    interrupt::enable(Interrupt::I2C_EXT0, interrupt::Priority::Priority1).unwrap();

    embassy::init(
        &clocks,
        esp32s3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    let neopixel_power_pin = io.pins.gpio17.into_push_pull_output();
    let neopixel_data_pin = io.pins.gpio18.into_push_pull_output();

    spawner
        .spawn(run_color_wheel(
            rmt,
            neopixel_power_pin.degrade(),
            neopixel_data_pin.degrade(),
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
    defmt::info!("Attempting to communicate with INA260");

    let i2c0_bus: &'static mut Mutex<
        CriticalSectionRawMutex,
        I2C<'static, esp32s3_hal::peripherals::I2C0>,
    > = make_static!({ Mutex::<CriticalSectionRawMutex, _>::new(i2c0) });

    let mut ina260 = match Ina260::new(I2cDevice::new(i2c0_bus)).await {
        Ok(device) => {
            defmt::info!("Established contact");
            device
        }
        Err(e) => {
            defmt::error!("ERROR, {}", e);
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
