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
#![test_runner(peripheral_controller::run_tests)]

extern crate alloc;

use bh1750_async::BH1750;
use defmt::Debug2Format;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::digital::Wait;
use esp32s3_hal::{
    self,
    clock::ClockControl,
    embassy, entry,
    gpio::{
        AnyPin, Gpio18, Gpio6, Input, Output, PullDown, PushPull, RTCPin, RTCPinWithResistors,
    },
    i2c::I2C,
    interrupt,
    mcpwm::{PeripheralClockConfig, MCPWM},
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    rtc_cntl::{
        get_reset_reason, get_wakeup_cause,
        sleep::{RtcioWakeupSource, WakeupLevel},
        SocResetReason,
    },
    system::SystemParts,
    uart, Rmt, Rtc, Uart, IO,
};
use esp_hal_procmacros::main;
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
use esp_println::logger::init_logger_from_env;
use hp_embedded_drivers::ina260_async::Ina260;
use peripheral_controller::{
    init_heap,
    stepper::{
        motor_constants::NEMA11_11HS18_0674S_CONSTANTS, tune::tune_driver,
        uart::Tmc2209UartConnection,
    },
    uart::bus::UartDevice,
    ui::{color::rgb_color_wheel, rgb_button::RgbButton},
};
use smart_leds::brightness;
use smart_leds_trait::*;
use static_cell::make_static;
use vl6180x_async::VL6180X;

type PeripheralI2cLink =
    I2cDevice<'static, CriticalSectionRawMutex, I2C<'static, esp32s3_hal::peripherals::I2C0>>;

#[main]
async fn main(spawner: Spawner) {
    init_heap();
    init_logger_from_env();

    defmt::info!("Init!");

    let peripherals = Peripherals::take();
    let system: SystemParts = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
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

    let rtc = Rtc::new(peripherals.RTC_CNTL);
    spawner
        .spawn(sleep_on_power_button_up(
            rtc,
            io.pins.gpio6.into_pull_down_input(),
            esp32s3_hal::Delay::new(&clocks),
        ))
        .unwrap();

    let uart1 = Uart::new_with_config(
        peripherals.UART1,
        uart::config::Config {
            baudrate: 115_200,
            ..uart::config::Config::default()
        },
        Some(uart::TxRxPins::new_tx_rx(io.pins.gpio35, io.pins.gpio37)),
        &clocks,
    );
    interrupt::enable(Interrupt::UART1, interrupt::Priority::Priority1).unwrap();

    let uart2 = Uart::new_with_config(
        peripherals.UART2,
        uart::config::Config {
            baudrate: 1024,
            ..uart::config::Config::default()
        },
        Some(uart::TxRxPins::new_tx_rx(io.pins.gpio1, io.pins.gpio2)),
        &clocks,
    );
    // uart2.set_at_cmd(AtCmdConfig::new(None, None, None, AT_CMD, None));
    interrupt::enable(Interrupt::UART2, interrupt::Priority::Priority1).unwrap();

    let i2c0 = I2C::new(
        peripherals.I2C0,
        io.pins.gpio8,
        io.pins.gpio9,
        400u32.kHz(),
        &clocks,
    );
    interrupt::enable(Interrupt::I2C_EXT0, interrupt::Priority::Priority1).unwrap();
    let i2c0_bus: &'static mut Mutex<
        CriticalSectionRawMutex,
        I2C<'static, esp32s3_hal::peripherals::I2C0>,
    > = make_static!({ Mutex::<CriticalSectionRawMutex, _>::new(i2c0) });

    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();
    let peripheral_clock_config =
        PeripheralClockConfig::with_frequency(&clocks, 40u32.MHz()).unwrap();
    let power_button = RgbButton::new(
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

    spawner.spawn(configure_stepper_drivers(uart2)).unwrap();
    spawner.spawn(run_host_communication(uart1)).unwrap();
    spawner
        .spawn(read_power_metrics(
            I2cDevice::new(i2c0_bus),
            I2cDevice::new(i2c0_bus),
        ))
        .unwrap();
    spawner
        .spawn(detect_user_proximity(I2cDevice::new(i2c0_bus)))
        .unwrap();

    spawner
        .spawn(run_color_wheel(
            rmt,
            io.pins.gpio17.into_push_pull_output().degrade(),
            io.pins.gpio18.into_push_pull_output().degrade(),
            power_button,
        ))
        .unwrap();

    loop {
        let uptime_seconds = Instant::now().as_millis() as f32 / 1000.0;

        defmt::debug!("Up {} seconds", uptime_seconds);
        Timer::after(Duration::from_secs(10)).await;
    }
}

#[embassy_executor::task]
async fn sleep_on_power_button_up(
    mut rtc: Rtc<'static>,
    mut sleep_trigger_pin: Gpio6<Input<PullDown>>,
    mut delay: esp32s3_hal::Delay,
) -> ! {
    let reason =
        get_reset_reason(esp32s3_hal::Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    defmt::info!("reset reason: {:?}", Debug2Format(&reason));
    let wake_reason = get_wakeup_cause();
    defmt::info!("wake reason: {:?}", Debug2Format(&wake_reason));

    sleep_trigger_pin.rtcio_pad_hold(true);
    sleep_trigger_pin.rtcio_pulldown(true);

    defmt::info!("Waiting for sleep trigger pin to go low");
    sleep_trigger_pin.wait_for_low().await.unwrap();
    defmt::info!("Sleep trigger pin is HIGH");
    defmt::info!("Going to sleep");

    let mut wakeup_pins: [(&mut dyn RTCPin, WakeupLevel); 1] =
        [(&mut sleep_trigger_pin, WakeupLevel::High)];
    let rtcio_wakeup = RtcioWakeupSource::new(&mut wakeup_pins);
    rtc.sleep_deep(&[&rtcio_wakeup], &mut delay);
}

#[embassy_executor::task]
async fn run_color_wheel(
    rmt: Rmt<'static>,
    mut power_pin: AnyPin<Output<PushPull>>,
    data_pin: AnyPin<Output<PushPull>>,
    mut power_button: RgbButton<'static>,
) {
    power_pin.set_high().unwrap();
    let data_pin: Gpio18<Output<PushPull>> = data_pin.try_into().unwrap();

    let mut led = <smartLedAdapter!(0, 1)>::new(rmt.channel0, data_pin);

    let mut color_index = 0u8;
    loop {
        let color = rgb_color_wheel(color_index);
        color_index = color_index.wrapping_add(1);

        led.write(brightness(core::iter::once(color), 60)).unwrap();
        power_button.set_color(color);

        Timer::after(Duration::from_millis(15)).await;
    }
}

#[embassy_executor::task]
async fn run_host_communication(mut uart1: Uart<'static, esp32s3_hal::peripherals::UART1>) {
    loop {
        match embedded_io_async::Write::write_all(&mut uart1, "hello hello\n".as_bytes())
            .await
        {
            Ok(_) => {
                //defmt::debug!("uart1 write successful")
            }
            Err(e) => defmt::error!("uart1 write failure, {}", e),
        }
        Timer::after(Duration::from_secs(2)).await;
    }
}

#[embassy_executor::task]
async fn configure_stepper_drivers(uart2: Uart<'static, esp32s3_hal::peripherals::UART2>) {
    defmt::info!("Configuring stepper drivers");
    Timer::after(Duration::from_secs(1)).await;

    let uart2_bus: &'static mut Mutex<
        CriticalSectionRawMutex,
        Uart<'static, esp32s3_hal::peripherals::UART2>,
    > = make_static!({ Mutex::<CriticalSectionRawMutex, _>::new(uart2) });
    let mut pan_driver =
        Tmc2209UartConnection::connect(UartDevice::new(uart2_bus), 0x00).await;
    defmt::info!("Connected to pan driver");

    // let mut tilt_driver = Tmc2209UartConnection::connect(&mut uart2, 0x01).await;

    defmt::info!("Tuning pan driver");
    tune_driver(&mut pan_driver, NEMA11_11HS18_0674S_CONSTANTS).await;
    defmt::info!("Tuned!");

    // tune_driver(&mut tilt_driver, NEMA8_S20STH30_0604A_CONSTANTS, &mut uart2).await;

    loop {
        Timer::after(Duration::from_secs(2)).await;
    }
}

#[embassy_executor::task]
async fn detect_light_levels(i2c_device: PeripheralI2cLink) {
    defmt::info!("Attempting to communicate with BH1750");
    let mut light_sensor = match BH1750::new(i2c_device, embassy_time::Delay).await {
        Ok(device) => {
            defmt::info!("Established contact");
            device
        }
        Err(e) => {
            defmt::error!("ERROR, {}", Debug2Format(&e));
            return;
        }
    };

    light_sensor
        .start_measurement(bh1750_async::ContinuesMeasurement::HIHGT_RES)
        .await
        .unwrap();
    loop {
        let sample_in_lux = light_sensor
            .get_measurement(bh1750_async::ContinuesMeasurement::HIHGT_RES)
            .await
            .unwrap();
        defmt::info!("Light level: {} lx", sample_in_lux);
        Timer::after(Duration::from_secs(4)).await;
    }
}

#[embassy_executor::task]
async fn detect_user_proximity(i2c_device: PeripheralI2cLink) {
    Timer::after(Duration::from_secs(1)).await;

    defmt::info!("Attempting to communicate with VL6180");

    static RANGE_INTER_MEASUREMENT_MS: u16 = 100;
    let mut c = vl6180x_async::Config::new();
    c.set_range_inter_measurement_period(RANGE_INTER_MEASUREMENT_MS)
        .unwrap();

    let tof_sensor = match VL6180X::with_config(i2c_device, &c).await {
        Ok(device) => {
            defmt::info!("Established contact");
            device
        }
        Err(e) => {
            defmt::error!("ERROR, {}", Debug2Format(&e));
            return;
        }
    };

    let mut tof_sensor = tof_sensor
        .start_interleaved_continuous_mode()
        .await
        .unwrap();

    loop {
        Timer::after(Duration::from_secs(2)).await;
        match tof_sensor.read_ambient_lux_blocking().await {
            Ok(lux) => {
                defmt::info!("Light level: {} lx", lux);
            }
            Err(e) => {
                defmt::warn!("Error while reading ambient light, {}", e);
            }
        };
        Timer::after(Duration::from_millis(RANGE_INTER_MEASUREMENT_MS as u64)).await;

        match tof_sensor.read_range_mm_blocking().await {
            Ok(mm) => defmt::info!("Range to object: {} mm", mm),
            Err(vl6180x_async::Error::RangeStatusError(code)) => {
                defmt::warn!("Range status error error: {}", code)
            }
            Err(e) => defmt::error!("Error while reading range, {}", e),
        };
        Timer::after(Duration::from_millis(RANGE_INTER_MEASUREMENT_MS as u64)).await;
    }
}

#[embassy_executor::task]
async fn read_power_metrics(
    ina260_i2c_device: PeripheralI2cLink,
    _husb238_i2c_device: PeripheralI2cLink,
) {
    defmt::info!("Attempting to communicate with INA260");
    let mut ina260 = match Ina260::new(ina260_i2c_device).await {
        Ok(device) => {
            defmt::info!("Established contact");
            device
        }
        Err(e) => {
            defmt::error!("ERROR, {}", e);
            return;
        }
    };

    // defmt::info!("Attempting to communicate with HUSB238");
    // let mut husb238 = match Husb238::new(husb238_i2c_device).await {
    //     Ok(device) => {
    //         defmt::info!("Established contact");
    //         device
    //     }
    //     Err(e) => {
    //         defmt::error!("ERROR, {}", e);
    //         return;
    //     }
    // };

    // husb238.is_voltage_detected(pd)

    loop {
        let m_volts = ina260.read_bus_voltage().await.unwrap();
        let m_amps = ina260.read_current().await.unwrap();
        let m_watts = ina260.read_power().await.unwrap();
        // let (source_voltage, source_current) = husb238.active_power_settings().await.unwrap();

        defmt::info!("{}mV {}mA {}mW", m_volts, m_amps, m_watts);
        // defmt::info!("power source: {}V@{}A", source_voltage, source_current);

        Timer::after(Duration::from_secs(4)).await;
    }
}
