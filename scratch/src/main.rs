#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(custom_test_frameworks)]
#![test_runner(scratch::run_tests)]
#![allow(unused)]

use core::cell::RefCell;

use defmt::{info, unwrap, warn, Debug2Format};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{AnyPin, Input, Pin, Pull},
    peripherals::{PIO0, UART0, USB},
    pio::{self, Pio},
    uart,
    uart::{BufferedInterruptHandler, BufferedUart},
    usb::{self},
};
use embassy_sync::{
    blocking_mutex::{raw::ThreadModeRawMutex, Mutex},
    signal::Signal,
};
use embassy_time::{Duration, Timer};
use panic_probe as _;
use scratch::{
    power_led::animate_power_led,
    stepper::{
        motor_constants::{NEMA11_11HS18_0674S_CONSTANTS, NEMA8_S20STH30_0604A_CONSTANTS},
        tune::tune_driver,
        uart::{Tmc2209UartConnection, UART_BAUD_RATE},
    },
};
use static_cell::make_static;
use tmc2209::reg;

bind_interrupts!(struct Irqs {
    // ADC_IRQ_FIFO => adc::InterruptHandler;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

#[derive(PartialEq, Clone, Copy)]
enum StepperMode {
    Accelerating,
    Decelerating,
}

static TARGET_VELOCITY: Mutex<ThreadModeRawMutex, RefCell<f32>> =
    Mutex::new(RefCell::new(0.0));

const PAN_DRIVER_UART_ADDRESS: u8 = 0;
const TILT_DRIVER_UART_ADDRESS: u8 = 1;

#[embassy_executor::task]
async fn stepper_drivers_task(mut uart: BufferedUart<'static, UART0>) {
    info!("Tuning stepper drivers");

    let mut pan_driver =
        Tmc2209UartConnection::connect(&mut uart, PAN_DRIVER_UART_ADDRESS).await;
    tune_driver(&mut pan_driver, NEMA11_11HS18_0674S_CONSTANTS, &mut uart).await;

    let mut tilt_driver =
        Tmc2209UartConnection::connect(&mut uart, TILT_DRIVER_UART_ADDRESS).await;
    tune_driver(&mut tilt_driver, NEMA8_S20STH30_0604A_CONSTANTS, &mut uart).await;

    let mut vactual = tmc2209::reg::VACTUAL::default();
    unwrap!(tilt_driver.write_register(&mut uart, vactual).await);

    const STEP: i32 = 2000;
    const MAX: i32 = 200000;

    loop {
        let mode = STEPPER_MODE.wait().await;

        info!("Accelerating stepper");

        let (from, to, step) = match mode {
            StepperMode::Accelerating => (vactual.get(), MAX, STEP),
            StepperMode::Decelerating => (vactual.get(), 0, -STEP),
        };

        let mut v = from;
        loop {
            if STEPPER_MODE.signaled() {
                break;
            }

            // let now = Instant::now();
            vactual.set(v);
            unwrap!(pan_driver.write_register(&mut uart, vactual).await);
            // info!("Write in {}us", (Instant::now() - now).as_micros());

            let tstep = pan_driver
                .read_register::<reg::TSTEP, _>(&mut uart)
                .await
                .unwrap();
            info!("TStep? {:?}", tstep.get());

            let sg = pan_driver
                .read_register::<reg::SG_RESULT, _>(&mut uart)
                .await
                .unwrap();
            info!("Stallguard? {:?}", sg.0);

            let status = pan_driver
                .read_register::<reg::IOIN, _>(&mut uart)
                .await
                .unwrap();
            info!("Pins? {:?}", Debug2Format(&status));

            Timer::after(Duration::from_hz(10)).await;

            if v == to {
                break;
            }

            // Increment
            if step < 0 {
                v = (v + step).max(to);
            } else {
                v = (v + step).min(to);
            }
        }
    }
}

static STEPPER_MODE: Signal<ThreadModeRawMutex, StepperMode> = Signal::new();

#[embassy_executor::task]
async fn trigger_stepper_task(button_pin: AnyPin) {
    let mut button_input = Input::new(button_pin, Pull::None);

    let mut mode = StepperMode::Decelerating;
    loop {
        button_input.wait_for_rising_edge().await;

        mode = if mode == StepperMode::Accelerating {
            StepperMode::Decelerating
        } else {
            StepperMode::Accelerating
        };
        STEPPER_MODE.signal(mode);
    }
}

#[embassy_executor::task]
async fn stepper_diagnostic(diag_pin: AnyPin) {
    let mut diag_input = Input::new(diag_pin, Pull::Up);
    loop {
        diag_input.wait_for_rising_edge().await;
        warn!("DIAG pin HIGH!!");
        diag_input.wait_for_low().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Power LED
    unwrap!(spawner.spawn(animate_power_led(p.PWM_CH4, p.PIN_25)));

    // LED display
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    // unwrap!(spawner.spawn(manage_display(Ws2812Chain::new(
    //     &mut common,
    //     sm0,
    //     p.DMA_CH0,
    //     p.PIN_2
    // ))));

    // Stepper diagnostic watch
    unwrap!(spawner.spawn(stepper_diagnostic(p.PIN_18.degrade())));

    // Stepper driver communication
    unwrap!(spawner.spawn(stepper_drivers_task(BufferedUart::new(
        p.UART0,
        Irqs,
        p.PIN_16,
        p.PIN_17,
        &mut make_static!([0u8; 1024])[..],
        &mut make_static!([0u8; 256])[..],
        {
            let mut cfg = uart::Config::default();
            cfg.baudrate = UART_BAUD_RATE;
            cfg
        },
    ))));

    // Wifi chip (for power management)
    // unwrap!(spawner.spawn(configure_power_management(
    //     spawner, p.PIN_23, p.PIN_25, p.PIO0, p.PIN_24, p.PIN_29, p.DMA_CH0
    // )));
}

// #[embassy_executor::task]
// async fn watch_speed_control(adc_p: ADC, input_pin: PIN_28) {
//     use typenum;
//     let adc_config = adc::Config::default();
//     let mut adc = Adc::new(adc_p, Irqs, adc_config);
//     let mut adc_channel = adc::Channel::new_pin(input_pin, Pull::None);

//     let mut filter = median::Filter::<u16, typenum::U40>::new();

//     let mut last_level = 0u16;
//     loop {
//         let sensor_value = adc.read(&mut adc_channel).await.unwrap() >> 2;
//         let level = filter.consume(sensor_value);
//         if level != last_level {
//             info!("Speed control {}", level);
//             last_level = level;
//         }

//         Timer::after(Duration::from_hz(120)).await;
//     }
// }
