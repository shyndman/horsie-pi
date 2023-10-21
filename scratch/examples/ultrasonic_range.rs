#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::{self, Write};

use defmt::{unwrap, warn};
// use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Level, Output},
    i2c::{Blocking, Config, I2c, Instance, SclPin, SdaPin},
    peripherals::I2C0,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker, Timer};
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use panic_probe as _;
use profont::{PROFONT_12_POINT, PROFONT_14_POINT, PROFONT_24_POINT};
use scratch::ultrasonic_range::hc_sr04::HcSr04RangeFinder;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

static RANGE_SIGNAL: Signal<CriticalSectionRawMutex, f32> = Signal::new();

async fn run_displays<P0: Instance>(bigger_i2c: I2c<'static, P0, Blocking>) {
    let mut bigger_display = Ssd1306::new(
        I2CDisplayInterface::new_alternate_address(bigger_i2c),
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    bigger_display.init().unwrap();
    bigger_display
        .set_brightness(Brightness::BRIGHTEST)
        .unwrap();

    let title_style = MonoTextStyleBuilder::new()
        .font(&PROFONT_12_POINT)
        .text_color(BinaryColor::On)
        .build();
    let data_text_style = MonoTextStyleBuilder::new()
        .font(&PROFONT_24_POINT)
        .text_color(BinaryColor::On)
        .build();
    let data_unit_text_style = MonoTextStyleBuilder::new()
        .font(&PROFONT_14_POINT)
        .text_color(BinaryColor::On)
        .build();

    let mut ticker = Ticker::every(Duration::from_hz(30));
    loop {
        let range_cm = RANGE_SIGNAL.wait().await;
        bigger_display.clear(BinaryColor::Off).unwrap();

        Text::with_baseline("Range", Point::zero(), title_style, Baseline::Top)
            .draw(&mut bigger_display)
            .unwrap();

        let mut w = FixedSizeWriter::<{ 3 + 1 + 1 }>::new();
        write!(w, "{:>5.1}", range_cm).unwrap();

        let pt = Text::with_baseline(
            w.string_value(),
            Point::new(0, title_style.font.character_size.height as i32 + 2),
            data_text_style,
            Baseline::Top,
        )
        .draw(&mut bigger_display)
        .unwrap();

        Text::with_baseline(
            "cm",
            pt + Size::new(2, data_text_style.font.character_size.height - 3),
            data_unit_text_style,
            Baseline::Bottom,
        )
        .draw(&mut bigger_display)
        .unwrap();

        bigger_display.flush().unwrap();

        ticker.next().await;
    }
}

struct FixedSizeWriter<const SIZE: usize> {
    string: [u8; SIZE],
    i: usize,
}

impl<const SIZE: usize> FixedSizeWriter<SIZE> {
    fn new() -> Self {
        Self {
            string: [0u8; SIZE],
            i: 0,
        }
    }

    fn string_value(&self) -> &str {
        core::str::from_utf8(&self.string[0..self.i]).unwrap()
    }
}

impl<const SIZE: usize> Write for FixedSizeWriter<SIZE> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.bytes() {
            self.string[self.i] = c;
            self.i += 1;
        }
        Ok(())
    }
}

/// Drives a HC-SR04 ultrasonic rangefinder
#[embassy_executor::task]
async fn report_ultrasonic_distance(mut range_finder: HcSr04RangeFinder<'static>) {
    loop {
        match range_finder.get_range_cm().await {
            Ok(cm) => {
                RANGE_SIGNAL.signal(cm);
            }
            Err(_) => {
                warn!("Timeout!");
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Keep LED on
    let mut led = Output::new(p.PIN_25, Level::High);
    led.set_high();

    // Begin measuring distance
    let range_finder = HcSr04RangeFinder::new(p.PIN_15.into(), p.PIN_14.into(), Some(20.0));
    unwrap!(spawner.spawn(report_ultrasonic_distance(range_finder)));

    // Configure OLED for displaying distance
    type I2C0SclPin = impl SclPin<I2C0>;
    type I2C0SdaPin = impl SdaPin<I2C0>;

    #[embassy_executor::task]
    async fn run_displays_internal(i2c0_p: I2C0, scl0: I2C0SclPin, sda0: I2C0SdaPin) {
        // drive_bigger_display(i2c_p, scl, sda, Irqs).await;
        run_displays(I2c::new_blocking(i2c0_p, scl0, sda0, {
            let mut c = Config::default();
            c.frequency = 400_000;
            c
        }))
        .await;
    }
    unwrap!(spawner.spawn(run_displays_internal(p.I2C0, p.PIN_21, p.PIN_20)));

    loop {
        Timer::after(Duration::from_hz(5)).await;
    }
}
