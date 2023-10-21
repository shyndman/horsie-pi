//! This example shows powerful PIO module in the RP2040 chip to communicate with WS2812 LED modules.
//! See (https://www.sparkfun.com/categories/tags/ws2812)

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Level, Output},
    i2c::{Blocking, Config, I2c, Instance, SclPin, SdaPin},
    peripherals::{I2C0, I2C1},
};
use embassy_time::{Duration, Ticker, Timer};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X13_BOLD, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Triangle},
    text::{Baseline, Text},
};
use panic_probe as _;
use profont::{PROFONT_7_POINT, PROFONT_9_POINT};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

// bind_interrupts!(struct Irqs {
//   I2C0_IRQ => InterruptHandler<I2C0>;
//   I2C1_IRQ => InterruptHandler<I2C1>;
// });

async fn run_displays<P1: Instance, P2: Instance>(
    bigger_i2c: I2c<'static, P2, Blocking>,
    longer_i2c: I2c<'static, P1, Blocking>,
) {
    let mut longer_display = Ssd1306::new(
        I2CDisplayInterface::new(longer_i2c),
        DisplaySize128x32,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    longer_display.init().unwrap();
    longer_display
        .set_brightness(Brightness::BRIGHTEST)
        .unwrap();

    let mut bigger_display = Ssd1306::new(
        I2CDisplayInterface::new_alternate_address(bigger_i2c),
        DisplaySize128x64,
        DisplayRotation::Rotate90,
    )
    .into_buffered_graphics_mode();
    bigger_display.init().unwrap();
    bigger_display
        .set_brightness(Brightness::BRIGHTEST)
        .unwrap();

    let primitive_style = PrimitiveStyleBuilder::new()
        .stroke_width(2)
        .stroke_color(BinaryColor::On)
        .build();

    let title_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X13_BOLD)
        .text_color(BinaryColor::On)
        .build();
    let data_text_style = MonoTextStyleBuilder::new()
        .font(&PROFONT_9_POINT)
        .text_color(BinaryColor::On)
        .build();

    let small_title_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X13_BOLD)
        .text_color(BinaryColor::On)
        .build();
    let small_data_text_style = MonoTextStyleBuilder::new()
        .font(&PROFONT_7_POINT)
        .text_color(BinaryColor::On)
        .build();

    let mut ticker = Ticker::every(Duration::from_hz(10));
    loop {
        {
            Text::with_baseline(
                "Horsie Pi!",
                Point::zero(),
                small_title_style,
                Baseline::Top,
            )
            .draw(&mut longer_display)
            .unwrap();

            Text::with_baseline(
                "CPU: 40.5%",
                Point::new(0, 13),
                small_data_text_style,
                Baseline::Top,
            )
            .draw(&mut longer_display)
            .unwrap();

            Text::with_baseline(
                "Disk Usage: 10.5%",
                Point::new(0, 22),
                small_data_text_style,
                Baseline::Top,
            )
            .draw(&mut longer_display)
            .unwrap();

            const YOFFSET: i32 = 0;
            const XOFFSET: i32 = 128 - 24;
            Triangle::new(
                Point::new(XOFFSET, 16 + YOFFSET),
                Point::new(XOFFSET + 16, 16 + YOFFSET),
                Point::new(XOFFSET + 8, YOFFSET),
            )
            .into_styled(primitive_style)
            .draw(&mut longer_display)
            .unwrap();
        }

        {
            Text::with_baseline("Horsie Pi!", Point::zero(), title_style, Baseline::Top)
                .draw(&mut bigger_display)
                .unwrap();

            let pt = Text::with_baseline(
                "CPU:\n40.5%",
                Point::new(0, title_style.font.character_size.height as i32 + 2),
                data_text_style,
                Baseline::Top,
            )
            .draw(&mut bigger_display)
            .unwrap();

            Text::with_baseline(
                "Disk Usage:\n10.5%",
                Point::new(
                    0,
                    pt.y + data_text_style.font.character_size.height as i32 + 2,
                ),
                data_text_style,
                Baseline::Top,
            )
            .draw(&mut bigger_display)
            .unwrap();

            const YOFFSET: i32 = 128 - 24;
            const XOFFSET: i32 = 64 - 24;
            Triangle::new(
                Point::new(XOFFSET, 16 + YOFFSET),
                Point::new(XOFFSET + 16, 16 + YOFFSET),
                Point::new(XOFFSET + 8, YOFFSET),
            )
            .into_styled(primitive_style)
            .draw(&mut bigger_display)
            .unwrap();
        }

        bigger_display.flush().unwrap();
        longer_display.flush().unwrap();

        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Start");
    let p = embassy_rp::init(Default::default());

    // Keep LED on
    let mut led = Output::new(p.PIN_25, Level::High);
    led.set_high();

    type I2C0SclPin = impl SclPin<I2C0>;
    type I2C0SdaPin = impl SdaPin<I2C0>;
    type I2C1SclPin = impl SclPin<I2C1>;
    type I2C1SdaPin = impl SdaPin<I2C1>;

    #[embassy_executor::task]
    async fn run_displays_internal(
        i2c0_p: I2C0,
        scl0: I2C0SclPin,
        sda0: I2C0SdaPin,
        i2c1_p: I2C1,
        scl1: I2C1SclPin,
        sda1: I2C1SdaPin,
    ) {
        // drive_bigger_display(i2c_p, scl, sda, Irqs).await;
        run_displays(
            I2c::new_blocking(i2c0_p, scl0, sda0, {
                let mut c = Config::default();
                c.frequency = 400_000;
                c
            }),
            I2c::new_blocking(i2c1_p, scl1, sda1, {
                let mut c = Config::default();
                c.frequency = 400_000;
                c
            }),
        )
        .await;
    }

    unwrap!(spawner.spawn(run_displays_internal(
        p.I2C0, p.PIN_21, p.PIN_20, p.I2C1, p.PIN_19, p.PIN_18
    )));

    loop {
        Timer::after(Duration::from_hz(5)).await;
    }
}
