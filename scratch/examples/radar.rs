//! This example shows powerful PIO module in the RP2040 chip to communicate with WS2812 LED modules.
//! See (https://www.sparkfun.com/categories/tags/ws2812)

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::PIO0,
    pio::{InterruptHandler, Pio},
};
use embassy_time::{Duration, Ticker};
use micromath::F32Ext;
use panic_probe as _;
use rgb::RGB8;
use scratch::ws2812::{
    brightness::adjust_brightness_f, Ws2812Chain, Ws2812FrameProvider,
};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

// #00cf00
const FRAME_DURATION: Duration = Duration::from_hz(120);

struct RadarFrameProvider<const LED_N: usize> {
    primary_color: RGB8,
    sweep_progress: usize,
    frame: [RGB8; LED_N],
    brightness: [f32; LED_N],
}
impl<const LED_N: usize> RadarFrameProvider<LED_N> {
    const STEPS_PER_LED: usize = 4;

    pub fn new(primary_color: RGB8) -> Self {
        Self {
            primary_color,
            sweep_progress: 0,
            frame: [RGB8::new(0, 0, 0); LED_N],
            brightness: [0.0; LED_N],
        }
    }

    pub fn advance(&mut self) {
        let ring_n = LED_N;
        let led0 = self.sweep_progress / Self::STEPS_PER_LED;
        let led1 = (led0 + 1) % ring_n;
        let led_progress: f32 =
            (self.sweep_progress % Self::STEPS_PER_LED) as f32 / Self::STEPS_PER_LED as f32;

        let led0_brightness = (1.0 - led_progress).powf(0.45);
        let led1_brightness = (led_progress).powf(0.45);

        for i in 0..ring_n {
            let brightness = if i == led0 {
                led0_brightness
            } else if i == led1 {
                led1_brightness
            } else {
                0.0
            };

            let cur_brightness = self.brightness[i];
            if cur_brightness < brightness {
                self.brightness[i] = brightness;
                self.frame[i] = adjust_brightness_f(self.primary_color, brightness);
            } else if cur_brightness > 0.0 {
                let brightness = (cur_brightness - 0.018).max(0.0);
                self.brightness[i] = brightness;
                self.frame[i] = adjust_brightness_f(self.primary_color, brightness);
            }
        }
        self.sweep_progress = (self.sweep_progress + 1) % (ring_n * Self::STEPS_PER_LED);
    }
}

impl<const LED_N: usize> Ws2812FrameProvider<LED_N> for RadarFrameProvider<LED_N> {
    fn frame_colors(&self) -> [RGB8; LED_N] {
        self.frame.clone()
    }
}

const OLED_BLUE_COLOR: RGB8 = RGB8::new(0x26, 0xEB, 0xFC);
#[allow(dead_code)]
const GREEN_COLOR: RGB8 = RGB8::new(0x00, 0xFF, 0x00);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Start");
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::High);
    led.set_high();

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);

    // Number of LEDs in the string
    const LED_N: usize = 24;
    let mut radar_frames = RadarFrameProvider::<LED_N>::new(OLED_BLUE_COLOR);
    let mut led_chain =
        Ws2812Chain::new(&mut common, sm0, p.DMA_CH0, p.PIN_16, &radar_frames).await;

    // Loop forever making RGB values and pushing them out to the WS2812.
    let mut ticker = Ticker::every(FRAME_DURATION);

    loop {
        radar_frames.advance();
        led_chain.draw(&radar_frames).await;
        ticker.next().await;
    }
}
