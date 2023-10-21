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
use panic_probe as _;
use rgb::RGB8;
use scratch::ws2812::{Ws2812Chain, Ws2812FrameProvider};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

/// Input a value 0 to 255 to get a color value
/// The colours are a transition r - g - b - back to r.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}

struct RainbowFrameSource<const LED_N: usize> {
    hue_position: u16,
    frame: [RGB8; LED_N],
}
impl<const LED_N: usize> RainbowFrameSource<LED_N> {
    pub fn new() -> Self {
        Self {
            hue_position: 0,
            frame: [RGB8::new(0, 0, 0); LED_N],
        }
    }

    pub fn advance(&mut self) {
        self.hue_position += 1;

        for i in 0..LED_N {
            self.frame[i] = wheel(
                (((i * 256) as u16 / LED_N as u16 + self.hue_position as u16) & 255) as u8,
            );
        }
    }
}
impl<const LED_N: usize> Ws2812FrameProvider<LED_N> for RainbowFrameSource<LED_N> {
    fn frame_colors(&self) -> [RGB8; LED_N] {
        self.frame.clone()
    }
}

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
    let mut rainbow_frames = RainbowFrameSource::<LED_N>::new();
    let mut led_chain = Ws2812Chain::<PIO0, 0, LED_N>::new(
        &mut common,
        sm0,
        p.DMA_CH0,
        p.PIN_16,
        &rainbow_frames,
    )
    .await;

    let mut ticker = Ticker::every(Duration::from_hz(120));
    loop {
        rainbow_frames.advance();
        led_chain.draw(&rainbow_frames).await;
        ticker.next().await;
    }
}
