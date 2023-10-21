#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

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
use scratch::ws2812::{
    effects::{EffectFrameProvider, MeteorFrameProvider},
    Ws2812Chain,
};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::High);
    led.set_high();

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);

    // Number of LEDs in the string
    const LED_N: usize = 16 * 16;
    let mut effect_frames = MeteorFrameProvider::<LED_N>::new(None, None, None);
    let mut led_chain = Ws2812Chain::<PIO0, 0, LED_N>::new(
        &mut common,
        sm0,
        p.DMA_CH0,
        p.PIN_16,
        &effect_frames,
    )
    .await;

    let mut ticker = Ticker::every(Duration::from_hz(60));
    loop {
        effect_frames.advance();
        led_chain.draw(&effect_frames).await;
        ticker.next().await;
    }
}
