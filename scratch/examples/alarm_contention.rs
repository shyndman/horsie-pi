//! This example shows powerful PIO module in the RP2040 chip to communicate with WS2812 LED modules.
//! See (https://www.sparkfun.com/categories/tags/ws2812)

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::{Duration, Instant, Timer};
use panic_probe as _;
use rand::{rngs::SmallRng, *};

const SLEEP_DURATION: Duration = Duration::from_secs(2);

#[embassy_executor::task(pool_size = 255)]
async fn sleep(i: u8) {
    let mut rng = SmallRng::seed_from_u64(i as u64);
    let ticks = SLEEP_DURATION.as_ticks();
    loop {
        let duration = Duration::from_ticks(rng.gen_range((ticks / 2)..(ticks * 3 / 2)));
        let ts = Instant::now();
        Timer::after(duration).await;
        info!(
            "[{}] delta us: {}",
            i,
            (ts.elapsed() - duration).as_micros()
        );
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Keep LED on
    let mut led = Output::new(p.PIN_25, Level::High);
    led.set_high();

    for i in 0u8..255 {
        unwrap!(spawner.spawn(sleep(i)));
    }

    loop {
        let ts = Instant::now();
        Timer::after(SLEEP_DURATION).await;
        info!("delta us: {}", (ts.elapsed() - SLEEP_DURATION).as_micros());
    }
}
