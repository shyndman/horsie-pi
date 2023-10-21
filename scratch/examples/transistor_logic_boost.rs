#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Level, Output},
    pwm::{self, Pwm},
};
use embassy_time::{Duration, Timer};
use fixed::traits::ToFixed;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Example booted");

    let p = embassy_rp::init(Default::default());
    let mut switch = Output::new(p.PIN_14, Level::Low);

    let pwm_out = Pwm::new_output_b(p.PWM_CH7, p.PIN_15, {
        let mut c = pwm::Config::default();
        c.divider = 255.to_fixed();
        c.top = 0xFFFF;
        c.compare_b = 0xFFFF / 2;
        c
    });

    loop {
        Timer::after(Duration::from_secs(1)).await;
        switch.toggle();
        info!("Count at {}", pwm_out.counter());
    }
}
