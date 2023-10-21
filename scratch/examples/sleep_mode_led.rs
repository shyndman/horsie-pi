#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use panic_probe as _;
use scratch::power_led::animate_power_led;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    unwrap!(spawner.spawn(animate_power_led(p.PWM_CH4, p.PIN_25)));
}
