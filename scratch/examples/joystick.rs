//! This example test the ADC (Analog to Digital Conversion) of the RS2040 pin 26, 27 and 28.
//! It also reads the temperature sensor in the chip.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    adc::{Adc, Channel, Config, InterruptHandler},
    bind_interrupts,
    gpio::{Input, Level, Pull},
};
use embassy_time::{Duration, Timer};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => InterruptHandler;
});

const ADC_DISCARD_BITS: u32 = 0;
const ADC_MAX: f32 = (4096 >> ADC_DISCARD_BITS) as f32;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("{}", ADC_MAX);

    let p = embassy_rp::init(Default::default());

    let mut adc = Adc::new(p.ADC, Irqs, Config::default());
    let mut x_channel = Channel::new_pin(p.PIN_26, Pull::None);
    let mut y_channel = Channel::new_pin(p.PIN_27, Pull::None);
    let button_input = Input::new(p.PIN_22, Pull::Up);

    let mut x_filter = median::Filter::<u16, typenum::U40>::new();
    let mut y_filter = median::Filter::<u16, typenum::U40>::new();

    let mut was_pressed = true;
    let mut counter = 0;
    loop {
        let x_sample = adc.read(&mut x_channel).await.unwrap();
        let x_filtered = x_filter.consume(x_sample >> ADC_DISCARD_BITS);
        let y_sample = adc.read(&mut y_channel).await.unwrap();
        let y_filtered = y_filter.consume(y_sample >> ADC_DISCARD_BITS);
        let pressed = button_input.get_level() == Level::Low;

        // Invert, and position between [-1, 1]
        let position = (
            1.0 - (x_filtered as f32 / ADC_MAX * 2.0),
            1.0 - (y_filtered as f32 / ADC_MAX * 2.0),
        );

        if counter % 180 == 0 {
            info!("position: {}", position);
        }
        if pressed != was_pressed {
            info!("pressed? {}", pressed);
        }

        Timer::after(Duration::from_hz(180)).await;

        counter += 1;
        was_pressed = pressed;
    }
}
