#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    i2c::{self, I2c},
    peripherals::I2C0,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use hp_embedded_drivers::ina260_async::Ina260;
use panic_probe as _;
use static_cell::make_static;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let i2c_bus: &'static mut Mutex<ThreadModeRawMutex, I2c<'static, I2C0, i2c::Async>> =
        make_static!({
            let b = I2c::new_async(p.I2C0, p.PIN_21, p.PIN_20, Irqs, i2c::Config::default());
            let b = Mutex::<ThreadModeRawMutex, _>::new(b);
            b
        });

    let _power_sensor = match Ina260::new(I2cDevice::new(i2c_bus)).await {
        Ok(dev) => dev,
        Err(_) => {
            error!("Failed to initialize INA260 power sensor");
            core::panic!("noo!");
        }
    };

    // NOTE: More devices can be added to the bus freely
    //
    // let i2c_dev2 = I2cDevice::new(&i2c_bus);

    loop {
        Timer::after(Duration::from_secs(10)).await;
    }
}
