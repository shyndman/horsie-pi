use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use esp32s3_hal as hal;
use esp_hal_common as hal_common;
use hal_common::prelude::_fugit_RateExtU32;

pub type PeripheralI2cLink<I2CP> =
    I2cDevice<'static, CriticalSectionRawMutex, hal::i2c::I2C<'static, I2CP>>;

pub fn new_i2c<
    I2CP: esp_hal_common::i2c::Instance,
    SDA: hal_common::gpio::InputPin + hal_common::gpio::OutputPin,
    SCL: hal_common::gpio::InputPin + hal_common::gpio::OutputPin,
>(
    peripheral: impl hal_common::peripheral::Peripheral<P = I2CP> + 'static,
    sda_pin: impl hal_common::peripheral::Peripheral<P = SDA> + 'static,
    scl_pin: impl hal_common::peripheral::Peripheral<P = SCL> + 'static,
    clocks: &hal_common::clock::Clocks,
) -> hal::i2c::I2C<'static, I2CP> {
    let i2c = hal::i2c::I2C::new(peripheral, sda_pin, scl_pin, 100u32.kHz(), clocks);

    // TODO(shyndman): How can we enable only the interrupt we need?
    hal::interrupt::enable(
        hal::peripherals::Interrupt::I2C_EXT0,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    i2c
}
