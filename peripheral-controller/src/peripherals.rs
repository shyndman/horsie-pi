use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use esp32s3_hal as hal;
use esp_hal_common as hal_common;
use hal_common::prelude::_fugit_RateExtU32;

use crate::shared_bus::DualModeI2cDevice;

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
    let i2c = hal::i2c::I2C::new(peripheral, sda_pin, scl_pin, 400u32.kHz(), clocks);

    // TODO(shyndman): How can we enable only the interrupt we need?
    hal::interrupt::enable(
        hal::peripherals::Interrupt::I2C_EXT0,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    i2c
}

pub type PeripheralI2cLink<I2CP> =
    DualModeI2cDevice<'static, CriticalSectionRawMutex, hal::i2c::I2C<'static, I2CP>>;

pub fn new_uart_bus<
    P: esp_hal_common::uart::Instance,
    TX: hal_common::gpio::OutputPin,
    RX: hal_common::gpio::InputPin,
>(
    peripheral: impl hal_common::peripheral::Peripheral<P = P> + 'static,
    tx_pin: impl hal_common::peripheral::Peripheral<P = TX>,
    rx_pin: impl hal_common::peripheral::Peripheral<P = RX>,
    baudrate: u32,
    clocks: &hal_common::clock::Clocks,
) -> hal::uart::Uart<'static, P> {
    let uart = hal::uart::Uart::new_with_config(
        peripheral,
        hal::uart::config::Config {
            baudrate,
            ..hal::uart::config::Config::default()
        },
        Some(hal::uart::TxRxPins::new_tx_rx(tx_pin, rx_pin)),
        clocks,
    );
    let reg = P::register_block();
    reg.conf0.modify(|_, w| {
        w.rxfifo_rst().set_bit();
        w
    });
    reg.conf0.modify(|_, w| {
        w.rxfifo_rst().clear_bit();
        w
    });
    reg.conf0.modify(|_, w| {
        w.txfifo_rst().set_bit();
        w
    });
    reg.conf0.modify(|_, w| {
        w.txfifo_rst().clear_bit();
        w
    });
    hal::interrupt::enable(
        hal::peripherals::Interrupt::UART0,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();
    hal::interrupt::enable(
        hal::peripherals::Interrupt::UART1,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();
    hal::interrupt::enable(
        hal::peripherals::Interrupt::UART2,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();
    uart
}