//! This was built to run on the S3Tiny

#![no_std]
#![no_main]
#![feature(
    asm_experimental_arch,
    async_fn_in_trait,
    const_mut_refs,
    custom_test_frameworks,
    exclusive_range_pattern,
    impl_trait_projections,
    return_position_impl_trait_in_trait,
    type_alias_impl_trait
)]

extern crate alloc;

use core::fmt::Debug;

use adafruit_seesaw_async::{
    prelude::{EncoderModule, GpioModule, PinMode, StatusModule},
    seesaw_device, Driver, SeesawDevice, SeesawDeviceInit, SeesawError,
};
use defmt::Debug2Format;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use esp32s3_hal::{
    self as hal,
    clock::ClockControl,
    embassy, interrupt,
    peripherals::{Interrupt, Peripherals},
    prelude::*,
    system::SystemParts,
    Rmt, IO,
};
use esp_hal_common::{
    clock::Clocks,
    gpio::{Gpio7, InputPin, OutputPin, Unknown},
    peripheral::Peripheral,
};
use esp_hal_procmacros::main;
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
use esp_println::logger::init_logger_from_env;
use peripheral_controller::init_heap;
use rgb::RGB8;
use smart_leds::brightness;
use smart_leds_trait::SmartLedsWrite;
use static_cell::make_static;

#[main]
async fn main(spawner: Spawner) {
    init_heap();
    init_logger_from_env();

    defmt::info!("Init!");

    let peripherals = Peripherals::take();
    let system: SystemParts = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();

    embassy::init(
        &clocks,
        esp32s3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    esp32s3_hal::interrupt::enable(
        esp32s3_hal::peripherals::Interrupt::GPIO,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    Timer::after(Duration::from_secs(1)).await;

    {
        io.pins.gpio17.into_push_pull_output().set_high().unwrap();
        let mut led = <smartLedAdapter!(0, 1)>::new(
            rmt.channel0,
            io.pins.gpio18.into_push_pull_output(),
        );
        led.write(brightness(
            core::iter::once(RGB8::new(0x33, 0x66, 0x00)),
            90,
        ))
        .unwrap();
    }

    let i2c0 = new_i2c(
        peripherals.I2C0,
        io.pins.gpio8.into_floating_input(),
        io.pins.gpio9.into_floating_input(),
        &clocks,
    );
    let i2c0_bus: &'static mut Mutex<
        CriticalSectionRawMutex,
        esp_hal_common::i2c::I2C<'static, esp_hal_common::peripherals::I2C0>,
    > = make_static!({
        Mutex::<
            CriticalSectionRawMutex,
            esp_hal_common::i2c::I2C<'static, esp_hal_common::peripherals::I2C0>,
        >::new(i2c0)
    });

    spawner
        .spawn(capture_input(I2cDevice::new(i2c0_bus), io.pins.gpio7))
        .unwrap();

    loop {
        Timer::after(Duration::from_secs(30)).await;
    }
}

#[embassy_executor::task]
async fn capture_input(
    encoder_link: PeripheralI2cLink<hal::peripherals::I2C0>,
    interrupt_pin: Gpio7<Unknown>,
) {
    defmt::info!("Initializing RotaryEncoder device");
    let rotary_encoder = AnoRotaryEncoder::new_with_default_addr(
        adafruit_seesaw_async::SeesawDriver::new(encoder_link, embassy_time::Delay),
    );
    let mut rotary_encoder = match rotary_encoder.init().await {
        Ok(encoder) => {
            defmt::info!("Rotary encoder initialized");
            encoder
        }
        Err(e) => {
            defmt::error!("Rotary encoder failed to initialize, {}", Debug2Format(&e));
            panic!("nooo!");
        }
    };

    let mut interrupt_pin = interrupt_pin.into_floating_input();
    loop {
        interrupt_pin.wait_for_low().await.ok();

        let press_states = rotary_encoder.button_states().await.unwrap();
        let position = rotary_encoder.position().await.unwrap();
        defmt::info!("Position: {}, Buttons: {}", position, press_states);

        rotary_encoder.consume_interrupt_state().await.unwrap();
        rotary_encoder.reset_interrupts().await.unwrap();
    }
}

use adafruit_seesaw_async::HardwareId;
seesaw_device! {
    name: AnoRotaryEncoder,
    hardware_id: HardwareId::ATTINY817,
    product_id: 5740,
    default_addr: 0x49,
    modules: [
        GpioModule,
        EncoderModule { button_pin: 0 },
    ]
}

impl<D: Driver> SeesawDeviceInit<D> for AnoRotaryEncoder<D> {
    async fn init(mut self) -> Result<Self, Self::Error> {
        self.reset_and_verify_seesaw().await?;
        self.init_buttons().await?;
        self.enable_interrupt().await?;
        Ok(self)
    }
}

impl<D: Driver> AnoRotaryEncoder<D> {
    /// Set the pin mode of the 4 buttons to input pullup:
    pub async fn init_buttons(&mut self) -> Result<(), SeesawError<D::Error>> {
        self.set_pin_mode_bulk(0b111111, PinMode::InputPullup)
            .await?;
        self.set_gpio_interrupts(!0u32, true).await?;
        Ok(())
    }

    pub async fn button_states(&mut self) -> Result<ButtonStates, SeesawError<D::Error>> {
        let pressed_bits = self.digital_read_bulk().await?;
        Ok(ButtonStates {
            select_pressed: (pressed_bits & 0b000010) == 0,
            left_pressed: (pressed_bits & 0b000100) == 0,
            down_pressed: (pressed_bits & 0b001000) == 0,
            right_pressed: (pressed_bits & 0b010000) == 0,
            up_pressed: (pressed_bits & 0b100000) == 0,
        })
    }

    pub async fn reset_interrupts(&mut self) -> Result<(), SeesawError<D::Error>> {
        self.enable_interrupt().await?;
        self.set_gpio_interrupts(!0u32, true).await.unwrap();
        Ok(())
    }
}

#[derive(defmt::Format)]
pub struct ButtonStates {
    pub select_pressed: bool,
    pub up_pressed: bool,
    pub left_pressed: bool,
    pub down_pressed: bool,
    pub right_pressed: bool,
}

type PeripheralI2cLink<I2CP> =
    I2cDevice<'static, CriticalSectionRawMutex, hal::i2c::I2C<'static, I2CP>>;

fn new_i2c<
    I2CP: esp_hal_common::i2c::Instance,
    SDA: InputPin + OutputPin,
    SCL: InputPin + OutputPin,
>(
    peripheral: impl Peripheral<P = I2CP> + 'static,
    sda_pin: impl Peripheral<P = SDA> + 'static,
    scl_pin: impl Peripheral<P = SCL> + 'static,
    clocks: &Clocks,
) -> hal::i2c::I2C<'static, I2CP> {
    let i2c = hal::i2c::I2C::new(peripheral, sda_pin, scl_pin, 100u32.kHz(), clocks);

    // TODO(shyndman): How can we enable only the interrupt we need?
    interrupt::enable(Interrupt::I2C_EXT0, interrupt::Priority::Priority1).unwrap();

    i2c
}
