use adafruit_seesaw_async::{
    prelude::{EncoderModule, GpioModule, PinMode, StatusModule},
    seesaw_device, Driver, HardwareId, SeesawDeviceInit, SeesawError,
};

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
