use embedded_hal_1::digital::OutputPin;
use esp32s3_hal::{
    gpio::{AnyPin, GpioPin, Output, PushPull, Unknown},
    mcpwm::{
        operator::{PwmPin, PwmPinConfig},
        timer::PwmWorkingMode,
        PeripheralClockConfig, MCPWM,
    },
    prelude::_fugit_RateExtU32,
};
use rgb::RGB8;

/// A PM162-K button, with red, green, and blue inputs.
///
/// https://cdn-shop.adafruit.com/product-files/4660/C13114++C15110.pdf
pub struct RgbButton<'d> {
    // While we don't use the field, we want to keep ownership
    _anode_pin: AnyPin<Output<PushPull>>,
    red_pin: PwmPin<'d, GpioPin<Unknown, 40>, esp32s3_hal::peripherals::MCPWM0, 0, true>,
    green_pin: PwmPin<'d, GpioPin<Unknown, 41>, esp32s3_hal::peripherals::MCPWM0, 1, true>,
    blue_pin: PwmPin<'d, GpioPin<Unknown, 42>, esp32s3_hal::peripherals::MCPWM0, 2, true>,
}

impl<'d> RgbButton<'d> {
    /// The provided pwm will be reconfigured appropriately for the button.
    pub fn new(
        red_pin: GpioPin<Unknown, 40>,
        green_pin: GpioPin<Unknown, 41>,
        blue_pin: GpioPin<Unknown, 42>,
        mut pwm: MCPWM<'static, esp32s3_hal::peripherals::MCPWM0>,
        mut anode_pin: AnyPin<Output<PushPull>>,
        clock_config: PeripheralClockConfig,
    ) -> Self {
        let timer = &mut pwm.timer0;
        pwm.operator0.set_timer(timer);
        let red_pin = pwm
            .operator0
            .with_pin_a(red_pin, PwmPinConfig::UP_ACTIVE_HIGH);
        pwm.operator1.set_timer(timer);
        let green_pin = pwm
            .operator1
            .with_pin_a(green_pin, PwmPinConfig::UP_ACTIVE_HIGH);
        pwm.operator2.set_timer(timer);
        let blue_pin = pwm
            .operator2
            .with_pin_a(blue_pin, PwmPinConfig::UP_ACTIVE_HIGH);

        anode_pin.set_high().unwrap();

        let button = Self {
            red_pin,
            green_pin,
            blue_pin,
            _anode_pin: anode_pin,
        };

        let timer_clock_cfg = clock_config
            .timer_clock_with_frequency(0xFF - 1, PwmWorkingMode::Increase, 10u32.kHz())
            .unwrap();
        timer.start(timer_clock_cfg);
        button
    }

    pub fn set_color(&mut self, color: RGB8) {
        self.red_pin
            .set_timestamp((0xFF - color.r as u16).saturating_sub(30));
        self.green_pin
            .set_timestamp((0xFF - color.g as u16).saturating_sub(30));
        self.blue_pin
            .set_timestamp((0xFF - color.b as u16).saturating_sub(30));
    }
}
