use embassy_rp::peripherals::{PIN_25, PWM_CH4};
use embassy_rp::pwm::{self, Pwm};
use embassy_time::{Duration, Instant, Timer};

use crate::animation::{quint_in_out, roundtrip, Tween};

const LED_FRAME_TIME: Duration = Duration::from_hz(60);

#[embassy_executor::task]
pub async fn animate_power_led(pwm_channel: PWM_CH4, pwm_pin: PIN_25) {
    let mut pwm_config = {
        let mut c = pwm::Config::default();
        c.top = 0x8000;
        c.compare_b = 8;
        c
    };
    let mut led = Pwm::new_output_b(pwm_channel, pwm_pin, pwm_config.clone());

    let breath_tween =
        Tween::new(32768.0 / 16.0, 32768.0, Duration::from_secs(5), |t: f32| {
            roundtrip(quint_in_out(t))
        });
    let start = Instant::now();
    loop {
        let duty = breath_tween.value(Instant::now() - start) as u16;
        pwm_config.compare_b = duty;
        led.set_config(&pwm_config);
        Timer::after(LED_FRAME_TIME).await;
    }
}
