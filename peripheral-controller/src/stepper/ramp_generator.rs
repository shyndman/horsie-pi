use embassy_time::Duration;

use crate::time::ticker::PreemptTicker;

const VACTUAL_TO_USTEP_HZ_FACTOR: f32 = 0.715;
const UPDATE_PERIOD: Duration = Duration::from_hz(15);

pub struct RampGenerator {
    degrees_per_step: f32,
    microsteps_per_step: u16,
    current_speed: i32,
    target_speed: i32,
    acceleration: u32,
    ticker: Option<PreemptTicker>,
}

impl RampGenerator {
    pub fn new(degrees_per_step: f32, microsteps_per_step: u16, acceleration: u32) -> Self {
        Self {
            degrees_per_step,
            microsteps_per_step,
            current_speed: 0,
            target_speed: 0,
            acceleration,
            ticker: None,
        }
    }

    /// Sets the target speed in degrees_per_second, immediately begining ramp generation
    ///
    /// TODO(shyndman): Refactor API to have this method return an object dedicated to
    /// receiving ramp speed updates.
    pub fn set_target_speed(&mut self, val: i32) {
        self.target_speed = val;
        self.ticker = Some(PreemptTicker::every(UPDATE_PERIOD))
    }

    /// Waits until it's time to apply the next speed change, then returns that speed as
    /// a `VACTUAL` that can be written directly to the stepper driver's register.
    ///
    pub async fn next(&mut self) -> i32 {
        if let Some(ref mut ticker) = self.ticker {
            let remaining = self.target_speed - self.current_speed;
            let direction = remaining.signum();

            if remaining.abs() as u32 > self.acceleration {
                ticker.next().await;
                self.current_speed += direction * self.acceleration as i32;
            } else {
                let step_fraction = remaining.abs() as f32 / self.acceleration as f32;
                ticker
                    .next_after(fractional_duration(step_fraction, UPDATE_PERIOD))
                    .await;
                self.current_speed = self.target_speed;
                self.ticker = None;
            }
        }

        self.to_vactual(self.current_speed)
    }

    fn to_vactual(&self, val: i32) -> i32 {
        let steps_per_second = val as f32 / self.degrees_per_step;
        let microsteps_per_second = steps_per_second * self.microsteps_per_step as f32;
        (microsteps_per_second / VACTUAL_TO_USTEP_HZ_FACTOR) as i32
    }
}

fn fractional_duration(fraction: f32, duration: Duration) -> Duration {
    Duration::from_micros((fraction * (duration.as_micros() as f32)) as u64)
}
