use embassy_time::{block_for, with_timeout, Duration, Instant, TimeoutError, Timer};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

/// Speed of sound at 0C in m/s.
const SOUND_SPEED_0C: f32 = 331.3;

/// Increase speed of sound over temperature factor m/[sC].
const SOUND_SPEED_INC_OVER_TEMP: f32 = 0.606;

/// Maximum measuring range for HC-SR04 sensor in m.
const MAX_RANGE: f32 = 4.0;

const MIN_TRIGGER_DURATION: Duration = Duration::from_micros(10);
const MIN_MEASUREMENT_CYCLE: Duration = Duration::from_millis(60);

pub struct HcSr04RangeFinder<O: OutputPin, I: InputPin + Wait> {
    // m/us
    speed_of_sound_at_temp: f32,
    echo_timeout: Duration,
    trigger_pin: O,
    echo_pin: I,
    last_trigger_ts: Instant,
}

impl<O: OutputPin, I: InputPin + Wait> HcSr04RangeFinder<O, I> {
    ///
    pub fn new(trigger_pin: O, echo_pin: I, temperature_c: Option<f32>) -> Self {
        // echo_pin.
        let (speed, timeout) =
            speed_and_echo_timeout_at_temperature(temperature_c.unwrap_or(20.0));

        Self {
            speed_of_sound_at_temp: speed,
            echo_timeout: timeout,
            trigger_pin: trigger_pin,
            echo_pin: echo_pin,
            last_trigger_ts: Instant::MIN,
        }
    }

    pub fn set_ambient_temperature(&mut self, temperature_c: f32) {
        (self.speed_of_sound_at_temp, self.echo_timeout) =
            speed_and_echo_timeout_at_temperature(temperature_c);
    }

    pub async fn get_range_cm(&mut self) -> Result<f32, TimeoutError> {
        let since_last_ranging = Instant::now() - self.last_trigger_ts;
        if since_last_ranging < MIN_MEASUREMENT_CYCLE {
            Timer::after(MIN_MEASUREMENT_CYCLE - since_last_ranging).await;
        }

        self.send_trigger_signal();

        with_timeout(self.echo_timeout, self.echo_pin.wait_for_high())
            .await?
            .unwrap();

        let elapsed_us = {
            let ts = Instant::now();
            with_timeout(self.echo_timeout, self.echo_pin.wait_for_low())
                .await?
                .unwrap();
            ts.elapsed().as_micros() as f32
        };

        Ok(self.speed_of_sound_at_temp * elapsed_us / 2. * 100.)
    }

    fn send_trigger_signal(&mut self) {
        self.last_trigger_ts = Instant::now();
        self.trigger_pin.set_high().unwrap();
        // TODO(shyndman): Determine whether the blocking is necessary
        block_for(MIN_TRIGGER_DURATION);
        self.trigger_pin.set_low().unwrap();
    }
}

fn speed_and_echo_timeout_at_temperature(temperature_c: f32) -> (f32, Duration) {
    let speed_mps = SOUND_SPEED_0C + SOUND_SPEED_INC_OVER_TEMP * temperature_c;
    let ranging_timeout =
        Duration::from_micros(((2. * MAX_RANGE) / speed_mps * 1_000_000.) as u64);

    (speed_mps / 1_000_000., ranging_timeout)
}
