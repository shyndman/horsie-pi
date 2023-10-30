use embassy_time::{Duration, Instant, Timer};

pub struct PreemptTicker {
    measured_from: Instant,
    duration: Duration,
}

impl PreemptTicker {
    /// Creates a new ticker that ticks at the specified duration interval
    pub fn every(duration: Duration) -> Self {
        Self {
            measured_from: Instant::now(),
            duration,
        }
    }

    /// Waits until the standard `duration` has elapsed since the last tick time
    pub async fn next(&mut self) {
        self.next_after(self.duration).await;
    }

    /// Waits until `duration` has elapsed since the last tick time
    pub async fn next_after(&mut self, duration: Duration) {
        let expires_at = self.measured_from + duration;
        if expires_at > Instant::now() {
            Timer::at(expires_at).await;
        }
        self.measured_from = expires_at;
    }
}
