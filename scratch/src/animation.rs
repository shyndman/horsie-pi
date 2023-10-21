use embassy_time::Duration;
use micromath::F32Ext;

pub struct Tween {
    from: f32,
    to: f32,
    duration_ticks: f32,
    easing_fn: EasingFn,
}

impl Tween {
    pub fn new(from: f32, to: f32, duration: Duration, easing_fn: EasingFn) -> Self {
        Self {
            from,
            to,
            duration_ticks: duration.as_ticks() as f32,
            easing_fn,
        }
    }

    pub fn value(&self, elapsed: Duration) -> f32 {
        let t = (elapsed.as_ticks() as f32 % self.duration_ticks) / self.duration_ticks;
        (self.easing_fn)(t) * (self.to - self.from) + self.from
    }
}

pub type EasingFn = fn(f32) -> f32;

pub fn quint_in_out(t: f32) -> f32 {
    if t < 0.5 {
        16.0 * t * t * t * t * t
    } else {
        1.0 - (-2.0 * t + 2.0).powi(5) / 2.0
    }
}

pub fn quad_in_out(t: f32) -> f32 {
    if t < 0.5 {
        2.0 * t * t
    } else {
        1.0 - (-2.0 * t + 2.0).powi(2) / 2.0
    }
}

pub fn cubic_in_out(t: f32) -> f32 {
    if t < 0.5 {
        4.0 * t * t * t
    } else {
        1.0 - (-2.0 * t + 2.0).powi(3) / 2.0
    }
}

pub fn roundtrip(t: f32) -> f32 {
    if t < 0.5 {
        t * 2.0
    } else {
        (1.0 - t) * 2.0
    }
}
