use rand::prelude::*;
use rgb::RGB8;

use super::EffectFrameProvider;
use crate::ws2812::{brightness::adjust_brightness_f, Ws2812FrameProvider};

pub struct MeteorFrameProvider<const LED_N: usize> {
    color: RGB8,
    size: usize,
    position: usize,
    fade: f32,
    random_color: bool,
    frame: [RGB8; LED_N],
}

impl<const LED_N: usize> MeteorFrameProvider<LED_N> {
    pub fn new(color: Option<RGB8>, size: Option<usize>, fade: Option<f32>) -> Self {
        Self {
            color: color.unwrap_or(RGB8::new(0xFF, 0xFF, 0xFF)),
            size: size.unwrap_or(4),
            fade: fade.unwrap_or(0.4),
            random_color: color.is_none(),
            position: 0,
            frame: [RGB8::new(0, 0, 0); LED_N],
        }
    }
}

impl<const LED_N: usize> EffectFrameProvider for MeteorFrameProvider<LED_N> {
    fn advance(&mut self) {
        let mut rng = SmallRng::seed_from_u64(0x00101010);

        for c in self.frame.iter_mut() {
            if rng.gen_range(0.0..1.0) < 0.5 {
                *c = adjust_brightness_f(*c, self.fade);
            }
        }

        for i in 0..self.size {
            if (self.position.saturating_sub(i) < LED_N) && (self.position >= i) {
                self.frame[self.position - i] = self.color;
            }
        }
        self.position += 1;
        if self.position > 2 * LED_N {
            if self.random_color {
                self.color = RGB8::new(
                    rng.gen_range(0..0xFF),
                    rng.gen_range(0..0xFF),
                    rng.gen_range(0..0xFF),
                );
            }
            self.position = 0;
        }
    }
}

impl<const LED_N: usize> Ws2812FrameProvider<LED_N> for MeteorFrameProvider<LED_N> {
    fn frame_colors(&self) -> [RGB8; LED_N] {
        self.frame.clone()
    }
}
