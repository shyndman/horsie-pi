use rand::prelude::*;
use rgb::RGB8;

use super::EffectFrameProvider;
use crate::ws2812::Ws2812FrameProvider;

pub struct FireFrameProvider<const LED_N: usize> {
    cooling: u8,
    sparking: u8,
    heat: [u8; LED_N],
    frame: [RGB8; LED_N],
    rng: SmallRng,
}

impl<const LED_N: usize> FireFrameProvider<LED_N> {
    pub fn new(cooling: Option<u8>, sparking: Option<u8>) -> Self {
        Self {
            cooling: cooling.unwrap_or(4),
            sparking: sparking.unwrap_or(120),
            heat: [0; LED_N],
            frame: [RGB8::new(0, 0, 0); LED_N],
            rng: SmallRng::seed_from_u64(0xDEADBEEF),
        }
    }
}

impl<const LED_N: usize> EffectFrameProvider for FireFrameProvider<LED_N> {
    fn advance(&mut self) {
        // Apply cooling
        for spark in self.heat.iter_mut() {
            let x = self.rng.gen_range(0..self.cooling) as u8;
            *spark = spark.saturating_sub(x);
        }

        // Apply heating
        for i in (2..LED_N).rev() {
            self.heat[i] = (self.heat[i - 1]
                .saturating_add(self.heat[i - 2])
                .saturating_add(self.heat[i - 2])) /
                3;
        }

        // Generate sparks
        if self.rng.gen_range(0..255) < self.sparking {
            let y = self.rng.gen_range(0..LED_N);
            self.heat[y] = self.heat[y].saturating_add(self.rng.gen_range(160..255));
        }

        // Fill the frame
        for (i, c) in self.heat.iter().map(|x| heat_to_colour(*x)).enumerate() {
            self.frame[i] = c;
        }
    }
}

impl<const LED_N: usize> Ws2812FrameProvider<LED_N> for FireFrameProvider<LED_N> {
    fn frame_colors(&self) -> [RGB8; LED_N] {
        self.frame.clone()
    }
}

fn heat_to_colour(val: u8) -> RGB8 {
    if val >= 0x85 {
        let heat_ramp = 3u8.saturating_mul(val - 0x85);
        RGB8::new(255, 255, heat_ramp)
    } else if val >= 0x40 {
        let heat_ramp = 3u8.saturating_mul(val - 0x40);
        RGB8::new(255, heat_ramp, 0)
    } else {
        let heat_ramp = 3u8.saturating_mul(val);
        RGB8::new(heat_ramp, 0, 0)
    }
}
