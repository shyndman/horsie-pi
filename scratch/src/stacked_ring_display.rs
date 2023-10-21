use embassy_rp::pio::Instance;
use rgb::RGB8;

use crate::ws2812::{Ws2812Chain, Ws2812FrameProvider};

struct StackedRingFrameProvider<'a, const LED_N: usize, const RING_LED_N: usize> {
    top_frames: &'a dyn Ws2812FrameProvider<RING_LED_N>,
    bottom_frames: &'a dyn Ws2812FrameProvider<RING_LED_N>,
}

impl<'a, const LED_N: usize> StackedRingFrameProvider<'a, LED_N, { LED_N / 2 }> {
    pub async fn new(
        top_frames: &'a dyn Ws2812FrameProvider<{ LED_N / 2 }>,
        bottom_frames: &'a dyn Ws2812FrameProvider<{ LED_N / 2 }>,
    ) -> Self {
        Self {
            top_frames,
            bottom_frames,
        }
    }
}

impl<'a, const LED_N: usize> Ws2812FrameProvider<LED_N>
    for StackedRingFrameProvider<'a, LED_N, { LED_N / 2 }>
{
    fn frame_colors(&self) -> [RGB8; LED_N] {
        let ring_n: usize = LED_N / 2;
        let top_ring_frame = self.top_frames.frame_colors();
        let bottom_ring_frame = self.bottom_frames.frame_colors();

        let mut colors = [RGB8::new(0, 0, 0); LED_N];
        for i in 0..ring_n {
            colors[i] = top_ring_frame[i];
        }
        for i in 0..ring_n {
            colors[i + ring_n] = bottom_ring_frame[ring_n - i - 1];
        }

        colors
    }
}

/// Represents a stack of two WS2812B pixel rings. This code assumes that the first
/// LED_N/2 pixels are the first ring, and the second LED_N/2 pixels are the second ring.
///
/// Pixels are indexed starting from the "back" of the rings, in clockwise order.
///
/// The rings are assumed to be facing each other. Pixel addresses are automatically
/// mapped so that for any pixel I in the top ring is in the same X,Y position as pixel
/// <code>I + (LED_N/2)</code> in the second.
pub struct StackedRingDisplay<'d, PIO: Instance, const SM: usize, const LED_N: usize> {
    pixel_chain: Ws2812Chain<'d, PIO, SM, LED_N>,
}

impl<'d, PIO: Instance, const SM: usize, const LED_N: usize>
    StackedRingDisplay<'d, PIO, SM, LED_N>
{
    pub async fn new(pixel_chain: Ws2812Chain<'d, PIO, SM, LED_N>) -> Self {
        Self { pixel_chain }
    }

    pub fn pixels_per_ring(&self) -> usize {
        LED_N / 2
    }
}
