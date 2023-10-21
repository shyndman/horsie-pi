pub mod brightness;
pub mod effects;
pub mod gamma;

use embassy_rp::{
    clocks,
    dma::{AnyChannel, Channel},
    into_ref,
    pio::{
        Common, Config, FifoJoin, Instance, PioPin, ShiftConfig, ShiftDirection, StateMachine,
    },
    Peripheral, PeripheralRef,
};
use fixed::types::U24F8;
use fixed_macro::fixed;
pub use gamma::correct_gamma;
use rgb::RGB8;

use self::brightness::adjust_iter_brightness;

pub trait Ws2812FrameProvider<const LED_N: usize> {
    fn frame_colors(&self) -> [RGB8; LED_N];
}

pub struct Ws2812Chain<'d, PIO: Instance, const SM: usize, const LED_N: usize> {
    pub brightness: u8,
    dma: PeripheralRef<'d, AnyChannel>,
    sm: StateMachine<'d, PIO, SM>,
}

impl<'d, PIO: Instance, const SM: usize, const LED_N: usize> Ws2812Chain<'d, PIO, SM, LED_N> {
    pub async fn new<FrameProvider: Ws2812FrameProvider<LED_N>>(
        pio: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        dma: impl Peripheral<P = impl Channel> + 'd,
        pin: impl PioPin,
        frame_provider: &FrameProvider,
    ) -> Ws2812Chain<'d, PIO, SM, LED_N> {
        into_ref!(dma);

        let program_w_defines = pio_proc::pio_file!("src/ws2812/ws2812.pio");
        let program = program_w_defines.program;
        let cycles_per_bit: u32 = (program_w_defines.public_defines.T1 +
            program_w_defines.public_defines.T2 +
            program_w_defines.public_defines.T3) as u32;

        // Pin config
        let pio_cfg = {
            let mut p = Config::default();

            let out_pin = pio.make_pio_pin(pin);
            p.set_out_pins(&[&out_pin]);
            p.set_set_pins(&[&out_pin]);
            p.use_program(&pio.load_program(&program), &[&out_pin]);

            // Clock config, measured in kHz to avoid overflows
            p.clock_divider = {
                let clock_freq = U24F8::from_num(clocks::clk_sys_freq() / 1000);
                let ws2812_freq = fixed!(800: U24F8);
                let bit_freq = ws2812_freq * cycles_per_bit;
                clock_freq / bit_freq
            };

            // FIFO config
            p.fifo_join = FifoJoin::TxOnly;
            p.shift_out = ShiftConfig {
                auto_fill: true,
                threshold: 24,
                direction: ShiftDirection::Left,
            };
            p
        };

        // Assign config to state machine and go!
        sm.set_config(&pio_cfg);
        sm.set_enable(true);

        let mut chain = Ws2812Chain {
            brightness: 0xFF,
            dma: dma.map_into(),
            sm,
        };
        chain.draw(frame_provider).await;
        chain
    }

    /// Updates the pixel array using the current data_source field.
    pub async fn draw<FrameProvider: Ws2812FrameProvider<LED_N>>(
        &mut self,
        frame_provider: &FrameProvider,
    ) {
        let c = frame_provider.frame_colors();
        let colors = c.iter().cloned();
        let colors = correct_gamma(colors);
        if self.brightness < 0xFF {
            self.write_iterator(adjust_iter_brightness(colors, self.brightness))
                .await;
        } else {
            self.write_iterator(colors).await;
        }
    }

    async fn write_iterator<I>(&mut self, colors: I)
    where
        I: ExactSizeIterator<Item = RGB8>,
    {
        assert!(
            LED_N == colors.len(),
            "Iterators provided to .write_iterator must be sized and match LED_N"
        );

        // Precompute the word bytes from the colors
        let mut words = [0u32; LED_N];
        for (i, c) in colors.enumerate() {
            words[i] = rgb8_to_word(c.r, c.g, c.b);
        }

        // DMA transfer
        self.sm.tx().dma_push(self.dma.reborrow(), &words).await;
    }
}

#[inline]
fn rgb8_to_word(red: u8, green: u8, blue: u8) -> u32 {
    (u32::from(green) << 24) | (u32::from(red) << 16) | (u32::from(blue) << 8)
}
