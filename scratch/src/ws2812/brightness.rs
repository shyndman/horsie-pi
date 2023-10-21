use rgb::RGB8;

/// An iterator that provides brightness reduction
///
/// Please be aware that using this after gamma correction the colours doesn't
/// work right.
pub struct Brightness<I> {
    iter: I,
    brightness: u8,
    len: usize,
}

impl<I> Iterator for Brightness<I>
where
    I: Iterator<Item = RGB8>,
{
    type Item = RGB8;
    fn next(&mut self) -> Option<RGB8> {
        self.iter
            .next()
            .map(|a| adjust_brightness(a, self.brightness))
    }
}

impl<I> ExactSizeIterator for Brightness<I>
where
    I: Iterator<Item = RGB8>,
{
    fn len(&self) -> usize {
        self.len
    }
}

#[inline]
pub fn adjust_brightness(c: RGB8, brightness: u8) -> RGB8 {
    RGB8 {
        r: (c.r as u16 * (brightness as u16 + 1) >> 8) as u8,
        g: (c.g as u16 * (brightness as u16 + 1) >> 8) as u8,
        b: (c.b as u16 * (brightness as u16 + 1) >> 8) as u8,
    }
}

pub fn adjust_brightness_f(c: RGB8, brightness: f32) -> RGB8 {
    adjust_brightness(c, (brightness.clamp(0.0, 1.0) * 255.0) as u8)
}

/// Pass your iterator into this function to get reduced brightness
pub fn adjust_iter_brightness<I>(iter: I, brightness: u8) -> Brightness<I>
where
    I: ExactSizeIterator<Item = RGB8>,
{
    Brightness {
        len: iter.len(),
        iter,
        brightness,
    }
}

/// Pass your iterator into this function to get reduced brightness
pub fn adjust_iter_brightness_f<I>(iter: I, brightness: f32) -> Brightness<I>
where
    I: ExactSizeIterator<Item = RGB8>,
{
    Brightness {
        len: iter.len(),
        iter,
        brightness: (brightness.clamp(0.0, 1.0) * 255.0) as u8,
    }
}
