use rgb::RGB8;

pub fn rgb_color_wheel(i: u8) -> RGB8 {
    match i % 255 {
        i @ 0..85 => RGB8::new(255 - i * 3, 0, i * 3),
        mut i @ 85..170 => {
            i -= 85;
            RGB8::new(0, i * 3, 255 - i * 3)
        }
        mut i @ _ => {
            i -= 170;
            RGB8::new(i * 3, 255 - i * 3, 0)
        }
    }
}
