#![no_std]
#![no_main]
#![feature(async_fn_in_trait, impl_trait_projections, type_alias_impl_trait)]

extern crate alloc;
use core::mem::MaybeUninit;

use defmt::Debug2Format;
use defmt_macros::unwrap;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32s3_hal::{
    self, clock::ClockControl, embassy, entry, peripherals::Peripherals, prelude::*,
    system::SystemParts, Rmt, IO,
};
use esp_backtrace as _;
use esp_hal_procmacros::main;
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
use smart_leds::brightness;
use smart_leds_trait::*;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[main]
async fn main(_spawner: Spawner) {
    init_heap();

    esp_println::println!("Init!");
    defmt::trace!("This is trace");
    defmt::debug!("This is debug");
    defmt::info!("This is info");
    defmt::warn!("This is warn");
    defmt::error!("This is error");

    let p = Peripherals::take();
    let system: SystemParts = p.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(p.GPIO, p.IO_MUX);
    let rmt = Rmt::new(p.RMT, 80u32.MHz(), &clocks).unwrap();
    embassy::init(&clocks, esp32s3_hal::systimer::SystemTimer::new(p.SYSTIMER));

    let mut neopixel_power = io.pins.gpio17.into_push_pull_output();
    unwrap!(neopixel_power.set_high());
    let mut led = <smartLedAdapter!(0, 1)>::new(rmt.channel0, io.pins.gpio18);

    let mut color_index = 0u8;
    loop {
        let color = rgb_color_wheel(color_index);
        defmt::debug!("{}: {}", color_index, Debug2Format(&color));
        unwrap!(led.write(brightness(core::iter::once(color), 160)));

        color_index = color_index.wrapping_add(1);
        Timer::after(Duration::from_millis(15)).await;
    }
}

fn rgb_color_wheel(mut i: u8) -> RGB8 {
    i %= 255;

    if i < 85 {
        RGB8::new(255 - i * 3, 0, i * 3)
    } else if i < 170 {
        i -= 85;
        RGB8::new(0, i * 3, 255 - i * 3)
    } else {
        i -= 170;
        RGB8::new(i * 3, 255 - i * 3, 0)
    }
}
