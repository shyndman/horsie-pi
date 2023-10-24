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
    self,
    clock::ClockControl,
    embassy, entry,
    gpio::{AnyPin, Gpio18, Output, PushPull},
    peripherals::Peripherals,
    prelude::*,
    system::SystemParts,
    Rmt, IO,
};
use esp_hal_procmacros::main;
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
use scratch as _;
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

#[embassy_executor::task]
async fn run_color_wheel(
    rmt: Rmt<'static>,
    mut power_pin: AnyPin<Output<PushPull>>,
    data_pin: AnyPin<Output<PushPull>>,
) {
    unwrap!(power_pin.set_high());
    let data_pin: Gpio18<Output<PushPull>> = data_pin.try_into().unwrap();

    let mut led = <smartLedAdapter!(0, 1)>::new(rmt.channel0, data_pin);

    let mut color_index = 0u8;
    loop {
        let color = rgb_color_wheel(color_index);
        defmt::debug!("{}: {}", color_index, Debug2Format(&color));
        unwrap!(led.write(brightness(core::iter::once(color), 90)));

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

#[main]
async fn main(spawner: Spawner) {
    init_heap();
    defmt::info!("Init!");

    let peripherals = Peripherals::take();
    let system: SystemParts = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();
    embassy::init(
        &clocks,
        esp32s3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    let neopixel_power_pin = io.pins.gpio17.into_push_pull_output();
    let neopixel_data_pin = io.pins.gpio18.into_push_pull_output();

    unwrap!(spawner.spawn(run_color_wheel(
        rmt,
        neopixel_power_pin.degrade(),
        neopixel_data_pin.degrade(),
    )));
}
