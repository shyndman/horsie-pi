//! Simulates a possible application scenario using the embassy_sync::pubsub system

#![no_std]
#![no_main]
#![feature(
    asm_experimental_arch,
    async_fn_in_trait,
    const_mut_refs,
    custom_test_frameworks,
    exclusive_range_pattern,
    impl_trait_projections,
    return_position_impl_trait_in_trait,
    type_alias_impl_trait
)]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    pubsub::{PubSubChannel, WaitResult},
};
use embassy_time::{Duration, Ticker, Timer};
use esp32s3_hal::{self as hal};
use esp_hal_procmacros::main;
use esp_hal_smartled::{smartLedAdapter, SmartLedsAdapter};
use esp_println::logger::init_logger_from_env;
use hal::{
    clock::ClockControl, embassy, interrupt, peripherals::Peripherals, prelude::*,
    system::SystemParts, Rmt, IO,
};
use peripheral_controller::init_heap;
use rand::prelude::*;
use rgb::RGB8;
use smart_leds::brightness;
use smart_leds_trait::SmartLedsWrite;
use static_cell::make_static;

#[main]
async fn main(spawner: Spawner) {
    init_heap();
    init_logger_from_env();

    defmt::info!("Init!");

    let peripherals = Peripherals::take();
    let system: SystemParts = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();

    embassy::init(
        &clocks,
        esp32s3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    esp32s3_hal::interrupt::enable(
        esp32s3_hal::peripherals::Interrupt::GPIO,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    {
        io.pins.gpio17.into_push_pull_output().set_high().unwrap();
        let mut led = <smartLedAdapter!(0, 1)>::new(
            rmt.channel0,
            io.pins.gpio18.into_push_pull_output(),
        );
        led.write(brightness(
            core::iter::once(RGB8::new(0x00, 0xFF, 0x00)),
            90,
        ))
        .unwrap();
    }

    let speed_topic: &'static mut SpeedTopic = make_static!(PubSubChannel::<
        CriticalSectionRawMutex,
        SpeedMsg,
        1,
        100,
        100,
    >::new());

    spawner.spawn(planner_task(speed_topic)).unwrap();
    spawner.spawn(stepper_task(speed_topic)).unwrap();
    spawner.spawn(display_task(speed_topic)).unwrap();

    loop {
        Timer::after(Duration::from_secs(30)).await;
    }
}

#[derive(Clone)]
struct SpeedMsg {
    degrees_per_second: i32,
}

type SpeedTopic = PubSubChannel<CriticalSectionRawMutex, SpeedMsg, 1, 100, 100>;

#[embassy_executor::task]
async fn planner_task(speed_topic: &'static SpeedTopic) {
    let publisher = speed_topic.publisher().unwrap();

    let mut rng = SmallRng::seed_from_u64(0xdeadbeef);
    loop {
        let degrees_per_second = rng.gen_range(40..(20 * 360));
        defmt::info!(
            "Planner indicating that target speed is now {}°/s",
            degrees_per_second
        );
        publisher.publish_immediate(SpeedMsg { degrees_per_second });

        // The planner sends new target speeds sporatically
        let time_until_next_update = Duration::from_secs(rng.gen_range(3u64..15));
        Timer::after(time_until_next_update).await;
    }
}

#[embassy_executor::task]
async fn stepper_task(speed_topic: &'static SpeedTopic) {
    let mut subscriber = speed_topic.subscriber().unwrap();

    let mut ticker = Ticker::every(Duration::from_hz(1));
    loop {
        if let Some(SpeedMsg { degrees_per_second }) = subscriber.try_next_message_pure() {
            defmt::info!("Setting ramp generator target to {}°/s", degrees_per_second);
        } else {
            defmt::info!("No speed changes. Continuing ramp.")
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn display_task(speed_topic: &'static SpeedTopic) {
    let mut subscriber = speed_topic.subscriber().unwrap();

    loop {
        match subscriber.next_message().await {
            WaitResult::Lagged(missed_count) => {
                defmt::warn!("display_task missed {} SpeedMsg messages", missed_count);
            }
            WaitResult::Message(SpeedMsg { degrees_per_second }) => {
                defmt::info!("Flushing {}°/s to the display", degrees_per_second);
            }
        }
    }
}
