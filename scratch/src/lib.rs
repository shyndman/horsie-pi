#![no_std]
#![no_main]
#![feature(const_mut_refs)]
#![feature(generic_const_exprs)]
#![feature(type_alias_impl_trait)]
#![feature(custom_test_frameworks)]
#![test_runner(run_tests)]
#![allow(incomplete_features)]
#![allow(unused)]

extern crate alloc;

mod _global_alloc;
pub mod animation;
pub mod joystick;
pub mod power;
pub mod power_led;
pub mod stacked_ring_display;
pub mod stepper;
pub mod ultrasonic_range;
pub mod ws2812;

pub fn run_tests(_: &[u8; 0]) {}
