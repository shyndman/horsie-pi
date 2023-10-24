#![no_std]
#![no_main]
#![feature(
    asm_experimental_arch,
    const_mut_refs,
    custom_test_frameworks,
    type_alias_impl_trait
)]
#![test_runner(run_tests)]

pub mod panic;

extern crate alloc;

pub fn run_tests(_: &[u8; 0]) {}
