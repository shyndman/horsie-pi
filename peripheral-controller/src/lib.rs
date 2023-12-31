#![no_std]
#![no_main]
#![feature(
    asm_experimental_arch,
    const_mut_refs,
    custom_test_frameworks,
    type_alias_impl_trait,
    impl_trait_projections,
    exclusive_range_pattern
)]
#![test_runner(run_tests)]

pub mod panic;
pub mod ui;

use core::mem::MaybeUninit;

extern crate alloc;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

pub fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

pub fn run_tests(_: &[u8; 0]) {}
