use cortex_m_rt;
use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;

use defmt::*;
use defmt_rtt as _;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();


// #[inline]
// pub fn heap_size() -> *mut u32 {
//     extern "C" {
//         static mut __heap_size: u32;
//     }

//     unsafe { &mut __heap_size }
// }


//static mut HEAP: [u8; 4096 * 64] = [0; 4096 * 64];


pub fn initialize_allocator() {
    let heap_start = cortex_m_rt::heap_start() as usize;
    let heap_size = 4096 * 128; //heap_size() as usize;

    unsafe {
        ALLOCATOR.init(
            heap_start,
            heap_size,
        )
    }
}


#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    defmt::error!("We're out of fucking memory!");
    defmt::panic!()
}

