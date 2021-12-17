#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

use bincode::config::Configuration;
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::{
    fixed_point::FixedPoint,
    rate::Extensions,
};
use panic_probe as _;

use embedded_graphics_core::{draw_target::DrawTarget, pixelcolor::Gray4};
use embedded_graphics::{prelude::*, image::{Image, ImageRaw}, pixelcolor::Gray8};

use display_interface_i2c::I2CInterface;

use pico as bsp;


use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};


mod allocator;


const BONGO_CAT: &[u8] = include_bytes!("../bongo-cat.bin");


#[derive(bincode::Decode)]
pub struct BinFrame {
    pub duration: u32,
    pub width: u32,
    pub height: u32,
    buffer: alloc::vec::Vec<u8>,
}


#[derive(bincode::Decode)]
pub struct BinFrames {
    pub frames: alloc::vec::Vec<BinFrame>
}


#[entry]
fn main() -> ! {
    allocator::initialize_allocator();

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio16.into_mode::<bsp::hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio17.into_mode::<bsp::hal::gpio::FunctionI2C>();

    let mut led_pin = pins.led.into_push_pull_output();

    let mut i2c = bsp::hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        800.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock,
    );

    let i2c_interface = I2CInterface::new(
        i2c,
        0x3D,
        0x40, // TODO: what am I
    );

    delay.delay_ms(250);

    let mut display = ssd1327::display::Ssd1327::new(i2c_interface);

    info!("Init");
    display.init().unwrap();
    info!("Finished Init");

    // Clear the display
    info!("Clearing");
    display.clear(Gray4::new(0)).unwrap();
    info!("Cleared");

    info!("Flushing");
    display.flush().unwrap();
    info!("Flushed");

    let config = Configuration::standard();
    let bin_frames: BinFrames = bincode::decode_from_slice(BONGO_CAT, config)
        .unwrap();

    loop {
        for frame in &bin_frames.frames {
            let raw = ImageRaw::<Gray8>::new(&frame.buffer, frame.height);
            let image = Image::new(&raw, Point::zero());

            image.draw(&mut display.color_converted())
                .unwrap();

            display.flush()
                .unwrap();

            delay.delay_ms(frame.duration);
        }
    }
}
