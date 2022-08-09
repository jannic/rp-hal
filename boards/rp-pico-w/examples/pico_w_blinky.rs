//! # Pico W Blinky Example
//!
//! Blinks the LED on a Pico W board.
//!
//! This will blink an LED attached to WL_GPIO0, which is the pin the Pico W uses for
//! the on-board LED. It is connected to the the Wifi chip so it cannot be set using RP2040 pins
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use rp_pico_w::entry;

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_probe as _;
use defmt::*;
use defmt_rtt as _;



// Pull in any important traits
use rp_pico_w::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico_w::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico_w::hal;

use embedded_hal_1 as eh_1;

use cyw43;
use embassy::executor::raw::TaskPool;
use embassy::executor::Executor;
use embassy::executor::Spawner;
use core::future::Future;
use embassy::time::{Duration, Timer};

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {

    info!("start");

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico_w::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();


    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico_w::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("init time driver!");
    let timer = hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);
    unsafe { rp_pico_w::embassy_time_driver::init(timer) };

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let mut executor = Executor::new();

    // Safety: function never returns, executor is never dropped
    let executor: &'static mut Executor = unsafe { forever_mut( &mut executor ) };

    let mut task_pool: TaskPool<_, 10> = TaskPool::new();
    let task_pool = unsafe { forever(&mut task_pool) };

    let state = cyw43::State::new();
    let state = forever(&cyw43::State::new());

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio6.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio7.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<hal::gpio::FunctionSpi>();
    let rp_spi = hal::Spi::<_, _, 8>::new(pac.SPI0);
    let mut rp_spi = rp_spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );


    executor.run(|spawner| {
        let spawn_token = task_pool.spawn(|| run(spawner, pins, delay, state, rp_spi));
        spawner.spawn(spawn_token);
    });

}

unsafe fn forever_mut<T>(r: &'_ mut T) -> &'static mut T {
    core::mem::transmute(r)
}

unsafe fn forever<T>(r: &'_ T) -> &'static T {
    core::mem::transmute(r)
}

use embedded_hal_1::spi::blocking::{SpiBusRead, SpiBusWrite, SpiBus, SpiDevice};

async fn run<SPI>(spawner: Spawner, pins:  rp_pico_w::Pins, mut delay: cortex_m::delay::Delay, state: &cyw43::State, spi: SPI) 
where
    SPI: SpiDevice,
    SPI::Bus: SpiBus + SpiBusWrite<u32> + SpiBusRead<u32>,

{
    // Set the LED to be an output
    // TODO fix this, the on-board LED is not directly accessible on a
    // GPIO pin, but only via the WLAN chip.
    // let mut led_pin = pins.led.into_push_pull_output();
    let mut led_pin = pins.gpio0.into_push_pull_output();
    //let mut other_pin = pins.gpio1.into_push_pull_output();

    let mut pwr = pins.wl_on.into_push_pull_output();

    let fw = include_bytes!("firmware/43439A0.bin");
    
    let (mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    // Blink the LED at 1 Hz
    loop {
        info!("on");
        led_pin.set_high().unwrap();
        use embedded_hal::digital::v2::OutputPin;
        Timer::after(Duration::from_millis(500)).await;
        //delay.delay_ms(500);
        info!("off");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
        //Timer::after(Duration::from_millis(500)).await;
    }
}
