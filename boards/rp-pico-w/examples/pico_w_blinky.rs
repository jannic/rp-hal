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


use core::convert::Infallible;

// Pull in any important traits
use rp_pico_w::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico_w::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico_w::hal;

use embedded_hal_1 as eh_1;

use hal::gpio::PushPullOutput;

use eh_1::spi::ErrorType;
use eh_1::spi::blocking::SpiBusFlush;

// use eh_1::blocking::OutputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;

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
    let state = unsafe { forever(&cyw43::State::new()) };

    info!("run spawner");
    executor.run(|spawner| {
        info!("create spawn token");
        let spawn_token = task_pool.spawn(|| run(spawner, pins, delay, state));
        info!("spawn it!");
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

async fn run(spawner: Spawner, pins:  rp_pico_w::Pins, mut delay: cortex_m::delay::Delay, state: &cyw43::State) 
{
    // These are implicitly used by the spi driver if they are in the correct mode
    let spi_clk = pins.voltage_monitor_wl_clk.into_push_pull_output();
    let spi_mosi_miso = pins.wl_d.into();
    let spi_cs = pins.wl_cs.into_push_pull_output();

    let bus = MySpi { clk: spi_clk, dio: spi_mosi_miso };
    let spi = eh_1::spi::blocking::ExclusiveDevice::new(bus, spi_cs);
    // Set the LED to be an output
    // TODO fix this, the on-board LED is not directly accessible on a
    // GPIO pin, but only via the WLAN chip.
    // let mut led_pin = pins.led.into_push_pull_output();
    let mut led_pin = pins.gpio0.into_push_pull_output();
    //let mut other_pin = pins.gpio1.into_push_pull_output();

    let mut pwr = pins.wl_on.into_push_pull_output();

    let fw = include_bytes!("firmware/43439A0.bin");
    
    info!("create cyw43 driver");
    let (mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    info!("created cyw43 driver");

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


////////////////////////////////////////////////////////////

struct MySpi {
    /// SPI clock
    clk: hal::gpio::Pin<hal::gpio::bank0::Gpio29, hal::gpio::Output<hal::gpio::PushPull>>,

    /// 4 signals, all in one!!
    /// - SPI MISO
    /// - SPI MOSI
    /// - IRQ
    /// - strap to set to gSPI mode on boot.
    dio: hal::gpio::dynpin::DynPin,
}

impl ErrorType for MySpi {
    type Error = Infallible;
}

impl SpiBusFlush for MySpi {
    fn flush<'a>(&'a mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl SpiBusRead<u32> for MySpi {
    fn read<'a>(&'a mut self, words: &'a mut [u32]) -> Result<(), Self::Error> {
        self.dio.into_floating_input();
        for word in words {
            let mut w = 0;
            for _ in 0..32 {
                w = w << 1;

                // rising edge, sample data
                if self.dio.is_high().unwrap() {
                    w |= 0x01;
                }
                self.clk.set_high();

                // falling edge
                self.clk.set_low();
            }
            *word = w
        }

        Ok(())
    }
}

impl SpiBusWrite<u32> for MySpi {
    fn write<'a>(&'a mut self, words: &'a [u32]) -> Result<(), Self::Error> {
        self.dio.into_push_pull_output();
        for word in words {
            let mut word = *word;
            for _ in 0..32 {
                // falling edge, setup data
                self.clk.set_low();
                if word & 0x8000_0000 == 0 {
                    self.dio.set_low();
                } else {
                    self.dio.set_high();
                }

                // rising edge
                self.clk.set_high();

                word = word << 1;
            }
        }
        self.clk.set_low();

        self.dio.into_floating_input();
        Ok(())
    }
}
