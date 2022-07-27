// copied from embassy-rp

use core::cell::Cell;
use core::cell::RefCell;

use atomic_polyfill::{AtomicU8, Ordering};
use critical_section::CriticalSection;
use embassy::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy::blocking_mutex::Mutex;
use embassy::time::driver::{AlarmHandle, Driver};

use rp2040_hal::pac::interrupt;
use rp2040_hal::pac;
use rp2040_hal::timer::Alarm;
use rp2040_hal::timer::Alarm0;
use rp2040_hal::timer::Timer;

struct AlarmState {
    timestamp: Cell<u64>,
    callback: Cell<Option<(fn(*mut ()), *mut ())>>,
}
unsafe impl Send for AlarmState {}

const ALARM_COUNT: usize = 4;
const DUMMY_ALARM: AlarmState = AlarmState {
    timestamp: Cell::new(0),
    callback: Cell::new(None),
};

struct TimerDriver {
    timer: Mutex<CriticalSectionRawMutex, RefCell<Option<Timer>>>,
    alarm: Mutex<CriticalSectionRawMutex, RefCell<Option<Alarm0>>>,
    alarms: Mutex<CriticalSectionRawMutex, [AlarmState; ALARM_COUNT]>,
    next_alarm: AtomicU8,
}

embassy::time_driver_impl!(static DRIVER: TimerDriver = TimerDriver{
    timer: Mutex::const_new(CriticalSectionRawMutex::new(), RefCell::new(None)),
    alarm:  Mutex::const_new(CriticalSectionRawMutex::new(), RefCell::new(None)),
    alarms:  Mutex::const_new(CriticalSectionRawMutex::new(), [DUMMY_ALARM; ALARM_COUNT]),
    next_alarm: AtomicU8::new(0),
});

impl Driver for TimerDriver {
    fn now(&self) -> u64 {
        critical_section::with(|cs| {
            let timer = self.timer.borrow(cs).borrow().unwrap();
            timer.get_counter()
        })
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        let id = self.next_alarm.fetch_update(Ordering::AcqRel, Ordering::Acquire, |x| {
            if x < ALARM_COUNT as u8 {
                Some(x + 1)
            } else {
                None
            }
        });

        match id {
            Ok(id) => Some(AlarmHandle::new(id)),
            Err(_) => None,
        }
    }

    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        let n = alarm.id() as usize;
        critical_section::with(|cs| {
            let alarm = &self.alarms.borrow(cs)[n];
            alarm.callback.set(Some((callback, ctx)));
        })
    }

    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) {
        let n = alarm.id() as usize;
        critical_section::with(|cs| {
            let alarms = self.alarms.borrow(cs);
            let alarm = &alarms[n];
            let timer = self.timer.borrow(cs).borrow().unwrap();
            alarm.timestamp.set(timestamp);

            let min_timestamp = alarms.iter().min().unwrap();

            // Arm it.
            // Note that we're not checking the high bits at all. This means the irq may fire early
            // if the alarm is more than 72 minutes (2^32 us) in the future. This is OK, since on irq fire
            // it is checked if the alarm time has passed.
            unsafe { timer.alarm(n).write_value(min_timestamp as u32) };

            let now = self.now();

            // If alarm timestamp has passed, trigger it instantly.
            // This disarms it.
            if timestamp <= now {
                self.trigger_alarm(n, cs);
            }
        })
    }
}

impl TimerDriver {
    fn arm(&self) {
        critical_section::with(|cs| {
            let alarms = self.alarms.borrow(cs);
            let min_timestamp = alarms.iter().min().unwrap();
            if min_timestamp != u64::MAX {
                let timer = self.timer.borrow(cs).borrow().unwrap();
                unsafe { timer.alarm.write_value(min_timestamp as u32) };
            } else {
                unsafe { pac::TIMER.armed().write(|w| w.set_armed(1 << n)) }
            }
    }

    fn check_alarm(&self, n: usize) {
        critical_section::with(|cs| {
            let alarm = &self.alarms.borrow(cs)[n];
            let timer = self.timer.borrow(cs).borrow().unwrap();
            let timestamp = alarm.timestamp.get();
            if timestamp <= self.now() {
                self.trigger_alarm(n, cs)
            } else {
                // Not elapsed, arm it again.
                // This can happen if it was set more than 2^32 us in the future.
                unsafe { timer.alarm(n).write_value(timestamp as u32) };
            }
        });

        // clear the irq
        unsafe { pac::TIMER.intr().write(|w| w.set_alarm(n, true)) }
    }

    fn trigger_alarm(&self, n: usize, cs: CriticalSection) {
        // disarm
        unsafe { pac::TIMER.armed().write(|w| w.set_armed(1 << n)) }

        let alarm = &self.alarms.borrow(cs)[n];
        alarm.timestamp.set(u64::MAX);

        // Call after clearing alarm, so the callback can set another alarm.
        if let Some((f, ctx)) = alarm.callback.get() {
            f(ctx);
        }
    }
}

/// safety: must be called exactly once at bootup
pub unsafe fn init(timer: Timer) {
    // init alarms
    critical_section::with(|cs| {
        // make sure the alarms are not yet taken,
        // and leak them, so they can be used 
        timer.alarm0().unwrap();
        timer.alarm1().unwrap();
        timer.alarm2().unwrap();
        timer.alarm3().unwrap();
        DRIVER.timer.borrow(cs).replace(Some(timer));
        let alarms = DRIVER.alarms.borrow(cs);
        for a in alarms {
            a.timestamp.set(u64::MAX);
        }
    });

    todo!();
    /*
    // enable all irqs
    pac::TIMER.inte().write(|w| {
        w.set_alarm(0, true);
        w.set_alarm(1, true);
        w.set_alarm(2, true);
        w.set_alarm(3, true);
    });

    interrupt::TIMER_IRQ_0::steal().enable();
    interrupt::TIMER_IRQ_1::steal().enable();
    interrupt::TIMER_IRQ_2::steal().enable();
    interrupt::TIMER_IRQ_3::steal().enable();
    */
}

#[interrupt]
unsafe fn TIMER_IRQ_0() {
    DRIVER.check_alarm(0)
}

#[interrupt]
unsafe fn TIMER_IRQ_1() {
    DRIVER.check_alarm(1)
}

#[interrupt]
unsafe fn TIMER_IRQ_2() {
    DRIVER.check_alarm(2)
}

#[interrupt]
unsafe fn TIMER_IRQ_3() {
    DRIVER.check_alarm(3)
}
