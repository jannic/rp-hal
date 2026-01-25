//! Pin Groups
//!
//! Lets you set multiple GPIOs simultaneously.

use embedded_hal::digital::PinState;
use frunk::{hlist::Plucker, HCons, HNil};

use crate::{
    gpio::{pin::pin_sealed::PinIdOps, DynPinId},
    typelevel::Sealed,
};

use super::{pin::pin_sealed::TypeLevelPinId, AnyPin, FunctionSio, SioConfig};

/// Generate a read mask for a pin list.
pub trait PinMask: Sealed {
    /// Generate a mask for a pin list.
    fn pin_mask(&self) -> u32;
    /// DynPinId of one if the pins. Can be used
    /// to determine the correct GPIO bank's registers.
    fn id(&self) -> Option<DynPinId>;
}

impl PinMask for HNil {
    fn pin_mask(&self) -> u32 {
        0
    }

    fn id(&self) -> Option<DynPinId> {
        None
    }
}
impl<H: AnyPin, T: PinMask> PinMask for HCons<H, T> {
    fn pin_mask(&self) -> u32 {
        (1 << self.head.borrow().id().num) | self.tail.pin_mask()
    }

    fn id(&self) -> Option<DynPinId> {
        Some(self.head.borrow().id())
    }
}

/// A group of pins to be controlled together and guaranty single cycle control of several pins.
///
/// ```no_run
/// # macro_rules! defmt { ($($a:tt)*) => {}}
/// use rp2040_hal::{pac, gpio::{bank0::Gpio12, Pin, Pins, PinState, PinGroup}, sio::Sio};
///
/// let mut peripherals = pac::Peripherals::take().unwrap();
/// let sio = Sio::new(peripherals.SIO);
/// let pins = Pins::new(peripherals.IO_BANK0,peripherals.PADS_BANK0,sio.gpio_bank0, &mut peripherals.RESETS);
///
/// let group = PinGroup::new();
/// let group = group.add_pin(pins.gpio0.into_pull_up_input());
/// let mut group = group.add_pin(pins.gpio4.into_push_pull_output_in_state(PinState::High));
///
/// defmt!("Group's state is: {}", group.read());
/// group.toggle();
/// defmt!("Group's state is: {}", group.read());
/// ```
pub struct PinGroup<T = HNil>(T)
where
    T: PinMask;
impl PinGroup<HNil> {
    /// Creates an empty pin group.
    pub fn new() -> Self {
        PinGroup(HNil)
    }

    /// Add a pin to the group.
    pub fn add_pin<P, C>(self, pin: P) -> PinGroup<HCons<P, HNil>>
    where
        C: SioConfig,
        P: AnyPin<Function = FunctionSio<C>>,
        P::Id: TypeLevelPinId,
    {
        PinGroup(HCons {
            head: pin,
            tail: self.0,
        })
    }
}
impl<T, H> PinGroup<HCons<H, T>>
where
    H::Id: TypeLevelPinId,
    H: AnyPin,
    T: PinMask,
{
    /// Add a pin to the group.
    pub fn add_pin<C, P>(self, pin: P) -> PinGroup<HCons<P, HCons<H, T>>>
    where
        C: SioConfig,
        P: AnyPin<Function = FunctionSio<C>>,
        P::Id: TypeLevelPinId<Bank = <H::Id as TypeLevelPinId>::Bank>,
    {
        PinGroup(HCons {
            head: pin,
            tail: self.0,
        })
    }

    /// Pluck a pin from the group.
    #[allow(clippy::type_complexity)]
    pub fn remove_pin<P, Index>(
        self,
    ) -> (P, PinGroup<<HCons<H, T> as Plucker<P, Index>>::Remainder>)
    where
        HCons<H, T>: Plucker<P, Index>,
        <HCons<H, T> as Plucker<P, Index>>::Remainder: PinMask,
    {
        let (p, rest): (P, _) = self.0.pluck();
        (p, PinGroup(rest))
    }
}
impl<T> PinGroup<T>
where
    T: PinMask,
{
    /// Read the whole group at once.
    ///
    /// The returned value is a bit field where each pin populates its own index. Therefore, there
    /// might be "holes" in the value. Unoccupied bits will always read as 0.
    ///
    /// For example, if the group contains Gpio1 and Gpio3, a read may yield:
    /// ```text
    /// 0b0000_0000__0000_0000__0000_0000__0000_1010
    ///                          This is Gpio3  ↑↑↑
    ///                      Gpio2 is not used   ||
    ///                          This is Gpio1    |
    /// ```
    pub fn read(&self) -> u32 {
        let mask = self.0.pin_mask();
        if let Some(head_id) = self.0.id() {
            head_id.sio_in().read().bits() & mask
        } else {
            0
        }
    }

    /// Write this set of pins all at the same time.
    ///
    /// This only affects output pins. Input pins in the
    /// set are ignored.
    pub fn set(&mut self, state: PinState) {
        use super::pin::pin_sealed::PinIdOps;
        let mask = self.0.pin_mask();
        if let Some(head_id) = self.0.id() {
            if state == PinState::Low {
                head_id.sio_out_clr().write(|w| unsafe { w.bits(mask) });
            } else {
                head_id.sio_out_set().write(|w| unsafe { w.bits(mask) });
            }
        }
    }

    /// Set this set of pins to the state given in a single operation.
    ///
    /// The state passed in must be a mask where each bit corresponds to a gpio.
    ///
    /// For example, if the group contains Gpio1 and Gpio3, a read may yield:
    /// ```text
    /// 0b0000_0000__0000_0000__0000_0000__0000_1010
    ///                          This is Gpio3  ↑↑↑
    ///                      Gpio2 is not used   ||
    ///                          This is Gpio1    |
    /// ```
    ///
    /// State corresponding to bins not in this group are ignored.
    pub fn set_u32(&mut self, state: u32) {
        use super::pin::pin_sealed::PinIdOps;
        let mask = self.0.pin_mask();
        let state_masked = mask & state;
        if let Some(head_id) = self.0.id() {
            // UNSAFE: this register is 32bit wide and all bits are valid.
            // The value set is masked
            head_id.sio_out().modify(|r, w| unsafe {
                // clear all bit part of this group
                let cleared = r.bits() & !mask;
                // set bits according to state
                w.bits(cleared | state_masked)
            });
        }
    }

    /// Toggles this set of pins all at the same time.
    ///
    /// This only affects output pins. Input pins in the
    /// set are ignored.
    pub fn toggle(&mut self) {
        use super::pin::pin_sealed::PinIdOps;
        let mask = self.0.pin_mask();
        if let Some(head_id) = self.0.id() {
            head_id.sio_out_xor().write(|w| unsafe { w.bits(mask) });
        }
    }
}
impl Default for PinGroup<HNil> {
    fn default() -> Self {
        Self::new()
    }
}
