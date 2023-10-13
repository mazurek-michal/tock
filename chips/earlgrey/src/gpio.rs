// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! GPIO instantiation.

use core::ops::{Index, IndexMut};

use crate::pinmux::Pad;
use crate::registers::top_earlgrey::GPIO_BASE_ADDR;
use crate::registers::top_earlgrey::{MuxedPads, PinmuxOutsel};

use kernel::utilities::StaticRef;

use kernel::hil::gpio;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{
    register_bitfields, register_structs, Field, LocalRegisterCopy, ReadOnly, ReadWrite, WriteOnly,
};

// Fix issue of this one ugly book that don't fit on the shelve!!!
#[path = "../../lowrisc/src/registers/gpio_regs.rs"]
mod gpio_regs;
use gpio_regs::{
    GpioRegisters, INTR, MASKED_OE_LOWER, MASKED_OE_UPPER, MASKED_OUT_LOWER, MASKED_OUT_UPPER,
};

pub const GPIO_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(GPIO_BASE_ADDR as *const GpioRegisters) };

pub struct Port<'a> {
    pins: [GpioPin<'a>; 32],
}

impl<'a> Port<'a> {
    pub const fn new() -> Self {
        Self {
            pins: [
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio0),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio1),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio2),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio3),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio4),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio5),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio6),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio7),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio8),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio9),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio10),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio11),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio12),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio13),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio14),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio15),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio16),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio17),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio18),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio19),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio20),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio21),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio22),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio23),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio24),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio25),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio26),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio27),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio28),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio29),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio30),
                GpioPin::new(MuxedPads::Ioa0, GpioNo::Gpio31),
            ],
        }
    }
}

impl<'a> Index<usize> for Port<'a> {
    type Output = GpioPin<'a>;

    fn index(&self, index: usize) -> &GpioPin<'a> {
        &self.pins[index]
    }
}

impl<'a> IndexMut<usize> for Port<'a> {
    fn index_mut(&mut self, index: usize) -> &mut GpioPin<'a> {
        &mut self.pins[index]
    }
}

register_bitfields![u32,
    pub GPIO [
        PIN_0 0,
        PIN_1 1,
        PIN_2 2,
        PIN_3 3,
        PIN_4 4,
        PIN_5 5,
        PIN_6 6,
        PIN_7 7,
        PIN_8 8,
        PIN_9 9,
        PIN_10 10,
        PIN_11 11,
        PIN_12 12,
        PIN_13 13,
        PIN_14 14,
        PIN_15 15,
        PIN_16 16,
        PIN_17 17,
        PIN_18 18,
        PIN_19 19,
        PIN_20 20,
        PIN_21 21,
        PIN_22 22,
        PIN_23 23,
        PIN_24 24,
        PIN_25 25,
        PIN_26 26,
        PIN_27 27,
        PIN_28 28,
        PIN_29 29,
        PIN_30 30,
        PIN_31 31
    ],
    MASK_HALF [
        DATA OFFSET(0) NUMBITS(16) [],
        MASK OFFSET(16) NUMBITS(16) []
    ]
];

#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum GpioNo {
    Gpio0 = 0,
    Gpio1 = 1,
    Gpio2 = 2,
    Gpio3 = 3,
    Gpio4 = 4,
    Gpio5 = 5,
    Gpio6 = 6,
    Gpio7 = 7,
    Gpio8 = 8,
    Gpio9 = 9,
    Gpio10 = 10,
    Gpio11 = 11,
    Gpio12 = 12,
    Gpio13 = 13,
    Gpio14 = 14,
    Gpio15 = 15,
    Gpio16 = 16,
    Gpio17 = 17,
    Gpio18 = 18,
    Gpio19 = 19,
    Gpio20 = 20,
    Gpio21 = 21,
    Gpio22 = 22,
    Gpio23 = 23,
    Gpio24 = 24,
    Gpio25 = 25,
    Gpio26 = 26,
    Gpio27 = 27,
    Gpio28 = 28,
    Gpio29 = 29,
    Gpio30 = 30,
    Gpio31 = 31,
}

type DirectGpioReg = LocalRegisterCopy<u32, GPIO::Register>;
type MaskedGpioReg = LocalRegisterCopy<u32, MASK_HALF::Register>;

impl GpioNo {
    pub fn field(&self) -> Field<u32, INTR::Register> {
        match *self {
            GpioNo::Gpio0 => INTR::GPIO_0,
            GpioNo::Gpio1 => INTR::GPIO_1,
            GpioNo::Gpio2 => INTR::GPIO_2,
            GpioNo::Gpio3 => INTR::GPIO_3,
            GpioNo::Gpio4 => INTR::GPIO_4,
            GpioNo::Gpio5 => INTR::GPIO_5,
            GpioNo::Gpio6 => INTR::GPIO_6,
            GpioNo::Gpio7 => INTR::GPIO_7,
            GpioNo::Gpio8 => INTR::GPIO_8,
            GpioNo::Gpio9 => INTR::GPIO_9,
            GpioNo::Gpio10 => INTR::GPIO_10,
            GpioNo::Gpio11 => INTR::GPIO_11,
            GpioNo::Gpio12 => INTR::GPIO_12,
            GpioNo::Gpio13 => INTR::GPIO_13,
            GpioNo::Gpio14 => INTR::GPIO_14,
            GpioNo::Gpio15 => INTR::GPIO_15,
            GpioNo::Gpio16 => INTR::GPIO_16,
            GpioNo::Gpio17 => INTR::GPIO_17,
            GpioNo::Gpio18 => INTR::GPIO_18,
            GpioNo::Gpio19 => INTR::GPIO_19,
            GpioNo::Gpio20 => INTR::GPIO_20,
            GpioNo::Gpio21 => INTR::GPIO_21,
            GpioNo::Gpio22 => INTR::GPIO_22,
            GpioNo::Gpio23 => INTR::GPIO_23,
            GpioNo::Gpio24 => INTR::GPIO_24,
            GpioNo::Gpio25 => INTR::GPIO_25,
            GpioNo::Gpio26 => INTR::GPIO_26,
            GpioNo::Gpio27 => INTR::GPIO_27,
            GpioNo::Gpio28 => INTR::GPIO_28,
            GpioNo::Gpio29 => INTR::GPIO_29,
            GpioNo::Gpio30 => INTR::GPIO_30,
            GpioNo::Gpio31 => INTR::GPIO_31,
        }
    }

    pub fn gpio_field(&self) -> Field<u32, GPIO::Register> {
        match *self {
            GpioNo::Gpio0 => GPIO::PIN_0,
            GpioNo::Gpio1 => GPIO::PIN_1,
            GpioNo::Gpio2 => GPIO::PIN_2,
            GpioNo::Gpio3 => GPIO::PIN_3,
            GpioNo::Gpio4 => GPIO::PIN_4,
            GpioNo::Gpio5 => GPIO::PIN_5,
            GpioNo::Gpio6 => GPIO::PIN_6,
            GpioNo::Gpio7 => GPIO::PIN_7,
            GpioNo::Gpio8 => GPIO::PIN_8,
            GpioNo::Gpio9 => GPIO::PIN_9,
            GpioNo::Gpio10 => GPIO::PIN_10,
            GpioNo::Gpio11 => GPIO::PIN_11,
            GpioNo::Gpio12 => GPIO::PIN_12,
            GpioNo::Gpio13 => GPIO::PIN_13,
            GpioNo::Gpio14 => GPIO::PIN_14,
            GpioNo::Gpio15 => GPIO::PIN_15,
            GpioNo::Gpio16 => GPIO::PIN_16,
            GpioNo::Gpio17 => GPIO::PIN_17,
            GpioNo::Gpio18 => GPIO::PIN_18,
            GpioNo::Gpio19 => GPIO::PIN_19,
            GpioNo::Gpio20 => GPIO::PIN_20,
            GpioNo::Gpio21 => GPIO::PIN_21,
            GpioNo::Gpio22 => GPIO::PIN_22,
            GpioNo::Gpio23 => GPIO::PIN_23,
            GpioNo::Gpio24 => GPIO::PIN_24,
            GpioNo::Gpio25 => GPIO::PIN_25,
            GpioNo::Gpio26 => GPIO::PIN_26,
            GpioNo::Gpio27 => GPIO::PIN_27,
            GpioNo::Gpio28 => GPIO::PIN_28,
            GpioNo::Gpio29 => GPIO::PIN_29,
            GpioNo::Gpio30 => GPIO::PIN_30,
            GpioNo::Gpio31 => GPIO::PIN_31,
        }
    }

    pub fn is_set(&self, val: u32) -> bool {
        let reg: DirectGpioReg = DirectGpioReg::new(val);
        reg.is_set(self.gpio_field())
    }

    pub fn output_selector(&self) -> PinmuxOutsel {
        // Add 3 to skip constant ConstantZero, ConstantOne, ConstantHighZ
        match PinmuxOutsel::try_from(*self as u32 + PinmuxOutsel::GpioGpio0 as u32) {
            Ok(select) => select,
            Err(_) => PinmuxOutsel::ConstantHighZ,
        }
    }
}

pub struct GpioPin<'a> {
    pin: GpioNo,
    pad: Pad,
    client: OptionalCell<&'a dyn gpio::Client>,
}

impl<'a> GpioPin<'a> {
    pub const fn new(pad: MuxedPads, pin: GpioNo) -> GpioPin<'a> {
        GpioPin {
            pin: pin,
            pad: Pad::Mio(pad),
            client: OptionalCell::empty(),
        }
    }

    #[inline(always)]
    fn half_set_oe(
        val: bool,
        pin: GpioNo,
        lower: &ReadWrite<u32, MASKED_OE_LOWER::Register>,
        upper: &ReadWrite<u32, MASKED_OE_UPPER::Register>,
    ) {
        // prepare place
        let shift = pin.field().shift;
        let bit = u32::from(val);

        if shift < 16 {
            //write
            lower.write(
                MASKED_OE_LOWER::DATA.val(bit << shift) + MASKED_OE_LOWER::MASK.val(1u32 << shift),
            );
        } else {
            // shift write
            let upper_shift = shift - 16;
            upper.write(
                MASKED_OE_UPPER::DATA.val(bit << upper_shift)
                    + MASKED_OE_UPPER::MASK.val(1u32 << upper_shift),
            );
        }
    }

    #[inline(always)]
    fn half_set_out(
        val: bool,
        pin: GpioNo,
        lower: &ReadWrite<u32, MASKED_OUT_LOWER::Register>,
        upper: &ReadWrite<u32, MASKED_OUT_UPPER::Register>,
    ) {
        // prepare place
        let shift = pin.field().shift;
        let bit = u32::from(val);

        if shift < 16 {
            //write
            lower.write(
                MASKED_OUT_LOWER::DATA.val(bit << shift)
                    + MASKED_OUT_LOWER::MASK.val(1u32 << shift),
            );
        } else {
            // shift write
            let upper_shift = shift - 16;
            upper.write(
                MASKED_OUT_UPPER::DATA.val(bit << upper_shift)
                    + MASKED_OUT_UPPER::MASK.val(1u32 << upper_shift),
            );
        }
    }

    pub fn handle_interrupt(&self) {
        if GPIO_BASE.intr_state.is_set(self.pin.field()) {
            GPIO_BASE.intr_state.modify(self.pin.field().val(1));
            self.client.map(|client| {
                client.fired();
            });
        }
    }
}

impl gpio::Configure for GpioPin<'_> {
    fn configuration(&self) -> gpio::Configuration {
        match self.pin.is_set(GPIO_BASE.direct_oe.get()) {
            true => gpio::Configuration::InputOutput,
            false => gpio::Configuration::Input,
        }
    }

    fn set_floating_state(&self, mode: gpio::FloatingState) {
        self.pad.set_floating_state(mode)
    }

    fn floating_state(&self) -> gpio::FloatingState {
        self.pad.floating_state()
    }

    fn deactivate_to_low_power(&self) {
        self.disable_input();
        self.disable_output();
    }

    fn make_output(&self) -> gpio::Configuration {
        GpioPin::half_set_oe(
            true,
            self.pin,
            &GPIO_BASE.masked_oe_lower,
            &GPIO_BASE.masked_oe_upper,
        );
        gpio::Configuration::InputOutput
    }

    fn disable_output(&self) -> gpio::Configuration {
        GpioPin::half_set_oe(
            false,
            self.pin,
            &GPIO_BASE.masked_oe_lower,
            &GPIO_BASE.masked_oe_upper,
        );
        gpio::Configuration::Input
    }

    fn make_input(&self) -> gpio::Configuration {
        self.configuration()
    }

    fn disable_input(&self) -> gpio::Configuration {
        self.configuration()
    }
}

impl gpio::Input for GpioPin<'_> {
    fn read(&self) -> bool {
        self.pin.is_set(GPIO_BASE.data_in.get())
    }
}

impl gpio::Output for GpioPin<'_> {
    fn toggle(&self) -> bool {
        let pin = self.pin;
        let new_state = !self.pin.is_set(GPIO_BASE.direct_out.get());

        GpioPin::half_set_out(
            new_state,
            self.pin,
            &GPIO_BASE.masked_out_lower,
            &GPIO_BASE.masked_out_upper,
        );
        new_state
    }

    fn set(&self) {
        GpioPin::half_set_out(
            true,
            self.pin,
            &GPIO_BASE.masked_out_lower,
            &GPIO_BASE.masked_out_upper,
        );
    }

    fn clear(&self) {
        GpioPin::half_set_out(
            false,
            self.pin,
            &GPIO_BASE.masked_out_lower,
            &GPIO_BASE.masked_out_upper,
        );
    }
}

impl<'a> gpio::Interrupt<'a> for GpioPin<'a> {
    fn set_client(&self, client: &'a dyn gpio::Client) {
        self.client.set(client);
    }

    fn enable_interrupts(&self, mode: gpio::InterruptEdge) {
        let pin = self.pin;

        match mode {
            gpio::InterruptEdge::RisingEdge => {
                GPIO_BASE.intr_ctrl_en_rising.modify(pin.field().val(1));
                GPIO_BASE.intr_ctrl_en_falling.modify(pin.field().val(0));
            }
            gpio::InterruptEdge::FallingEdge => {
                GPIO_BASE.intr_ctrl_en_rising.modify(pin.field().val(0));
                GPIO_BASE.intr_ctrl_en_falling.modify(pin.field().val(1));
            }
            gpio::InterruptEdge::EitherEdge => {
                GPIO_BASE.intr_ctrl_en_rising.modify(pin.field().val(1));
                GPIO_BASE.intr_ctrl_en_falling.modify(pin.field().val(1));
            }
        }
        GPIO_BASE.intr_state.modify(pin.field().val(1));
        GPIO_BASE.intr_enable.modify(pin.field().val(1));
    }

    fn disable_interrupts(&self) {
        let pin = self.pin;

        GPIO_BASE.intr_enable.modify(pin.field().val(0));
        // Clear any pending interrupt
        GPIO_BASE.intr_state.modify(pin.field().val(1));
    }

    fn is_pending(&self) -> bool {
        GPIO_BASE.intr_state.is_set(self.pin.field())
    }
}
