// Generated register constants for rv_timer.
// This file is licensed under either of:
//   Apache License, Version 2.0 (LICENSE-APACHE <http://www.apache.org/licenses/LICENSE-2.0>)
//   MIT License (LICENSE-MIT <http://opensource.org/licenses/MIT>)

// Built for earlgrey_silver_release_v5-8164-g7a7139c8a
// https://github.com/lowRISC/opentitan/tree/7a7139c8af345f423ac27a0186febdda027f7127
// Tree status: clean
// Build date: 2022-10-25T11:35:38

// Original reference file: hw/ip/rv_timer/data/rv_timer.hjson
use kernel::utilities::registers::ReadWrite;
use kernel::utilities::registers::{register_bitfields, register_structs};
// Number of harts
pub const RV_TIMER_PARAM_N_HARTS: u32 = 1;
// Number of timers per Hart
pub const RV_TIMER_PARAM_N_TIMERS: u32 = 1;
// Number of alerts
pub const RV_TIMER_PARAM_NUM_ALERTS: u32 = 1;
// Register width
pub const RV_TIMER_PARAM_REG_WIDTH: u32 = 32;

register_structs! {
    pub RvTimerRegisters {
        // Alert Test Register
        (0x0000 => pub(crate) alert_test: ReadWrite<u32, ALERT_TEST::Register>),
        // Control register
        (0x0004 => pub(crate) ct: [ReadWrite<u32, CT::Register>; 1]),
        // Interrupt Enable
        (0x0100 => pub(crate) intr_enabl: [ReadWrite<u32, INTR_ENABL::Register>; 1]),
        // Interrupt Status
        (0x0104 => pub(crate) intr_stat: [ReadWrite<u32, INTR_STAT::Register>; 1]),
        // Interrupt test register
        (0x0108 => pub(crate) intr_tes: [ReadWrite<u32, INTR_TES::Register>; 1]),
        // Configuration for Hart 0
        (0x010c => pub(crate) cfg0: ReadWrite<u32, CFG0::Register>),
        // Timer value Lower
        (0x0110 => pub(crate) timer_v_lower0: ReadWrite<u32, TIMER_V_LOWER0::Register>),
        // Timer value Upper
        (0x0114 => pub(crate) timer_v_upper0: ReadWrite<u32, TIMER_V_UPPER0::Register>),
        // Timer value Lower
        (0x0118 => pub(crate) compare_lower0_0: ReadWrite<u32, COMPARE_LOWER0_0::Register>),
        // Timer value Upper
        (0x011c => pub(crate) compare_upper0_0: ReadWrite<u32, COMPARE_UPPER0_0::Register>),
        (0x0120 => @END),
    }
}

register_bitfields![u32,
    pub(crate) ALERT_TEST [
        FATAL_FAULT OFFSET(0) NUMBITS(1) [],
    ],
    pub(crate) CT [
        ACTIVE_0 OFFSET(0) NUMBITS(1) [],
    ],
    pub(crate) INTR_ENABL [
        IE_0 OFFSET(0) NUMBITS(1) [],
    ],
    pub(crate) INTR_STAT [
        IS_0 OFFSET(0) NUMBITS(1) [],
    ],
    pub(crate) INTR_TES [
        T_0 OFFSET(0) NUMBITS(1) [],
    ],
    pub(crate) CFG0 [
        PRESCALE OFFSET(0) NUMBITS(12) [],
        STEP OFFSET(16) NUMBITS(8) [],
    ],
    pub(crate) TIMER_V_LOWER0 [
        V OFFSET(0) NUMBITS(32) [],
    ],
    pub(crate) TIMER_V_UPPER0 [
        V OFFSET(0) NUMBITS(32) [],
    ],
    pub(crate) COMPARE_LOWER0_0 [
        V OFFSET(0) NUMBITS(32) [],
    ],
    pub(crate) COMPARE_UPPER0_0 [
        V OFFSET(0) NUMBITS(32) [],
    ],
];

// End generated register constants for rv_timer
