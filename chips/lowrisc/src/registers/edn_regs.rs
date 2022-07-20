// Generated register constants for edn.
// This file is licensed under either of:
//   Apache License, Version 2.0 (LICENSE-APACHE <http://www.apache.org/licenses/LICENSE-2.0>)
//   MIT License (LICENSE-MIT <http://opensource.org/licenses/MIT>)

// Built for earlgrey_silver_release_v5-6422-g05dcfbd00
// https://github.com/lowRISC/opentitan/tree/05dcfbd00ca893dba034b468d1754f3f50780080
// Tree status: clean
// Build date: 2022-07-20T21:02:26

// Original reference file: hw/ip/edn/data/edn.hjson
use kernel::utilities::registers::ReadWrite;
use kernel::utilities::registers::{register_bitfields, register_structs};
// Number of alerts
pub const EDN_PARAM_NUM_ALERTS: u32 = 2;
// Register width
pub const EDN_PARAM_REG_WIDTH: u32 = 32;

register_structs! {
    pub EdnRegisters {
        // Interrupt State Register
        (0x0000 => pub(crate) intr_state: ReadWrite<u32, INTR::Register>),
        // Interrupt Enable Register
        (0x0004 => pub(crate) intr_enable: ReadWrite<u32, INTR::Register>),
        // Interrupt Test Register
        (0x0008 => pub(crate) intr_test: ReadWrite<u32, INTR::Register>),
        // Alert Test Register
        (0x000c => pub(crate) alert_test: ReadWrite<u32, ALERT_TEST::Register>),
        // Register write enable for all control registers
        (0x0010 => pub(crate) regwen: ReadWrite<u32, REGWEN::Register>),
        // EDN control register
        (0x0014 => pub(crate) ctrl: ReadWrite<u32, CTRL::Register>),
        // EDN boot instantiate command register
        (0x0018 => pub(crate) boot_ins_cmd: ReadWrite<u32, BOOT_INS_CMD::Register>),
        // EDN boot generate command register
        (0x001c => pub(crate) boot_gen_cmd: ReadWrite<u32, BOOT_GEN_CMD::Register>),
        // EDN csrng app command request register
        (0x0020 => pub(crate) sw_cmd_req: ReadWrite<u32, SW_CMD_REQ::Register>),
        // EDN command status register
        (0x0024 => pub(crate) sw_cmd_sts: ReadWrite<u32, SW_CMD_STS::Register>),
        // EDN csrng reseed command register
        (0x0028 => pub(crate) reseed_cmd: ReadWrite<u32, RESEED_CMD::Register>),
        // EDN csrng generate command register
        (0x002c => pub(crate) generate_cmd: ReadWrite<u32, GENERATE_CMD::Register>),
        // EDN maximum number of requests between reseeds register
        (0x0030 => pub(crate) max_num_reqs_between_reseeds: ReadWrite<u32, MAX_NUM_REQS_BETWEEN_RESEEDS::Register>),
        // Recoverable alert status register
        (0x0034 => pub(crate) recov_alert_sts: ReadWrite<u32, RECOV_ALERT_STS::Register>),
        // Hardware detection of error conditions status register
        (0x0038 => pub(crate) err_code: ReadWrite<u32, ERR_CODE::Register>),
        // Test error conditions register
        (0x003c => pub(crate) err_code_test: ReadWrite<u32, ERR_CODE_TEST::Register>),
        // Main state machine state debug register
        (0x0040 => pub(crate) main_sm_state: ReadWrite<u32, MAIN_SM_STATE::Register>),
        (0x0044 => @END),
    }
}

register_bitfields![u32,
    // Common Interrupt Offsets
    pub(crate) INTR [
        EDN_CMD_REQ_DONE OFFSET(0) NUMBITS(1) [],
        EDN_FATAL_ERR OFFSET(1) NUMBITS(1) [],
    ],
    pub(crate) ALERT_TEST [
        RECOV_ALERT OFFSET(0) NUMBITS(1) [],
        FATAL_ALERT OFFSET(1) NUMBITS(1) [],
    ],
    pub(crate) REGWEN [
        REGWEN OFFSET(0) NUMBITS(1) [],
    ],
    pub(crate) CTRL [
        EDN_ENABLE OFFSET(0) NUMBITS(4) [],
        BOOT_REQ_MODE OFFSET(4) NUMBITS(4) [],
        AUTO_REQ_MODE OFFSET(8) NUMBITS(4) [],
        CMD_FIFO_RST OFFSET(12) NUMBITS(4) [],
    ],
    pub(crate) BOOT_INS_CMD [],
    pub(crate) BOOT_GEN_CMD [],
    pub(crate) SW_CMD_REQ [],
    pub(crate) SW_CMD_STS [
        CMD_RDY OFFSET(0) NUMBITS(1) [],
        CMD_STS OFFSET(1) NUMBITS(1) [],
    ],
    pub(crate) RESEED_CMD [],
    pub(crate) GENERATE_CMD [],
    pub(crate) MAX_NUM_REQS_BETWEEN_RESEEDS [],
    pub(crate) RECOV_ALERT_STS [
        EDN_ENABLE_FIELD_ALERT OFFSET(0) NUMBITS(1) [],
        BOOT_REQ_MODE_FIELD_ALERT OFFSET(1) NUMBITS(1) [],
        AUTO_REQ_MODE_FIELD_ALERT OFFSET(2) NUMBITS(1) [],
        CMD_FIFO_RST_FIELD_ALERT OFFSET(3) NUMBITS(1) [],
        EDN_BUS_CMP_ALERT OFFSET(12) NUMBITS(1) [],
    ],
    pub(crate) ERR_CODE [
        SFIFO_RESCMD_ERR OFFSET(0) NUMBITS(1) [],
        SFIFO_GENCMD_ERR OFFSET(1) NUMBITS(1) [],
        EDN_ACK_SM_ERR OFFSET(20) NUMBITS(1) [],
        EDN_MAIN_SM_ERR OFFSET(21) NUMBITS(1) [],
        EDN_CNTR_ERR OFFSET(22) NUMBITS(1) [],
        FIFO_WRITE_ERR OFFSET(28) NUMBITS(1) [],
        FIFO_READ_ERR OFFSET(29) NUMBITS(1) [],
        FIFO_STATE_ERR OFFSET(30) NUMBITS(1) [],
    ],
    pub(crate) ERR_CODE_TEST [
        ERR_CODE_TEST OFFSET(0) NUMBITS(5) [],
    ],
    pub(crate) MAIN_SM_STATE [
        MAIN_SM_STATE OFFSET(0) NUMBITS(9) [],
    ],
];

// End generated register constants for edn