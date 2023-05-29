// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

use kernel::utilities::StaticRef;
use lowrisc::aon_timer::AonTimerRegisters;
use crate::top::top_earlgrey::TOP_EARLGREY_AON_TIMER_AON_BASE_ADDR;

// Refer: https://github.com/lowRISC/opentitan/blob/217a0168ba118503c166a9587819e3811eeb0c0c/hw/top_earlgrey/sw/autogen/top_earlgrey_memory.h#L247
// This is based on the latest support commit of OpenTitan for Tock.
pub const AON_TIMER_BASE: StaticRef<AonTimerRegisters> =
    unsafe { StaticRef::new(TOP_EARLGREY_AON_TIMER_AON_BASE_ADDR as *const AonTimerRegisters) };
