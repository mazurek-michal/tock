// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

use kernel::utilities::StaticRef;
use lowrisc::pwrmgr::PwrMgrRegisters;
use crate::top::top_earlgrey::TOP_EARLGREY_PWRMGR_AON_BASE_ADDR;

pub(crate) const PWRMGR_BASE: StaticRef<PwrMgrRegisters> =
    unsafe { StaticRef::new(TOP_EARLGREY_PWRMGR_AON_BASE_ADDR as *const PwrMgrRegisters) };
