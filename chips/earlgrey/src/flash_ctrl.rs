// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

use kernel::utilities::StaticRef;
use lowrisc::flash_ctrl::FlashCtrlRegisters;
use crate::top::top_earlgrey::TOP_EARLGREY_FLASH_CTRL_CORE_BASE_ADDR;

pub const FLASH_CTRL_BASE: StaticRef<FlashCtrlRegisters> =
    unsafe { StaticRef::new(TOP_EARLGREY_FLASH_CTRL_CORE_BASE_ADDR as *const FlashCtrlRegisters) };
