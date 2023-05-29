// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

use kernel::utilities::StaticRef;
use lowrisc::csrng::CsRngRegisters;
use crate::top::top_earlgrey::TOP_EARLGREY_CSRNG_BASE_ADDR;

pub const CSRNG_BASE: StaticRef<CsRngRegisters> =
    unsafe { StaticRef::new(TOP_EARLGREY_CSRNG_BASE_ADDR as *const CsRngRegisters) };
