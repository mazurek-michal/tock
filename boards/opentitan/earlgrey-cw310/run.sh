#!/bin/bash

# Licensed under the Apache License, Version 2.0 or the MIT License.
# SPDX-License-Identifier: Apache-2.0 OR MIT
# Copyright Tock Contributors 2023.

BUILD_DIR="verilator_build/"

# Preemptively cleanup layout (incase this was a test) so that following apps (non-tests) load the correct layout.
rm $TOCK_ROOT_DIRECTORY/target/$TARGET/release/deps/layout.ld

if [[ "${VERILATOR}" == "yes" ]]; then
		if [ -d "$BUILD_DIR" ]; then
			# Cleanup before we build again
			printf "\n[CW-130: Verilator Tests]: Cleaning up verilator_build...\n\n"
			rm -R "$BUILD_DIR"/*
		else
			printf "\n[CW-130: Verilator Tests]: Setting up verilator_build...\n\n"
			mkdir "$BUILD_DIR"
		fi


	${OPENTITAN_TREE}/bazelisk.sh build \
		//hw:verilator \
		//hw/ip/otp_ctrl/data:img_rma \
		//sw/device/lib/testing/test_rom:test_rom
	VERILATOR_DIR=$(${OPENTITAN_TREE}/bazelisk.sh outquery //hw:verilator)
	OTP_IMAGE=$(${OPENTITAN_TREE}/bazelisk.sh outquery //hw/ip/otp_ctrl/data:img_rma)
	TEST_ROM=$(${OPENTITAN_TREE}/bazelisk.sh outquery //sw/device/lib/testing/test_rom:test_rom_sim_verilator_scr_vmem)

	# Copy in and covert from cargo test output to binary
	${OBJCOPY} ${1} "$BUILD_DIR"/earlgrey-cw310-tests.elf
	if [[ "${APP}" != "" ]]; then
		# An app was specified, copy it in
		printf "[CW-130: Verilator Tests]: Linking APP\n\n"
		${OBJCOPY} --update-section .apps=${APP} "$BUILD_DIR"/earlgrey-cw310-tests.elf "$BUILD_DIR"/earlgrey-cw310-tests.elf
	fi
	${OBJCOPY} --output-target=binary "$BUILD_DIR"/earlgrey-cw310-tests.elf "$BUILD_DIR"/earlgrey-cw310-tests.bin
	# Create VMEM file from test binary
	srec_cat "$BUILD_DIR"/earlgrey-cw310-tests.bin\
		--binary --offset 0 --byte-swap 8 --fill 0xff \
		-within "$BUILD_DIR"/earlgrey-cw310-tests.bin\
		-binary -range-pad 8 --output "$BUILD_DIR"/binary.64.vmem --vmem 64
	${OPENTITAN_TREE}/${VERILATOR_DIR}/sim-verilator/Vchip_sim_tb \
		--meminit=rom,${OPENTITAN_TREE}/${TEST_ROM} \
		--meminit=flash,./"$BUILD_DIR"/binary.64.vmem \
		--meminit=otp,${OPENTITAN_TREE}/${OTP_IMAGE}
elif [[ "${OPENTITAN_TREE}" != "" ]]; then
	${OBJCOPY} --update-section .apps=${APP} ${1} bundle.elf
	${OBJCOPY} --output-target=binary bundle.elf binary

	${OPENTITAN_TREE}/bazelisk.sh run //sw/host/opentitantool -- \
        --rcfile= \
		--interface=cw310 --conf=${OPENTITAN_TREE}/sw/host/opentitantool/config/opentitan_cw310.json \
        bootstrap $(realpath binary)
else
	../../../tools/qemu/build/qemu-system-riscv32 -M opentitan -nographic -serial stdio -monitor none -semihosting -kernel "${1}" -global driver=riscv.lowrisc.ibex.soc,property=resetvec,value=${QEMU_ENTRY_POINT}
fi
