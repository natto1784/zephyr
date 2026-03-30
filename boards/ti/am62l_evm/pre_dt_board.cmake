# Texas Instruments Sitara AM62L EVM
#
# Copyright (c) 2026 Texas Instruments Incorporated
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_AM62L_EVM_IO_EXPANDER)
  list(APPEND EXTRA_DTC_OVERLAY_FILE "${BOARD_DIR}/dts/io_expander.overlay")
endif()
