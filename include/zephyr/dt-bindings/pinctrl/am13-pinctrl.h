/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AM13_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AM13_PINCTRL_H_

#include <zephyr/dt-bindings/pinctrl/mspm0-pinctrl.h>

/*
 * AM13's PINCM array is 0-based and starts at the IOMUX base with no
 * leading reserved word (unlike mspm0/mspm33c's 1-based PINCMx labeling),
 * so the PINCM index is used directly here with no -1 adjustment.
 */
#define AM13_PINMUX(pincm, function) ((pincm) << 0x10 | (function))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_AM13_PINCTRL_H_ */
