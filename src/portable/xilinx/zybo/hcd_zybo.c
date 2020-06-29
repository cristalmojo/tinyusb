/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include <inttypes.h>

#include "tusb_option.h"
#include "xparameters.h"
#include "xscugic.h"
#include "xusbps_hw.h"

#if TUSB_OPT_HOST_ENABLED && (CFG_TUSB_MCU == OPT_MCU_ZYBO)

extern XScuGic IntcInstance;

// LPC18xx and 43xx use EHCI driver

void hcd_int_enable(uint8_t rhport)
{
    XScuGic_Enable(&IntcInstance, XPAR_XUSBPS_0_INTR);
}

void hcd_int_disable(uint8_t rhport)
{
	XScuGic_Disable(&IntcInstance, XPAR_XUSBPS_0_INTR);
}

uint32_t hcd_ehci_register_addr(uint8_t rhport)
{
    // TODO: cleanup
    return (rhport ? XPS_USB1_BASEADDR : XPS_USB0_BASEADDR) + XUSBPS_CMD_OFFSET;
}

#endif
