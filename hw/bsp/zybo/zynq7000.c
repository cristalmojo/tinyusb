/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Christian H. Meyer (Christian.H.Meyer@t-online.de),
 *                    Maximilian Klimm (Klimm.Max@gmail.com)
 * Ha Thach (tinyusb.org)
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

#include "../board.h"

#include "xparameters.h"
#include "xparameters_ps.h"
#include "xgpiops.h"
#include "xuartps.h"
#include "xstatus.h"
#include "xplatform_info.h"
#include "xscutimer.h"
#include "xscugic.h"
#include "xil_exception.h"
#include <xil_printf.h>

#include "xusbps.h" /* USB controller driver */
#include "xpseudo_asm.h"
#include "xreg_cortexa9.h"
#include "xil_cache.h"

/************************** Constant Definitions ****************************/

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define GPIO_DEVICE_ID XPAR_XGPIOPS_0_DEVICE_ID
#define UART_DEVICE_ID XPAR_XUARTPS_0_DEVICE_ID
#define TIMER_DEVICE_ID XPAR_XSCUTIMER_0_DEVICE_ID
#define INTC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID
#define TIMER_IRPT_ID XPAR_SCUTIMER_INTR
#define USB_IRPT_ID XPAR_XUSBPS_0_INTR

#define printf xil_printf /* Smalller foot-print printf */

#define LED_PIN 7
#define BUTTON_PIN 51

#define INPUT_DIRECTION 0
#define OUTPUT_DIRECTION 1

#define SYSTEM_FREQUENCY 650000000

XGpioPs Gpio; // GPIO driver instance
XUartPs Uart; // UART Driver instance
XUsbPs UsbInstance;	/* The instance of the USB Controller */
XScuGic IntcInstance;


void count_millis(void);
uint32_t board_millis(void);
void USB_IRQHandler(void *HandlerRef);
void dummy (void* something, long unsigned int someelse);

/*****************************************************************************/
/**
*
* This function sets up the interrupt system such that interrupts can occur
* for the device.
*
* @param	IntcInstancePtr is a pointer to the instance of XScuGic driver.
* @param	TimerInstancePtr is a pointer to the instance of XScuTimer
*		driver.
* @param	TimerIntrId is the Interrupt Id of the XScuTimer device.
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None.
*
******************************************************************************/
void TimerSetupIntrSystem(XScuGic *IntcInstancePtr,
                          XScuTimer *TimerInstancePtr, u16 TimerIntrId)
{
  XScuGic_Config *IntcConfig;

  /*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
  IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);

  XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
                        IntcConfig->CpuBaseAddress);

  Xil_ExceptionInit();

  /*
	 * Connect the interrupt controller interrupt handler to the hardware
	 * interrupt handling logic in the processor.
	 */
  Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
                               (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                               IntcInstancePtr);

  /*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
  XScuGic_Connect(IntcInstancePtr, TimerIntrId,
                           (Xil_ExceptionHandler)count_millis, (void *)TimerInstancePtr);

  // Enable the interrupt for the device.
  XScuGic_Enable(IntcInstancePtr, TimerIntrId);

  // Enable the timer interrupts for timer mode.
  XScuTimer_EnableInterrupt(TimerInstancePtr);

  // Enable interrupts in the Processor.
  Xil_ExceptionEnable();
}

void board_init(void)
{
  XGpioPs_Config *GpioConfigPtr;
  XUartPs_Config *UartConfigPtr;
  XScuTimer_Config *TimerConfigPtr;
  XUsbPs_Config *UsbConfigPtr;
  XScuTimer TimerInstance; /* Cortex A9 Scu Private Timer Instance */

  Xil_ICacheEnable();
  Xil_DCacheEnable();

  // Initialize the GPIO driver
  GpioConfigPtr = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
  XGpioPs_CfgInitialize(&Gpio, GpioConfigPtr, GpioConfigPtr->BaseAddr);
  XGpioPs_SetDirectionPin(&Gpio, LED_PIN, OUTPUT_DIRECTION);
  XGpioPs_SetDirectionPin(&Gpio, BUTTON_PIN, INPUT_DIRECTION);

  // Initialize UART driver
  UartConfigPtr = XUartPs_LookupConfig(UART_DEVICE_ID);
  XUartPs_CfgInitialize(&Uart, UartConfigPtr, UartConfigPtr->BaseAddress);
  XUartPs_SetBaudRate(&Uart, CFG_BOARD_UART_BAUDRATE);

  // Initialize USB driver
	u32 ModeValue = XUSBPS_MODE_CM_HOST_MASK;

  UsbConfigPtr = XUsbPs_LookupConfig(XPAR_XUSBPS_0_DEVICE_ID);
  XUsbPs_CfgInitialize(&UsbInstance, UsbConfigPtr, UsbConfigPtr->BaseAddress);
  XUsbPs_Reset(&UsbInstance);
  XUsbPs_IntrSetHandler(&UsbInstance, dummy, dummy, XUSBPS_IXR_ALL);
	XUsbPs_WriteReg(UsbConfigPtr->BaseAddress, XUSBPS_MODE_OFFSET, ModeValue);

  
  // Initialize Timer
#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  //ScuTimerIntrExample(&IntcInstance, &TimerInstance, TIMER_DEVICE_ID, TIMER_IRPT_INTR);

  TimerConfigPtr = XScuTimer_LookupConfig(TIMER_DEVICE_ID);
  XScuTimer_CfgInitialize(&TimerInstance, TimerConfigPtr, TimerConfigPtr->BaseAddr);
  XScuTimer_SelfTest(&TimerInstance);
  TimerSetupIntrSystem(&IntcInstance, &TimerInstance, TIMER_IRPT_ID);
  XScuTimer_EnableAutoReload(&TimerInstance);
  XScuTimer_LoadTimer(&TimerInstance, (int)SYSTEM_FREQUENCY / 2000);
  XScuTimer_Start(&TimerInstance);
#endif

  //XScuGic_Connect(&IntcInstance, USB_IRPT_ID, XUsbPs_IntrHandler, &UsbInstance);
  XScuGic_Connect(&IntcInstance, USB_IRPT_ID, USB_IRQHandler, &UsbInstance);
  XScuGic_Enable(&IntcInstance, USB_IRPT_ID);
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  XGpioPs_WritePin(&Gpio, LED_PIN, state);
}

// TODO: test
uint32_t board_button_read(void)
{
  return XGpioPs_ReadPin(&Gpio, BUTTON_PIN);
}

// TODO: test
int board_uart_read(uint8_t *buf, int len)
{
  return XUartPs_Recv(&Uart, buf, len);
}

/*
 * XUartPs_Send does not block on invocation. Instead, it returns immediately
 * the count of sent bytes. Therefore, we do some busy waiting to make sure
 * that everything has been sent.
 */
int board_uart_write(void const *buf, int len)
{
  int sent_bytes = 0;
  u8 *buf_pos = (u8 *) buf;
  while (sent_bytes < len) {
    sent_bytes += XUartPs_Send(&Uart, buf_pos, len-sent_bytes);
    buf_pos = (u8 *) buf + sent_bytes;
  } 

  return sent_bytes;
}

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB_IRQHandler(void *HandlerRef)
{
  printf("USB Interrupt\n");
  tuh_isr(0);
  //tuh_int_handler(0);
}

void dummy (void* something, long unsigned int someelse) {
  return;
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t cnt = 0;
void count_millis(void)
{
  cnt++;
}

uint32_t board_millis(void)
{
  return cnt;
}
#endif
