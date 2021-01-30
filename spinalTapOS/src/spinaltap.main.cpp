#include "platform.h"

#include "xscugic.h"
#include "xusbps.h"

#include "xil_printf.h"
#include "xusb_spinaltap.hpp"
#include <stdio.h>
#include <stdlib.h>

static int setup_interrupts(XScuGic *intc) {
  XScuGic_Config *intc_config =
      XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
  if (intc_config == 0)
    return XST_FAILURE;

  int status =
      XScuGic_CfgInitialize(intc, intc_config, intc_config->CpuBaseAddress);
  if (status != XST_SUCCESS)
    return status;

  Xil_ExceptionInit();
  Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
                               (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                               intc);
  return XST_SUCCESS;
}

static void readFromUsb(uint8_t *buffer, uint32_t len) {
  do {
    const uint32_t fromCurrent = std::min<size_t>(len, rb.available());
    if (fromCurrent == 0)
      continue;

    rb.read(buffer, fromCurrent);
    buffer += fromCurrent;
    len -= fromCurrent;
  } while (len > 0);
}

fixed_ringbuffer<uint8_t, 2 * 1024> resp;

template <typename T> static T readFromUsb() {
  T val;
  readFromUsb((uint8_t *)&val, sizeof(T));
  return val;
}

static const uintptr_t baseAddress = 0x43c00000UL;
static void process_write(uint8_t source) {
  uint16_t address = readFromUsb<uint16_t>();
  uint32_t value = readFromUsb<uint16_t>();
  *(uint32_t *)(baseAddress + address) = value;
  resp.write(source);
  resp.write(0);
}

static void process_read(uint8_t source) {
  uint16_t address = readFromUsb<uint16_t>();
  uint32_t value = *(uint32_t *)(baseAddress + address);
  resp.write(source);
  resp.write(0);
  resp.write((uint8_t *)&value, sizeof(value));
}

enum class cmd : uint8_t { write = 0x01, read = 0x02 };

static void process_cmd(XUsbPs *usb) {
  while (1) {
    struct header_t {
      uint8_t source;
      uint8_t cmd;
    } __attribute__((packed));
    const header_t header = readFromUsb<header_t>();
    switch (static_cast<cmd>(header.cmd)) {
    case cmd::write:
      process_write(header.source);
      break;
    case cmd::read:
      process_read(header.source);
      break;
    default:
      break;
    }

    if (rb.available() == 0) {
      uint8_t buffer[512];
      while (resp.available() != 0) {
        // TODO: better API to not have to copy
        const uint32_t available = resp.available();
        const size_t frameSize = std::min<size_t>(available, sizeof(buffer));
        resp.read(buffer, frameSize);
        if (available == frameSize)
          XUsbPs_EpBufferSend(usb, 1, buffer, frameSize);
        else
          XUsbPs_EpBufferSendWithZLT(usb, 1, buffer, frameSize);
      }
    }
  }
}

struct pwm_t {
  uint32_t divider;
  uint32_t width1;
  uint32_t width2;
  uint32_t width3;
};
#define PWM ((pwm_t *)0x43c00000UL)

int main(void) {
  init_platform();

  PWM->divider = 1000;
  PWM->width1 = 1;
  PWM->width2 = 128;
  PWM->width3 = 255;

  print("Hello World\n\r");
  print("Successfully ran Hello World application\n\r");

  XScuGic intc;
  int status = setup_interrupts(&intc);
  if (status != XST_SUCCESS) {
    xil_printf("ERROR: Unable to initialize interrupt system: %d\n\r", status);
    exit(1);
  }

  XUsbPs usb;
  status =
      xusbps_spinaltap_register_interrupt(&intc, &usb, XPAR_PS7_USB_0_INTR);
  if (status != XST_SUCCESS) {
    xil_printf("ERROR: Unable to register USB interrupts: %d\n\r", status);
    exit(1);
  }

  // Enable CPU IRQs
  Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);

  status = xusbps_spinaltap_init(&usb, XPAR_PS7_USB_0_DEVICE_ID,
                                 XPAR_PS7_USB_0_INTR, 64 * 1024);
  if (status != XST_SUCCESS) {
    xil_printf("ERROR: Unable to set up USB controller: %d\n\r", status);
    exit(1);
  }

  while (1) {
    process_cmd(&usb);
  }

  cleanup_platform();
  return 0;
}
