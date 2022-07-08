#include <stdio.h>
#include <stdlib.h>

#include "debug.hpp"
#include "platform.h"
#include "xil_printf.h"
#include "xscugic.h"
#include "xusb_spinaltap.hpp"
#include "xusbps.h"

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

[[nodiscard]] constexpr bool valid_address(uint16_t address) noexcept {
  return (address & 3) == 0;
}

static const uintptr_t baseAddress = 0x43c00000UL;
static void process_write(uint8_t source) {
  uint16_t address = readFromUsb<uint16_t>();
  uint32_t value = readFromUsb<uint32_t>();

  if (!valid_address(address)) {
    xil_printf("invalid write address %04x", (uint32_t)address);
    resp.write(source);
    resp.write(1);
    return;
  }

  xil_printf("writing @%04x=%x\r\n", (uint32_t)address, value);
  *(uint32_t *)(baseAddress + address) = value;
  resp.write(source);
  resp.write(0);
}

static void process_read(uint8_t source) {
  uint16_t address = readFromUsb<uint16_t>();
  uint32_t value = *(uint32_t *)(baseAddress + address);
  // TODO move read after check, adapt library accordingly
  if (!valid_address(address)) {
    xil_printf("invalid read address %04x", (uint32_t)address);
    resp.write(source);
    resp.write(1);
    return;
  }

  xil_printf("reading @%04x=%x\r\n", (uint32_t)address, value);
  resp.write(source);
  resp.write(0);
  resp.write((uint8_t *)&value, sizeof(value));
}

static void process_write_stream_8(uint8_t source) {
  uint16_t address = readFromUsb<uint16_t>();
  uint16_t size = readFromUsb<uint16_t>();

  xil_printf("writing @%04x=xx (%d)\r\n", (uint32_t)address, (uint32_t)size);

  while (size--) {
    uint8_t value = readFromUsb<uint8_t>();
    *(uint32_t *)(baseAddress + address) = value;
  }
  resp.write(source);
  resp.write(0);
}

static void process_read_stream_8(uint8_t source) {
  uint16_t address = readFromUsb<uint16_t>();
  uint16_t size = readFromUsb<uint16_t>();

  xil_printf("reading 8bit @%04x=xx (%d)\r\n", (uint32_t)address, (uint32_t)size);

  resp.write(source);
  resp.write(0);
  while(size--) {
	  uint32_t v = *(uint32_t *)(baseAddress + address);
	  resp.write((uint8_t)v);
  }
}

static void process_read_stream_32(uint8_t source) {
  uint16_t address = readFromUsb<uint16_t>();
  uint16_t size = readFromUsb<uint16_t>();

  xil_printf("reading 32bit @%04x=xx (%d)\r\n", (uint32_t)address, (uint32_t)size);

  resp.write(source);
  resp.write(0);
  while(size--) {
	  uint32_t v = *(uint32_t *)(baseAddress + address);
	  resp.write(v);
  }
}

static void process_read_modify_write(uint8_t source) {
  uint16_t address = readFromUsb<uint16_t>();
  uint32_t mask = readFromUsb<uint32_t>();
  uint32_t value = readFromUsb<uint32_t>();

  xil_printf("rmw @%04x = %04x mask %04x", (uint32_t)address, (uint32_t)value, (uint32_t)mask);

  uint32_t current = *(uint32_t *)(baseAddress + address);
  uint32_t updated = (current & ~mask) | (current & mask);
  *(uint32_t *)(baseAddress + address) = updated;

  resp.write(source);
  resp.write(0);
}

enum class cmd : uint8_t {
  write = 0x01,
  read = 0x02,
  write_stream_8 = 0x03,
  read_stream_8 = 0x04,
  read_stream_32 = 0x05,
  read_modify_write = 0x06
};
#include <array>
static void process_cmd(XUsbPs *usb) {
  using func = void (*)(uint8_t);
  std::array<func, 6> f = {process_write, process_read, process_write_stream_8,
                           process_read_stream_8, process_read_stream_32,
						   process_read_modify_write};
  while (1) {
    struct header_t {
      uint8_t source;
      uint8_t cmd;
    } __attribute__((packed));
    const header_t header = readFromUsb<header_t>();

    uint8_t idx = header.cmd - 1;
    if (idx < f.size() && f[idx] != nullptr)
      f[idx](header.source);
    else
      xil_printf("invalid cmd");

    if (rb.available() == 0) {
      uint8_t buffer[512] ALIGNMENT_CACHELINE;
      while (resp.available() != 0) {
        // TODO: better API to not have to copy
        const uint32_t available = resp.available();
        const size_t frameSize = std::min<size_t>(available, sizeof(buffer));
        resp.read(buffer, frameSize);
        xil_printf("resp: ");
        dump_buffer(buffer, frameSize);
        xil_printf("tx zlt\r\n");
        if (auto status = XUsbPs_EpBufferSendWithZLT(usb, 1, buffer, frameSize);
            status != XST_SUCCESS) {
          xil_printf("send error: %d\n\r", status);
          return;
        }
      }
    }
  }
}

#include "pwm.h"

int main(void) {
  init_platform();
  PWM->prescaler = 1000;
  PWM->max = 255;
  PWM->ctrl = PWM_CTRL_RUN;
  PWM->level[0] = 1;
  PWM->level[1] = 128;
  PWM->level[2] = 255;

  print("Initializing\n\r");

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

  process_cmd(&usb);

  cleanup_platform();
  return 0;
}
