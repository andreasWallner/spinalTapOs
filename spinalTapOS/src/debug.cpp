#include "debug.hpp"
#include "xil_printf.h"

void dump_buffer(u8 *buffer, u32 length) {
  xil_printf("0x%04X: ", (int)buffer);
  while (length--) {
    xil_printf("%02X ", *buffer++);
  }
  xil_printf("\r\n");
}
