#include "xusb_spinaltap.hpp"

#include <array>
#include <string_view>

#include "inttypes.h" // for printing...
#include "sleep.h"
#include "spinaltap.hpp"
#include "usb/constants.hpp"
#include "usb/descriptors.hpp"
#include "usb/device.hpp"
#include "xil_cache.h"

// TODO replace
#define be2le(val) (u32)(val)
#define be2les(x) (u16)(x)

fixed_ringbuffer<uint8_t, 10 * 512> rb;

static void usb_HandleStdDevRequest(XUsbPs *usb, XUsbPs_SetupData *setupData);
static void usb_HandleClassIfRequest(XUsbPs *usb, XUsbPs_SetupData *setupData);
static int usb_HandleVendorReq(XUsbPs *usb, XUsbPs_SetupData *setupData);

uint32_t usb_ch9_SetupStringDescReply(uint8_t reply[], uint32_t replySize,
                                      uint8_t index);
void usb_SetConfiguration(XUsbPs *usb, int configIdx);

void xusb_spinaltap_irq_handler(void *callbackRef, uint32_t mask);
void xusb_spinaltap_ep0_irq_handler(void *callbackRef, uint8_t endpoint,
                                    uint8_t eventType, void *data);
void xusb_spinaltap_ep1_irq_handler(void *callbackRef, uint8_t endpoint,
                                    uint8_t eventType, void *data);
void xusb_spinaltap_ep2_irq_handler(void *callbackRef, uint8_t endpoint,
                                    uint8_t eventType, void *data);
void xusb_spinaltap_ep3_irq_handler(void *callbackRef, uint8_t endpoint,
                                    uint8_t eventType, void *data);

typedef enum {
  STATE_UNCONNECTED,
  STATE_CONFIG,
  STATE_CONNECTED
} spinaltap_state_t;
volatile spinaltap_state_t usb_driver_state;

int xusbps_spinaltap_register_interrupt(XScuGic *intc, XUsbPs *usb,
                                        u16 usb_irq) {
  int status;
  if (!intc->IsReady) {
    return XST_FAILURE;
  }

  /* Connect the generic USB controller handler for the device that will be
   * called for all USB events. This handler will then call other specific
   * routines for handling particular events.
   */
  status = XScuGic_Connect(
      intc, usb_irq, (Xil_ExceptionHandler)XUsbPs_IntrHandler, (void *)usb);
  if (status != XST_SUCCESS) {
    return status;
  }

  /* Enable the interrupt for the device */
  XScuGic_Enable(intc, usb_irq);

  return XST_SUCCESS;
}

#define USB_DMA_BUFFER_SIZE (64 * 1024)
static uint8_t usb_dma_buffer[USB_DMA_BUFFER_SIZE] ALIGNMENT_CACHELINE;

int xusbps_spinaltap_init(XUsbPs *usb, uint16_t usbDeviceId, uint16_t usbIrq,
                          uint32_t rxBufSize) {
  XUsbPs_Config *usbCfg;
  int status;
  usbCfg = XUsbPs_LookupConfig(usbDeviceId);
  if (usbCfg == 0)
    goto cleanup;
  status = XUsbPs_CfgInitialize(usb, usbCfg, usbCfg->BaseAddress);
  if (status != XST_SUCCESS)
    goto cleanup;

  // TODO get buffer via XUsbPs_DeviceMemRequired & alloc
  const static XUsbPs_DeviceConfig deviceConfig = {
      .NumEndpoints = 4,
      .EpCfg = {{.Out = {.Type = XUSBPS_EP_TYPE_CONTROL,
                         .NumBufs = 2,
                         .BufSize = 64,
                         .MaxPacketSize = 64},
                 .In = {.Type = XUSBPS_EP_TYPE_CONTROL,
                        .NumBufs = 2,
                        .MaxPacketSize = 64}},
                {.Out = {.Type = XUSBPS_EP_TYPE_BULK,
                         .NumBufs = 16,
                         .BufSize = 512,
                         .MaxPacketSize = 512},
                 .In = {.Type = XUSBPS_EP_TYPE_BULK,
                        .NumBufs = 16,
                        .MaxPacketSize = 512}},
                {.Out = {.Type = XUSBPS_EP_TYPE_BULK,
                         .NumBufs = 16,
                         .BufSize = 512,
                         .MaxPacketSize = 512},
                 .In = {.Type = XUSBPS_EP_TYPE_BULK,
                        .NumBufs = 16,
                        .MaxPacketSize = 512}},
                {.Out = {.Type = XUSBPS_EP_TYPE_BULK,
                         .NumBufs = 16,
                         .BufSize = 512,
                         .MaxPacketSize = 512},
                 .In = {.Type = XUSBPS_EP_TYPE_BULK,
                        .NumBufs = 16,
                        .MaxPacketSize = 512}},
                {0},
                {0},
                {0},
                {0},
                {0},
                {0},
                {0},
                {0}},
      .DMAMemPhys = (uint32_t)usb_dma_buffer};
  memset(usb_dma_buffer, 0, USB_DMA_BUFFER_SIZE);
  Xil_DCacheFlushRange((unsigned int)usb_dma_buffer, USB_DMA_BUFFER_SIZE);

  status = XUsbPs_ConfigureDevice(usb, &deviceConfig);
  if (status != XST_SUCCESS)
    goto cleanup;

  status = XUsbPs_IntrSetHandler(usb, xusb_spinaltap_irq_handler, 0,
                                 XUSBPS_IXR_UE_MASK);
  if (status != XST_SUCCESS)
    goto cleanup;

  XUsbPs_EpSetHandler(usb, 0, XUSBPS_EP_DIRECTION_OUT,
                      xusb_spinaltap_ep0_irq_handler,
                      usb); // TODO check return values
  XUsbPs_EpSetHandler(usb, 1, XUSBPS_EP_DIRECTION_IN,
                      xusb_spinaltap_ep1_irq_handler, usb);
  XUsbPs_EpSetHandler(usb, 2, XUSBPS_EP_DIRECTION_OUT,
                      xusb_spinaltap_ep2_irq_handler, usb);
  XUsbPs_EpSetHandler(usb, 3, XUSBPS_EP_DIRECTION_IN,
                      xusb_spinaltap_ep3_irq_handler, usb);

  XUsbPs_IntrEnable(usb, XUSBPS_IXR_UR_MASK | XUSBPS_IXR_UI_MASK);

  // initialize internal data structures

  XUsbPs_Start(usb);
  return XST_SUCCESS;

cleanup:
  XUsbPs_Stop(usb);
  XUsbPs_IntrDisable(usb, XUSBPS_IXR_ALL);
  (void)XUsbPs_IntrSetHandler(usb, 0, 0, 0);
  return status;
}

int xusb_cdc_handle_ch9_setup_packet(XUsbPs *usb, XUsbPs_SetupData *SetupData) {
  int Status = XST_SUCCESS;

#ifdef XUSB_CDC_DEBUG
  printf("Handle setup packet\n");
#endif

  xil_printf("setup type: %x request: %x value: %x length: %d\r\n",
             (int)SetupData->bmRequestType, (int)SetupData->bRequest,
             (int)SetupData->wValue, (int)SetupData->wLength);

  switch ((usb::std::requests)(SetupData->bmRequestType &
                               usb::std::REQ_TYPE_MASK)) {
  case usb::std::requests::STD:
    usb_HandleStdDevRequest(usb, SetupData);
    break;

  case usb::std::requests::CLASS:
    usb_HandleClassIfRequest(usb, SetupData);
    break;

  case usb::std::requests::VENDOR:

#ifdef XUSB_CDC_DEBUG
    printf("vendor request %x\n", SetupData->bRequest);
#endif
    Status = usb_HandleVendorReq(usb, SetupData);
    break;

  default:
    /* Stall on Endpoint 0 */
#ifdef XUSB_CDC_DEBUG
    printf("unknown class req, stall 0 in out\n");
#endif
    XUsbPs_EpStall(usb, 0, XUSBPS_EP_DIRECTION_IN | XUSBPS_EP_DIRECTION_OUT);
    break;
  }

  return Status;
}

constexpr uint8_t USB_ENDPOINT0_MAXP =
    0x40; // TODO: match against peripheral config
constexpr static usb::chapter9::device_t deviceDesc = {
    .bLength = sizeof(usb::chapter9::device_t),
    .bDescriptorType = USB_TYPE_DEVICE_DESC,
    .bcdUSB = be2les(0x0200),
    .bDeviceClass = USB_CLASS_VENDOR,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = USB_ENDPOINT0_MAXP,
    .idVendor = be2les(0x1209),
    .idProduct = be2les(0x1337),
    .bcdDevice = be2les(0x0010),
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01};
// TODO: correct descriptor, depending on wether we are full speed / high
// speed?
static const usb::chapter9::qualifier_t qualifierDesc = {
    .bLength = sizeof(usb::chapter9::qualifier_t),
    .bDescriptorType = USB_TYPE_DEVICE_QUALIFIER,
    .bcdUSB = be2les(0x0200),
    .bDeviceClass = USB_CLASS_VENDOR,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = USB_ENDPOINT0_MAXP,
    .bNumConfigurations = 1,
    .bReserved = 0};
static const spinaltap::usb::configuration_0_t config = {
    .configuration =
        {
            .bLength = sizeof(usb::chapter9::configuration_t),
            .bDescriptorType = USB_TYPE_CONFIG_DESC,
            .wTotalLength = be2les(sizeof(spinaltap::usb::configuration_0_t)),
            .bNumInterfaces = 0x02,
            .bConfigurationValue = 0x01,
            .iConfiguration = 0x04,
            .bmAttributes = 0xc0, // TODO: ?
            .bMaxPower = 0x00     // TODO: !
        },
    .readerInterface = {.bLength = sizeof(usb::chapter9::interface_t),
                        .bDescriptorType = USB_TYPE_IF_CFG_DESC,
                        .bInterfaceNumber = 0x00,
                        .bAlternateSetting = 0x00,
                        .bNumEndPoints = 0x02,
                        .bInterfaceClass = USB_CLASS_VENDOR,
                        .bInterfaceSubClass = 0x01,
                        .bInterfaceProtocol = 0x00,
                        .iInterface = 0x05},
    .readerInEp = {.bLength = sizeof(usb::chapter9::endpoint_t),
                   .bDescriptorType = USB_TYPE_ENDPOINT_CFG_DESC,
                   .bEndpointAddress = 0x80 | 1,
                   .bmAttributes = USB_EP_BULK, // TODO ?
                   .wMaxPacketSize = be2les(0x200),
                   .bInterval = 0x00},
    .readerOutEp = {.bLength = sizeof(usb::chapter9::endpoint_t),
                    .bDescriptorType = USB_TYPE_ENDPOINT_CFG_DESC,
                    .bEndpointAddress = 2,
                    .bmAttributes = USB_EP_BULK, // TODO ?
                    .wMaxPacketSize = be2les(0x200),
                    .bInterval = 0x00},
    .analyzerInterface = {.bLength = sizeof(usb::chapter9::interface_t),
                          .bDescriptorType = USB_TYPE_IF_CFG_DESC,
                          .bInterfaceNumber = 0x01,
                          .bAlternateSetting = 0x00,
                          .bNumEndPoints = 0x01,
                          .bInterfaceClass = USB_CLASS_VENDOR,
                          .bInterfaceSubClass = 0x01,
                          .bInterfaceProtocol = 0x00,
                          .iInterface = 0x06},
    .analyzerInEp = {.bLength = sizeof(usb::chapter9::endpoint_t),
                     .bDescriptorType = USB_TYPE_ENDPOINT_CFG_DESC,
                     .bEndpointAddress = 0x80 | 3,
                     .bmAttributes = USB_EP_BULK, // TODO ?
                     .wMaxPacketSize = be2les(0x200),
                     .bInterval = 0x00}};
constexpr const std::array<std::string_view, 7> strings = {
    "UNUSED",
    "innovative-solutions",
    "spinalTap",
    "Serial Number...",
    "Default Configuration",
    "Reader Interface",
    "Analyzer Interface"};
constexpr usb::Device<spinaltap::usb::configuration_0_t, 7> device{
    deviceDesc, qualifierDesc, config, strings};

#define USB_REQ_REPLY_LEN 1024
static void usb_HandleStdDevRequest(XUsbPs *usb, XUsbPs_SetupData *setupData) {
  if (setupData->wLength > USB_REQ_REPLY_LEN) {
    return;
  }

  int replyLen;
  int error = 0;
  static uint8_t reply[USB_REQ_REPLY_LEN] ALIGNMENT_CACHELINE;
  switch ((usb::std::std_requests)setupData->bRequest) {
  case usb::std::std_requests::GET_STATUS:
    switch (setupData->bmRequestType & USB_STATUS_MASK) {
    case USB_STATUS_DEVICE: {
      *((uint16_t *)reply) = 0x0100;
      break;
    }
    case USB_STATUS_INTERFACE: {
      *((uint16_t *)reply) = 0x0;
      break;
    }
    case USB_STATUS_ENDPOINT: {
      const int ep = setupData->wIndex;
      const uint32_t status = XUsbPs_ReadReg(usb->Config.BaseAddress,
                                             XUSBPS_EPCRn_OFFSET(ep & 0x0f));
      const uint32_t mask =
          (ep & 0x80) ? XUSBPS_EPCR_TXS_MASK : XUSBPS_EPCR_RXS_MASK;
      *((uint16_t *)reply) = (status & mask) ? 0x0100 : 0x0000;
      break;
    }
    default: {
      break;
    }
    }
    XUsbPs_EpBufferSend(usb, 0, reply, setupData->wLength);
    break;

  case usb::std::std_requests::SET_ADDRESS: {
    XUsbPs_SetDeviceAddress(usb, setupData->wValue);
    XUsbPs_EpBufferSend(usb, 0, 0, 0);
    break;
  }
  case usb::std::std_requests::GET_INTERFACE: {
    reply[0] = usb->CurrentAltSetting;
    XUsbPs_EpBufferSend(usb, 0, reply, 1);
    break;
  }
  case usb::std::std_requests::GET_DESCRIPTOR: {
    xil_printf("reading descriptor\r\n");
    switch ((setupData->wValue >> 8) & 0xff) {
    case USB_TYPE_DEVICE_DESC:
      replyLen = device.setupDeviceDescReply(reply, USB_REQ_REPLY_LEN);
      break;
    case USB_TYPE_DEVICE_QUALIFIER:
      replyLen = device.setupDeviceQualifierReply(reply, USB_REQ_REPLY_LEN);
      break;

    case USB_TYPE_CONFIG_DESC:
      replyLen = device.setupConfigDescriptorReply(reply, USB_REQ_REPLY_LEN);
      break;
    case USB_TYPE_STRING_DESC:
      replyLen = device.setupStringDescReply(reply, USB_REQ_REPLY_LEN,
                                             setupData->wValue & 0xFF);
      break;
    default:
      error = 1;
      break;
    }
    if (error == 0) {
      replyLen = std::min<int>(replyLen, setupData->wLength);
      XUsbPs_EpBufferSend(usb, 0, reply,
                          replyLen); // TODO: handle status
    }
    break;
  }
  case usb::std::std_requests::SET_CONFIGURATION: {
    const int configuration = setupData->wValue & 0xff;
    if (configuration != 0x01) {
      error = 1;
      break;
    }

    usb_SetConfiguration(usb, configuration);
    XUsbPs_EpBufferSend(usb, 0, 0, 0);
    usb_driver_state = STATE_CONNECTED;
    break;
  }
  case usb::std::std_requests::GET_CONFIGURATION: {
    reply[0] = usb->CurrentAltSetting;
    XUsbPs_EpBufferSend(usb, 0, reply, 1);
    break;
  }
  case usb::std::std_requests::CLEAR_FEATURE: {
    switch (setupData->bmRequestType & USB_STATUS_MASK) {
    case USB_STATUS_ENDPOINT:
      if (setupData->wValue == USB_ENDPOINT_HALT) {
        const int ep = setupData->wIndex;
        const int epId = (ep & 0x80) ? ep & 0xf : ep;
        const uint32_t toClear =
            (ep & 0x80) ? XUSBPS_EPCR_TXS_MASK : XUSBPS_EPCR_RXS_MASK;
        XUsbPs_ClrBits(usb, XUSBPS_EPCRn_OFFSET(epId), toClear);
      }
      XUsbPs_EpBufferSend(usb, 0, 0, 0);
      break;
    default:
      error = 1;
      break;
    }
    break;
  }
  case usb::std::std_requests::SET_FEATURE: {
    switch (setupData->bmRequestType & USB_STATUS_MASK) {
    case USB_STATUS_ENDPOINT:
      if (setupData->wValue == USB_ENDPOINT_HALT) {
        const int ep = setupData->wIndex;
        const int epId = (ep & 0x80) ? ep & 0xf : ep;
        const uint32_t toSet =
            (ep & 0x80) ? XUSBPS_EPCR_TXS_MASK : XUSBPS_EPCR_RXS_MASK;
        XUsbPs_SetBits(usb, XUSBPS_EPCRn_OFFSET(epId), toSet);
      }
      XUsbPs_EpBufferSend(usb, 0, 0, 0);
      break;

    case USB_STATUS_DEVICE: {
      if (setupData->wValue == USB_TEST_MODE) {
        const int testSel = (setupData->wIndex >> 8) & 0xff;

        XUsbPs_EpBufferSend(usb, 0, 0, 0);
        usleep(1000);

        switch (testSel) {
        case USB_TEST_J:
        case USB_TEST_K:
        case USB_TEST_SE0_NAK:
        case USB_TEST_PACKET:
        case USB_TEST_FORCE_ENABLE:
          XUsbPs_SetBits(usb, XUSBPS_PORTSCR1_OFFSET, testSel << 16);
          break;
        default:
          break;
        }
      }
      break;
    }
    default: {
      error = 1;
      break;
    }
    }
    break;
  }
  case usb::std::std_requests::SET_INTERFACE: {
    // not supported, so just ACK it
    XUsbPs_EpBufferSend(usb, 0, 0, 0);
    break;
  }
  default: {
    error = 1;
    break;
  }
  }

  if (error)
    XUsbPs_EpStall(usb, 0, XUSBPS_EP_DIRECTION_IN | XUSBPS_EP_DIRECTION_OUT);
}

static void usb_HandleClassIfRequest(XUsbPs *usb, XUsbPs_SetupData *setupData) {
}

static int usb_HandleVendorReq(XUsbPs *usb, XUsbPs_SetupData *setupData) {
  return XST_FAILURE;
}

void usb_SetConfiguration(XUsbPs *usb, int configIdx) {
  if (configIdx != 1)
    return;

  XUsbPs_EpEnable(usb, 1, XUSBPS_EP_DIRECTION_IN);
  XUsbPs_EpEnable(usb, 2, XUSBPS_EP_DIRECTION_OUT);
  XUsbPs_EpEnable(usb, 3, XUSBPS_EP_DIRECTION_IN);

  XUsbPs_SetBits(usb, XUSBPS_EPCR1_OFFSET,
                 XUSBPS_EPCR_TXT_BULK_MASK | XUSBPS_EPCR_RXT_BULK_MASK |
                     XUSBPS_EPCR_TXR_MASK | XUSBPS_EPCR_RXR_MASK);
  XUsbPs_SetBits(usb, XUSBPS_EPCR2_OFFSET,
                 XUSBPS_EPCR_TXT_BULK_MASK | XUSBPS_EPCR_RXT_BULK_MASK |
                     XUSBPS_EPCR_TXR_MASK | XUSBPS_EPCR_RXR_MASK);
  XUsbPs_SetBits(usb, XUSBPS_EPCR3_OFFSET,
                 XUSBPS_EPCR_TXT_BULK_MASK | XUSBPS_EPCR_RXT_BULK_MASK |
                     XUSBPS_EPCR_TXR_MASK | XUSBPS_EPCR_RXR_MASK);

  XUsbPs_EpPrime(usb, 2, XUSBPS_EP_DIRECTION_OUT);
}

#define ENSURE_ALIGNED(x) (((x) + 31) & ~31)

void xusb_spinaltap_irq_handler(void *callbackRef, uint32_t mask) {
  xil_printf("generic spinaltap IRQ: %d", mask);
}

void xusb_spinaltap_ep0_irq_handler(void *callbackRef, uint8_t endpoint,
                                    uint8_t eventType, void *data) {
  Xil_AssertVoid(callbackRef != 0);
  XUsbPs *usb = (XUsbPs *)callbackRef;
  xil_printf("ep%d irq %d\r\n", endpoint, eventType);
  switch (eventType) {
  case XUSBPS_EP_EVENT_SETUP_DATA_RECEIVED: {
    XUsbPs_SetupData setupData;
    int status = XUsbPs_EpGetSetupData(usb, endpoint, &setupData);
    if (status == XST_SUCCESS) {
      (void)xusb_cdc_handle_ch9_setup_packet(usb, &setupData);
    }
    break;
  }

  // There will be RX events for zero-length packets on EP0.
  // Receive them and immediately release them to clear the IRQ.
  case XUSBPS_EP_EVENT_DATA_RX: {
    uint8_t *buffer;
    uint32_t bufferLength;
    uint32_t handle;
    uint32_t status =
        XUsbPs_EpBufferReceive(usb, endpoint, &buffer, &bufferLength, &handle);
    if (status == XST_SUCCESS) {
      XUsbPs_EpBufferRelease(handle);
    }
    break;
  }
  default:
    break;
  }
}

void dump_buffer(u8 *buffer, u32 length) {
  xil_printf("0x%04X: ", (int)buffer);
  while (length--) {
    xil_printf("%02X ", *buffer++);
  }
  xil_printf("\r\n");
}

uint32_t xusb_spinaltap_handle_bulk_request(XUsbPs *usb, uint8_t endpoint,
                                            uint8_t *buffer,
                                            uint32_t bufferLength) {
  xil_printf("%d: ", endpoint);
  dump_buffer(buffer, bufferLength);
  XUsbPs_EpBufferSendWithZLT(usb, 1, buffer, bufferLength);
  return XST_SUCCESS;
}

void xusb_spinaltap_ep1_irq_handler(void *callbackRef, uint8_t endpoint,
                                    uint8_t eventType, void *data) {
  Xil_AssertVoid(callbackRef);
  XUsbPs *usb = (XUsbPs *)callbackRef;
  xil_printf("ep%d irq %d data %08xp\r\n", endpoint, eventType, data);
  switch (eventType) {
  case XUSBPS_EP_EVENT_DATA_TX: {
    // TODO: count number of available descriptors...
    break;
  }
  }
}

void xusb_spinaltap_ep2_irq_handler(void *callbackRef, uint8_t endpoint,
                                    uint8_t eventType, void *data) {
  Xil_AssertVoid(callbackRef);
  XUsbPs *usb = (XUsbPs *)callbackRef;
  xil_printf("ep%d irq %d data %08xp\r\n", endpoint, eventType, data);
  switch (eventType) {
  case XUSBPS_EP_EVENT_DATA_RX: {
    uint8_t *buffer;
    uint32_t bufferLength;
    uint32_t handle;
    int status =
        XUsbPs_EpBufferReceive(usb, endpoint, &buffer, &bufferLength, &handle);
    if (status != XST_SUCCESS)
      return;
    uint32_t invalidateLength = ENSURE_ALIGNED(bufferLength);
    Xil_DCacheInvalidateRange((intptr_t)buffer, invalidateLength);

    xil_printf("pushing %08x %08x", buffer, handle);
    rb.write(buffer, bufferLength);
    XUsbPs_EpBufferRelease(handle);
    break;
  }
  }
}

void xusb_spinaltap_ep3_irq_handler(void *callbackRef, uint8_t endpoint,
                                    uint8_t eventType, void *data) {
  Xil_AssertVoid(callbackRef);
  XUsbPs *usb = (XUsbPs *)callbackRef;
  xil_printf("ep%d irq %d\r\n", endpoint, eventType);
  switch (eventType) {
  case XUSBPS_EP_EVENT_DATA_RX: {
    uint8_t *buffer;
    uint32_t bufferLength;
    uint32_t handle;
    int status =
        XUsbPs_EpBufferReceive(usb, endpoint, &buffer, &bufferLength, &handle);
    if (status != XST_SUCCESS)
      break;
    uint32_t invalidateLength = ENSURE_ALIGNED(bufferLength);
    Xil_DCacheInvalidateRange((intptr_t)buffer, invalidateLength);

    (void)xusb_spinaltap_handle_bulk_request(usb, endpoint, buffer,
                                             bufferLength);
    XUsbPs_EpBufferRelease(handle);
    break;
  }
  }
}
