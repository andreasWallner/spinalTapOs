#pragma once

#include "xscugic.h"
#include "xusbps.h"
#include "xusbps_hw.h"

#include "queue.hpp"

/**
 * @name Feature Selectors
 * @{
 */
#define USB_ENDPOINT_HALT 0x00
#define USB_DEVICE_REMOTE_WAKEUP 0x01
#define USB_TEST_MODE 0x02
/* @} */

/**
 * @name Descriptor Types
 * @{
 */
#define USB_TYPE_DEVICE_DESC 0x01
#define USB_TYPE_CONFIG_DESC 0x02
#define USB_TYPE_STRING_DESC 0x03
#define USB_TYPE_IF_CFG_DESC 0x04
#define USB_TYPE_ENDPOINT_CFG_DESC 0x05
#define USB_TYPE_DEVICE_QUALIFIER 0x06
#define USB_TYPE_HID_DESC 0x21

#define USB_TYPE_REPORT_DESC 0x22
/* @} */

/**
 * @name USB Device States
 * @{
 */
#define USB_DEVICE_ATTACHED 0x00
#define USB_DEVICE_POWERED 0x01
#define USB_DEVICE_DEFAULT 0x02
#define USB_DEVICE_ADDRESSED 0x03
#define USB_DEVICE_CONFIGURED 0x04
#define USB_DEVICE_SUSPENDED 0x05
/* @} */

/**
 * @name Status type
 * @{
 */
#define USB_STATUS_MASK 0x3
#define USB_STATUS_DEVICE 0x0
#define USB_STATUS_INTERFACE 0x1
#define USB_STATUS_ENDPOINT 0x2
/* @} */

/**
 * @name EP Types
 * @{
 */
#define USB_EP_CONTROL 0
#define USB_EP_ISOCHRONOUS 1
#define USB_EP_BULK 2
#define USB_EP_INTERRUPT 3
/* @} */

/**
 * @name Device Classes
 * @{
 */
#define USB_CLASS_CDC 0x02
#define USB_CLASS_HID 0x03
#define USB_CLASS_STORAGE 0x08
#define USB_CLASS_VENDOR 0xFF
/* @} */

/**
 * @name Test Mode Selectors
 * @{
 */
#define USB_TEST_J 0x01
#define USB_TEST_K 0x02
#define USB_TEST_SE0_NAK 0x03
#define USB_TEST_PACKET 0x04
#define USB_TEST_FORCE_ENABLE 0x05
/* @} */

#define ALIGNMENT_CACHELINE __attribute__((aligned(32)))
#define DCACHE_INVALIDATE_SIZE(a) ((a) % 32) ? ((((a) / 32) * 32) + 32) : (a)

int xusbps_spinaltap_init(XUsbPs *usb, uint16_t usbDeviceId, uint16_t usbIrq,
                          uint32_t rxBufSize);
int xusbps_spinaltap_register_interrupt(XScuGic *intc, XUsbPs *usb,
                                        u16 usb_irq);
int xusb_cdc_handle_ch9_setup_packet(XUsbPs *InstancePtr,
                                     XUsbPs_SetupData *SetupData);

extern fixed_ringbuffer<uint8_t, 5 * 1024> rb;
