#pragma once

#include "descriptors.hpp"
#include "helper.hpp"

#include <array>
#include <cstring>
#include <string_view>

namespace usb {

template <typename T, int stringCnt> class Device {
public:
  constexpr Device(
      const chapter9::device_t &deviceDesc,
      const chapter9::qualifier_t &qualifierDesc, const T &configDesc,
      const ::std::array<::std::string_view, stringCnt> &strings) noexcept
      : deviceDesc_(deviceDesc), qualifierDesc_(qualifierDesc),
        configDesc_(configDesc), strings_(strings) {}

  uint32_t setupDeviceDescReply(uint8_t reply[],
                                uint32_t replySize) const noexcept {
    if (replySize < sizeof(usb::chapter9::device_t))
      return 0;
    memcpy(reply, &deviceDesc_, sizeof(deviceDesc_));
    return sizeof(deviceDesc_);
  }

  uint32_t setupDeviceQualifierReply(uint8_t reply[],
                                     uint32_t replySize) const noexcept {
    if (replySize < sizeof(configDesc_))
      return 0;

    memcpy(reply, &configDesc_, sizeof(configDesc_));
    return sizeof(configDesc_);
  }

  uint32_t setupConfigDescriptorReply(uint8_t reply[],
                                      uint32_t replySize) const noexcept {
    if (replySize < sizeof(configDesc_))
      return 0;

    memcpy(reply, &configDesc_, sizeof(configDesc_));
    return sizeof(configDesc_);
  }

  uint32_t setupStringDescReply(uint8_t reply[], uint32_t replySize,
                                uint8_t index) const {
    if (index >= strings_.size())
      return 0;

    const ::std::string_view &string = strings_[index];
    const size_t stringLen = string.size();

    if (replySize < 2 + stringLen * 2)
      return 0;

    usb::chapter9::string_t *pDesc = (usb::chapter9::string_t *)reply;
    pDesc->bDescriptorType = USB_TYPE_STRING_DESC;
    if (index == 0) { // TODO: why?
      pDesc->bLength = 4;
      pDesc->wLANGID[0] = helper::host2le(uint16_t(0x0409));
    } else {
      pDesc->bLength = 2 + stringLen * 2;
      for (unsigned int i = 0; i < stringLen; i++)
        pDesc->wLANGID[i] = helper::host2le(uint16_t(string[i]));
    }

    return pDesc->bLength;
  }
/*
  static const USB_REQ_REPLY_LEN = 1024;
  static void handleStdDevRequest(XUsbPs *usb, XusbPs_SetupData *setupData) {
    static uint8_t reply[USB_REQ_REPLY_LEN] ALIGNMENT_CACHELINE;
    uint8_t error = 0;
    switch (static_cast<usb::std::std::requests>(setupData->bRequest)) {
    case usb::std::std_requests::GET_STATUS:
      uint16_t reply16;
      handleStdGetStatus(setupData->bmRequestType, setupData->wIndex, reply16,
                         usb);
      memcpy(reply, &reply16, sizeof(reply16));
      XUsbPs_EpBufferSend(usb, 0, reply, setupData->wLength);
      break;

    case usb::std::std_requests::SET_ADDRESS:
      XUsbPs_SetDeviceAddress(usb, setupData->wValue);
      XUsbPs_EpBufferSend(usb, 0, 0, 0);
      break;

    case usb::std::std_requests::GET_INTERFACE:
      reply[0] = usb->CurrentAltSetting;
      XUsbPs_EpBufferSend(usb, 0, reply, 1);
      break;

    case usb::std::std_requests::GET_DESCRIPTOR:
      if (handleStdGetDescriptor(reply, setupData->wValue)) {
        replyLen = std::min<int>(replyLen, setupData->wLength);
        XUsbPs_EpBufferSend(usb, 0, reply,
                            replyLen); // TODO: handle status
      }
      break;

    case usb::std::std_requests::SET_CONFIGURATION:
      const int configuration = setupData->wValue & 0xff;
      if (configuration != 0x01) {
        error = 1;
        break;
      }
      usb_SetConfiguration(usb, configuration);
      XUsbPs_EpBufferSend(usb, 0, 0, 0);
      usb_driver_state = STATE_CONNECTED;
      break;

    case usb::std::std_requests::GET_CONFIGURATION:
      reply[0] = usb->CurrentAltSetting;
      XUsbPs_EpBufferSend(usb, 0, reply, 1);
      break;

    case usb::std::std_requests::CLEAR_FEATURE:
      if (handleStdClearFeature(setupData->bmRequestType, setupData->wValue,
                                setupData->wIndex))
        XUsbPs_EpBufferSend(usb, 0, 0, 0);
      else
        error = 1;
      break;

    case usb::std::std_requests::SET_FEATURE:
      error = handleStdSetFeature(setupData->bmRequestType, setupData->wValue,
                                  setupData->wIndex, usb);
      break;

    case usb::std::std_requests::SET_INTERFACE:
      // not supported, so just ACK it
      XUsbPs_EpBufferSend(usb, 0, 0, 0);
      break;

    default:
      error = 1;
    }

    if (error)
      XUsbPs_EpStall(usb, 0, XUSBPS_EP_DIRECTION_IN | XUSBPS_EP_DIRECTION_OUT);
  }

  // TODO: remove usb dependency
  static void handleStdGetStatus(uint8_t requestType, uint16_t index,
                                 uint16_t &reply, XUsbPs *usb) {
    switch (requestType & USB_STATUS_MASK) {
    case USB_STATUS_DEVICE:
      reply = 0x0100;
      break;

    case USB_STATUS_INTERFACE:
      reply = 0x0000;
      break;

    case USB_STATUS_ENDPOINT:
      const int ep = setupData->wIndex;
      const uint32_t XUsbPs_ReadReg(usb->Config.BaseAddress,
                                    XUSBPS_EPCRn_OFFSET(ep & 0x0f));
      const uint32_t mask =
          (ep & 0x80) ? XUSBPS_EPCR_TXS_MASK : XUSBPS_EPCR_RXS_MASK;
      reply = (status & mask) ? 0x0100 : 0x0000;
      break;

    default:
      reply = 0;
      break;
    }
  }

  int handleStdGetDescriptor(uint8_t reply[USB_REQ_REPLY_LEN],
                             uint16_t wValue) {
    switch ((setupData->wValue >> 8) & 0xff) {
    case USB_TYPE_DEVICE_DESC:
      replyLen = device.setupDeviceDescReply(reply, USB_REQ_REPLY_LEN);
      return 0;

    case USB_TYPE_DEVICE_QUALIFIER:
      replyLen = device.setupDeviceQualifierReply(reply, USB_REQ_REPLY_LEN);
      return 0;

    case USB_TYPE_CONFIG_DESC:
      replyLen = device.setupConfigDescriptorReply(reply, USB_REQ_REPLY_LEN);
      return 0;

    case USB_TYPE_STRING_DESC:
      replyLen = device.setupStringDescReply(reply, USB_REQ_REPLY_LEN,
                                             setupData->wValue & 0xFF);
      return 0;

    default:
      return 1;
    }
  }

  int handleStdClearFeature(uint16_t bmRequestType, uint16_t wValue,
                            uint16_t wIndex) {
    switch (bmRequestType & USB_STATUS_MASK) {
    case USB_STATUS_ENDPOINT:
      if (wValue == USB_ENDPOINT_HALT) {
        const int ep = wIndex;
        const int epId = (ep & 0x80) ? ep & 0xf : ep;
        const uint32_t toClear =
            (ep & 0x80) ? XUSBPS_EPCR_TXS_MASK : XUSBPS_EPCR_RXS_MASK;
        XUsbPs_ClrBits(usb, XUSBPS_EPCRn_OFFSET(epId), toClear);
      }
      return 0;

    default:
      return 1;
    }
  }

  int handleStdSetFeature(uint16_t bmRequestType, uint16_t wIndex,
                          uint16_t wValue, XUsbPs *usb) {
    switch (bmRequestType & USB_STATUS_MASK) {
    case USB_STATUS_ENDPOINT:
      if (wValue == USB_ENDPOINT_HALT) {
        const int ep = wIndex;
        const int epId = (ep & 0x80) ? ep & 0xf : ep;
        const uint32_t toSet =
            (ep & 0x80) ? XUSBPS_EPCR_TXS_MASK : XUSBPS_EPCR_RXS_MASK;
        XUsbPs_SetBits(usb, XUSBPS_EPCRn_OFFSET(epId), toSet);
      }
      XUsbPs_EpBufferSend(usb, 0, 0, 0);
      return 0;

    case USB_STATUS_DEVICE:
      if (wValue == USB_TEST_MODE) {
        const int testSel = (wIndex >> 8) & 0xff;

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
      return 0;

    default:
      return 1;
    }
  }
  */

  private:
    const chapter9::device_t &deviceDesc_;
    const chapter9::qualifier_t &qualifierDesc_;
    const T &configDesc_;
    const ::std::array<::std::string_view, stringCnt> &strings_;
  };
  // TODO: CTAD guide for array length
} // namespace usb
