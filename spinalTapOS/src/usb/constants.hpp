#pragma once

namespace usb {
namespace std {

/**
 * \name Request types
 * \see USB spec, Rev 2.0, 9.3 USB Device Requests, Table 9-2 Format of Setup
 * Data, p. 248
 * \{
 */
constexpr uint8_t REQ_TYPE_MASK = 0x60;
enum class requests : uint8_t {
  STD = 0x00,
  CLASS = 0x20,
  INTERFACE = 0x21,
  VENDOR = 0x40
};
/* \} */

/**
 * @name Request Values
 * @{
 */
enum class std_requests : uint8_t {
  GET_STATUS = 0x00,
  CLEAR_FEATURE = 0x01,
  SET_FEATURE = 0x03,
  SET_ADDRESS = 0x05,
  GET_DESCRIPTOR = 0x06,
  SET_DESCRIPTOR = 0x07,
  GET_CONFIGURATION = 0x08,
  SET_CONFIGURATION = 0x09,
  GET_INTERFACE = 0x0a,
  SET_INTERFACE = 0x0b,
  SYNC = 0x0c
};
/* @} */

} // namespace std
} // namespace usb
