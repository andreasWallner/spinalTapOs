#pragma once

#include "usb/descriptors.hpp"

namespace spinaltap {
namespace usb {

typedef struct {
  ::usb::chapter9::configuration_t configuration;
  ::usb::chapter9::interface_t readerInterface;
  ::usb::chapter9::endpoint_t readerInEp;
  ::usb::chapter9::endpoint_t readerOutEp;
  ::usb::chapter9::interface_t analyzerInterface;
  ::usb::chapter9::endpoint_t analyzerInEp;
} __attribute__((__packed__)) configuration_0_t;

} // namespace usb
} // namespace spinaltap
