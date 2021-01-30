#pragma once

#include <type_traits>

namespace usb {
namespace helper {

inline constexpr uint16_t host2le(uint16_t val) noexcept { return val; }

inline constexpr uint32_t host2le(uint32_t val) noexcept { return val; }

} // namespace helper
} // namespace usb
