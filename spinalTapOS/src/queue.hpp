#pragma once

#include <array>

template <typename T> constexpr bool is_two_to_n(T x) {
  return (x != 0) && ((x & (x - 1)) == 0);
}

// TODO check against type trait for primitive type (such that memcpy is ok)
template <typename T, int size> class fixed_ringbuffer {
public:
  using value_type = T;
  static constexpr size_t capacity = size - 1;
  // TODO typical typedefs
  // static_assert(is_two_to_n(size), "ringbuffer size must be 2**n");

  fixed_ringbuffer() : read_(0), write_(0) {}
  fixed_ringbuffer(const fixed_ringbuffer &) = delete;
  fixed_ringbuffer(fixed_ringbuffer &&) = delete; // allow?
  fixed_ringbuffer &operator=(const fixed_ringbuffer &) = delete;
  fixed_ringbuffer &operator=(fixed_ringbuffer &&) = delete;

  // TODO return value?
  bool read(T data[], uint32_t len) noexcept {
    if (available() < len)
      return false;

    const uint32_t read = read_;
    if (read + len < size) {
      memcpy(data, buffer_ + read, len * sizeof(T));
      read_ = (read + len) < size ? (read + len) : 1;
    } else {
      const uint32_t trailing = size - read;
      const uint32_t leading = len - trailing;
      memcpy(data, buffer_ + read, trailing * sizeof(T));
      memcpy(data + trailing, buffer_, leading * sizeof(T));
      read_ = leading;
    }
    return true;
  }

  template <std::size_t S> bool read(T (&data)[S]) noexcept {
    return read(data, S);
  }

  bool read(T &data) noexcept { return read(&data, 1); }
  T read() noexcept {
    T val;
    read(&val, 1);
    return val;
  }

  bool write(const T data[], uint32_t len) noexcept {
    if (free() < len)
      return false;

    const uint32_t write = write_;
    if (write + len < size) {
      memcpy(buffer_ + write, data, len * sizeof(T));
      write_ = (write + len) < size ? (write + len) : 0;
    } else {
      const uint32_t trailing = size - write;
      const uint32_t leading = len - trailing;
      memcpy(buffer_ + write, data, trailing * sizeof(T));
      memcpy(buffer_, data + trailing, leading * sizeof(T));
      write_ = leading;
    }

    return true;
  }

  template <std::size_t S> bool write(const T (&data)[S]) noexcept {
    return write(data, S);
  }

  bool
  write(const T data) noexcept { // TODO better version, + one for rvalue ref
    return write(&data, 1);
  }

  size_t free() const noexcept {
    if (read_ > write_)
      return read_ - write_ - 1;
    else
      return read_ + (capacity - write_);
  }
  size_t available() const noexcept {
    if (write_ >= read_)
      return write_ - read_;
    else
      return write_ + (size - read_);
  }

private:
  int read_; // std::atomic?
  int write_;
  T buffer_[size]; // use storage for uninitialized type
};
