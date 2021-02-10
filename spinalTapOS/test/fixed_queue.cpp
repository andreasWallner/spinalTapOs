#include "Catch2/catch_test_macros.hpp"
#include "Catch2/generators/catch_generators_all.hpp"
#include "catch2/catch_all.hpp"
//#include "Catch2/generators/"
#include "queue.hpp"

SCENARIO("standard use-cases") {
  GIVEN("empty queue") {
    fixed_ringbuffer<uint8_t, 32> queue;
    REQUIRE(queue.free() == queue.capacity);
    REQUIRE(queue.available() == 0);

    WHEN("pushing single element") {
      const uint8_t value = GENERATE(take(2, random(0, 255)));
      REQUIRE(queue.write(value) == true);

      THEN("counts must be correct") {
        REQUIRE(queue.free() == queue.capacity - 1);
        REQUIRE(queue.available() == 1);
      }
      THEN("reading single element must return correct value") {
        REQUIRE(queue.read() == value);
        REQUIRE(queue.free() == queue.capacity);
        REQUIRE(queue.available() == 0);
      }
      THEN("reading array must return correct value") {
        uint8_t read[2] = {0};
        REQUIRE(queue.read(read, 1) == true);
        REQUIRE(read[0] == value);
        REQUIRE(read[1] == 0);
      }
      THEN("reading reference must return value") {
        uint8_t read;
        REQUIRE(queue.read(read) == true);
        REQUIRE(read == value);
      }
      THEN("reading too much must fail") {
        uint8_t read[2];
        REQUIRE(queue.read(read) == false);
        REQUIRE(queue.available() == 1);
      }
    }

    WHEN("pushing array") {
      const uint8_t value0 = GENERATE(take(2, random(0, 255)));
      const uint8_t value1 = GENERATE(take(2, random(0, 255)));
      const uint8_t value2 = GENERATE(take(2, random(0, 255)));
      const uint8_t value[3] = {value0, value1, value2};

      REQUIRE(queue.write(value) == true);
      THEN("counts must be correct") {
        REQUIRE(queue.free() == queue.capacity - 3);
        REQUIRE(queue.available() == 3);
      }
      THEN("reading elements must return correct values") {
        REQUIRE(queue.read() == value0);
        REQUIRE(queue.free() == queue.capacity - 2);
        REQUIRE(queue.available() == 2);

        REQUIRE(queue.read() == value1);
        REQUIRE(queue.free() == queue.capacity - 1);
        REQUIRE(queue.available() == 1);

        REQUIRE(queue.read() == value2);
        REQUIRE(queue.free() == queue.capacity);
        REQUIRE(queue.available() == 0);
      }
      THEN("reading array[2] must return correct values") {
        uint8_t read[2];
        REQUIRE(queue.read(read) == true);
        REQUIRE(read[0] == value0);
        REQUIRE(read[1] == value1);
        REQUIRE(queue.free() == queue.capacity - 1);
        REQUIRE(queue.available() == 1);
      }
      THEN("reading array[3] must return correct values") {
        uint8_t read[3];
        REQUIRE(queue.read(read) == true);
        REQUIRE(read[0] == value0);
        REQUIRE(read[1] == value1);
        REQUIRE(read[2] == value2);
        REQUIRE(queue.free() == queue.capacity);
        REQUIRE(queue.available() == 0);
      }
      THEN("read too much must return error") {
        uint8_t read[4];
        REQUIRE(queue.read(read) == false);
        REQUIRE(queue.available() == 3);
        REQUIRE(queue.free() == queue.capacity - 3);
      }
    }
  }
  GIVEN("full queue (capacity - 1)") {
    fixed_ringbuffer<uint8_t, 32> queue;
    auto preload32 = GENERATE(take(1, chunk(31, random(0, 255))));
    const std::vector<uint8_t> preload(begin(preload32), end(preload32));
    REQUIRE(queue.write(preload.data(), preload.size()) == true);

    THEN("counts must be correct") {
      REQUIRE(queue.available() == queue.capacity);
      REQUIRE(queue.free() == 0);
    }

    WHEN("pusing last element (as single element") {
      uint8_t value = GENERATE(take(1, random(0, 255)));
      REQUIRE(queue.write(value) == false);

      THEN("counts must be correct") {
        REQUIRE(queue.available() == queue.capacity);
        REQUIRE(queue.free() == 0);
      }

      THEN("same elements must still be in queue") {
        std::vector<uint8_t> read;
        read.resize(queue.capacity);

        REQUIRE(queue.read(read.data(), queue.capacity) == true);
        REQUIRE(read == preload);
      }
    }
  }

  GIVEN("pointers at end of queue") {
    fixed_ringbuffer<uint8_t, 32> queue;
    for (int i = 0; i < queue.capacity; i++) {
      queue.write((uint8_t)i);
      (void)queue.read();
    }

    WHEN("wrapping around") {
      auto value32 = GENERATE(take(1, chunk(15, random(0, 255))));
      const std::vector<uint8_t> value(begin(value32), end(value32));
      REQUIRE(queue.write(value.data(), value.size()) == true);

      THEN("counts must be correct") {
        REQUIRE(queue.available() == value.size());
        REQUIRE(queue.free() == queue.capacity - value.size());
      }

      THEN("read values must be correct") {
        std::vector<uint8_t> read;
        read.resize(15);
        REQUIRE(queue.read(read.data(), read.size()) == true);
        REQUIRE(read == value);
      }

      // todo push too much data
      // todo push remaining space
    }
  }

  GIVEN("wrapped around queue") {
    fixed_ringbuffer<uint8_t, 32> queue;
    std::vector<uint8_t> values;
    for (int i = 0; i < queue.capacity + 5; i++) {
      queue.write((uint8_t)i);
      if (i < 10)
        (void)queue.read();
      else
        values.push_back(i);
    }

    THEN("counts must be correct") {
      REQUIRE(queue.available() == queue.capacity + 5 - 10);
      REQUIRE(queue.free() == 5);
    }

    WHEN("reading data") {
      std::vector<uint8_t> read;
      read.resize(queue.available());
      REQUIRE(queue.read(read.data(), read.size()) == true);
      REQUIRE(read == values);
    }
  }
}

// queue of pointers?