// Copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once
#include <cstdint>
#include <istream>
#include <vector>

namespace Haply
{
    namespace HardwareAPI
    {

        /** Simple representation of a 16 byte UUID. */
        struct UUID
        {
            //! Number of bytes in the UUID.
            static constexpr size_t SIZE = 16;

            //! Byte container for the the UUID.
            uint8_t bytes[SIZE]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

            /**
             * Copies the values of b into the current instance of the UUID.
             *
             * @param b a const byte array that must be at least 16-bytes wide
             * where any excess bytes will be ignored. An array that less then
             * 16-bytes will lead to undefined behaviour.
             */
            void SetBytes(const uint8_t* b);

            /**
             * Prints the contents of the byte array to a given output stream
             *
             * The output will be formatted as:
             * `xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxx`
             *
             * @param os Output stream that will be written to.
             */
            void Print(std::ostream& os);
        };
    }  // namespace HardwareAPI
}  // namespace Haply
