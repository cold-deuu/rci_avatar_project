// Copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once

#include <cassert>
#include <iomanip>
#include <iostream>
#include <istream>
#include <string>

struct serialib;

namespace Haply
{
    namespace HardwareAPI
    {
        namespace IO
        {

            /**
             * Provides an std::iostream compatible stream over for a serial
             * port.
             *
             * Meant to be used in conjunction with the
             * [Inverse3](#Devices::Inverse3) and the [Handle](#Devices::Handle)
             * classes. The API provides the
             * [DeviceDetection](#Devices::DeviceDetection) class to easily find
             * the serial ports that are currently associated with a Haply
             * device.
             *
             * [OpenDevice](#OpenDevice) or passing true to the `open` parameter
             * of [SerialStream](#SerialStream) is required before the object
             * can be used with one of the device classes.
             */
            struct SerialStream : public std::iostream
            {
                /**
                 * Constructs a SerialStream object from the given serial port.
                 *
                 * @warning If open is passed as `true` and an error is raised
                 * while opening the port, an error will be written to stderr
                 * and the resulting object will not be valid.
                 *
                 * @param port The serial port to open. On Windows serial ports
                 * above 9 must be prefixed with `\\.\` to be valid. As an
                 * example, `"\\\\.\\COM12"` or `R"(\\.\COM12)"` should be used
                 * to open the serial port `COM12`.
                 *
                 * @param open If true, [Opendevice](#OpenDevice) will be called
                 * in the constructor.
                 */
                explicit SerialStream(const char* port, bool open = true);

                /**
                 * Closes the serial port and frees any associated resources.
                 */
                virtual ~SerialStream();

                /**
                 * Opens the serial port making it usable by one of the device
                 * classes.
                 *
                 * @returns Returns the value zero if the port was successfully
                 * opened. Returns a negative value if the serial port could not
                 * be opened and an error will be printed to stderr.
                 */
                char OpenDevice();

                /**
                 * This function closes the serial port.
                 *
                 * Called by the destructor and can be safely ignored in most
                 * circumstances.
                 */
                virtual void CloseDevice();

                /**
                 * This function returns the number of bytes available to read
                 * from the serial port.
                 *
                 * @returns The number of bytes available to read from the
                 * serial port.
                 */
                virtual int Available();

                /**
                 * This function writes the given data to the serial port.
                 *
                 * @param data The data to write to the serial port.
                 * @param size The size of the data to write to the serial port.
                 */
                virtual void WriteBytes(const char* data, size_t size);

             private:
                const char* address;
                serialib* serialbuf{nullptr};
                bool open;

                const char* newAddress;
                std::string buffer;
            };
        }  // namespace IO
    }      // namespace HardwareAPI
}  // namespace Haply
