// copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "SerialDeviceProvider.h"

namespace Haply
{
    namespace HardwareAPI
    {
        namespace Devices
        {

            /**
             * Utilities to automatically find the serial ports associated with
             * Haply devices.
             *
             * It's worth noting that on Windows serial ports numbered above 9
             * must be prefixed with `\\.\` to be valid. As an example,
             * `"\\\\.\\COM12"` or `R"(\\.\)"` should be used to open the serial
             * port `COM12`.
             */
            struct DeviceDetection
            {
                /**
                 * Automatically detects the Inverse3 device(s) on the computer.
                 *
                 * This function provides a cleaner interface then
                 * AutoDetectInverse3 and the internals have been reworked to be
                 * more stable.
                 *
                 * @return A vector of strings representing the COM ports of the
                 * Inverse3 device(s) connected on the computer. An empty vector
                 * indicate that no Inverse3 devices are currently connected to
                 * the computer.
                 */
                static std::vector<std::string> DetectInverse3s();

                /**
                 * Automatically detects the handle device(s) on the computer.
                 *
                 * This function provides a cleaner interface then
                 * AutoDetectHandle and the internals have been reworked to be
                 * more stable. It is also provided for backwards compatibility
                 * for the old wireless handles.
                 *
                 * @return A vector of strings representing the COM ports of the
                 * handle device(s) connected on the computer. An empty vector
                 * indicate that no handle devices are currently connected to
                 * the computer.
                 */
                static std::vector<std::string> DetectHandles();

                /**
                 * Automatically detects the wired handle device(s) on the
                 * computer. This function should be prefered over
                 * `DetectHandles` because it will skip COM ports that tend to
                 * be very slow to test.
                 *
                 * This function provides a cleaner interface then
                 * AutoDetectHandle and the internals have been reworked to be
                 * more stable. It should also be prefered over DetectHandles
                 * because it will skip COM ports that tend to be very slow to
                 * test.
                 *
                 * @return A vector of strings representing the COM ports of the
                 * wired handle device(s) connected on the computer. An empty
                 * vector indicate that no wired handle devices are currently
                 * connected to the computer.
                 */
                static std::vector<std::string> DetectWiredHandles();

                /**
                 * Automatically detects the wireless handle device(s) on the
                 * computer. This function is provided for backwards
                 * compatibility with the old wireless handles.
                 *
                 * This function provides a cleaner interface then
                 * AutoDetectHandle and the internals have been reworked to be
                 * more stable. It is also provided for backwards compatibility
                 * for the old wireless handles.
                 *
                 * @return A vector of strings representing the COM ports of the
                 * wireless handle device(s) connected on the computer. An empty
                 * vector indicate that no wireless handle devices are currently
                 * connected to the computer.
                 */
                static std::vector<std::string> DetectWirelessHandles();

                /**
                 * Automatically detects the Inverse3 device(s) on the computer.
                 *
                 * @deprecated Deprecated in favour of
                 * [DetectInverse3s](#DetectInverse3s).
                 */
                [[deprecated(
                    "Deprecated in favour of DetectInverse3s.")]] static int
                AutoDetectInverse3(std::string portNames[]);

                /**
                 * Automatically detects the Handle device(s) on the computer.
                 *
                 * @deprecated Deprecated in favour of
                 * [DetectHandles](#DetectHandles).
                 */
                [[deprecated(
                    "Deprecated in favour of DetectHandles.")]] static int
                AutoDetectHandle(std::string portNames[]);

                /**
                 * Test a serial port to determine if it's connected to an
                 * inverse3 device.
                 *
                 * \warning This function is mostly meant for internal use only.
                 *
                 * @param port The serial port to test.
                 *
                 * @return A non-zero value if the device connected to port
                 * is an inverse3 device.
                 */
                static int IsInverse3(const char* port);

                /**
                 * Test a serial port to determine if it's connected to a
                 * handle.
                 *
                 * \warning This function is mostly meant for internal use only.
                 *
                 * @param port The serial port to test.
                 *
                 * @return A non-zero value if the device connected to port is
                 * an inverse3 device.
                 */
                static bool IsHandle(const char* port);

             private:
                static int InverseThread(const char* port);
                static int HandleThread(const char* port);
                static int AutoDetect(std::string portNames[],
                                      std::function<int(const char*)> func,
                                      const wchar_t* portType);
            };
        }  // namespace Devices
    }      // namespace HardwareAPI
}  // namespace Haply
