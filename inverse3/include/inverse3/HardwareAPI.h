// copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once

#include "DeviceDetection.h"
#include "Handle.h"
#include "Inverse3.h"
#include "SerialStream.h"
#include "UUID.h"

//! Top level namespace for the Haply SDK.
namespace Haply
{
    //! Contains the logic required to interact with Haply's hardware.
    namespace HardwareAPI
    {
        //! Namespace that contains device specific classes.
        namespace Devices
        {
        }

        //! Namespace that contains communication specific classes.
        namespace IO
        {
        }

        /**
         * Prints the version of the library to the stdout.
         */
        void PrintLibraryVersion();

        /**
         * Returns the semver version of the HardwareAPI.
         *
         * @return A pointer to a static string which should not be freed.
         */
        const char* GetLibraryVersion();

    }  // namespace HardwareAPI
}  // namespace Haply
