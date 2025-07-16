// Copyright 2022 Haply Robotics Inc. All rights reserved.

#pragma once
#include <cstdint>
#include <cstring>
#include <istream>
#include <vector>

#include "Device.h"

namespace Haply
{
    namespace HardwareAPI
    {
        namespace Devices
        {

            /**
             * Encapsulates the logic required to interact with a Haply handle.
             *
             * @warning Handles are still an area of active development and the
             * API is not stable or guaranteed to be backwards compatible.
             *
             * This class must be specialized to override one of the protected
             * virtual methods which will be invoked when a particular event is
             * received from the device:
             *
             * - [OnReceiveHandleInfo](#OnReceiveHandleInfo): Invoked after
             *   waking up the device.
             *
             * - [OnReceiveHandleStatusMessage](#OnReceiveHandleStatusMessage)
             *   Invoked when a new state update is received from the device.
             *
             * - [OnReceiveHandleErrorResponse](#OnReceiveHandleErrorResponse)
             *   Invoked the device reports an error.
             *
             * An instance of the [SerialStream](#IO::SerialStream) class
             * representing the serial port of the handle is required as a
             * parameter to the constructor. Once constructed, the
             * [SendDeviceWakeup](#SendDeviceWakeup) command will wakeup the
             * handle and allow for [RequestStatus](#RequestStatus) commands to
             * be sent to retrieve the latest device status. The
             * [Receive](#Receive) method should follow any command method to
             * process the device's response and trigger one of the overloaded
             * virtual methods.
             *
             * Handle implementations are customizable using a `user_data` field
             * in the device's status whose length is determined by the
             * `user_len` field. The Haply Quill's user fields are defined as
             * follows:
             *
             * - `user_data[0]`: The state of the button on the flat surface of
             *   the handle where 1 indicate that it's held down.
             *
             * - `user_data[3]`: The battery level where the value 100 means
             *   that the battery is full.
             *
             * All methods within this class will block until their respective
             * IO operations are completed or error out. Handles operate at a
             * frequency of approximately 50Hz. If multiple devices are managed
             * by the same thread, the overall processing frequency will cap at
             * the lowest frequency of all the device. Mixing handles and
             * Inverse3 within the same thread is therefore not recommended.
             */
            struct Handle : public Haply::HardwareAPI::Devices::Device
            {
                /**
                 * The maximum amount of bytes that can be contained
                 * in the user field of any any specialized implementations of
                 * the Haply handle.
                 */
                static constexpr uint8_t USER_DATA_MAX = 255;
                static constexpr uint8_t MAX_EXTENSION_INCOMING_DATA_SIZE = 12;
                static constexpr uint8_t MAX_EXTENSION_OUTGOING_DATA_SIZE = 20;

                /**
                 * Constructs a Handle object from the provided stream.
                 *
                 * @param stream The stream object representing the serial port
                 * associated with the device. The lifetime of the stream must
                 * match or exceed the lifetime of the Handle object. We
                 * recommend constructing this stream using the
                 * [SerialStream](#IO::SerialStream) class.
                 *
                 * @param timeout The maximum amount of time to wait for a
                 * response from the device. If the device fails to respond
                 * within this time, the operation will fail and an error will
                 * be printed to stderr.
                 * @remark The default timeout is 5 seconds,
                 * a negative value will disable the timeout.
                 */
                explicit Handle(Haply::HardwareAPI::IO::SerialStream* stream,
                                float timeout = 5.0f)
                    : Device(stream, timeout)
                {
                }

                /**
                 * Basic device information for the handle.
                 */
                struct HandleInfoResponse
                {
                    /**
                     * Short non-unique device id for the device.
                     *
                     * Device identifier for the handle. This short device is
                     * not guaranteed to be unique and may be overloaded as the
                     * [hardware_version](#HandleInfoResponse::hardware_version)
                     * and the
                     * [device_model_number](#HandleInfoResponse::device_model_number)
                     * changes.
                     */
                    uint16_t device_id{0};

                    /**
                     * @deprecated This field is being repurposed and should not
                     * be used or relied upon.
                     */
                    [[deprecated(
                        "This field is being repurposed and should not be used "
                        "or relied upon.")]] uint8_t handle_data_remaining{0};

                    /**
                     * Used to identify the type of device being used.
                     *
                     * @deprecated This field is being repurposed and should not
                     * be used or relied upon.
                     */
                    [[deprecated(
                        "This field is being repurposed and should not be used "
                        "or relied upon.")]] uint8_t device_model_number{0};

                    /**
                     * Version of the device's hardware.
                     *
                     * @deprecated This field is being repurposed and should not
                     * be used or relied upon.
                     */
                    [[deprecated(
                        "This field is being repurposed and should not be used "
                        "or relied upon.")]] uint8_t hardware_version{0};

                    /**
                     * Version of the device's firmware.
                     *
                     * @deprecated This field is being repurposed and should not
                     * be used or relied upon.
                     */
                    [[deprecated(
                        "This field is being repurposed and should not be used "
                        "or relied upon.")]] uint8_t firmware_version{0};
                };

                /**
                 * The current state of the handle.
                 */
                struct HandleStatusResponse
                {
                    /// @copydoc HandleInfoResponse::device_id
                    uint16_t device_id{0};

                    /**
                     * The orientation of the handle represented as a quaternion
                     * in the WXYZ order.
                     *
                     * The handle rotation is calibrated when it's powered on
                     * and can be reset using the calibration button.
                     */
                    float quaternion[QUATERNION_SIZE]{0, 0, 0, 0};

                    /**
                     * Indicates whether an error was detected.
                     *
                     * @deprecated This field is being repurposed and should not
                     * be used or relied upon.
                     */
                    [[deprecated(
                        "This field is being repurposed and should not be used "
                        "or relied upon.")]] uint8_t error_flag{0};

                    /**
                     * Indicates whether the device is attached to an Inverse3
                     * device.
                     *
                     * The value 1 indicates that the handle is attached to an
                     * Invers3.
                     *
                     * For environments that uses multiple Inverse3
                     * simultaneously, it is currently not possible to determine
                     * programmatically which Inverse3 a given handle is
                     * attached to.
                     */
                    uint8_t handle_connection_sensor{0};

                    /**
                     * The number of bytes that can be read in the
                     * [user_data](#user_data) field.
                     *
                     * This value can't exceed [USER_DATA_MAX](#USER_DATA_MAX).
                     */
                    uint8_t user_data_length{0};

                    /**
                     * User bytes whose representation is specific to the
                     * handle.
                     *
                     * Consult the documentation for your handle to determine
                     * how to interpret the bytes.
                     */
                    uint8_t user_data[USER_DATA_MAX]{};
                };

                /**
                 * Error notification from the device.
                 *
                 * This message type currently not being generated by any
                 * handle implementation and is subject to change.
                 */
                struct HandleErrorResponse
                {
                    /// @copydoc HandleInfoResponse::device_id
                    uint16_t device_id{0};

                    /**
                     * The code of the error being reported.
                     *
                     * There's currently no define error code table as this
                     * message is not currently being generated.
                     */
                    uint8_t error_code{0};
                };

                /**
                 * Wakeup the handle to allow for additional commands.
                 *
                 * This command is required to be sent to the device before any
                 * other commands can be sent.
                 *
                 * Must be followed by a call to [Receive](#Receive) to process
                 * the handle's response which will in turn call the
                 * [OnReceiveHandleInfo](#OnReceiveHandleInfo) overloaded
                 * method.
                 *
                 */
                void SendDeviceWakeup();

                /**
                 * Request the latest state from the handle.
                 *
                 * Must be followed by a call to [Receive](#Receive) to process
                 * the handle's response which will in turn call the
                 * [OnReceiveHandleStatusMessage](#OnReceiveHandleStatusMessage)
                 * overloaded method.
                 */
                void RequestStatus();

                /**
                 * Update the handle-specific user state with the provided
                 * arguments.
                 *
                 * Must be followed by a call to [Receive](#Receive) to process
                 * the handle's response which will in turn call the
                 * [OnReceiveHandleStatusMessage](#OnReceiveHandleStatusMessage)
                 * overloaded method.
                 *
                 * @param device_id The id of the device. This field is
                 * currently ignored by the handle and can be safely set to 0.
                 *
                 * @param user_data_length The number of bytes to read from
                 * user_data. Must be smaller then
                 * [USER_DATA_MAX](#USER_DATA_MAX).
                 *
                 * @param user_data The bytes that will be read and used to
                 * update the handle-specific state. Consult the documentation
                 * of your handle to determine how to format these bytes.
                 */
                void SendHandleState(const uint16_t device_id,
                                     const uint8_t user_data_length,
                                     const uint8_t* user_data);

                /**
                 * Request the latest error information from the handle.
                 *
                 * @note This message type is currently not being processed by
                 * any handle implementation and is subject to change.
                 *
                 * Must be followed by a call to [Receive](#Receive) to process
                 * the handle's response which will in turn call the
                 * [OnReceiveHandleErrorResponse](#OnReceiveHandleErrorResponse)
                 * overloaded method.
                 *
                 * @param device_id The id of the device. This field is
                 * currently be ignored by the handle and can be safely set to
                 * 0.
                 */
                void SendHandleErrorRequest(const uint16_t device_id);

                /**
                 * Receive and processe a response from the handle.
                 *
                 * Must follow any call to one of the following methods and will
                 * invoke the associated overloaded method:
                 *
                 * - [SendDeviceWakeup](#SendDeviceWakeup) ->
                 *   [OnReceiveHandleInfo](#OnReceiveHandleInfo)
                 *
                 * - [RequestStatus](#RequestStatus) ->
                 *   [OnReceiveHandleStatusMessage](#OnReceiveHandleStatusMessage)
                 *
                 * - [SendHandleState](#SendHandleState) ->
                 *   [OnReceiveHandleStatusMessage](#OnReceiveHandleStatusMessage)
                 *
                 * - [SendHandleErrorRequest](#SendHandleErrorRequest) ->
                 *   [OnReceiveHandleErrorResponse](#OnReceiveHandleErrorResponse)
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered.
                 *
                 * @return 0 if the operation completed successfully. If an
                 * error occurred then the method will return a negative value,
                 * an error will be printed to stderr and `stream->good()` may
                 * return false.
                 */
                int Receive();

                /**
                 * VerseGrip handle status response
                 * The VerseGrip Status is not valid when the error_flag is set
                 * to a non zero value.
                 */
                struct VersegripStatusResponse
                {
                    uint8_t device_id{0};
                    float quaternion[QUATERNION_SIZE]{0, 0, 0, 0};
                    uint8_t error_flag{0};
                    uint8_t hall_effect_sensor_level{0};
                    uint8_t buttons{0};
                    float battery_level{0};
                    // x, y, z, w quaternion
                    struct Quaternion
                    {
                        float x{0}, y{0}, z{0}, w{0};
                    } q;

                    uint8_t extension_data[MAX_EXTENSION_INCOMING_DATA_SIZE];
                };

                /**
                 * VerseGrip handle extended data update
                 */
                struct VersegripExtendedDataUpdate
                {
                    uint16_t device_id;
                    uint8_t data[MAX_EXTENSION_OUTGOING_DATA_SIZE];
                };

                /**
                 * @brief Get the Versegrip Status object as a response
                 *
                 * The new VerseGrip is designed to work with a dongle connected
                 * on a usb port and combine the ease of use of the wired handle
                 * to the freedom of the wireless handle.
                 * As we achieved more than 1kHz communication on wireless
                 * communication it’s now possible to use the handle in a
                 * wireless mode in a very simple way. 'GetVersegripStatus' can
                 * simply be called in the standard loop of the application to
                 * get the status of the handle.
                 *
                 * When the VerseGripStatusResponse error_flag i set to non zero
                 * the content of the response is invald and can be discarded.
                 *
                 * @return VersegripStatusResponse
                 */
                VersegripStatusResponse GetVersegripStatus(
                    bool extended = false);

                void SendVersegripExtendedData(
                    VersegripExtendedDataUpdate update);

             protected:
                /**
                 * Invoked by [Receive](#Receive) when processing the handle
                 * response to the [SendDeviceWakeup](#SendDeviceWakeup)
                 * command.
                 *
                 * Overriding either of the overloads for this method is
                 * optional.
                 *
                 * @param response The device's response.
                 */
                virtual void OnReceiveHandleInfo(HandleInfoResponse& response);

                /// @copydoc OnReceiveHandleInfo
                virtual void OnReceiveHandleInfo(uint8_t handle_data_remaining,
                                                 uint16_t device_id,
                                                 uint8_t device_model_number,
                                                 uint8_t hardware_version,
                                                 uint8_t firmware_version);

                /**
                 * Invoked by [Receive](#Receive) when processing the handle
                 * response to the [RequestStatus](#RequestStatus) or the
                 * [SendHandleState](#SendHandleState) command.
                 *
                 * Overriding either of the overloads for this method is
                 * optional. We recommend overriding at least one as there are
                 * no other methods of extracting the handle state.
                 *
                 * @param response The device's response.
                 */
                virtual void OnReceiveHandleStatusMessage(
                    HandleStatusResponse& response);

                /// @copydoc OnReceiveHandleInfo
                virtual void OnReceiveHandleStatusMessage(
                    uint16_t device_id, float* quaternion, uint8_t error_flag,
                    uint8_t hall_effect_sensor_level, uint8_t user_data_length,
                    uint8_t* user_data);

                /**
                 * Invoked by [Receive](#Receive) when processing the handle
                 * response to the
                 * [SendHandleErrorRequest](#SendHandleErrorRequest) command.
                 *
                 * Overriding either of the overloads for this method is
                 * optional. We recommend not overriding this method as this
                 * message is currently not in use by any handle implementation.
                 *
                 * @param response The device's response.
                 */
                virtual void OnReceiveHandleErrorResponse(
                    HandleErrorResponse& response);

                /// @copydoc OnReceiveHandleErrorResponse
                virtual void OnReceiveHandleErrorResponse(uint16_t device_id,
                                                          uint8_t error_code);

             private:
                int Receive(uint8_t* header_code);

                int ReceiveHandleInfo();
                int ReceiveHandleInfo(uint8_t* handle_data_remaining,
                                      uint16_t* device_id,
                                      uint8_t* device_model_number,
                                      uint8_t* hardware_version,
                                      uint8_t* firmware_version);
                int ReceiveHandleStatusMessage();
                int ReceiveHandleStatusMessage(
                    uint16_t* device_id, float* quaternion, uint8_t* error_flag,
                    uint8_t* hall_effect_sensor_level,
                    uint8_t* user_data_length, uint8_t* user_data);
                int ReceiveHandleErrorResponse();
                int ReceiveHandleErrorResponse(uint16_t* device_id,
                                               uint8_t* error_code);
            };
        }  // namespace Devices
    }      // namespace HardwareAPI
}  // namespace Haply
