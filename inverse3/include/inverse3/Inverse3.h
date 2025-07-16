// Copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once
#include <cstdint>
#include <cstring>
#include <istream>

#include "Device.h"
#include "UUID.h"

namespace Haply
{
    namespace HardwareAPI
    {
        namespace Devices
        {

            /**
             * Encapsulates the logic required to interact with an Inverse3
             * device.
             *
             * An instance of the [SerialStream](#IO::SerialStream) class
             * representing the serial port of the Inverse3 is required as a
             * parameter to the constructor. Once constructed, the
             * [DeviceWakeup](#DeviceWakeup) command will wakeup the device and
             * allow enable the send methods. All send methods must be followed
             * by their respective receive methods to process the device's
             * response.
             *
             * @remark This class uses the term end-effector to refer to the
             * device's cursor.
             *
             * All methods within this class will block until their IO
             * operations are completed. An Inverse3 device can operate
             * frequencies of up to 8kHz where a recommended minimum of 1kHz is
             * required to ensure a smooth haptic experience. If multiple
             * devices are managed by the same thread, the overall processing
             * frequency will cap at the lowest frequency of all the
             * device. Mixing handles and Inverse3 within the same thread is
             * therefore not recommended.
             */
            struct Inverse3 : public Device
            {
                /**
                 * Number of motors/joints in an inverse3 device.
                 */
                static constexpr uint8_t NUM_JOINTS = 3;

                /**
                 * Constant for a left-handed device to be used in conjunction
                 * with [DeviceHandednessPayload](#DeviceHandednessPayload).
                 */
                static constexpr uint8_t LEFT_HANDED = 0x00;

                /**
                 * Constant for a right-handed device to be used in conjunction
                 * with [DeviceHandednessPayload](#DeviceHandednessPayload).
                 */
                static constexpr uint8_t RIGHT_HANDED = 0x01;

                /**
                 * Constructs an Inverse3 object from the provided stream.
                 *
                 * @param stream The stream object representing the serial port
                 * associated with the device. The lifetime of the stream must
                 * match or exceed the lifetime of the Inverse3 object. We
                 * recommend constructing this stream using the
                 * [SerialStream](#IO::SerialStream) class.
                 *
                 * @param timeout The maximum amount of time
                 * to wait for a response from the device. If the device fails
                 * to respond within this time, the operation will fail and an
                 * error will be printed to stderr.
                 * @remark The default timeout is 5 seconds, a negative value
                 * will disable the timeout.
                 */
                explicit Inverse3(Haply::HardwareAPI::IO::SerialStream* stream,
                                  float timeout = 5.0f);
                /**
                 * Basic device information for the Inverse3.
                 *
                 * Response from the [DeviceWakeup](#DeviceWakeup) method
                 * containing the basic information of the Inverse3 device
                 * represented by this object.
                 */
                struct DeviceInfoResponse
                {
                    /**
                     * Short non-unique device id for the device.
                     *
                     * Device identifier for the Inverse3 device. This short
                     * device is not guaranteed to be unique and may be
                     * overloaded as the [hardware_version](#hardware_version)
                     * and the [device_model_number](#device_model_number)
                     * changes. For a unique identifier use the
                     * [device_id_ext](#device_id_ext) field instead.
                     */
                    uint16_t device_id;

                    /**
                     * Unique UUID identifier for the  device.
                     *
                     * This identifier is guaranteed to unique regardless of the
                     * model number and device type currently being used.
                     */
                    UUID device_id_ext;

                    /**
                     * Used to identify the type of device being used.
                     *
                     * @deprecated This field is being repurposed and should not
                     * be used or relied upon.
                     */
                    [[deprecated(
                        "This field is being repurposed and should not be used "
                        "or relied upon.")]] uint8_t device_model_number;

                    /**
                     * Version of the device's hardware.
                     *
                     * @deprecated This field is being repurposed and should not
                     * be used or relied upon.
                     */
                    [[deprecated(
                        "This field is being repurposed and should not be used "
                        "or relied upon.")]] uint8_t hardware_version;

                    /**
                     * Version of the device's firmware.
                     *
                     * @deprecated This field is being repurposed and should not
                     * be used or relied upon.
                     */
                    [[deprecated(
                        "This field is being repurposed and should not be used "
                        "or relied upon.")]] uint8_t firmware_version;
                };

                /**
                 * Wakeup a device to allow for additional commands.
                 *
                 * This command is required to be sent to the device before any
                 * other commands can be sent. This command also has the
                 * side-effect of resetting the internal state of the device
                 * (i.e. simulation mode).
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error
                 * will be printed to stderr and `stream->good()` may return
                 * false.
                 *
                 * @remark Functionally equivalent to
                 * [SendDeviceWakeup](#SendDeviceWakeup) but provides no error
                 * handling mechanism.
                 *
                 * @return Basic information about the device.
                 */
                DeviceInfoResponse DeviceWakeup();

                /**
                 * @copybrief DeviceWakeup
                 *
                 * This command is required to be sent to the device before any
                 * other commands can be sent. This command also has the
                 * side-effect of resetting the internal state of the device
                 * (i.e. simulation mode). Must be followed by a call to
                 * [ReceiveDeviceInfo](#ReceiveDeviceInfo) to process the
                 * device's response.
                 *
                 * @remark Functionally equivalent to
                 * [DeviceWakeup](#DeviceWakeup) but provides a basic error
                 * handling mechanism.
                 */
                void SendDeviceWakeup();

                /**
                 * Receive the response from
                 * [SendDeviceWakeup](#SendDeviceWakeup) discarding the results.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered.
                 *
                 * @return Returns 0 if the operation completed successfully. If
                 * an error occurred while communicating with the device via the
                 * stream then the method will return a negative value, an error
                 * will be printed to stderr and `stream->good()` may return
                 * false.
                 */
                int ReceiveDeviceInfo();

                /**
                 * Receive the response from
                 * [SendDeviceWakeup](#SendDeviceWakeup) storing the result in
                 * the provided arguments.
                 *
                 * @copydetails ReceiveDeviceInfo
                 *
                 * @see DeviceInfoResponse for a description of the parameters.
                 */
                int ReceiveDeviceInfo(uint16_t* device_id,
                                      uint8_t* device_model_number,
                                      uint8_t* hardware_version,
                                      uint8_t* firmware_version,
                                      UUID* device_id_ext);

                /**
                 *  The device's unique identifier.
                 */
                struct DeviceIdExtResponse
                {
                    /**
                     * Unique UUID identifier for the  device.
                     *
                     * This identifier is guaranteed to unique regardless of the
                     * model number and device type currently being used.
                     */
                    UUID device_id{};
                };

                /**
                 * Query the UUID of the device.
                 *
                 * Returns the same UUID as provided in the response to the
                 * [DeviceWakeup](#DeviceWakeup) method.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendDeviceIdExtQuery](#SendDeviceIdExtQuery) but provides no
                 * error handling mechanism.
                 *
                 * @return The UUID of the device.
                 */
                DeviceIdExtResponse DeviceIdExtQuery();

                /**
                 * @copybrief DeviceIdExtQuery
                 *
                 * Returns the same UUID as provided in the response to the
                 * [DeviceWakeup](#DeviceWakeup) method. Must be followed by a
                 * call to [ReceiveDeviceIdExt](#ReceiveDeviceIdExt) to process
                 * the device's response.
                 *
                 * @remark Functionally equivalent to
                 * [DeviceIdExtQuery](#DeviceIdExtQuery) but provides a basic
                 * error handling mechanism.
                 */
                void SendDeviceIdExtQuery();

                /**
                 * Receive the response from
                 * [SendDeviceIdExtQuery](#SendDeviceIdExtQuery) discarding the
                 * results.
                 *
                 * @copydetails ReceiveDeviceInfo
                 */
                int ReceiveDeviceIdExt();

                /**
                 * Receive the response from
                 * [SendDeviceIdExtQuery](#SendDeviceIdExtQuery) storing the
                 * result in the provided argument.
                 *
                 * @copydetails ReceiveDeviceIdExt
                 *
                 * @see DeviceIdExtResponse for a description of the parameters.
                 */
                int ReceiveDeviceIdExt(UUID* device_id);

                /**
                 * The device's extended firmware version.
                 *
                 * @warning This command is still under review and its behaviour
                 * should not be relied upon.
                 */
                struct FirmwareVersionExtResponse
                {
                    /**
                     * UUID representing the firmware version of the device.
                     *
                     * @warning This field is still under review and its value
                     * should not be relied upon.
                     */
                    UUID firmware_version_id{};
                };

                /**
                 * Query the device for its extended firmware version.
                 *
                 * @warning This command is still under review and its behaviour
                 * should not be relied upon.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendFirmwareVersionExtQuery](#SendFirmwareVersionExtQuery)
                 * but provides no error handling mechanism.
                 *
                 * @return The firmware version of the device.
                 */
                FirmwareVersionExtResponse FirmwareVersionExtQuery();

                /**
                 * @copybrief FirmwareVersionExtQuery
                 *
                 * Must be followed by a call to
                 * [ReceiveFirmwareVersionExt](#ReceiveFirmwareVersionExt) to
                 * process the device's response.
                 *
                 * @remark Functionally equivalent to
                 * [FirmwareVersionExtResponse](#FirmwareVersionExtResponse) but
                 * provides a basic error handling mechanism.
                 */
                void SendFirmwareVersionExtQuery();

                /**
                 * Receive the response from
                 * [SendFirmwareVersionExtQuery](#SendFirmwareVersionExtQuery)
                 * discarding the results.
                 *
                 * @copydetails ReceiveDeviceInfo
                 */
                int ReceiveFirmwareVersionExt();

                /**
                 * Receive the response from
                 * [SendFirmwareVersionExtQuery](#SendFirmwareVersionExtQuery)
                 * storing the result in the provided arguments.
                 *
                 * @copydetails ReceiveFirmwareVersionExt
                 *
                 * @see FirmwareVersionExtResponse for a description the
                 * parameters.
                 */
                int ReceiveFirmwareVersionExt(UUID* firmware_version_id);

                /**
                 * The state of the device's joints.
                 *
                 * Acts as the response to the [JointTorques](#JointTorques) and
                 * [JointAngles](#JointAngles) commands.
                 */
                struct JointStatesResponse
                {
                    /**
                     * Current angle of each joints in degrees.
                     *
                     * @todo joint angle range values.
                     */
                    float angles[NUM_JOINTS]{0, 0, 0};

                    /**
                     * The current angular velocity of each joints in degrees
                     * per second.
                     */
                    float angularVelocities[NUM_JOINTS]{0, 0, 0};
                };

                /**
                 * Request to apply the given torque to the associated device
                 * motors.
                 */
                struct JointTorquesRequest
                {
                    /**
                     * Torque to apply to each of the device's motors.
                     *
                     * Torque values are in newtons millimeters where a value of
                     * at least ~20Nmm must be provided to overcome the internal
                     * friction of the device.
                     */
                    float torques[NUM_JOINTS]{0, 0, 0};
                };

                /**
                 * Apply the provided torque to the device's motors.
                 *
                 * This command will switch the device to the joints simulation
                 * mode and will maintain the provided torque until overridden
                 * by a new torque value or a new simulation mode.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendJointTorques](#SendJointTorques) but provides no error
                 * handling mechanism.
                 *
                 * @param request The torque to apply to the motors.
                 *
                 * @return The current state of the device's joints.
                 */
                JointStatesResponse JointTorques(
                    const JointTorquesRequest& request);

                /**
                 * Halts the generation of torque by the device's motors.
                 */
                void SendJointTorques();

                /**
                 * @copybrief JointTorques
                 *
                 * This command will switch the device to the joints simulation
                 * mode and will maintain the provided torque until overridden
                 * by a new torque value or a new simulation mode.
                 *
                 * Must be followed by a call to
                 * [ReceiveJointStates](#ReceiveJointStates) to process the
                 * device's response.
                 *
                 * @remark Functionally equivalent to
                 * [JointTorques](#JointTorques) but provides a basic
                 * error handling mechanism.
                 */
                void SendJointTorques(const float torques[3]);

                /**
                 * Request to set the device's joint at the provided angles.
                 */
                struct JointAnglesRequest
                {
                    /**
                     * Angle value for each of the device's joint in degrees.
                     */
                    float angles[NUM_JOINTS]{0, 0, 0};
                };

                /**
                 * Set the angle for the device's joints.
                 *
                 * This command will switch the device to the joints simulation
                 * mode and will maintain the provided angles until overridden
                 * by new angles or a new simulation mode.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendJointAngles](#SendJointAngles) but provides no error
                 * handling mechanism.
                 *
                 * @param request The angles to apply to the motors.
                 *
                 * @return The current state of the device's joints.
                 */
                JointStatesResponse JointAngles(
                    const JointAnglesRequest& request);

                /**
                 * Halts the generation of torque by the device's motors.
                 */
                void SendJointAngles();

                /**
                 * @copybrief JointAngles
                 *
                 * This command will switch the device to the joints simulation
                 * mode and will maintain the provided angles until overridden
                 * by new angles or a new simulation mode.
                 *
                 * Must be followed by a call to
                 * [ReceiveJointStates](#ReceiveJointStates) to process the
                 * device's response.
                 *
                 * @remark Functionally equivalent to
                 * [JointAngles](#JointAngles) but provides a basic
                 * error handling mechanism.
                 */
                void SendJointAngles(const float angles[3]);

                /**
                 * Receive the response from
                 * [SendJointTorques](#SendJointTorques) or
                 * [SendJointAngles](#SendJointAngles) discarding the results.
                 *
                 * @copydetails ReceiveDeviceInfo
                 */
                int ReceiveJointStates();

                /**
                 * Receive the response from
                 * [SendJointTorques](#SendJointTorques) or
                 * [SendJointAngles](#SendJointAngles) storing the results in
                 * the provided parameters.
                 *
                 * @copydetails ReceiveJointStates
                 *
                 * @see JointStatesResponse for a description of the parameters.
                 */
                int ReceiveJointStates(float* angles, float* angularVelocities);

                /**
                 * The state of the device's cursor.
                 *
                 * Acts as the response for the
                 * [EndEffectorForce](#EndEffectorForce) and
                 * [EndEffectorPosition](#EndEffectorPosition) commands.
                 */
                struct EndEffectorStateResponse
                {
                    /**
                     * Current position of the device's cursor in meters.
                     */
                    float position[VECTOR_SIZE]{0, 0, 0};

                    /**
                     * Current velocity of the device's cursor in meters per
                     * second.
                     */
                    float velocity[VECTOR_SIZE]{0, 0, 0};
                };

                /**
                 * Request that the provided force values be applied to the
                 * device's cursor.
                 */
                struct EndEffectorForceRequest
                {
                    /**
                     * Force to be applied to the cursor in newtons.
                     */
                    float force[VECTOR_SIZE]{0, 0, 0};
                };

                /**
                 * Apply the provided force to the device's cursor.
                 *
                 * This command will switch the device to the cursor simulation
                 * mode and will maintain the provided force until overridden by
                 * a new force or a new simulation mode.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * x[SendEndEffectorForce](#SendEndEffectorForce) but provides
                 * no error handling mechanism.
                 *
                 * @param request The force to apply to the cursor.
                 * @param onboard Whether to use the onboard kinematics or not.
                 *
                 * @return The current state of the device's cursor.
                 */
                EndEffectorStateResponse EndEffectorForce(
                    const EndEffectorForceRequest& request,
                    bool onboard = true);

                /**
                 * Read position and velocity of the device's cursor.
                 *
                 * This command will switch the device to the cursor simulation
                 * mode and will maintain 0 force until overridden by a new
                 * force or a new simulation mode.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @param onboard Whether to use the onboard kinematics or not.
                 *
                 * @return The current state of the device's cursor.
                 */
                EndEffectorStateResponse GetEndEffectorPosition(
                    bool onboard = true);

                /**
                 * Halts the generation of forces to the device's cursor.
                 */
                void SendEndEffectorForce();

                /**
                 * @copybrief EndEffectorForce
                 *
                 * This command will switch the device to the cursor simulation
                 * mode and will maintain the provided force until overridden by
                 * a new force or a new simulation mode.
                 *
                 * Must be followed by a call to
                 * [ReceiveEndEffectorState](#ReceiveEndEffectorState) to
                 * process the device's response.
                 *
                 * @remark Functionally equivalent to
                 * [EndEffectorForce](#EndEffectorForce) but provides a basic
                 * error handling mechanism.
                 */
                void SendEndEffectorForce(const float force[3]);

                /**
                 * Request to set the device's cursor at the provided position.
                 */
                struct EndEffectorPositionRequest
                {
                    /**
                     * Position of the cursor in meters.
                     */
                    float position[VECTOR_SIZE]{0, 0, 0};
                };

                /**
                 * Set the position of the device's cursor.
                 *
                 * This command will switch the device to the cursor simulation
                 * mode and will maintain the provided position until overridden
                 * by a new position or a new simulation mode.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * x[SendEndEffectorPosition](#SendEndEffectorPosition) but
                 * provides no error handling mechanism.
                 *
                 * @param request The desired position for the cursor.
                 *
                 * @return The current state of the device's cursor.
                 */
                EndEffectorStateResponse EndEffectorPosition(
                    const EndEffectorPositionRequest& request);

                /**
                 * Halts the generation of forces to the device's cursor.
                 */
                void SendEndEffectorPosition();

                /**
                 * @copybrief EndEffectorPosition
                 *
                 * This command will switch the device to the cursor simulation
                 * mode and will maintain the provided position until overridden
                 * by a new position or a new simulation mode.
                 *
                 * Must be followed by a call to
                 * [ReceiveEndEffectorState](#ReceiveEndEffectorState) to
                 * process the device's response.
                 *
                 * @remark Functionally equivalent to
                 * [EndEffectorPosition](#EndEffectorPosition) but provides a
                 * basic error handling mechanism.
                 */
                void SendEndEffectorPosition(const float position[3]);

                /**
                 * Receive the response from
                 * [SendEndEffectorForce](#SendEndEffectorForce) or
                 * [SendEndEffectorPosition](#SendEndEffectorPosition)
                 * discarding the results.
                 *
                 * @copydetails ReceiveDeviceInfo
                 */
                int ReceiveEndEffectorState();

                /**
                 * Receive the response from
                 * [SendEndEffectorForce](#SendEndEffectorForce) or
                 * [SendEndEffectorPosition](#SendEndEffectorPosition) storing
                 * the results in the provided arguments.
                 *
                 * @copydetails ReceiveEndEffectorState
                 *
                 * @see EndEffetorStatesResponse for a description of the
                 * parameters.
                 */
                int ReceiveEndEffectorState(float* position, float* velocity);

                /**
                 * The orientation of the device's body.
                 *
                 * Acts as the response to
                 * [DeviceOrientationQuery](#DeviceOrientationQuery).
                 */
                struct DeviceOrientationResponse
                {
                    /**
                     * Quaternion representing the orientation of the device's
                     * body in WXYZ order.
                     */
                    float quaternion[QUATERNION_SIZE]{0, 0, 0, 0};
                };

                /**
                 * Query the orientation of the device's body.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendDeviceOrientationQuery](#SendDeviceOrientationQuery) but
                 * provides no error handling mechanism.
                 *
                 * @return The orientation of the device's body.
                 */
                DeviceOrientationResponse DeviceOrientationQuery();

                /**
                 * @copybrief DeviceOrientationQuery
                 *
                 * Must be followed by a call to
                 * [ReceiveDeviceOrientation](#ReceiveDeviceOrientation) to
                 * process the device's response.
                 *
                 * @remark Functionally equivalent to
                 * [DeviceOrientationQuery](#DeviceOrientationQuery) but
                 * provides a basic error handling mechanism.
                 */
                void SendDeviceOrientationQuery();

                /**
                 * Receive the response from
                 * [SendDeviceOrientationQuery](#SendDeviceOrientationQuery)
                 * discarding the results.
                 *
                 * @copydetails ReceiveDeviceInfo
                 */
                int ReceiveDeviceOrientation();

                /**
                 * Receive the response from
                 * [SendDeviceOrientationQuery](#SendDeviceOrientationQuery)
                 * storing the result in the provided arguments.
                 *
                 * @copydetails ReceiveDeviceOrientation
                 *
                 * @see DeviceOrientationResponse for a description the
                 * parameters.
                 */
                int ReceiveDeviceOrientation(float quaternion[4]);

                /**
                 * Indicates whether the device is connected to a power supply
                 * or not.
                 *
                 * An unpowered device is not able to apply forces or torque but
                 * can still provide the position and velocity of it's cursor as
                 * well as the angle and angular velocity of it's joints.
                 */
                struct DevicePowerResponse
                {
                    /**
                     * Indicates whether the device is connected to a power
                     * supply.
                     */
                    bool powered{false};
                };

                /**
                 * Query the powered state of the device.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendDevicePowerQuery](#SendDevicePowerQuery) but provides no
                 * error handling mechanism.
                 *
                 * @return The powered state of the device.
                 */
                DevicePowerResponse DevicePowerQuery();

                /**
                 * @copybrief DevicePowerQuery
                 *
                 * Must be followed by a call to
                 * [ReceiveDevicePower](#ReceiveDevicePower) to process the
                 * device's response.
                 *
                 * @remark Functionally equivalent to
                 * [DevicePowerQuery](#DevicePowerQuery) but provides a basic
                 * error handling mechanism.
                 */
                void SendDevicePowerQuery();

                /**
                 * Receive the response from
                 * [SendDevicePowerQuery](#SendDevicePowerQuery) discarding the
                 * results.
                 *
                 * @copydetails ReceiveDeviceInfo
                 */
                int ReceiveDevicePower();

                /**
                 * Receive the response from
                 * [SendDevicePowerQuery](#SendDevicePowerQuery) storing the
                 * result in the provided arguments.
                 *
                 * @copydetails ReceiveDevicePower
                 *
                 * @see DevicePowerResponse for a description the parameters.
                 */
                int ReceiveDevicePower(bool* powered);

                /**
                 * Temperature of the device's body.
                 *
                 * @remark Not all versions of the Inverse3 have a temperature
                 * sensor in which case the returned value is
                 * non-deterministic. There's currently no method to
                 * programmatically determine whether a temperature sensor is
                 * present or not.
                 */
                struct DeviceTemperatureResponse
                {
                    /**
                     * Temperature of the device in celsius.
                     */
                    float temperature{0};
                };

                /**
                 * Query the device's temperature.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred while then an
                 * error will be printed to stderr and `stream->good()` may
                 * return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendDeviceTemperatureQuery](#SendDeviceTemperatureQuery) but
                 * provides no error handling mechanism.
                 *
                 * @return The temperature of the device.
                 */
                DeviceTemperatureResponse DeviceTemperatureQuery();

                /**
                 * @copybrief DeviceTemperatureQuery
                 *
                 * Must be followed by a call to
                 * [ReceiveDeviceTemperature](#ReceiveDeviceTemperature) to
                 * process the device's response.
                 *
                 * @remark Functionally equivalent to
                 * [DeviceTemperatureQuery](#DeviceTemperatureQuery) but
                 * provides a basic error handling mechanism.
                 */
                void SendDeviceTemperatureQuery();

                /**
                 * Receive the response from
                 * [SendDeviceTemperatureQuery](#SendDeviceTemperatureQuery)
                 * discarding the results.
                 *
                 * @copydetails ReceiveDeviceInfo
                 */
                int ReceiveDeviceTemperature();

                /**
                 * Receive the response from
                 * [SendDeviceTemperatureQuery](#SendDeviceTemperatureQuery)
                 * storing the result in the provided arguments.
                 *
                 * @copydetails ReceiveDeviceTemperature
                 *
                 * @see DeviceTemperatureResponse for a description the
                 * parameters.
                 */
                int ReceiveDeviceTemperature(float* temperature);

                /**
                 * The current being applied to the device's motors.
                 */
                struct MotorCurrentsResponse
                {
                    /**
                     * Current being sent to each motors in amps.
                     */
                    float currents[NUM_JOINTS]{0, 0, 0};
                };

                /**
                 * Query the current being applied to the device's motors.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendMotorCurrentsQuery](#SendMotorCurrentsQuery) but
                 * provides no error handling mechanism.
                 *
                 * @return The current for each of the device's motors.
                 */
                MotorCurrentsResponse MotorCurrentsQuery();

                /**
                 * @copybrief MotorCurrentsQuery
                 *
                 * Must be followed by a call to
                 * [ReceiveMotorCurrents](#ReceiveMotorCurrents) to process the
                 * device's response.
                 *
                 * @remark Functionally equivalent to
                 * [MotorCurrentsQuery](#MotorCurrentsQuery) but provides a
                 * basic error handling mechanism.
                 */
                void SendMotorCurrentsQuery();

                /**
                 * Receive the response from
                 * [SendMotorCurrentsQuery](#SendMotorCurrentsQuery) discarding
                 * the results.
                 *
                 * @copydetails ReceiveDeviceInfo
                 */
                int ReceiveMotorCurrents();

                /**
                 * Receive the response from
                 * [SendMotorCurrentsQuery](#SendMotorCurrentsQuery) storing the
                 * result in the provided arguments.
                 *
                 * @copydetails ReceiveMotorCurrents
                 *
                 * @see MotorCurrentsResponse for a description the parameters.
                 */
                int ReceiveMotorCurrents(float* currents);

                /**
                 * Whether a device is right-handed or left-handed.
                 *
                 * @remark xThe handedness of the device affects the frame of
                 * reference of the device.
                 */
                struct DeviceHandednessPayload
                {
                    /**
                     * Whether the device is left-handed or right-handed.
                     *
                     * This value is meant to be compared against the
                     * [LEFT_HANDED](#LEFT_HANDED) and
                     * [RIGHT_HANDED](#RIGHT_HANDED) constants.
                     */
                    uint8_t handedness{0};
                };

                /**
                 * Query the device's handedness.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendGetDeviceHandedness](#SendGetDeviceHandedness) but
                 * provides no error handling mechanism.
                 *
                 * @return The handedness of the device.
                 */
                DeviceHandednessPayload GetDeviceHandedness();

                /**
                 * @copybrief GetDeviceHandedness
                 *
                 * Must be followed by a call to
                 * [ReceiveDeviceHandedness](#ReceiveDeviceHandedness) to
                 * process the device's response.
                 *
                 * @remark Functionally equivalent to
                 * [GetDeviceHandedness](#GetDeviceHandedness) but provides a
                 * basic error handling mechanism.
                 */
                void SendGetDeviceHandedness();

                /**
                 * Set the device's handedness.
                 *
                 * Changing the handedness will cause a device to change it's
                 * frame of reference and reset its internal state. This include
                 * the device's calibration, torque/angle/force/position values
                 * and simulation mode. Any changes to the handedness will also
                 * necessitate a physical change to the device's arms. Consult
                 * the Haply documentation hub to find a guide on how to effect
                 * this change.
                 *
                 * The change made by this method will only persist until the
                 * next power-cycle of the device. [SaveConfig](#SaveConfig) can
                 * be used to persist the change permanently.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendSetDeviceHandedness](#SendSetDeviceHandedness) but
                 * provides no error handling mechanism.
                 *
                 * @return The handedness of the device.
                 */
                DeviceHandednessPayload SetDeviceHandedness(
                    const DeviceHandednessPayload& payload);

                /**
                 * @copybrief SetDeviceHandedness
                 *
                 * Changing the handedness will cause a device to change it's
                 * frame of reference and reset its internal state. This include
                 * the device's calibration, torque/angle/force/position values
                 * and simulation mode. Any changes to the handedness will also
                 * necessitate a physical change to the device's arms. Consult
                 * the Haply documentation hub to find a guide on how to effect
                 * this change.
                 *
                 * The change made by this method will only persist until the
                 * next power-cycle of the device. [SaveConfig](#SaveConfig) can
                 * be used to persist the change permanently.
                 *
                 * The change made by this method will only persist until the
                 * next power-cycle of the device. [SaveConfig](#SaveConfig) can
                 * be used to persist the change permanently.
                 *
                 * Must be followed by a call to
                 * [ReceiveDeviceHandedness](#ReceiveDeviceHandedness) to
                 * process the device's response.
                 *
                 * @remark Functionally equivalent to
                 * [SetDeviceHandedness](#SetDeviceHandedness) but provides a
                 * basic error handling mechanism.
                 */
                void SendSetDeviceHandedness(const uint8_t& handedness);

                /**
                 * Receive the response from
                 * [SendGetDeviceHandedness](#SendGetDeviceHandedness) or
                 * [SendSetDeviceHandedness](#SendSetDeviceHandedness)
                 * discarding the results.
                 *
                 * @copydetails ReceiveDeviceInfo
                 */
                int ReceiveDeviceHandedness();

                /**
                 * Receive the response from
                 * [SendGetDeviceHandedness](#SendGetDeviceHandedness) or
                 * [SendSetDeviceHandedness](#SendSetDeviceHandedness) storing
                 * the result in the provided arguments.
                 *
                 * @copydetails ReceiveDeviceHandedness
                 *
                 * @see DeviceHandednessResponse for a description the
                 * parameters.
                 */
                int ReceiveDeviceHandedness(uint8_t* handedness);

                /**
                 * Whether torque scaling is enabled in the device or not.
                 */
                struct TorqueScalingPayload
                {
                    /**
                     * Whether torque scaling is enabled or not.
                     */
                    bool enabled{false};
                };

                /**
                 * Query the device's torque scaling state.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendGetTorqueScaling](#SendGetTorqueScaling) but provides no
                 * error handling mechanism.
                 *
                 * @return The torque scaling state of the device.
                 */
                TorqueScalingPayload GetTorqueScaling();

                /**
                 * @copybrief GetTorqueScaling
                 *
                 * Must be followed by a call to
                 * [ReceiveTorqueScaling](#ReceiveTorqueScaling) to process the
                 * device's response.
                 *
                 * @remark Functionally equivalent to
                 * [GetTorqueScaling](#GetTorqueScaling) but provides a basic
                 * error handling mechanism.
                 */
                void SendGetTorqueScaling();

                /**
                 * Set the device's torque scaling state to the provided value.
                 *
                 * The change made by this method will only persist until the
                 * next power-cycle of the device. [SaveConfig](#SaveConfig) can
                 * be used to persist the change permanently.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendGetTorqueScaling](#SendGetTorqueScaling) but provides no
                 * error handling mechanism.
                 *
                 * @return The torque scaling state of the device.
                 */
                TorqueScalingPayload SetTorqueScaling(
                    const TorqueScalingPayload& payload);

                /**
                 * @copybrief SetTorqueScaling
                 *
                 * The change made by this method will only persist until the
                 * next power-cycle of the device. [SaveConfig](#SaveConfig) can
                 * be used to persist the change permanently.
                 *
                 * Must be followed by a call to
                 * [ReceiveTorqueScaling](#ReceiveTorqueScaling) to process the
                 * device's response.
                 *
                 * @remark Functionally equivalent to
                 * [SetTorqueScaling](#SetTorqueScaling) but provides a basic
                 * error handling mechanism.
                 */
                void SendSetTorqueScaling(const bool& enabled);

                /**
                 * Receive the response from
                 * [SendGetTorqueScaling](#SendGetTorqueScaling) or
                 * [SendSetTorqueScaling](#SendSetTorqueScaling)
                 * discarding the results.
                 *
                 * @copydetails ReceiveDeviceInfo
                 */
                int ReceiveTorqueScaling();

                /**
                 * Receive the response from
                 * [SendGetTorqueScaling](#SendGetTorqueScaling) or
                 * [SendSetTorqueScaling](#SendSetTorqueScaling)
                 * storing the result in the provided arguments.
                 *
                 * @copydetails ReceiveTorqueScaling
                 *
                 * @see TorqueScalingPayload for a description the parameters.
                 */
                int ReceiveTorqueScaling(bool* enabled);

                /**
                 * Whether gravity compensation is enabled in the device or not.
                 */
                struct GravityCompensationPayload
                {
                    /**
                     * Indicates whether gravity compensation is enabled.
                     */
                    bool enabled{false};

                    /**
                     * The scaling factor applied to the gravity compensation
                     * calculations.
                     *
                     * Defaults to 0.75.
                     */
                    float gravity_scale_factor{0};
                };

                /**
                 * Query the device's gravity compensation state.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendGetGravityCompensation](#SendGetGravityCompensation) but
                 * provides no error handling mechanism.
                 *
                 * @return The device's gravity compensation state.
                 */
                GravityCompensationPayload GetGravityCompensation();

                /**
                 * @copybrief GetGravityCompensation
                 *
                 * Must be followed by a call to
                 * [ReceiveGravityCompensation](#ReceiveGravityCompensation) to
                 * process the device's response.
                 *
                 * @remark Functionally equivalent to
                 * [GetGravityCompensation](#GetGravityCompensation) but
                 * provides a basic error handling mechanism.
                 */
                void SendGetGravityCompensation();

                /**
                 * Set the device's gravity compensation state to the provided
                 * value.
                 *
                 * The change made by this method will only persist until the
                 * next power-cycle of the device. [SaveConfig](#SaveConfig) can
                 * be used to persist the change permanently.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendSetGravityCompensation](#SendSetGravityCompensation) but
                 * provides no error handling mechanism.
                 *
                 * @return The gravity compensation state of the device.
                 */
                GravityCompensationPayload SetGravityCompensation(
                    const GravityCompensationPayload& payload);

                /**
                 * @copybrief SetGravityCompensation
                 *
                 * The change made by this method will only persist until the
                 * next power-cycle of the device. [SaveConfig](#SaveConfig) can
                 * be used to persist the change permanently.
                 *
                 * Must be followed by a call to
                 * [ReceiveGravityCompensation](#ReceiveGravityCompensation) to
                 * process the device's response.
                 *
                 * @remark Functionally equivalent to
                 * [GetGravityCompensation](#SetGravityCompensation) but
                 * provides a basic error handling mechanism.
                 */
                void SendSetGravityCompensation(
                    const bool& enabled, const float& gravity_scale_factor);

                /**
                 * Receive the response from
                 * [SendGetGravityCompensation](#SendGetGravityCompensation) or
                 * [SendSetGravityCompensation](#SendSetGravityCompensation)
                 * discarding the results.
                 *
                 * @copydetails ReceiveDeviceInfo
                 */
                int ReceiveGravityCompensation();

                /**
                 * Receive the response from
                 * [SendGetGravityCompensation](#SendGetGravityCompensation) or
                 * [SendSetGravityCompensation](#SendSetGravityCompensation)
                 * storing the result in the provided arguments.
                 *
                 * @copydetails ReceiveGravityCompensation
                 *
                 * @see GravityCompensationPayload for a description the
                 * parameters.
                 */
                int ReceiveGravityCompensation(bool* enabled,
                                               float* gravity_scale_factor);

                /**
                 * The result of persisting the device's configuration state.
                 */
                struct SaveConfigResponse
                {
                    /**
                     * Numbers of configuration parameters that were persisted.
                     *
                     * If the value is zero then no configuration parameters
                     * were persisted.
                     */
                    uint8_t parameters_updated{0};
                };

                /**
                 * Permanently persists any modified configuration parameter.
                 *
                 * The device's persistent storage has a write limit which is
                 * consumed with every invocation of this method. As a result,
                 * this method should only be used seldomly.
                 *
                 * Will block until a message is received from the device or an
                 * error is encountered. If an error occurred then an error will
                 * be printed to stderr and `stream->good()` may return false.
                 *
                 * @remark Functionally equivalent to
                 * [SendSaveConfig](#SendSaveConfig) but provides no error
                 * handling mechanism.
                 *
                 * @return The result of the command.
                 */
                SaveConfigResponse SaveConfig();

                /**
                 * @copybrief SaveConfig
                 *
                 * The device's persistent storage has a write limit which is
                 * consumed with every invocation of this method. As a result,
                 * this method should only be used seldomly.
                 *
                 * Must be followed by a call to
                 * [ReceiveSaveConfig](#ReceiveSaveConfig) to process the
                 * device's response.
                 *
                 * @remark Functionally equivalent to [SaveConfig](#SaveConfig)
                 * but provides a basic error handling mechanism.
                 */
                void SendSaveConfig();

                /**
                 * Receive the response from [SendSaveConfig](#SendSaveConfig)
                 * storing the results in the provided argument.
                 *
                 * @copydetails ReceiveDeviceInfo
                 *
                 * @see SaveConfigResponse for a description of the parameters.
                 */
                int ReceiveSaveConfig(uint8_t* parameters_updated);

                void AdjustAngles(float* angles);
                void AdjustLengths(float a1, float a2, float b1, float b2,
                                   float c3);
                void AdjustGearRatios(float head, float inner_arm,
                                      float outer_arm);
                void ResetOrientation();
            };
        }  // namespace Devices
    }      // namespace HardwareAPI
}  // namespace Haply
