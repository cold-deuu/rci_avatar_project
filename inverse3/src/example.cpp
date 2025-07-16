#include <string.h>


#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>


#include "inverse3/HardwareAPI.h"


// Used to reduce verbosity within the examples.
namespace API = Haply::HardwareAPI;
using namespace std::chrono_literals;

// Vector types used throughout the example.
typedef std::array<float, 3> vec3;
typedef std::array<float, 4> vec4;

// Implementation of the Quill handle for our example where, unlike our previous
// examples, we'll make use of a thread to update the state of our handle.
//
// The use of a thread is necessary because handles operate at a much lower
// frequency (~50Hz) then an Inverse3 device (>1000Hz) where the synchronous
// nature of the API would force the main thread to operate at the frequency of
// the slowest device, in this case 50Hz. The thread allows the handle to be
// polled at 50Hz without affecting the polling rate of the main loop.
//
// The downside of using threads is that it does require the use of
// synchronization primitives to correctly communicate the state information
// between the handle thread and the main loop. In this example we'll use a
// simple mutex which, in our experience, has been performant enough for simple
// use cases.
struct QuillHandle : public API::Devices::Handle {

    // Our constructor from prior examples has been augmented to also start our
    // polling thread which will execute the `run` function.
    QuillHandle(API::IO::SerialStream *stream)
        : Handle(stream), thread(&QuillHandle::run, this) {}

    // We've also added a destructor to properly cleanup and terminate the
    // thread.
    ~QuillHandle() {
        exit.store(true);
        thread.join();
    }

    // This function is used to safely read the latest cached state received
    // from the handle. We make use of our mutex to avoid data-races when
    // reading the data.
    void read(vec4 &quaternion, bool &button) {
        std::lock_guard<std::mutex> guard(mutex);
        quaternion = this->quaternion;
        button = this->button;
    }

  protected:
    // Each state update received will be cached within the object while holding
    // the mutex to avoid data-races with the `read` function used by the main
    // thread.
    void OnReceiveHandleStatusMessage(uint16_t, float *quaternion, uint8_t,
                                      uint8_t, uint8_t,
                                      uint8_t *user) override {

        std::lock_guard<std::mutex> lock{mutex};

        button = user[0];
        for (size_t i = 0; i < 4; ++i)
            this->quaternion[i] = quaternion[i];
    }

  private:
    // Implementation of the polling loop for our handle which will run in a
    // background thread. See previous examples for a more detailed explanations
    // of the steps.
    void run() {
        SendDeviceWakeup();
        (void)Receive();

        typedef std::chrono::high_resolution_clock clock;
        auto next = clock::now();
        auto delay = 20ms; // 50 Hz

        while (!exit.load()) {
            RequestStatus();
            (void)Receive();

            next += delay;
            while (next > clock::now())
                ;
        }
    }

    // Synchronization primitive used to prevent data-races between the handle
    // thread and the main thread.
    std::mutex mutex{};

    // These variables represent the cached state of the handle which must only
    // be accessed while holding a lock on `mutex` to prevent data-races.
    bool button;
    vec4 quaternion{0, 0, 0, 0};

    // State required to run and control the thread.
    std::atomic<bool> exit{false};
    std::thread thread;
};

// Basic quaternion conjugate function which we will use to calibrate our
// handle.
void quaternion_conjugate(vec4 &q) {
    q[1] = -q[1];
    q[2] = -q[2];
    q[3] = -q[3];
}

// Basic quaternion multiplication function which we will use to calibrate our
// handle.
vec4 quaternion_multiply(const vec4 &q, const vec4 &m) {
    // Worth remembering that handle quaternions are in RIJK order.
    static constexpr size_t r = 0, i = 1, j = 2, k = 3;

    vec4 tmp;
    tmp[r] = q[r] * m[r] - q[i] * m[i] - q[j] * m[j] - q[k] * m[k];
    tmp[i] = q[r] * m[i] + q[i] * m[r] + q[j] * m[k] - q[k] * m[j];
    tmp[j] = q[r] * m[j] - q[i] * m[k] + q[j] * m[r] + q[k] * m[i];
    tmp[k] = q[r] * m[k] + q[i] * m[j] - q[j] * m[i] + q[k] * m[r];
    return tmp;
}

// This functions computes the calibration quaternion that will be used to
// translate the raw quaternion of the handle into coordinate system of the
// inverse3 device. This quaternion is computed using the following formula:
//
//     Qc = Q' * Qk * Qr
//
// Where:
//
// - Q': The conjugate of the quaternion obtained during the calibration
//   process.
//
// - Qk: The known quaternion that represents the handle flat on a surface
//   parallel to the Y+ axis of the Inverse3 device. This quaternion can be
//   adjusted to represent alternative calibration position but will require
//   that the Qr quaternion be adjusted accordingly.
//
// - Qr: The rotation quaternion that will translate the coordinate system of
//   the handle into the coordinate system of the Inverse3 device. This constant
//   assumes the calibration position specified in Qk and that the device is
//   positioned upright (not tilted in anyway).
//
// The coordinate system of the handle is as follows assuming that the handle is
// flat on a table with the face button pointing upwards.
//
// - X: Runs along the body of the handle where the gimbal represents positive X
//   and the USB port represents negative X.
//
// - Y: Perpendicular to the body of the handle along the table's surface where
//   the right side of the handle represents positive Y and the left side
//   represents negative Y.
//
// - Z: The last remaining direction where into the table represents positive Z
//   and away from the table represents negative Z.
void quaternion_calibrate(vec4 &q) {
    quaternion_conjugate(q);

    vec4 qk{1, 0, 0, 0};
    q = quaternion_multiply(qk, q);

    vec4 qr{0, 0.7071068f, 0.7071068f, 0};
    q = quaternion_multiply(qr, q);
}

// This function adjusts the current handle quaternion with the calibration
// quaternion calculated by quaternion_calibrate using the following formula:
//
//     Qf = Qc * Q
//
// Where:
//
// - Qc: The calibration quaternion obtained from quaternion_calibrate.
//
// - Q: The quaternion to adjust
void quaternion_offset(vec4 &q, const vec4 &qc) {
    q = quaternion_multiply(qc, q);
}

// Converts a quaternion into a direction vector representing where the handle
// is pointing using the following formula:
//
//    R(Q) * [1; 0; 0]
//
// Where we're converting the input quaternion into its rotation matrix and
// extracting its first column to get the direction vector that represents where
// the handle is pointing.
void quaternion_to_direction(const vec4 &q, vec3 &dir) {
    float s = 0.0;
    for (size_t i = 0; i < 4; ++i)
        s += q[i] * q[i];
    s = 1 / s;

    // Worth remembering that handle quaternions are in RIJK order.
    static constexpr size_t r = 0, i = 1, j = 2, k = 3;
    dir[0] = 1 - 2 * s * (q[j] * q[j] + q[k] * q[k]);
    dir[1] = 2 * s * (q[i] * q[j] + q[k] * q[r]);
    dir[2] = 2 * s * (q[i] * q[k] - q[j] * q[r]);
}

int main(int argc, const char *argv[]) {
    if (argc < 3) {
        std::fprintf(
            stderr,
            "usage: 04-combined [inverse3-com-port] [handle-com-port]\n");
        return 1;
    }

    // Basic inverse3 device setup. See earlier examples for a more detailed
    // explanation.
    API::IO::SerialStream inverse3_stream{argv[1]};
    API::Devices::Inverse3 inverse3{&inverse3_stream};
    (void)inverse3.DeviceWakeup();

    // Basic inverse3 handle setup which will also start our background polling
    // thread. See earlier examples for a more detailed explanation.
    API::IO::SerialStream quill_stream{argv[2]};
    QuillHandle quill{&quill_stream};

    // Beginning of our calibration process with the specific calibration
    // position expected by our `quaternion_offset` function.
    fprintf(stdout, "Position the handle flat on a surface parallel to the "
                    "Inverse3's Y+ axis.\n"
                    "Press ENTER when ready to calibrate...");
    while (std::getc(stdin) != '\n')
        ;

    // Read the state of our handle to compute our calibration quaternion.
    bool button = false;
    vec4 calibration{0, 0, 0, 0};
    quill.read(calibration, button);
    quaternion_calibrate(calibration);

    fprintf(stdout, "Calibration complete; Press ENTER to begin...");
    while (std::getc(stdin) != '\n')
        ;

    // Details of our Inverse3 polling loop frequency. See earlier examples for
    // a more detailed explanation.
    typedef std::chrono::high_resolution_clock clock;
    auto next = clock::now();
    auto delay = 200us; // 5kHz

    // Read the initial position of the end-effector which we will be adjusting.
    vec3 position;
    auto state = inverse3.EndEffectorPosition({{0, 0, 0}});
    for (size_t i = 0; i < 3; ++i)
        position[i] = state.position[i];

    while (true) {

        // Read the latest cached orientation quaternion and button state from
        // the handle thread.
        vec4 quaternion;
        quill.read(quaternion, button);

        // Adjust the current handle quaternion with our calibration quaternion.
        quaternion_offset(quaternion, calibration);

        // Transform our quaternion into a direction vector representing where
        // the handle is facing.
        vec3 dir;
        quaternion_to_direction(quaternion, dir);

        // Compute a new end-effector position which will be updated based on
        // where the handle is pointing while the handle button is held down.
        // The multiplication constant used here is somewhat arbitrary and was
        // obtained through trial and error.
        API::Devices::Inverse3::EndEffectorPositionRequest request;
        for (size_t i = 0; i < 3; ++i) {
            if (button)
                position[i] += dir[i] * 0.0005f;
            request.position[i] = static_cast<float>(position[i]);
        }

        // Update our end-effector position to reflect the desired position.
        (void)inverse3.EndEffectorPosition(request);

        fprintf(stderr,
                "\r"
                "move=%u, "
                "dir=[ % 0.3f % 0.3f % 0.3f ], "
                "pos=[ % 0.3f % 0.3f % 0.3f ]",
                button, dir[0], dir[1], dir[2], request.position[0],
                request.position[1], request.position[2]);

        // Busy loop. See previous examples for more details.
        next += delay;
        while (next > clock::now())
            ;
    }

    return 0;
}

