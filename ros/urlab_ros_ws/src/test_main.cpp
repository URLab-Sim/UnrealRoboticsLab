// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Standalone harness that drives the whole UrlabRclCore API and self-verifies.
// It lives inside the ROS workspace, so it is allowed to use rcl directly for
// the loopback probe that exercises the subscription leg; the core-side sub path
// stays the code under test.
//
// Modes:
//   (default) / --selftest : create every publisher and subscription, then run
//       an in-process loopback that publishes a deterministic Float64MultiArray
//       and Twist to the core's own subscriptions and asserts the callbacks fire
//       with the exact values.
//   --publish N            : publish N rounds of deterministic values on every
//       publisher at 50 Hz for an external `ros2 topic echo` cross-check.
//
// Exit 0 = pass, non-zero = failure.

#include "UrlabRclCore.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <thread>

#if defined(_WIN32)
#include <windows.h>
#include <timeapi.h>  // timeBeginPeriod / timeEndPeriod (link winmm)
#endif

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <geometry_msgs/msg/twist.h>

namespace
{
constexpr const char* kCtrlTopic = "/urlab_test/cmd_ctrl";
constexpr const char* kTwistTopic = "/urlab_test/cmd_vel";
constexpr int64_t kSpinTimeoutNs = 10 * 1000 * 1000;  // 10 ms

// Callback capture targets, populated by the core's subscription callbacks.
struct FCtrlCapture
{
    bool bReceived = false;
    int32_t Count = 0;
    double Values[16] = {0.0};
};

struct FTwistCapture
{
    bool bReceived = false;
    double Linear[3] = {0.0, 0.0, 0.0};
    double Angular[3] = {0.0, 0.0, 0.0};
};

void OnCtrl(const double* Values, int32_t Count, void* User)
{
    FCtrlCapture* Cap = static_cast<FCtrlCapture*>(User);
    Cap->bReceived = true;
    Cap->Count = Count;
    const int32_t N = Count < 16 ? Count : 16;
    for (int32_t i = 0; i < N; ++i)
    {
        Cap->Values[i] = Values[i];
    }
}

void OnTwist(const double Linear[3], const double Angular[3], void* User)
{
    FTwistCapture* Cap = static_cast<FTwistCapture*>(User);
    Cap->bReceived = true;
    for (int i = 0; i < 3; ++i)
    {
        Cap->Linear[i] = Linear[i];
        Cap->Angular[i] = Angular[i];
    }
}

bool NearlyEqual(double A, double B)
{
    const double D = A - B;
    return (D < 1e-9) && (D > -1e-9);
}

// Every core handle the harness creates, so both modes share one setup/teardown.
struct FHarness
{
    UrlabRclContext* Ctx = nullptr;
    UrlabRclJointStatePub* JointState = nullptr;
    UrlabRclImuPub* Imu = nullptr;
    UrlabRclTfPub* Tf = nullptr;
    UrlabRclTfPub* TfStatic = nullptr;
    UrlabRclTwistStampedPub* TwistStamped = nullptr;
    UrlabRclClockPub* Clock = nullptr;
    UrlabRclImagePub* Image = nullptr;
    UrlabRclCtrlSub* CtrlSub = nullptr;
    UrlabRclTwistSub* TwistSub = nullptr;
    FCtrlCapture CtrlCapture;
    FTwistCapture TwistCapture;
};

bool CreateHarness(FHarness& H)
{
    H.Ctx = UrlabRcl_Init("urlab_test", "", -1);
    if (!H.Ctx)
    {
        std::fprintf(stderr, "UrlabRcl_Init failed: %s\n", UrlabRcl_LastError());
        return false;
    }
    std::printf("distro: %s\n", UrlabRcl_DistroName());

    const char* JointNames[3] = {"joint_a", "joint_b", "joint_c"};
    H.JointState = UrlabRcl_CreateJointStatePub(H.Ctx,
        "/urlab_test/joint_states", JointNames, 3);
    H.Imu = UrlabRcl_CreateImuPub(H.Ctx, "/urlab_test/imu", "urlab_test/imu");
    H.Tf = UrlabRcl_CreateTfPub(H.Ctx, 0);
    H.TfStatic = UrlabRcl_CreateTfPub(H.Ctx, 1);
    H.TwistStamped = UrlabRcl_CreateTwistStampedPub(H.Ctx,
        "/urlab_test/twist", "urlab_test/base");
    H.Clock = UrlabRcl_CreateClockPub(H.Ctx);
    H.Image = UrlabRcl_CreateImagePub(H.Ctx, "/urlab_test/image",
        "urlab_test/camera", 4, 4, "rgb8");
    H.CtrlSub = UrlabRcl_CreateCtrlSub(H.Ctx, kCtrlTopic, &OnCtrl, &H.CtrlCapture);
    H.TwistSub = UrlabRcl_CreateTwistSub(H.Ctx, kTwistTopic, &OnTwist, &H.TwistCapture);

    if (!H.JointState || !H.Imu || !H.Tf || !H.TfStatic || !H.TwistStamped ||
        !H.Clock || !H.Image || !H.CtrlSub || !H.TwistSub)
    {
        std::fprintf(stderr, "handle creation failed: %s\n", UrlabRcl_LastError());
        return false;
    }
    return true;
}

void DestroyHarness(FHarness& H)
{
    UrlabRcl_DestroyCtrlSub(H.CtrlSub);
    UrlabRcl_DestroyTwistSub(H.TwistSub);
    UrlabRcl_DestroyImagePub(H.Image);
    UrlabRcl_DestroyClockPub(H.Clock);
    UrlabRcl_DestroyTwistStampedPub(H.TwistStamped);
    UrlabRcl_DestroyTfPub(H.TfStatic);
    UrlabRcl_DestroyTfPub(H.Tf);
    UrlabRcl_DestroyImuPub(H.Imu);
    UrlabRcl_DestroyJointStatePub(H.JointState);
    UrlabRcl_Shutdown(H.Ctx);
}

int PublishRound(FHarness& H, int32_t Round, int64_t SimTimeNs)
{
    const double Positions[3] = {1.0, 2.0, 3.0};
    const double Velocities[3] = {0.1, 0.2, 0.3};
    const double Efforts[3] = {10.0, 20.0, 30.0};
    if (UrlabRcl_PublishJointState(H.JointState, Positions, Velocities, Efforts, 3, SimTimeNs) != 0)
    {
        std::fprintf(stderr, "PublishJointState failed: %s\n", UrlabRcl_LastError());
        return 1;
    }

    const double AngularVel[3] = {0.01, 0.02, 0.03};
    const double LinearAccel[3] = {0.0, 0.0, 9.81};
    const double OrientationXyzw[4] = {0.0, 0.0, 0.0, 1.0};
    UrlabRcl_PublishImu(H.Imu, AngularVel, LinearAccel, OrientationXyzw, SimTimeNs);

    const char* Parents[1] = {"world"};
    const char* Children[1] = {"urlab_test/base"};
    const double Translation[3] = {static_cast<double>(Round) * 0.001, 0.0, 0.5};
    const double Rotation[4] = {0.0, 0.0, 0.0, 1.0};
    UrlabRcl_PublishTf(H.Tf, Parents, Children, Translation, Rotation, 1, SimTimeNs);

    const char* StaticParents[1] = {"urlab_test/base"};
    const char* StaticChildren[1] = {"urlab_test/imu"};
    const double StaticTranslation[3] = {0.0, 0.0, 0.1};
    const double StaticRotation[4] = {0.0, 0.0, 0.0, 1.0};
    UrlabRcl_PublishTf(H.TfStatic, StaticParents, StaticChildren,
        StaticTranslation, StaticRotation, 1, SimTimeNs);

    const double Linear[3] = {0.5, 0.0, 0.0};
    const double Angular[3] = {0.0, 0.0, 0.2};
    UrlabRcl_PublishTwistStamped(H.TwistStamped, Linear, Angular, SimTimeNs);

    UrlabRcl_PublishClock(H.Clock, SimTimeNs);

    uint8_t Pixels[4 * 4 * 3];
    for (int i = 0; i < 4 * 4 * 3; ++i)
    {
        Pixels[i] = static_cast<uint8_t>(i);
    }
    UrlabRcl_PublishImage(H.Image, Pixels, 4 * 3, SimTimeNs);

    return 0;
}

// A minimal rcl publisher pair used only by the selftest to feed the core's own
// subscriptions from within this process (loopback over DDS).
struct FProbe
{
    rcl_context_t Context = rcl_get_zero_initialized_context();
    rcl_init_options_t InitOptions = rcl_get_zero_initialized_init_options();
    rcl_node_t Node = rcl_get_zero_initialized_node();
    rcl_publisher_t CtrlPub = rcl_get_zero_initialized_publisher();
    rcl_publisher_t TwistPub = rcl_get_zero_initialized_publisher();
    bool bValid = false;
};

bool ProbeInit(FProbe& P)
{
    rcl_allocator_t Allocator = rcl_get_default_allocator();
    if (rcl_init_options_init(&P.InitOptions, Allocator) != RCL_RET_OK)
    {
        return false;
    }
    if (rcl_init(0, nullptr, &P.InitOptions, &P.Context) != RCL_RET_OK)
    {
        return false;
    }
    rcl_node_options_t NodeOptions = rcl_node_get_default_options();
    if (rcl_node_init(&P.Node, "urlab_test_probe", "", &P.Context, &NodeOptions) != RCL_RET_OK)
    {
        return false;
    }

    rcl_publisher_options_t PubOptions = rcl_publisher_get_default_options();
    const rosidl_message_type_support_t* CtrlTs =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray);
    if (rcl_publisher_init(&P.CtrlPub, &P.Node, CtrlTs, kCtrlTopic, &PubOptions) != RCL_RET_OK)
    {
        return false;
    }
    const rosidl_message_type_support_t* TwistTs =
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
    if (rcl_publisher_init(&P.TwistPub, &P.Node, TwistTs, kTwistTopic, &PubOptions) != RCL_RET_OK)
    {
        return false;
    }
    P.bValid = true;
    return true;
}

void ProbeShutdown(FProbe& P)
{
    rcl_publisher_fini(&P.TwistPub, &P.Node);
    rcl_publisher_fini(&P.CtrlPub, &P.Node);
    rcl_node_fini(&P.Node);
    rcl_shutdown(&P.Context);
    rcl_context_fini(&P.Context);
    rcl_init_options_fini(&P.InitOptions);
}

int ProbePublishCtrl(FProbe& P, const double* Values, int32_t Count)
{
    std_msgs__msg__Float64MultiArray Msg;
    std_msgs__msg__Float64MultiArray__init(&Msg);
    rosidl_runtime_c__double__Sequence__init(&Msg.data, Count);
    for (int32_t i = 0; i < Count; ++i)
    {
        Msg.data.data[i] = Values[i];
    }
    const rcl_ret_t Ret = rcl_publish(&P.CtrlPub, &Msg, nullptr);
    std_msgs__msg__Float64MultiArray__fini(&Msg);
    return Ret == RCL_RET_OK ? 0 : 1;
}

int ProbePublishTwist(FProbe& P, const double Linear[3], const double Angular[3])
{
    geometry_msgs__msg__Twist Msg;
    geometry_msgs__msg__Twist__init(&Msg);
    Msg.linear.x = Linear[0];
    Msg.linear.y = Linear[1];
    Msg.linear.z = Linear[2];
    Msg.angular.x = Angular[0];
    Msg.angular.y = Angular[1];
    Msg.angular.z = Angular[2];
    const rcl_ret_t Ret = rcl_publish(&P.TwistPub, &Msg, nullptr);
    geometry_msgs__msg__Twist__fini(&Msg);
    return Ret == RCL_RET_OK ? 0 : 1;
}

int RunSelftest()
{
    FHarness H;
    if (!CreateHarness(H))
    {
        return 1;
    }

    FProbe Probe;
    if (!ProbeInit(Probe))
    {
        std::fprintf(stderr, "probe init failed\n");
        DestroyHarness(H);
        return 1;
    }

    const double CtrlValues[4] = {1.5, 2.5, 3.5, 4.5};
    const double ProbeLinear[3] = {0.11, 0.22, 0.33};
    const double ProbeAngular[3] = {0.44, 0.55, 0.66};

    // Publish and spin repeatedly; DDS discovery between the probe and the core
    // subscriptions can take a moment on the first sample.
    int Result = 1;
    for (int Attempt = 0; Attempt < 300; ++Attempt)
    {
        ProbePublishCtrl(Probe, CtrlValues, 4);
        ProbePublishTwist(Probe, ProbeLinear, ProbeAngular);
        UrlabRcl_SpinSome(H.Ctx, kSpinTimeoutNs);
        if (H.CtrlCapture.bReceived && H.TwistCapture.bReceived)
        {
            Result = 0;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (Result != 0)
    {
        std::fprintf(stderr, "selftest: subscription callbacks did not fire "
            "(ctrl=%d twist=%d)\n", H.CtrlCapture.bReceived ? 1 : 0,
            H.TwistCapture.bReceived ? 1 : 0);
    }
    else
    {
        if (H.CtrlCapture.Count != 4)
        {
            std::fprintf(stderr, "selftest: ctrl count %d != 4\n", H.CtrlCapture.Count);
            Result = 1;
        }
        for (int i = 0; i < 4 && Result == 0; ++i)
        {
            if (!NearlyEqual(H.CtrlCapture.Values[i], CtrlValues[i]))
            {
                std::fprintf(stderr, "selftest: ctrl[%d] %f != %f\n",
                    i, H.CtrlCapture.Values[i], CtrlValues[i]);
                Result = 1;
            }
        }
        for (int i = 0; i < 3 && Result == 0; ++i)
        {
            if (!NearlyEqual(H.TwistCapture.Linear[i], ProbeLinear[i]) ||
                !NearlyEqual(H.TwistCapture.Angular[i], ProbeAngular[i]))
            {
                std::fprintf(stderr, "selftest: twist component %d mismatch\n", i);
                Result = 1;
            }
        }
    }

    if (Result == 0)
    {
        std::printf("selftest: OK (ctrl + twist loopback verified)\n");
    }

    ProbeShutdown(Probe);
    DestroyHarness(H);
    return Result;
}

int RunPublish(int32_t Rounds)
{
    FHarness H;
    if (!CreateHarness(H))
    {
        return 1;
    }

    const int64_t StepNs = 20 * 1000 * 1000;  // 50 Hz
    int64_t SimTimeNs = 0;
    int Result = 0;
#if defined(_WIN32)
    // Windows' default ~15.6 ms scheduler tick rounds a 20 ms sleep up to ~31 ms
    // (~32 Hz). Request 1 ms timer resolution so the 50 Hz cadence is accurate.
    timeBeginPeriod(1);
#endif
    const auto Start = std::chrono::steady_clock::now();
    for (int32_t r = 0; r < Rounds; ++r)
    {
        if (PublishRound(H, r, SimTimeNs) != 0)
        {
            Result = 1;
            break;
        }
        UrlabRcl_SpinSome(H.Ctx, 0);
        SimTimeNs += StepNs;
        // Pace on a fixed cadence relative to Start so per-round publish cost
        // does not accumulate into drift.
        std::this_thread::sleep_until(Start + std::chrono::milliseconds(20) * (r + 1));
    }
#if defined(_WIN32)
    timeEndPeriod(1);
#endif

    if (Result == 0)
    {
        std::printf("publish: OK (%d rounds)\n", Rounds);
    }
    DestroyHarness(H);
    return Result;
}

int RunReinitRoundTrip()
{
    UrlabRclContext* Ctx = UrlabRcl_Init("urlab_test_reinit", "", -1);
    if (!Ctx)
    {
        std::fprintf(stderr, "re-init round trip failed: %s\n", UrlabRcl_LastError());
        return 1;
    }
    UrlabRcl_Shutdown(Ctx);
    std::printf("reinit: OK\n");
    return 0;
}
}  // namespace

int main(int argc, char** argv)
{
    int32_t PublishRounds = -1;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--publish") == 0 && i + 1 < argc)
        {
            PublishRounds = std::atoi(argv[i + 1]);
            ++i;
        }
    }

    int Result;
    if (PublishRounds >= 0)
    {
        Result = RunPublish(PublishRounds);
    }
    else
    {
        Result = RunSelftest();
    }

    if (Result == 0)
    {
        Result = RunReinitRoundTrip();
    }

    if (Result == 0)
    {
        std::printf("urlab_rcl_test: PASS\n");
    }
    else
    {
        std::printf("urlab_rcl_test: FAIL\n");
    }
    return Result;
}
