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

// Everything below is fenced behind URLAB_WITH_ROS2 so that UBT, which compiles
// every .cpp under the module unconditionally, sees an empty translation unit in
// every UE build until the ROS build wiring defines the macro. The defined()
// guard keeps builds green while the macro does not exist at all. The standalone
// ROS workspace (ros/urlab_ros_ws) defines URLAB_WITH_ROS2=1 itself.
#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2

#include "UrlabRclCore.h"

#include <cstring>
#include <cstdlib>
#include <vector>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rcl/subscription.h>
#include <rcl/wait.h>
#include <rmw/qos_profiles.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <builtin_interfaces/msg/time.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/image.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <tf2_msgs/msg/tf_message.h>
#include <rosgraph_msgs/msg/clock.h>

#ifndef URLAB_ROS_DISTRO_NAME
#define URLAB_ROS_DISTRO_NAME "humble"
#endif

namespace
{
// Last error string, captured from rcutils/rcl on each failed call.
char GLastError[1024] = {0};

void CaptureError()
{
    const rcutils_error_string_t Err = rcl_get_error_string();
    std::strncpy(GLastError, Err.str, sizeof(GLastError) - 1);
    GLastError[sizeof(GLastError) - 1] = '\0';
    rcl_reset_error();
}

void ClearError()
{
    GLastError[0] = '\0';
}

void FillStamp(builtin_interfaces__msg__Time& Stamp, int64_t SimTimeNs)
{
    Stamp.sec = static_cast<int32_t>(SimTimeNs / 1000000000LL);
    Stamp.nanosec = static_cast<uint32_t>(SimTimeNs % 1000000000LL);
}

// Subscription record kinds; SpinSome dispatches per kind.
enum class ESubKind : uint8_t
{
    Ctrl,
    Twist
};

struct FSubRecord
{
    ESubKind Kind;
    rcl_subscription_t Sub;
    UrlabRclContext* Ctx;
    UrlabRclCtrlCallback CtrlCallback;
    UrlabRclTwistCallback TwistCallback;
    void* User;
    std_msgs__msg__Float64MultiArray CtrlMsg;
    geometry_msgs__msg__Twist TwistMsg;
};
}  // namespace

// --- Opaque handle definitions ---------------------------------------------

struct UrlabRclContext
{
    rcl_context_t Context;
    rcl_init_options_t InitOptions;
    rcl_node_t Node;
    rcl_allocator_t Allocator;
    bool bNodeValid;
    // Subscriptions registered against this context, spun by UrlabRcl_SpinSome.
    std::vector<FSubRecord*> Subs;
};

struct UrlabRclJointStatePub
{
    UrlabRclContext* Ctx;
    rcl_publisher_t Pub;
    sensor_msgs__msg__JointState Msg;
};

struct UrlabRclImuPub
{
    UrlabRclContext* Ctx;
    rcl_publisher_t Pub;
    sensor_msgs__msg__Imu Msg;
};

struct UrlabRclTfPub
{
    UrlabRclContext* Ctx;
    rcl_publisher_t Pub;
    tf2_msgs__msg__TFMessage Msg;
};

struct UrlabRclTwistStampedPub
{
    UrlabRclContext* Ctx;
    rcl_publisher_t Pub;
    geometry_msgs__msg__TwistStamped Msg;
};

struct UrlabRclClockPub
{
    UrlabRclContext* Ctx;
    rcl_publisher_t Pub;
    rosgraph_msgs__msg__Clock Msg;
};

struct UrlabRclImagePub
{
    UrlabRclContext* Ctx;
    rcl_publisher_t Pub;
    sensor_msgs__msg__Image Msg;
    int32_t Width;
    int32_t Height;
};

struct UrlabRclCtrlPub
{
    UrlabRclContext* Ctx;
    rcl_publisher_t Pub;
    std_msgs__msg__Float64MultiArray Msg;
};

struct UrlabRclCtrlSub
{
    FSubRecord Rec;
};

struct UrlabRclTwistSub
{
    FSubRecord Rec;
};

namespace
{
// Publisher creation helper shared by every telemetry publisher: initialises the
// rcl publisher against the context node with the given type support and QoS.
bool InitPublisher(UrlabRclContext* Ctx, rcl_publisher_t& OutPub,
    const rosidl_message_type_support_t* TypeSupport, const char* Topic,
    const rmw_qos_profile_t& Qos)
{
    OutPub = rcl_get_zero_initialized_publisher();
    rcl_publisher_options_t Options = rcl_publisher_get_default_options();
    Options.qos = Qos;
    const rcl_ret_t Ret = rcl_publisher_init(&OutPub, &Ctx->Node, TypeSupport, Topic, &Options);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        return false;
    }
    return true;
}

void SetString(rosidl_runtime_c__String& Str, const char* Value)
{
    rosidl_runtime_c__String__assign(&Str, Value ? Value : "");
}

void DetachSub(FSubRecord* Rec)
{
    if (!Rec || !Rec->Ctx)
    {
        return;
    }
    std::vector<FSubRecord*>& List = Rec->Ctx->Subs;
    for (size_t i = 0; i < List.size(); ++i)
    {
        if (List[i] == Rec)
        {
            List.erase(List.begin() + i);
            break;
        }
    }
}
}  // namespace

// --- Context ---------------------------------------------------------------

UrlabRclContext* UrlabRcl_Init(const char* NodeName, const char* NodeNamespace, int32_t DomainId)
{
    ClearError();
    UrlabRclContext* Ctx = new UrlabRclContext();
    Ctx->Context = rcl_get_zero_initialized_context();
    Ctx->InitOptions = rcl_get_zero_initialized_init_options();
    Ctx->Node = rcl_get_zero_initialized_node();
    Ctx->Allocator = rcl_get_default_allocator();
    Ctx->bNodeValid = false;

    rcl_ret_t Ret = rcl_init_options_init(&Ctx->InitOptions, Ctx->Allocator);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        delete Ctx;
        return nullptr;
    }

    const size_t Domain = (DomainId < 0)
        ? static_cast<size_t>(RCL_DEFAULT_DOMAIN_ID)
        : static_cast<size_t>(DomainId);
    Ret = rcl_init_options_set_domain_id(&Ctx->InitOptions, Domain);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        rcl_init_options_fini(&Ctx->InitOptions);
        delete Ctx;
        return nullptr;
    }

    Ret = rcl_init(0, nullptr, &Ctx->InitOptions, &Ctx->Context);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        rcl_init_options_fini(&Ctx->InitOptions);
        delete Ctx;
        return nullptr;
    }

    rcl_node_options_t NodeOptions = rcl_node_get_default_options();
    Ret = rcl_node_init(&Ctx->Node, NodeName ? NodeName : "urlab",
        NodeNamespace ? NodeNamespace : "", &Ctx->Context, &NodeOptions);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        rcl_shutdown(&Ctx->Context);
        rcl_context_fini(&Ctx->Context);
        rcl_init_options_fini(&Ctx->InitOptions);
        delete Ctx;
        return nullptr;
    }
    Ctx->bNodeValid = true;
    return Ctx;
}

void UrlabRcl_Shutdown(UrlabRclContext* Ctx)
{
    if (!Ctx)
    {
        return;
    }
    for (FSubRecord* Rec : Ctx->Subs)
    {
        if (Rec)
        {
            rcl_subscription_fini(&Rec->Sub, &Ctx->Node);
            if (Rec->Kind == ESubKind::Ctrl)
            {
                std_msgs__msg__Float64MultiArray__fini(&Rec->CtrlMsg);
            }
            else
            {
                geometry_msgs__msg__Twist__fini(&Rec->TwistMsg);
            }
            delete Rec;
        }
    }
    Ctx->Subs.clear();

    if (Ctx->bNodeValid)
    {
        rcl_node_fini(&Ctx->Node);
        Ctx->bNodeValid = false;
    }
    rcl_shutdown(&Ctx->Context);
    rcl_context_fini(&Ctx->Context);
    rcl_init_options_fini(&Ctx->InitOptions);
    delete Ctx;
}

const char* UrlabRcl_DistroName()
{
    return URLAB_ROS_DISTRO_NAME;
}

const char* UrlabRcl_LastError()
{
    return GLastError;
}

// --- JointState ------------------------------------------------------------

UrlabRclJointStatePub* UrlabRcl_CreateJointStatePub(UrlabRclContext* Ctx,
    const char* Topic, const char** JointNames, int32_t JointCount)
{
    ClearError();
    if (!Ctx)
    {
        return nullptr;
    }
    UrlabRclJointStatePub* Pub = new UrlabRclJointStatePub();
    Pub->Ctx = Ctx;
    sensor_msgs__msg__JointState__init(&Pub->Msg);
    SetString(Pub->Msg.header.frame_id, "");

    const int32_t Count = JointCount > 0 ? JointCount : 0;
    if (Count > 0)
    {
        rosidl_runtime_c__String__Sequence__init(&Pub->Msg.name, Count);
        for (int32_t i = 0; i < Count; ++i)
        {
            SetString(Pub->Msg.name.data[i], JointNames ? JointNames[i] : "");
        }
        rosidl_runtime_c__double__Sequence__init(&Pub->Msg.position, Count);
        rosidl_runtime_c__double__Sequence__init(&Pub->Msg.velocity, Count);
        rosidl_runtime_c__double__Sequence__init(&Pub->Msg.effort, Count);
    }

    const rosidl_message_type_support_t* Ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);
    if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
    {
        sensor_msgs__msg__JointState__fini(&Pub->Msg);
        delete Pub;
        return nullptr;
    }
    return Pub;
}

int UrlabRcl_PublishJointState(UrlabRclJointStatePub* Pub,
    const double* Positions, const double* Velocities, const double* Efforts,
    int32_t Count, int64_t SimTimeNs)
{
    ClearError();
    if (!Pub)
    {
        return -1;
    }
    FillStamp(Pub->Msg.header.stamp, SimTimeNs);

    const int32_t Cap = static_cast<int32_t>(Pub->Msg.position.capacity);
    const int32_t N = Count < Cap ? Count : Cap;

    if (Positions)
    {
        std::memcpy(Pub->Msg.position.data, Positions, sizeof(double) * N);
        Pub->Msg.position.size = N;
    }
    else
    {
        Pub->Msg.position.size = 0;
    }
    if (Velocities)
    {
        std::memcpy(Pub->Msg.velocity.data, Velocities, sizeof(double) * N);
        Pub->Msg.velocity.size = N;
    }
    else
    {
        Pub->Msg.velocity.size = 0;
    }
    if (Efforts)
    {
        std::memcpy(Pub->Msg.effort.data, Efforts, sizeof(double) * N);
        Pub->Msg.effort.size = N;
    }
    else
    {
        Pub->Msg.effort.size = 0;
    }

    const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        return -static_cast<int>(Ret);
    }
    return 0;
}

void UrlabRcl_DestroyJointStatePub(UrlabRclJointStatePub* Pub)
{
    if (!Pub)
    {
        return;
    }
    rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
    sensor_msgs__msg__JointState__fini(&Pub->Msg);
    delete Pub;
}

// --- Imu -------------------------------------------------------------------

UrlabRclImuPub* UrlabRcl_CreateImuPub(UrlabRclContext* Ctx, const char* Topic, const char* FrameId)
{
    ClearError();
    if (!Ctx)
    {
        return nullptr;
    }
    UrlabRclImuPub* Pub = new UrlabRclImuPub();
    Pub->Ctx = Ctx;
    sensor_msgs__msg__Imu__init(&Pub->Msg);
    SetString(Pub->Msg.header.frame_id, FrameId);

    const rosidl_message_type_support_t* Ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);
    if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
    {
        sensor_msgs__msg__Imu__fini(&Pub->Msg);
        delete Pub;
        return nullptr;
    }
    return Pub;
}

int UrlabRcl_PublishImu(UrlabRclImuPub* Pub, const double AngularVel[3],
    const double LinearAccel[3], const double OrientationXyzw[4], int64_t SimTimeNs)
{
    ClearError();
    if (!Pub)
    {
        return -1;
    }
    FillStamp(Pub->Msg.header.stamp, SimTimeNs);

    if (OrientationXyzw)
    {
        Pub->Msg.orientation.x = OrientationXyzw[0];
        Pub->Msg.orientation.y = OrientationXyzw[1];
        Pub->Msg.orientation.z = OrientationXyzw[2];
        Pub->Msg.orientation.w = OrientationXyzw[3];
        Pub->Msg.orientation_covariance[0] = 0.0;
    }
    else
    {
        // REP 145: leading covariance element -1 marks orientation absent.
        Pub->Msg.orientation.x = 0.0;
        Pub->Msg.orientation.y = 0.0;
        Pub->Msg.orientation.z = 0.0;
        Pub->Msg.orientation.w = 1.0;
        Pub->Msg.orientation_covariance[0] = -1.0;
    }
    if (AngularVel)
    {
        Pub->Msg.angular_velocity.x = AngularVel[0];
        Pub->Msg.angular_velocity.y = AngularVel[1];
        Pub->Msg.angular_velocity.z = AngularVel[2];
        Pub->Msg.angular_velocity_covariance[0] = 0.0;
    }
    else
    {
        Pub->Msg.angular_velocity.x = 0.0;
        Pub->Msg.angular_velocity.y = 0.0;
        Pub->Msg.angular_velocity.z = 0.0;
        Pub->Msg.angular_velocity_covariance[0] = -1.0;
    }
    if (LinearAccel)
    {
        Pub->Msg.linear_acceleration.x = LinearAccel[0];
        Pub->Msg.linear_acceleration.y = LinearAccel[1];
        Pub->Msg.linear_acceleration.z = LinearAccel[2];
        Pub->Msg.linear_acceleration_covariance[0] = 0.0;
    }
    else
    {
        Pub->Msg.linear_acceleration.x = 0.0;
        Pub->Msg.linear_acceleration.y = 0.0;
        Pub->Msg.linear_acceleration.z = 0.0;
        Pub->Msg.linear_acceleration_covariance[0] = -1.0;
    }

    const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        return -static_cast<int>(Ret);
    }
    return 0;
}

void UrlabRcl_DestroyImuPub(UrlabRclImuPub* Pub)
{
    if (!Pub)
    {
        return;
    }
    rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
    sensor_msgs__msg__Imu__fini(&Pub->Msg);
    delete Pub;
}

// --- Tf --------------------------------------------------------------------

UrlabRclTfPub* UrlabRcl_CreateTfPub(UrlabRclContext* Ctx, int32_t bStatic)
{
    ClearError();
    if (!Ctx)
    {
        return nullptr;
    }
    UrlabRclTfPub* Pub = new UrlabRclTfPub();
    Pub->Ctx = Ctx;
    tf2_msgs__msg__TFMessage__init(&Pub->Msg);

    rmw_qos_profile_t Qos = rmw_qos_profile_default;
    const char* Topic = "/tf";
    if (bStatic != 0)
    {
        // /tf_static: latch the last set so late joiners receive it.
        Qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        Qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        Qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        Qos.depth = 1;
        Topic = "/tf_static";
    }

    const rosidl_message_type_support_t* Ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage);
    if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, Qos))
    {
        tf2_msgs__msg__TFMessage__fini(&Pub->Msg);
        delete Pub;
        return nullptr;
    }
    return Pub;
}

int UrlabRcl_PublishTf(UrlabRclTfPub* Pub, const char** ParentFrameIds,
    const char** ChildFrameIds, const double* TranslationsXyz,
    const double* RotationsXyzw, int32_t Count, int64_t SimTimeNs)
{
    ClearError();
    if (!Pub)
    {
        return -1;
    }
    const int32_t N = Count > 0 ? Count : 0;

    // Resize the transform sequence to exactly N, reinitialising each element.
    tf2_msgs__msg__TFMessage__fini(&Pub->Msg);
    tf2_msgs__msg__TFMessage__init(&Pub->Msg);
    if (N > 0)
    {
        geometry_msgs__msg__TransformStamped__Sequence__init(&Pub->Msg.transforms, N);
        for (int32_t i = 0; i < N; ++i)
        {
            geometry_msgs__msg__TransformStamped& T = Pub->Msg.transforms.data[i];
            FillStamp(T.header.stamp, SimTimeNs);
            SetString(T.header.frame_id, ParentFrameIds ? ParentFrameIds[i] : "");
            SetString(T.child_frame_id, ChildFrameIds ? ChildFrameIds[i] : "");
            if (TranslationsXyz)
            {
                T.transform.translation.x = TranslationsXyz[i * 3 + 0];
                T.transform.translation.y = TranslationsXyz[i * 3 + 1];
                T.transform.translation.z = TranslationsXyz[i * 3 + 2];
            }
            if (RotationsXyzw)
            {
                T.transform.rotation.x = RotationsXyzw[i * 4 + 0];
                T.transform.rotation.y = RotationsXyzw[i * 4 + 1];
                T.transform.rotation.z = RotationsXyzw[i * 4 + 2];
                T.transform.rotation.w = RotationsXyzw[i * 4 + 3];
            }
            else
            {
                T.transform.rotation.w = 1.0;
            }
        }
    }

    const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        return -static_cast<int>(Ret);
    }
    return 0;
}

void UrlabRcl_DestroyTfPub(UrlabRclTfPub* Pub)
{
    if (!Pub)
    {
        return;
    }
    rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
    tf2_msgs__msg__TFMessage__fini(&Pub->Msg);
    delete Pub;
}

// --- TwistStamped ----------------------------------------------------------

UrlabRclTwistStampedPub* UrlabRcl_CreateTwistStampedPub(UrlabRclContext* Ctx,
    const char* Topic, const char* FrameId)
{
    ClearError();
    if (!Ctx)
    {
        return nullptr;
    }
    UrlabRclTwistStampedPub* Pub = new UrlabRclTwistStampedPub();
    Pub->Ctx = Ctx;
    geometry_msgs__msg__TwistStamped__init(&Pub->Msg);
    SetString(Pub->Msg.header.frame_id, FrameId);

    const rosidl_message_type_support_t* Ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped);
    if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
    {
        geometry_msgs__msg__TwistStamped__fini(&Pub->Msg);
        delete Pub;
        return nullptr;
    }
    return Pub;
}

int UrlabRcl_PublishTwistStamped(UrlabRclTwistStampedPub* Pub,
    const double Linear[3], const double Angular[3], int64_t SimTimeNs)
{
    ClearError();
    if (!Pub)
    {
        return -1;
    }
    FillStamp(Pub->Msg.header.stamp, SimTimeNs);
    if (Linear)
    {
        Pub->Msg.twist.linear.x = Linear[0];
        Pub->Msg.twist.linear.y = Linear[1];
        Pub->Msg.twist.linear.z = Linear[2];
    }
    if (Angular)
    {
        Pub->Msg.twist.angular.x = Angular[0];
        Pub->Msg.twist.angular.y = Angular[1];
        Pub->Msg.twist.angular.z = Angular[2];
    }

    const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        return -static_cast<int>(Ret);
    }
    return 0;
}

void UrlabRcl_DestroyTwistStampedPub(UrlabRclTwistStampedPub* Pub)
{
    if (!Pub)
    {
        return;
    }
    rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
    geometry_msgs__msg__TwistStamped__fini(&Pub->Msg);
    delete Pub;
}

// --- Clock -----------------------------------------------------------------

UrlabRclClockPub* UrlabRcl_CreateClockPub(UrlabRclContext* Ctx)
{
    ClearError();
    if (!Ctx)
    {
        return nullptr;
    }
    UrlabRclClockPub* Pub = new UrlabRclClockPub();
    Pub->Ctx = Ctx;
    rosgraph_msgs__msg__Clock__init(&Pub->Msg);

    const rosidl_message_type_support_t* Ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(rosgraph_msgs, msg, Clock);
    if (!InitPublisher(Ctx, Pub->Pub, Ts, "/clock", rmw_qos_profile_default))
    {
        rosgraph_msgs__msg__Clock__fini(&Pub->Msg);
        delete Pub;
        return nullptr;
    }
    return Pub;
}

int UrlabRcl_PublishClock(UrlabRclClockPub* Pub, int64_t SimTimeNs)
{
    ClearError();
    if (!Pub)
    {
        return -1;
    }
    FillStamp(Pub->Msg.clock, SimTimeNs);
    const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        return -static_cast<int>(Ret);
    }
    return 0;
}

void UrlabRcl_DestroyClockPub(UrlabRclClockPub* Pub)
{
    if (!Pub)
    {
        return;
    }
    rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
    rosgraph_msgs__msg__Clock__fini(&Pub->Msg);
    delete Pub;
}

// --- Image -----------------------------------------------------------------

UrlabRclImagePub* UrlabRcl_CreateImagePub(UrlabRclContext* Ctx, const char* Topic,
    const char* FrameId, int32_t Width, int32_t Height, const char* Encoding)
{
    ClearError();
    if (!Ctx)
    {
        return nullptr;
    }
    UrlabRclImagePub* Pub = new UrlabRclImagePub();
    Pub->Ctx = Ctx;
    Pub->Width = Width > 0 ? Width : 0;
    Pub->Height = Height > 0 ? Height : 0;
    sensor_msgs__msg__Image__init(&Pub->Msg);
    SetString(Pub->Msg.header.frame_id, FrameId);
    SetString(Pub->Msg.encoding, Encoding);
    Pub->Msg.width = static_cast<uint32_t>(Pub->Width);
    Pub->Msg.height = static_cast<uint32_t>(Pub->Height);
    Pub->Msg.is_bigendian = 0;

    const rosidl_message_type_support_t* Ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image);
    if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
    {
        sensor_msgs__msg__Image__fini(&Pub->Msg);
        delete Pub;
        return nullptr;
    }
    return Pub;
}

int UrlabRcl_PublishImage(UrlabRclImagePub* Pub, const uint8_t* Data,
    int32_t StrideBytes, int64_t SimTimeNs)
{
    ClearError();
    if (!Pub)
    {
        return -1;
    }
    FillStamp(Pub->Msg.header.stamp, SimTimeNs);
    Pub->Msg.step = static_cast<uint32_t>(StrideBytes > 0 ? StrideBytes : 0);

    const size_t Total = static_cast<size_t>(Pub->Msg.step) * static_cast<size_t>(Pub->Height);
    if (Data && Total > 0)
    {
        if (Pub->Msg.data.capacity < Total)
        {
            rosidl_runtime_c__uint8__Sequence__fini(&Pub->Msg.data);
            rosidl_runtime_c__uint8__Sequence__init(&Pub->Msg.data, Total);
        }
        std::memcpy(Pub->Msg.data.data, Data, Total);
        Pub->Msg.data.size = Total;
    }
    else
    {
        Pub->Msg.data.size = 0;
    }

    const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        return -static_cast<int>(Ret);
    }
    return 0;
}

void UrlabRcl_DestroyImagePub(UrlabRclImagePub* Pub)
{
    if (!Pub)
    {
        return;
    }
    rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
    sensor_msgs__msg__Image__fini(&Pub->Msg);
    delete Pub;
}

// --- Ctrl publisher (control injection) ------------------------------------

UrlabRclCtrlPub* UrlabRcl_CreateCtrlPub(UrlabRclContext* Ctx, const char* Topic)
{
    ClearError();
    if (!Ctx)
    {
        return nullptr;
    }
    UrlabRclCtrlPub* Pub = new UrlabRclCtrlPub();
    Pub->Ctx = Ctx;
    std_msgs__msg__Float64MultiArray__init(&Pub->Msg);

    const rosidl_message_type_support_t* Ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray);
    if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
    {
        std_msgs__msg__Float64MultiArray__fini(&Pub->Msg);
        delete Pub;
        return nullptr;
    }
    return Pub;
}

int UrlabRcl_PublishCtrl(UrlabRclCtrlPub* Pub, const double* Values, int32_t Count)
{
    ClearError();
    if (!Pub)
    {
        return -1;
    }
    const int32_t N = Count > 0 ? Count : 0;
    // Resize the data sequence to exactly N.
    if (static_cast<int32_t>(Pub->Msg.data.capacity) < N)
    {
        rosidl_runtime_c__double__Sequence__fini(&Pub->Msg.data);
        rosidl_runtime_c__double__Sequence__init(&Pub->Msg.data, N);
    }
    if (Values && N > 0)
    {
        std::memcpy(Pub->Msg.data.data, Values, sizeof(double) * N);
    }
    Pub->Msg.data.size = N;

    const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        return -static_cast<int>(Ret);
    }
    return 0;
}

void UrlabRcl_DestroyCtrlPub(UrlabRclCtrlPub* Pub)
{
    if (!Pub)
    {
        return;
    }
    rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
    std_msgs__msg__Float64MultiArray__fini(&Pub->Msg);
    delete Pub;
}

// --- Subscriptions ---------------------------------------------------------

UrlabRclCtrlSub* UrlabRcl_CreateCtrlSub(UrlabRclContext* Ctx, const char* Topic,
    UrlabRclCtrlCallback Callback, void* User)
{
    ClearError();
    if (!Ctx)
    {
        return nullptr;
    }
    UrlabRclCtrlSub* Sub = new UrlabRclCtrlSub();
    Sub->Rec.Kind = ESubKind::Ctrl;
    Sub->Rec.Ctx = Ctx;
    Sub->Rec.CtrlCallback = Callback;
    Sub->Rec.TwistCallback = nullptr;
    Sub->Rec.User = User;
    std_msgs__msg__Float64MultiArray__init(&Sub->Rec.CtrlMsg);

    Sub->Rec.Sub = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t Options = rcl_subscription_get_default_options();
    const rosidl_message_type_support_t* Ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray);
    const rcl_ret_t Ret = rcl_subscription_init(&Sub->Rec.Sub, &Ctx->Node, Ts, Topic, &Options);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        std_msgs__msg__Float64MultiArray__fini(&Sub->Rec.CtrlMsg);
        delete Sub;
        return nullptr;
    }
    Ctx->Subs.push_back(&Sub->Rec);
    return Sub;
}

void UrlabRcl_DestroyCtrlSub(UrlabRclCtrlSub* Sub)
{
    if (!Sub)
    {
        return;
    }
    DetachSub(&Sub->Rec);
    rcl_subscription_fini(&Sub->Rec.Sub, &Sub->Rec.Ctx->Node);
    std_msgs__msg__Float64MultiArray__fini(&Sub->Rec.CtrlMsg);
    delete Sub;
}

UrlabRclTwistSub* UrlabRcl_CreateTwistSub(UrlabRclContext* Ctx, const char* Topic,
    UrlabRclTwistCallback Callback, void* User)
{
    ClearError();
    if (!Ctx)
    {
        return nullptr;
    }
    UrlabRclTwistSub* Sub = new UrlabRclTwistSub();
    Sub->Rec.Kind = ESubKind::Twist;
    Sub->Rec.Ctx = Ctx;
    Sub->Rec.CtrlCallback = nullptr;
    Sub->Rec.TwistCallback = Callback;
    Sub->Rec.User = User;
    geometry_msgs__msg__Twist__init(&Sub->Rec.TwistMsg);

    Sub->Rec.Sub = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t Options = rcl_subscription_get_default_options();
    const rosidl_message_type_support_t* Ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
    const rcl_ret_t Ret = rcl_subscription_init(&Sub->Rec.Sub, &Ctx->Node, Ts, Topic, &Options);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        geometry_msgs__msg__Twist__fini(&Sub->Rec.TwistMsg);
        delete Sub;
        return nullptr;
    }
    Ctx->Subs.push_back(&Sub->Rec);
    return Sub;
}

void UrlabRcl_DestroyTwistSub(UrlabRclTwistSub* Sub)
{
    if (!Sub)
    {
        return;
    }
    DetachSub(&Sub->Rec);
    rcl_subscription_fini(&Sub->Rec.Sub, &Sub->Rec.Ctx->Node);
    geometry_msgs__msg__Twist__fini(&Sub->Rec.TwistMsg);
    delete Sub;
}

int UrlabRcl_SpinSome(UrlabRclContext* Ctx, int64_t TimeoutNs)
{
    ClearError();
    if (!Ctx)
    {
        return -1;
    }
    const size_t N = Ctx->Subs.size();
    if (N == 0)
    {
        return 0;
    }

    rcl_wait_set_t WaitSet = rcl_get_zero_initialized_wait_set();
    rcl_ret_t Ret = rcl_wait_set_init(&WaitSet, N, 0, 0, 0, 0, 0, &Ctx->Context, Ctx->Allocator);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        return -static_cast<int>(Ret);
    }

    Ret = rcl_wait_set_clear(&WaitSet);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        rcl_wait_set_fini(&WaitSet);
        return -static_cast<int>(Ret);
    }
    for (FSubRecord* Rec : Ctx->Subs)
    {
        rcl_wait_set_add_subscription(&WaitSet, &Rec->Sub, nullptr);
    }

    Ret = rcl_wait(&WaitSet, TimeoutNs);
    if (Ret == RCL_RET_TIMEOUT)
    {
        rcl_wait_set_fini(&WaitSet);
        return 0;
    }
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        rcl_wait_set_fini(&WaitSet);
        return -static_cast<int>(Ret);
    }

    for (size_t i = 0; i < N; ++i)
    {
        if (WaitSet.subscriptions[i] == nullptr)
        {
            continue;
        }
        FSubRecord* Rec = Ctx->Subs[i];
        if (Rec->Kind == ESubKind::Ctrl)
        {
            rmw_message_info_t Info = rmw_get_zero_initialized_message_info();
            const rcl_ret_t Take = rcl_take(&Rec->Sub, &Rec->CtrlMsg, &Info, nullptr);
            if (Take == RCL_RET_OK && Rec->CtrlCallback)
            {
                Rec->CtrlCallback(Rec->CtrlMsg.data.data,
                    static_cast<int32_t>(Rec->CtrlMsg.data.size), Rec->User);
            }
        }
        else
        {
            rmw_message_info_t Info = rmw_get_zero_initialized_message_info();
            const rcl_ret_t Take = rcl_take(&Rec->Sub, &Rec->TwistMsg, &Info, nullptr);
            if (Take == RCL_RET_OK && Rec->TwistCallback)
            {
                const double Linear[3] = {
                    Rec->TwistMsg.linear.x, Rec->TwistMsg.linear.y, Rec->TwistMsg.linear.z};
                const double Angular[3] = {
                    Rec->TwistMsg.angular.x, Rec->TwistMsg.angular.y, Rec->TwistMsg.angular.z};
                Rec->TwistCallback(Linear, Angular, Rec->User);
            }
        }
    }

    rcl_wait_set_fini(&WaitSet);
    return 0;
}

// --- Zero-copy Clock -------------------------------------------------------

int UrlabRcl_ClockCanLoan(UrlabRclClockPub* Pub)
{
    if (!Pub)
    {
        return 0;
    }
    return rcl_publisher_can_loan_messages(&Pub->Pub) ? 1 : 0;
}

int UrlabRcl_PublishClockLoaned(UrlabRclClockPub* Pub, int64_t SimTimeNs)
{
    ClearError();
    if (!Pub)
    {
        return -1;
    }
    if (!rcl_publisher_can_loan_messages(&Pub->Pub))
    {
        return UrlabRcl_PublishClock(Pub, SimTimeNs);
    }

    const rosidl_message_type_support_t* Ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(rosgraph_msgs, msg, Clock);
    void* Loaned = nullptr;
    rcl_ret_t Ret = rcl_borrow_loaned_message(&Pub->Pub, Ts, &Loaned);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        return -static_cast<int>(Ret);
    }
    rosgraph_msgs__msg__Clock* Msg = static_cast<rosgraph_msgs__msg__Clock*>(Loaned);
    FillStamp(Msg->clock, SimTimeNs);
    Ret = rcl_publish_loaned_message(&Pub->Pub, Loaned, nullptr);
    if (Ret != RCL_RET_OK)
    {
        CaptureError();
        rcl_return_loaned_message_from_publisher(&Pub->Pub, Loaned);
        return -static_cast<int>(Ret);
    }
    return 0;
}

#endif  // URLAB_WITH_ROS2
