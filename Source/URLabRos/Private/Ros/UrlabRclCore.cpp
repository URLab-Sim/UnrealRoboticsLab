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
#include <rcl/service.h>
#include <rcl/wait.h>
#include <rmw/qos_profiles.h>

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <builtin_interfaces/msg/time.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/camera_info.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/wrench_stamped.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float64.h>
#include <shape_msgs/msg/solid_primitive.h>
#include <shape_msgs/msg/mesh.h>
#include <shape_msgs/msg/mesh_triangle.h>
#include <geometry_msgs/msg/point.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_msgs/msg/collision_object.h>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <nav_msgs/msg/odometry.h>
#include <rosgraph_msgs/msg/clock.h>
#include <std_srvs/srv/trigger.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/msg/detail/point_field__functions.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <octomap_msgs/msg/octomap.h>

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
	Twist,
	JointState
};

struct FSubRecord
{
	ESubKind Kind;
	rcl_subscription_t Sub;
	UrlabRclContext* Ctx;
	UrlabRclCtrlCallback CtrlCallback;
	UrlabRclTwistCallback TwistCallback;
	UrlabRclJointStateCallback JointStateCallback;
	void* User;
	std_msgs__msg__Float64MultiArray CtrlMsg;
	geometry_msgs__msg__Twist TwistMsg;
	sensor_msgs__msg__JointState JointStateMsg;
};

// Service record; SpinSome takes the request, runs the callback, sends the reply.
struct FSrvRecord
{
	rcl_service_t Srv;
	UrlabRclContext* Ctx;
	UrlabRclTriggerCallback Callback;
	void* User;
	std_srvs__srv__Trigger_Request Request;
	std_srvs__srv__Trigger_Response Response;
};
} // namespace

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
	// Services registered against this context, also served by UrlabRcl_SpinSome.
	std::vector<FSrvRecord*> Srvs;
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

struct UrlabRclStringPub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	std_msgs__msg__String Msg;
};

struct UrlabRclWrenchStampedPub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	geometry_msgs__msg__WrenchStamped Msg;
};

struct UrlabRclRangePub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	sensor_msgs__msg__Range Msg;
};

struct UrlabRclMagneticFieldPub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	sensor_msgs__msg__MagneticField Msg;
};

struct UrlabRclFloat64MultiArrayPub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	std_msgs__msg__Float64MultiArray Msg;
};

struct UrlabRclOdometryPub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	nav_msgs__msg__Odometry Msg;
};

struct UrlabRclPoseWithCovariancePub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	geometry_msgs__msg__PoseWithCovarianceStamped Msg;
};

struct UrlabRclCameraInfoPub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	sensor_msgs__msg__CameraInfo Msg;
};

struct UrlabRclBoolPub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	std_msgs__msg__Bool Msg;
};

struct UrlabRclFloat64Pub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	std_msgs__msg__Float64 Msg;
};

struct UrlabRclVector3Pub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	geometry_msgs__msg__Vector3 Msg;
};

struct UrlabRclPoseStampedPub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	geometry_msgs__msg__PoseStamped Msg;
};

struct UrlabRclCtrlSub
{
	FSubRecord Rec;
};

struct UrlabRclTwistSub
{
	FSubRecord Rec;
};

struct UrlabRclJointStateSub
{
	FSubRecord Rec;
};

struct UrlabRclTriggerService
{
	FSrvRecord Rec;
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

void DetachSrv(FSrvRecord* Rec)
{
	if (!Rec || !Rec->Ctx)
	{
		return;
	}
	std::vector<FSrvRecord*>& List = Rec->Ctx->Srvs;
	for (size_t i = 0; i < List.size(); ++i)
	{
		if (List[i] == Rec)
		{
			List.erase(List.begin() + i);
			break;
		}
	}
}
} // namespace

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
			else if (Rec->Kind == ESubKind::Twist)
			{
				geometry_msgs__msg__Twist__fini(&Rec->TwistMsg);
			}
			else
			{
				sensor_msgs__msg__JointState__fini(&Rec->JointStateMsg);
			}
			delete Rec;
		}
	}
	Ctx->Subs.clear();

	for (FSrvRecord* Rec : Ctx->Srvs)
	{
		if (Rec)
		{
			rcl_service_fini(&Rec->Srv, &Ctx->Node);
			std_srvs__srv__Trigger_Request__fini(&Rec->Request);
			std_srvs__srv__Trigger_Response__fini(&Rec->Response);
			delete Rec;
		}
	}
	Ctx->Srvs.clear();

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

// --- String publisher (latched robot_description) --------------------------

UrlabRclStringPub* UrlabRcl_CreateStringPub(UrlabRclContext* Ctx, const char* Topic)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclStringPub* Pub = new UrlabRclStringPub();
	Pub->Ctx = Ctx;
	std_msgs__msg__String__init(&Pub->Msg);

	// Latch the last spec so late-joining subscribers (rviz, MoveIt) receive
	// it without a re-publish, matching robot_state_publisher's QoS.
	rmw_qos_profile_t Qos = rmw_qos_profile_default;
	Qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
	Qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
	Qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
	Qos.depth = 1;

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, Qos))
	{
		std_msgs__msg__String__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishString(UrlabRclStringPub* Pub, const char* Text)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	SetString(Pub->Msg.data, Text);
	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyStringPub(UrlabRclStringPub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	std_msgs__msg__String__fini(&Pub->Msg);
	delete Pub;
}

// --- WrenchStamped ---------------------------------------------------------

UrlabRclWrenchStampedPub* UrlabRcl_CreateWrenchStampedPub(UrlabRclContext* Ctx,
	const char* Topic, const char* FrameId)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclWrenchStampedPub* Pub = new UrlabRclWrenchStampedPub();
	Pub->Ctx = Ctx;
	geometry_msgs__msg__WrenchStamped__init(&Pub->Msg);
	SetString(Pub->Msg.header.frame_id, FrameId);

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, WrenchStamped);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		geometry_msgs__msg__WrenchStamped__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishWrenchStamped(UrlabRclWrenchStampedPub* Pub,
	const double Force[3], const double Torque[3], int64_t SimTimeNs)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	FillStamp(Pub->Msg.header.stamp, SimTimeNs);
	Pub->Msg.wrench.force.x = Force ? Force[0] : 0.0;
	Pub->Msg.wrench.force.y = Force ? Force[1] : 0.0;
	Pub->Msg.wrench.force.z = Force ? Force[2] : 0.0;
	Pub->Msg.wrench.torque.x = Torque ? Torque[0] : 0.0;
	Pub->Msg.wrench.torque.y = Torque ? Torque[1] : 0.0;
	Pub->Msg.wrench.torque.z = Torque ? Torque[2] : 0.0;

	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyWrenchStampedPub(UrlabRclWrenchStampedPub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	geometry_msgs__msg__WrenchStamped__fini(&Pub->Msg);
	delete Pub;
}

// --- Range -----------------------------------------------------------------

UrlabRclRangePub* UrlabRcl_CreateRangePub(UrlabRclContext* Ctx, const char* Topic,
	const char* FrameId, uint8_t RadiationType, float FieldOfView, float MinRange,
	float MaxRange)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclRangePub* Pub = new UrlabRclRangePub();
	Pub->Ctx = Ctx;
	sensor_msgs__msg__Range__init(&Pub->Msg);
	SetString(Pub->Msg.header.frame_id, FrameId);
	// Constant fields; only the reading + stamp change per publish.
	Pub->Msg.radiation_type = RadiationType;
	Pub->Msg.field_of_view = FieldOfView;
	Pub->Msg.min_range = MinRange;
	Pub->Msg.max_range = MaxRange;

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		sensor_msgs__msg__Range__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishRange(UrlabRclRangePub* Pub, float Range, int64_t SimTimeNs)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	FillStamp(Pub->Msg.header.stamp, SimTimeNs);
	Pub->Msg.range = Range;

	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyRangePub(UrlabRclRangePub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	sensor_msgs__msg__Range__fini(&Pub->Msg);
	delete Pub;
}

// --- MagneticField ---------------------------------------------------------

UrlabRclMagneticFieldPub* UrlabRcl_CreateMagneticFieldPub(UrlabRclContext* Ctx,
	const char* Topic, const char* FrameId)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclMagneticFieldPub* Pub = new UrlabRclMagneticFieldPub();
	Pub->Ctx = Ctx;
	sensor_msgs__msg__MagneticField__init(&Pub->Msg);
	SetString(Pub->Msg.header.frame_id, FrameId);
	// Exact ground truth: leading covariance element 0 (not -1 "unknown").
	Pub->Msg.magnetic_field_covariance[0] = 0.0;

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		sensor_msgs__msg__MagneticField__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishMagneticField(UrlabRclMagneticFieldPub* Pub, const double Field[3],
	int64_t SimTimeNs)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	FillStamp(Pub->Msg.header.stamp, SimTimeNs);
	Pub->Msg.magnetic_field.x = Field ? Field[0] : 0.0;
	Pub->Msg.magnetic_field.y = Field ? Field[1] : 0.0;
	Pub->Msg.magnetic_field.z = Field ? Field[2] : 0.0;

	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyMagneticFieldPub(UrlabRclMagneticFieldPub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	sensor_msgs__msg__MagneticField__fini(&Pub->Msg);
	delete Pub;
}

// --- Float64MultiArray (total-coverage sensor fallback) --------------------

UrlabRclFloat64MultiArrayPub* UrlabRcl_CreateFloat64MultiArrayPub(UrlabRclContext* Ctx,
	const char* Topic)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclFloat64MultiArrayPub* Pub = new UrlabRclFloat64MultiArrayPub();
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

int UrlabRcl_PublishFloat64MultiArray(UrlabRclFloat64MultiArrayPub* Pub,
	const double* Values, int32_t Count)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	const int32_t N = Count > 0 ? Count : 0;
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

void UrlabRcl_DestroyFloat64MultiArrayPub(UrlabRclFloat64MultiArrayPub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	std_msgs__msg__Float64MultiArray__fini(&Pub->Msg);
	delete Pub;
}

// --- Odometry --------------------------------------------------------------

namespace
{
// Ground-truth pose/twist is exact; a small nonzero diagonal keeps EKF consumers
// (robot_localization) from rejecting the message while still reading as
// "essentially certain".
constexpr double GGroundTruthVariance = 1.0e-6;

void SetCovarianceDiagonal(double Cov[36], double Value)
{
	std::memset(Cov, 0, sizeof(double) * 36);
	for (int i = 0; i < 6; ++i)
	{
		Cov[i * 6 + i] = Value;
	}
}
} // namespace

UrlabRclOdometryPub* UrlabRcl_CreateOdometryPub(UrlabRclContext* Ctx, const char* Topic,
	const char* FrameId, const char* ChildFrameId)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclOdometryPub* Pub = new UrlabRclOdometryPub();
	Pub->Ctx = Ctx;
	nav_msgs__msg__Odometry__init(&Pub->Msg);
	SetString(Pub->Msg.header.frame_id, FrameId);
	SetString(Pub->Msg.child_frame_id, ChildFrameId);
	SetCovarianceDiagonal(Pub->Msg.pose.covariance, GGroundTruthVariance);
	SetCovarianceDiagonal(Pub->Msg.twist.covariance, GGroundTruthVariance);

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		nav_msgs__msg__Odometry__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishOdometry(UrlabRclOdometryPub* Pub, const double PositionXyz[3],
	const double OrientationXyzw[4], const double LinearBody[3],
	const double AngularBody[3], int64_t SimTimeNs)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	FillStamp(Pub->Msg.header.stamp, SimTimeNs);
	geometry_msgs__msg__Pose& P = Pub->Msg.pose.pose;
	P.position.x = PositionXyz ? PositionXyz[0] : 0.0;
	P.position.y = PositionXyz ? PositionXyz[1] : 0.0;
	P.position.z = PositionXyz ? PositionXyz[2] : 0.0;
	P.orientation.x = OrientationXyzw ? OrientationXyzw[0] : 0.0;
	P.orientation.y = OrientationXyzw ? OrientationXyzw[1] : 0.0;
	P.orientation.z = OrientationXyzw ? OrientationXyzw[2] : 0.0;
	P.orientation.w = OrientationXyzw ? OrientationXyzw[3] : 1.0;
	geometry_msgs__msg__Twist& T = Pub->Msg.twist.twist;
	T.linear.x = LinearBody ? LinearBody[0] : 0.0;
	T.linear.y = LinearBody ? LinearBody[1] : 0.0;
	T.linear.z = LinearBody ? LinearBody[2] : 0.0;
	T.angular.x = AngularBody ? AngularBody[0] : 0.0;
	T.angular.y = AngularBody ? AngularBody[1] : 0.0;
	T.angular.z = AngularBody ? AngularBody[2] : 0.0;

	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyOdometryPub(UrlabRclOdometryPub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	nav_msgs__msg__Odometry__fini(&Pub->Msg);
	delete Pub;
}

// --- PoseWithCovarianceStamped ---------------------------------------------

UrlabRclPoseWithCovariancePub* UrlabRcl_CreatePoseWithCovariancePub(UrlabRclContext* Ctx,
	const char* Topic, const char* FrameId)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclPoseWithCovariancePub* Pub = new UrlabRclPoseWithCovariancePub();
	Pub->Ctx = Ctx;
	geometry_msgs__msg__PoseWithCovarianceStamped__init(&Pub->Msg);
	SetString(Pub->Msg.header.frame_id, FrameId);
	SetCovarianceDiagonal(Pub->Msg.pose.covariance, GGroundTruthVariance);

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseWithCovarianceStamped);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		geometry_msgs__msg__PoseWithCovarianceStamped__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishPoseWithCovariance(UrlabRclPoseWithCovariancePub* Pub,
	const double PositionXyz[3], const double OrientationXyzw[4], int64_t SimTimeNs)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	FillStamp(Pub->Msg.header.stamp, SimTimeNs);
	geometry_msgs__msg__Pose& P = Pub->Msg.pose.pose;
	P.position.x = PositionXyz ? PositionXyz[0] : 0.0;
	P.position.y = PositionXyz ? PositionXyz[1] : 0.0;
	P.position.z = PositionXyz ? PositionXyz[2] : 0.0;
	P.orientation.x = OrientationXyzw ? OrientationXyzw[0] : 0.0;
	P.orientation.y = OrientationXyzw ? OrientationXyzw[1] : 0.0;
	P.orientation.z = OrientationXyzw ? OrientationXyzw[2] : 0.0;
	P.orientation.w = OrientationXyzw ? OrientationXyzw[3] : 1.0;

	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyPoseWithCovariancePub(UrlabRclPoseWithCovariancePub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	geometry_msgs__msg__PoseWithCovarianceStamped__fini(&Pub->Msg);
	delete Pub;
}

// --- CameraInfo ------------------------------------------------------------

UrlabRclCameraInfoPub* UrlabRcl_CreateCameraInfoPub(UrlabRclContext* Ctx, const char* Topic,
	const char* FrameId, int32_t Width, int32_t Height, const double K9[9])
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclCameraInfoPub* Pub = new UrlabRclCameraInfoPub();
	Pub->Ctx = Ctx;
	sensor_msgs__msg__CameraInfo__init(&Pub->Msg);
	SetString(Pub->Msg.header.frame_id, FrameId);
	Pub->Msg.width = static_cast<uint32_t>(Width > 0 ? Width : 0);
	Pub->Msg.height = static_cast<uint32_t>(Height > 0 ? Height : 0);

	// Zero-distortion pinhole: plumb_bob with five zero coefficients.
	SetString(Pub->Msg.distortion_model, "plumb_bob");
	rosidl_runtime_c__double__Sequence__init(&Pub->Msg.d, 5);
	for (int i = 0; i < 5; ++i)
	{
		Pub->Msg.d.data[i] = 0.0;
	}

	// K (row-major 3x3) straight from the caller; R = identity; P = K with a zero
	// translation column (monocular, no baseline).
	for (int i = 0; i < 9; ++i)
	{
		Pub->Msg.k[i] = K9 ? K9[i] : 0.0;
	}
	for (int i = 0; i < 9; ++i)
	{
		Pub->Msg.r[i] = 0.0;
	}
	Pub->Msg.r[0] = Pub->Msg.r[4] = Pub->Msg.r[8] = 1.0;
	for (int i = 0; i < 12; ++i)
	{
		Pub->Msg.p[i] = 0.0;
	}
	Pub->Msg.p[0] = Pub->Msg.k[0]; // fx
	Pub->Msg.p[2] = Pub->Msg.k[2]; // cx
	Pub->Msg.p[5] = Pub->Msg.k[4]; // fy
	Pub->Msg.p[6] = Pub->Msg.k[5]; // cy
	Pub->Msg.p[10] = 1.0;

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CameraInfo);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		sensor_msgs__msg__CameraInfo__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishCameraInfo(UrlabRclCameraInfoPub* Pub, int64_t SimTimeNs)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	FillStamp(Pub->Msg.header.stamp, SimTimeNs);
	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyCameraInfoPub(UrlabRclCameraInfoPub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	sensor_msgs__msg__CameraInfo__fini(&Pub->Msg);
	delete Pub;
}

// --- Bool (typed user channel) ---------------------------------------------

UrlabRclBoolPub* UrlabRcl_CreateBoolPub(UrlabRclContext* Ctx, const char* Topic)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclBoolPub* Pub = new UrlabRclBoolPub();
	Pub->Ctx = Ctx;
	std_msgs__msg__Bool__init(&Pub->Msg);

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		std_msgs__msg__Bool__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishBool(UrlabRclBoolPub* Pub, int32_t bValue)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	Pub->Msg.data = bValue != 0;
	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyBoolPub(UrlabRclBoolPub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	std_msgs__msg__Bool__fini(&Pub->Msg);
	delete Pub;
}

// --- Float64 (typed user channel) ------------------------------------------

UrlabRclFloat64Pub* UrlabRcl_CreateFloat64Pub(UrlabRclContext* Ctx, const char* Topic)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclFloat64Pub* Pub = new UrlabRclFloat64Pub();
	Pub->Ctx = Ctx;
	std_msgs__msg__Float64__init(&Pub->Msg);

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		std_msgs__msg__Float64__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishFloat64(UrlabRclFloat64Pub* Pub, double Value)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	Pub->Msg.data = Value;
	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyFloat64Pub(UrlabRclFloat64Pub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	std_msgs__msg__Float64__fini(&Pub->Msg);
	delete Pub;
}

// --- Vector3 (typed user channel) ------------------------------------------

UrlabRclVector3Pub* UrlabRcl_CreateVector3Pub(UrlabRclContext* Ctx, const char* Topic)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclVector3Pub* Pub = new UrlabRclVector3Pub();
	Pub->Ctx = Ctx;
	geometry_msgs__msg__Vector3__init(&Pub->Msg);

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		geometry_msgs__msg__Vector3__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishVector3(UrlabRclVector3Pub* Pub, const double Xyz[3])
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	Pub->Msg.x = Xyz ? Xyz[0] : 0.0;
	Pub->Msg.y = Xyz ? Xyz[1] : 0.0;
	Pub->Msg.z = Xyz ? Xyz[2] : 0.0;
	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyVector3Pub(UrlabRclVector3Pub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	geometry_msgs__msg__Vector3__fini(&Pub->Msg);
	delete Pub;
}

// --- PoseStamped (typed user channel) --------------------------------------

UrlabRclPoseStampedPub* UrlabRcl_CreatePoseStampedPub(UrlabRclContext* Ctx,
	const char* Topic, const char* FrameId)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclPoseStampedPub* Pub = new UrlabRclPoseStampedPub();
	Pub->Ctx = Ctx;
	geometry_msgs__msg__PoseStamped__init(&Pub->Msg);
	SetString(Pub->Msg.header.frame_id, FrameId);

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		geometry_msgs__msg__PoseStamped__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishPoseStamped(UrlabRclPoseStampedPub* Pub, const double PositionXyz[3],
	const double OrientationXyzw[4], int64_t SimTimeNs)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	FillStamp(Pub->Msg.header.stamp, SimTimeNs);
	geometry_msgs__msg__Pose& P = Pub->Msg.pose;
	P.position.x = PositionXyz ? PositionXyz[0] : 0.0;
	P.position.y = PositionXyz ? PositionXyz[1] : 0.0;
	P.position.z = PositionXyz ? PositionXyz[2] : 0.0;
	P.orientation.x = OrientationXyzw ? OrientationXyzw[0] : 0.0;
	P.orientation.y = OrientationXyzw ? OrientationXyzw[1] : 0.0;
	P.orientation.z = OrientationXyzw ? OrientationXyzw[2] : 0.0;
	P.orientation.w = OrientationXyzw ? OrientationXyzw[3] : 1.0;

	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyPoseStampedPub(UrlabRclPoseStampedPub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	geometry_msgs__msg__PoseStamped__fini(&Pub->Msg);
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

UrlabRclJointStateSub* UrlabRcl_CreateJointStateSub(UrlabRclContext* Ctx, const char* Topic,
	UrlabRclJointStateCallback Callback, void* User)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclJointStateSub* Sub = new UrlabRclJointStateSub();
	Sub->Rec.Kind = ESubKind::JointState;
	Sub->Rec.Ctx = Ctx;
	Sub->Rec.CtrlCallback = nullptr;
	Sub->Rec.TwistCallback = nullptr;
	Sub->Rec.JointStateCallback = Callback;
	Sub->Rec.User = User;
	sensor_msgs__msg__JointState__init(&Sub->Rec.JointStateMsg);

	Sub->Rec.Sub = rcl_get_zero_initialized_subscription();
	rcl_subscription_options_t Options = rcl_subscription_get_default_options();
	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);
	const rcl_ret_t Ret = rcl_subscription_init(&Sub->Rec.Sub, &Ctx->Node, Ts, Topic, &Options);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		sensor_msgs__msg__JointState__fini(&Sub->Rec.JointStateMsg);
		delete Sub;
		return nullptr;
	}
	Ctx->Subs.push_back(&Sub->Rec);
	return Sub;
}

void UrlabRcl_DestroyJointStateSub(UrlabRclJointStateSub* Sub)
{
	if (!Sub)
	{
		return;
	}
	DetachSub(&Sub->Rec);
	rcl_subscription_fini(&Sub->Rec.Sub, &Sub->Rec.Ctx->Node);
	sensor_msgs__msg__JointState__fini(&Sub->Rec.JointStateMsg);
	delete Sub;
}

// --- Services --------------------------------------------------------------

UrlabRclTriggerService* UrlabRcl_CreateTriggerService(UrlabRclContext* Ctx,
	const char* ServiceName, UrlabRclTriggerCallback Callback, void* User)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclTriggerService* Srv = new UrlabRclTriggerService();
	Srv->Rec.Ctx = Ctx;
	Srv->Rec.Callback = Callback;
	Srv->Rec.User = User;
	std_srvs__srv__Trigger_Request__init(&Srv->Rec.Request);
	std_srvs__srv__Trigger_Response__init(&Srv->Rec.Response);

	Srv->Rec.Srv = rcl_get_zero_initialized_service();
	rcl_service_options_t Options = rcl_service_get_default_options();
	const rosidl_service_type_support_t* Ts =
		ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);
	const rcl_ret_t Ret = rcl_service_init(&Srv->Rec.Srv, &Ctx->Node, Ts, ServiceName, &Options);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		std_srvs__srv__Trigger_Request__fini(&Srv->Rec.Request);
		std_srvs__srv__Trigger_Response__fini(&Srv->Rec.Response);
		delete Srv;
		return nullptr;
	}
	Ctx->Srvs.push_back(&Srv->Rec);
	return Srv;
}

void UrlabRcl_DestroyTriggerService(UrlabRclTriggerService* Srv)
{
	if (!Srv)
	{
		return;
	}
	DetachSrv(&Srv->Rec);
	rcl_service_fini(&Srv->Rec.Srv, &Srv->Rec.Ctx->Node);
	std_srvs__srv__Trigger_Request__fini(&Srv->Rec.Request);
	std_srvs__srv__Trigger_Response__fini(&Srv->Rec.Response);
	delete Srv;
}

int UrlabRcl_SpinSome(UrlabRclContext* Ctx, int64_t TimeoutNs)
{
	ClearError();
	if (!Ctx)
	{
		return -1;
	}
	const size_t NSubs = Ctx->Subs.size();
	const size_t NSrvs = Ctx->Srvs.size();
	if (NSubs == 0 && NSrvs == 0)
	{
		return 0;
	}

	rcl_wait_set_t WaitSet = rcl_get_zero_initialized_wait_set();
	rcl_ret_t Ret = rcl_wait_set_init(&WaitSet, NSubs, 0, 0, 0, NSrvs, 0,
		&Ctx->Context, Ctx->Allocator);
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
	for (FSrvRecord* Rec : Ctx->Srvs)
	{
		rcl_wait_set_add_service(&WaitSet, &Rec->Srv, nullptr);
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

	for (size_t i = 0; i < NSubs; ++i)
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
		else if (Rec->Kind == ESubKind::Twist)
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
		else // ESubKind::JointState
		{
			rmw_message_info_t Info = rmw_get_zero_initialized_message_info();
			const rcl_ret_t Take = rcl_take(&Rec->Sub, &Rec->JointStateMsg, &Info, nullptr);
			if (Take == RCL_RET_OK && Rec->JointStateCallback)
			{
				const sensor_msgs__msg__JointState& Msg = Rec->JointStateMsg;
				const size_t Count = Msg.name.size < Msg.position.size
									   ? Msg.name.size
									   : Msg.position.size;
				std::vector<const char*> Names(Count);
				for (size_t j = 0; j < Count; ++j)
				{
					Names[j] = Msg.name.data[j].data ? Msg.name.data[j].data : "";
				}
				Rec->JointStateCallback(Count > 0 ? Names.data() : nullptr,
					Count > 0 ? Msg.position.data : nullptr,
					static_cast<int32_t>(Count), Rec->User);
			}
		}
	}

	for (size_t i = 0; i < NSrvs; ++i)
	{
		if (WaitSet.services[i] == nullptr)
		{
			continue;
		}
		FSrvRecord* Rec = Ctx->Srvs[i];
		rmw_request_id_t Header;
		std::memset(&Header, 0, sizeof(Header));
		const rcl_ret_t Take = rcl_take_request(&Rec->Srv, &Header, &Rec->Request);
		if (Take != RCL_RET_OK)
		{
			continue;
		}
		int32_t bSuccess = 0;
		char MessageBuf[512] = {0};
		if (Rec->Callback)
		{
			Rec->Callback(Rec->User, &bSuccess, MessageBuf, static_cast<int32_t>(sizeof(MessageBuf)));
		}
		MessageBuf[sizeof(MessageBuf) - 1] = '\0';
		Rec->Response.success = bSuccess != 0;
		SetString(Rec->Response.message, MessageBuf);
		const rcl_ret_t Send = rcl_send_response(&Rec->Srv, &Header, &Rec->Response);
		if (Send != RCL_RET_OK)
		{
			CaptureError();
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

// --- moveit_msgs/PlanningScene --------------------------------------------
struct UrlabRclPlanningScenePub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	moveit_msgs__msg__PlanningScene Msg;
};

struct UrlabRclPointCloud2Pub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	sensor_msgs__msg__PointCloud2 Msg;
	int32_t MaxPoints;
};

struct UrlabRclOccupancyGridPub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	nav_msgs__msg__OccupancyGrid Msg;
};

struct UrlabRclOctomapPub
{
	UrlabRclContext* Ctx;
	rcl_publisher_t Pub;
	octomap_msgs__msg__Octomap Msg;
};

UrlabRclPlanningScenePub* UrlabRcl_CreatePlanningScenePub(UrlabRclContext* Ctx,
	const char* Topic, const char* FrameId, const char** Ids,
	const int32_t* PrimTypes, const double* Dims, const int32_t* MeshVertCounts,
	const double* MeshVerts, const int32_t* MeshTriCounts, const int32_t* MeshTris,
	int32_t Count)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclPlanningScenePub* Pub = new UrlabRclPlanningScenePub();
	Pub->Ctx = Ctx;
	moveit_msgs__msg__PlanningScene__init(&Pub->Msg);
	Pub->Msg.is_diff = true;

	const int32_t N = Count > 0 ? Count : 0;
	// Running offsets into the flattened mesh arrays (advanced past every object,
	// primitive or mesh, so a primitive contributes 0 and mesh objects stay aligned).
	int64_t VertOff = 0;
	int64_t TriOff = 0;
	moveit_msgs__msg__CollisionObject__Sequence__init(&Pub->Msg.world.collision_objects, N);
	for (int32_t i = 0; i < N; ++i)
	{
		moveit_msgs__msg__CollisionObject* CO = &Pub->Msg.world.collision_objects.data[i];
		SetString(CO->header.frame_id, FrameId ? FrameId : "world");
		SetString(CO->id, (Ids && Ids[i]) ? Ids[i] : "");
		CO->operation = 0;            // ADD
		CO->pose.orientation.w = 1.0; // object frame; placement filled per-publish

		const uint8_t Type = PrimTypes ? (uint8_t)PrimTypes[i] : 1;
		const int32_t VertCount = MeshVertCounts ? MeshVertCounts[i] : 0;
		const int32_t TriCount = MeshTriCounts ? MeshTriCounts[i] : 0;

		if (Type == 4) // MESH
		{
			shape_msgs__msg__Mesh__Sequence__init(&CO->meshes, 1);
			shape_msgs__msg__Mesh* Me = &CO->meshes.data[0];
			geometry_msgs__msg__Point__Sequence__init(&Me->vertices, VertCount);
			for (int32_t v = 0; v < VertCount; ++v)
			{
				const double* Vp = &MeshVerts[(VertOff + v) * 3];
				Me->vertices.data[v].x = Vp[0];
				Me->vertices.data[v].y = Vp[1];
				Me->vertices.data[v].z = Vp[2];
			}
			shape_msgs__msg__MeshTriangle__Sequence__init(&Me->triangles, TriCount);
			for (int32_t t = 0; t < TriCount; ++t)
			{
				const int32_t* Tp = &MeshTris[(TriOff + t) * 3];
				Me->triangles.data[t].vertex_indices[0] = (uint32_t)Tp[0];
				Me->triangles.data[t].vertex_indices[1] = (uint32_t)Tp[1];
				Me->triangles.data[t].vertex_indices[2] = (uint32_t)Tp[2];
			}
			geometry_msgs__msg__Pose__Sequence__init(&CO->mesh_poses, 1);
			CO->mesh_poses.data[0].orientation.w = 1.0; // identity vs the object pose
		}
		else
		{
			shape_msgs__msg__SolidPrimitive__Sequence__init(&CO->primitives, 1);
			shape_msgs__msg__SolidPrimitive* P = &CO->primitives.data[0];
			P->type = Type;
			const int DimN = (Type == 1) ? 3 : (Type == 3 ? 2 : 1); // BOX 3, CYL 2, SPH 1
			rosidl_runtime_c__double__Sequence__init(&P->dimensions, DimN);
			for (int k = 0; k < DimN; ++k)
			{
				P->dimensions.data[k] = Dims ? Dims[i * 3 + k] : 0.0;
			}

			geometry_msgs__msg__Pose__Sequence__init(&CO->primitive_poses, 1);
			CO->primitive_poses.data[0].orientation.w = 1.0; // identity vs the object pose
		}
		VertOff += VertCount;
		TriOff += TriCount;
	}

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(moveit_msgs, msg, PlanningScene);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		moveit_msgs__msg__PlanningScene__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishPlanningScene(UrlabRclPlanningScenePub* Pub,
	const double* Poses, int32_t Count, int64_t SimTimeNs)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	const int32_t N = static_cast<int32_t>(Pub->Msg.world.collision_objects.size);
	const int32_t M = Count < N ? Count : N;
	for (int32_t i = 0; i < M; ++i)
	{
		moveit_msgs__msg__CollisionObject* CO = &Pub->Msg.world.collision_objects.data[i];
		FillStamp(CO->header.stamp, SimTimeNs);
		CO->operation = 0; // ADD (re-add replaces, keeping the scene current)
		const double* P = &Poses[i * 7];
		CO->pose.position.x = P[0];
		CO->pose.position.y = P[1];
		CO->pose.position.z = P[2];
		CO->pose.orientation.x = P[3];
		CO->pose.orientation.y = P[4];
		CO->pose.orientation.z = P[5];
		CO->pose.orientation.w = P[6];
	}
	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyPlanningScenePub(UrlabRclPlanningScenePub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	moveit_msgs__msg__PlanningScene__fini(&Pub->Msg);
	delete Pub;
}

// --- PointCloud2 ----------------------------------------------------------

namespace
{
constexpr int32_t GDefaultMaxPoints = 200'000;

void InitPointField(sensor_msgs__msg__PointField& F, const char* Name,
	uint32_t Offset, uint8_t DataType)
{
	SetString(F.name, Name);
	F.offset = Offset;
	F.datatype = DataType;
	F.count = 1;
}
} // namespace

UrlabRclPointCloud2Pub* UrlabRcl_CreatePointCloud2Pub(UrlabRclContext* Ctx,
	const char* Topic, const char* FrameId, int32_t MaxPoints)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclPointCloud2Pub* Pub = new UrlabRclPointCloud2Pub();
	Pub->Ctx = Ctx;
	Pub->MaxPoints = MaxPoints > 0 ? MaxPoints : GDefaultMaxPoints;
	sensor_msgs__msg__PointCloud2__init(&Pub->Msg);
	SetString(Pub->Msg.header.frame_id, FrameId);
	Pub->Msg.height = 1;
	Pub->Msg.is_bigendian = false;
	Pub->Msg.is_dense = true;
	Pub->Msg.point_step = 12; // 3 * float32
	sensor_msgs__msg__PointField__Sequence__init(&Pub->Msg.fields, 3);
	InitPointField(Pub->Msg.fields.data[0], "x", 0, 7);
	InitPointField(Pub->Msg.fields.data[1], "y", 4, 7);
	InitPointField(Pub->Msg.fields.data[2], "z", 8, 7);
	rosidl_runtime_c__uint8__Sequence__init(&Pub->Msg.data,
		static_cast<size_t>(Pub->MaxPoints) * 12);

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		sensor_msgs__msg__PointCloud2__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishPointCloud2(UrlabRclPointCloud2Pub* Pub,
	const float* Points, int32_t N, int64_t SimTimeNs)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	const int32_t Count = N > 0 ? N : 0;
	const int32_t Capped = Count > Pub->MaxPoints ? Pub->MaxPoints : Count;
	const size_t ByteSize = static_cast<size_t>(Capped) * 12;
	FillStamp(Pub->Msg.header.stamp, SimTimeNs);
	Pub->Msg.width = static_cast<uint32_t>(Capped);
	Pub->Msg.row_step = static_cast<uint32_t>(ByteSize);
	if (Pub->Msg.data.capacity < ByteSize)
	{
		rosidl_runtime_c__uint8__Sequence__fini(&Pub->Msg.data);
		rosidl_runtime_c__uint8__Sequence__init(&Pub->Msg.data, ByteSize);
	}
	if (Points && Capped > 0)
	{
		std::memcpy(Pub->Msg.data.data, Points, ByteSize);
		Pub->Msg.data.size = ByteSize;
	}
	else
	{
		Pub->Msg.data.size = 0;
		Pub->Msg.width = 0;
		Pub->Msg.row_step = 0;
	}
	const rcl_ret_t Ret = rcl_publish(&Pub->Pub, &Pub->Msg, nullptr);
	if (Ret != RCL_RET_OK)
	{
		CaptureError();
		return -static_cast<int>(Ret);
	}
	return 0;
}

void UrlabRcl_DestroyPointCloud2Pub(UrlabRclPointCloud2Pub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	sensor_msgs__msg__PointCloud2__fini(&Pub->Msg);
	delete Pub;
}

// --- OccupancyGrid --------------------------------------------------------

UrlabRclOccupancyGridPub* UrlabRcl_CreateOccupancyGridPub(UrlabRclContext* Ctx,
	const char* Topic, const char* FrameId, double Resolution,
	int32_t Width, int32_t Height, double OriginX, double OriginY)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclOccupancyGridPub* Pub = new UrlabRclOccupancyGridPub();
	Pub->Ctx = Ctx;
	nav_msgs__msg__OccupancyGrid__init(&Pub->Msg);
	SetString(Pub->Msg.header.frame_id, FrameId);
	Pub->Msg.info.resolution = static_cast<float>(Resolution);
	Pub->Msg.info.width = static_cast<uint32_t>(Width > 0 ? Width : 0);
	Pub->Msg.info.height = static_cast<uint32_t>(Height > 0 ? Height : 0);
	Pub->Msg.info.origin.position.x = OriginX;
	Pub->Msg.info.origin.position.y = OriginY;
	Pub->Msg.info.origin.position.z = 0.0;
	Pub->Msg.info.origin.orientation.w = 1.0;
	const size_t CellCount = static_cast<size_t>(Pub->Msg.info.width) *
		static_cast<size_t>(Pub->Msg.info.height);
	rosidl_runtime_c__int8__Sequence__init(&Pub->Msg.data, CellCount);
	if (CellCount > 0)
	{
		std::memset(Pub->Msg.data.data, 0xFF, CellCount);
		Pub->Msg.data.size = CellCount;
	}

	rmw_qos_profile_t Qos = rmw_qos_profile_default;
	Qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
	Qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
	Qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
	Qos.depth = 1;

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, OccupancyGrid);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, Qos))
	{
		nav_msgs__msg__OccupancyGrid__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishOccupancyGrid(UrlabRclOccupancyGridPub* Pub,
	const int8_t* Data, int64_t SimTimeNs)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	FillStamp(Pub->Msg.header.stamp, SimTimeNs);
	const size_t CellCount = static_cast<size_t>(Pub->Msg.info.width) *
		static_cast<size_t>(Pub->Msg.info.height);
	if (Data && CellCount > 0)
	{
		std::memcpy(Pub->Msg.data.data, Data, CellCount);
		Pub->Msg.data.size = CellCount;
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

void UrlabRcl_DestroyOccupancyGridPub(UrlabRclOccupancyGridPub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	nav_msgs__msg__OccupancyGrid__fini(&Pub->Msg);
	delete Pub;
}

// --- Octomap --------------------------------------------------------------

UrlabRclOctomapPub* UrlabRcl_CreateOctomapPub(UrlabRclContext* Ctx,
	const char* Topic, const char* FrameId, double Resolution)
{
	ClearError();
	if (!Ctx)
	{
		return nullptr;
	}
	UrlabRclOctomapPub* Pub = new UrlabRclOctomapPub();
	Pub->Ctx = Ctx;
	octomap_msgs__msg__Octomap__init(&Pub->Msg);
	SetString(Pub->Msg.header.frame_id, FrameId);
	Pub->Msg.binary = true;
	SetString(Pub->Msg.id, "OcTree");
	Pub->Msg.resolution = Resolution;

	const rosidl_message_type_support_t* Ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(octomap_msgs, msg, Octomap);
	if (!InitPublisher(Ctx, Pub->Pub, Ts, Topic, rmw_qos_profile_default))
	{
		octomap_msgs__msg__Octomap__fini(&Pub->Msg);
		delete Pub;
		return nullptr;
	}
	return Pub;
}

int UrlabRcl_PublishOctomap(UrlabRclOctomapPub* Pub,
	const uint8_t* Data, int32_t Size, int64_t SimTimeNs)
{
	ClearError();
	if (!Pub)
	{
		return -1;
	}
	FillStamp(Pub->Msg.header.stamp, SimTimeNs);
	const size_t S = static_cast<size_t>(Size > 0 ? Size : 0);
	if (Pub->Msg.data.capacity < S)
	{
		rosidl_runtime_c__int8__Sequence__fini(&Pub->Msg.data);
		rosidl_runtime_c__int8__Sequence__init(&Pub->Msg.data, S);
	}
	if (Data && S > 0)
	{
		std::memcpy(Pub->Msg.data.data, Data, S);
		Pub->Msg.data.size = S;
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

void UrlabRcl_DestroyOctomapPub(UrlabRclOctomapPub* Pub)
{
	if (!Pub)
	{
		return;
	}
	rcl_publisher_fini(&Pub->Pub, &Pub->Ctx->Node);
	octomap_msgs__msg__Octomap__fini(&Pub->Msg);
	delete Pub;
}

#endif // URLAB_WITH_ROS2
