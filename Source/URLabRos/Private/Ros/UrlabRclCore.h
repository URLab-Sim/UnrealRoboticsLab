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
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

// UrlabRclCore is the single UE-agnostic seam that owns every rcl / rosidl call
// in the project. This header exposes only opaque handles and POD/C-friendly
// free functions: no UE types, no rcl types, no rosidl types, no STL in any
// signature. It is includable from UE module code and from the standalone ROS
// workspace harness alike; its only include is <stdint.h>.
//
// Return convention: functions returning int use 0 = ok, negative = a mapped
// rcl_ret_t. Create* functions return a handle pointer, null on failure.
// UrlabRcl_LastError() returns the rcutils error string from the most recent
// failed call, for logging. Every Create* has a matching Destroy that takes its
// handle; all destroy/shutdown functions are null-safe.
//
// Time is sim time in nanoseconds (int64_t); the core splits it into sec/nsec.
// Quaternions are xyzw ordered.
//
// Threading: the core takes no locks. Create and destroy all handles from one
// thread, serialize calls per handle, and note that UrlabRcl_SpinSome runs
// subscription callbacks on its caller's thread.

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	// Opaque handles. Definitions live only in UrlabRclCore.cpp.
	struct UrlabRclContext; // rcl init options + context + one node
	struct UrlabRclJointStatePub;
	struct UrlabRclImuPub;
	struct UrlabRclTfPub;
	struct UrlabRclTwistStampedPub;
	struct UrlabRclClockPub;
	struct UrlabRclImagePub;
	struct UrlabRclCtrlPub;
	struct UrlabRclStringPub;
	struct UrlabRclWrenchStampedPub;
	struct UrlabRclRangePub;
	struct UrlabRclMagneticFieldPub;
	struct UrlabRclFloat64MultiArrayPub;
	struct UrlabRclOdometryPub;
	struct UrlabRclPoseWithCovariancePub;
	struct UrlabRclCameraInfoPub;
	struct UrlabRclBoolPub;
	struct UrlabRclFloat64Pub;
	struct UrlabRclVector3Pub;
	struct UrlabRclPoseStampedPub;
	struct UrlabRclPlanningScenePub;
	struct UrlabRclPointCloud2Pub;
	struct UrlabRclOccupancyGridPub;
	struct UrlabRclOctomapPub;
	struct UrlabRclCtrlSub;
	struct UrlabRclTwistSub;
	struct UrlabRclJointStateSub;
	struct UrlabRclTriggerService;

	// --- Context ---------------------------------------------------------------
	// DomainId -1 = use the ROS_DOMAIN_ID environment variable. Returns null on
	// failure (query UrlabRcl_LastError for the reason).
	struct UrlabRclContext* UrlabRcl_Init(const char* NodeName,
		const char* NodeNamespace, int32_t DomainId);
	void UrlabRcl_Shutdown(struct UrlabRclContext* Ctx); // fini node/context, reverse order
	const char* UrlabRcl_DistroName();                   // compile-time pin, for the facts file
	const char* UrlabRcl_LastError();

	// --- Publishers ------------------------------------------------------------
	// Joint-name arrays are copied at create time (sized to the model); message
	// structs are preallocated per handle.
	struct UrlabRclJointStatePub* UrlabRcl_CreateJointStatePub(struct UrlabRclContext* Ctx,
		const char* Topic, const char** JointNames, int32_t JointCount);
	int UrlabRcl_PublishJointState(struct UrlabRclJointStatePub* Pub,
		const double* Positions, const double* Velocities, const double* Efforts,
		int32_t Count, int64_t SimTimeNs); // Velocities/Efforts may be null
	void UrlabRcl_DestroyJointStatePub(struct UrlabRclJointStatePub* Pub);

	struct UrlabRclImuPub* UrlabRcl_CreateImuPub(struct UrlabRclContext* Ctx,
		const char* Topic, const char* FrameId);
	int UrlabRcl_PublishImu(struct UrlabRclImuPub* Pub, const double AngularVel[3],
		const double LinearAccel[3], const double OrientationXyzw[4],
		int64_t SimTimeNs); // any array may be null (unpaired gyro)
	void UrlabRcl_DestroyImuPub(struct UrlabRclImuPub* Pub);

	struct UrlabRclTfPub* UrlabRcl_CreateTfPub(struct UrlabRclContext* Ctx, int32_t bStatic);
	// bStatic != 0: /tf_static with transient-local QoS; else /tf
	int UrlabRcl_PublishTf(struct UrlabRclTfPub* Pub, const char** ParentFrameIds,
		const char** ChildFrameIds, const double* TranslationsXyz /* 3*Count */,
		const double* RotationsXyzw /* 4*Count */, int32_t Count,
		int64_t SimTimeNs);
	void UrlabRcl_DestroyTfPub(struct UrlabRclTfPub* Pub);

	// moveit_msgs/PlanningScene (is_diff) on /planning_scene: the world's non-robot
	// collision geometry as CollisionObjects. Create fixes the object set (ids +
	// shapes); Publish updates their world poses each step. PrimTypes are
	// shape_msgs/SolidPrimitive.type constants (1=BOX, 2=SPHERE, 3=CYLINDER) with Dims
	// 3 per object (BOX: full extents x,y,z; SPHERE: [radius,_,_]; CYLINDER:
	// [height, radius, _]), or 4=MESH which reads geometry from the mesh arrays
	// instead of Dims. Mesh arrays are flattened across all objects: for object i,
	// MeshVertCounts[i] vertices (3 doubles each) and MeshTriCounts[i] triangles
	// (3 int indices each), concatenated in object order (0 for primitive objects).
	struct UrlabRclPlanningScenePub* UrlabRcl_CreatePlanningScenePub(
		struct UrlabRclContext* Ctx, const char* Topic, const char* FrameId,
		const char** Ids, const int32_t* PrimTypes, const double* Dims,
		const int32_t* MeshVertCounts, const double* MeshVerts,
		const int32_t* MeshTriCounts, const int32_t* MeshTris, int32_t Count);
	int UrlabRcl_PublishPlanningScene(struct UrlabRclPlanningScenePub* Pub,
		const double* Poses /* 7*Count: px,py,pz,qx,qy,qz,qw */, int32_t Count,
		int64_t SimTimeNs);
	void UrlabRcl_DestroyPlanningScenePub(struct UrlabRclPlanningScenePub* Pub);

	// sensor_msgs/PointCloud2, unordered (height=1), frame_id set at create.
	// max_points caps the pre-allocated data array; 0 uses a default (200 kpts).
	struct UrlabRclPointCloud2Pub* UrlabRcl_CreatePointCloud2Pub(struct UrlabRclContext* Ctx,
		const char* Topic, const char* FrameId, int32_t MaxPoints);
	int UrlabRcl_PublishPointCloud2(struct UrlabRclPointCloud2Pub* Pub,
		const float* Points, int32_t N, int64_t SimTimeNs);
	void UrlabRcl_DestroyPointCloud2Pub(struct UrlabRclPointCloud2Pub* Pub);

	// nav_msgs/OccupancyGrid, latched (transient-local), fixed grid at create.
	// OriginX/Y are the world-frame coordinate of the grid's bottom-left cell centre.
	struct UrlabRclOccupancyGridPub* UrlabRcl_CreateOccupancyGridPub(struct UrlabRclContext* Ctx,
		const char* Topic, const char* FrameId, double Resolution,
		int32_t Width, int32_t Height, double OriginX, double OriginY);
	int UrlabRcl_PublishOccupancyGrid(struct UrlabRclOccupancyGridPub* Pub,
		const int8_t* Data, int64_t SimTimeNs);
	void UrlabRcl_DestroyOccupancyGridPub(struct UrlabRclOccupancyGridPub* Pub);

	// octomap_msgs/Octomap (binary=true). Data is the serialised octree bytes;
	// the tree id and resolution are fixed at create.
	struct UrlabRclOctomapPub* UrlabRcl_CreateOctomapPub(struct UrlabRclContext* Ctx,
		const char* Topic, const char* FrameId, double Resolution);
	int UrlabRcl_PublishOctomap(struct UrlabRclOctomapPub* Pub,
		const uint8_t* Data, int32_t Size, int64_t SimTimeNs);
	void UrlabRcl_DestroyOctomapPub(struct UrlabRclOctomapPub* Pub);

	struct UrlabRclTwistStampedPub* UrlabRcl_CreateTwistStampedPub(struct UrlabRclContext* Ctx,
		const char* Topic, const char* FrameId);
	int UrlabRcl_PublishTwistStamped(struct UrlabRclTwistStampedPub* Pub,
		const double Linear[3], const double Angular[3], int64_t SimTimeNs);
	void UrlabRcl_DestroyTwistStampedPub(struct UrlabRclTwistStampedPub* Pub);

	struct UrlabRclClockPub* UrlabRcl_CreateClockPub(struct UrlabRclContext* Ctx); // topic /clock
	int UrlabRcl_PublishClock(struct UrlabRclClockPub* Pub, int64_t SimTimeNs);
	void UrlabRcl_DestroyClockPub(struct UrlabRclClockPub* Pub);

	struct UrlabRclImagePub* UrlabRcl_CreateImagePub(struct UrlabRclContext* Ctx,
		const char* Topic, const char* FrameId, int32_t Width, int32_t Height,
		const char* Encoding); // ROS encoding string, e.g. "rgb8"/"bgra8"
	int UrlabRcl_PublishImage(struct UrlabRclImagePub* Pub, const uint8_t* Data,
		int32_t StrideBytes, int64_t SimTimeNs);
	void UrlabRcl_DestroyImagePub(struct UrlabRclImagePub* Pub);

	// Control injection: the publish counterpart to the /<art>/cmd_ctrl
	// subscription below (std_msgs/Float64MultiArray). It exists so control can be
	// injected in-process for loopback and tests without a second ROS node.
	struct UrlabRclCtrlPub* UrlabRcl_CreateCtrlPub(struct UrlabRclContext* Ctx,
		const char* Topic);
	int UrlabRcl_PublishCtrl(struct UrlabRclCtrlPub* Pub, const double* Values,
		int32_t Count);
	void UrlabRcl_DestroyCtrlPub(struct UrlabRclCtrlPub* Pub);

	// Latched std_msgs/String publisher, transient-local + reliable + keep-last
	// depth 1, for the per-art /<art>/robot_description URDF. The QoS matches
	// robot_state_publisher so late-joining subscribers (rviz, MoveIt) receive the
	// last published spec. The text is copied on each publish.
	struct UrlabRclStringPub* UrlabRcl_CreateStringPub(struct UrlabRclContext* Ctx,
		const char* Topic);
	int UrlabRcl_PublishString(struct UrlabRclStringPub* Pub, const char* Text);
	void UrlabRcl_DestroyStringPub(struct UrlabRclStringPub* Pub);

	// geometry_msgs/WrenchStamped, for MuJoCo force + torque sensors paired on a
	// site. Force and Torque are 3-vectors in the sensor frame; either may be null
	// (an unpaired force or torque publishes its half, the other left zero).
	struct UrlabRclWrenchStampedPub* UrlabRcl_CreateWrenchStampedPub(struct UrlabRclContext* Ctx,
		const char* Topic, const char* FrameId);
	int UrlabRcl_PublishWrenchStamped(struct UrlabRclWrenchStampedPub* Pub,
		const double Force[3], const double Torque[3], int64_t SimTimeNs);
	void UrlabRcl_DestroyWrenchStampedPub(struct UrlabRclWrenchStampedPub* Pub);

	// sensor_msgs/Range, for MuJoCo rangefinder sensors. The constant fields
	// (radiation type per sensor_msgs/Range: 0 = ultrasound, 1 = infrared; field of
	// view; min/max range) are fixed at create time; publish sets only the reading.
	struct UrlabRclRangePub* UrlabRcl_CreateRangePub(struct UrlabRclContext* Ctx,
		const char* Topic, const char* FrameId, uint8_t RadiationType,
		float FieldOfView, float MinRange, float MaxRange);
	int UrlabRcl_PublishRange(struct UrlabRclRangePub* Pub, float Range, int64_t SimTimeNs);
	void UrlabRcl_DestroyRangePub(struct UrlabRclRangePub* Pub);

	// sensor_msgs/MagneticField, for MuJoCo magnetometer sensors. The field is a
	// 3-vector in tesla; the covariance leading element is set to 0 (exact
	// ground truth) per REP 145.
	struct UrlabRclMagneticFieldPub* UrlabRcl_CreateMagneticFieldPub(struct UrlabRclContext* Ctx,
		const char* Topic, const char* FrameId);
	int UrlabRcl_PublishMagneticField(struct UrlabRclMagneticFieldPub* Pub,
		const double Field[3], int64_t SimTimeNs);
	void UrlabRcl_DestroyMagneticFieldPub(struct UrlabRclMagneticFieldPub* Pub);

	// std_msgs/Float64MultiArray, the total-coverage fallback for any sensor with no
	// standard typed message (touch, subtree, user, ...). The data sequence grows to
	// fit on publish. Distinct from the ctrl publisher above so the two roles read
	// clearly at the call site; it also serves the planned cmd_ctrl echo and typed
	// user-channel array topics.
	struct UrlabRclFloat64MultiArrayPub* UrlabRcl_CreateFloat64MultiArrayPub(
		struct UrlabRclContext* Ctx, const char* Topic);
	int UrlabRcl_PublishFloat64MultiArray(struct UrlabRclFloat64MultiArrayPub* Pub,
		const double* Values, int32_t Count);
	void UrlabRcl_DestroyFloat64MultiArrayPub(struct UrlabRclFloat64MultiArrayPub* Pub);

	// nav_msgs/Odometry, the ground-truth base odometry for a free-base articulation.
	// FrameId is the header frame (REP-105 "odom"); ChildFrameId is the base link
	// ("<art>/<base>"). Both are fixed at create. Per the MuJoCo free-joint
	// convention the caller passes position + orientation (world) and the twist
	// ALREADY resolved into the base frame (linear rotated world->body, angular is
	// native body-frame qvel). Covariance is a small fixed ground-truth diagonal set
	// at create so EKF consumers (robot_localization) accept the message.
	struct UrlabRclOdometryPub* UrlabRcl_CreateOdometryPub(struct UrlabRclContext* Ctx,
		const char* Topic, const char* FrameId, const char* ChildFrameId);
	int UrlabRcl_PublishOdometry(struct UrlabRclOdometryPub* Pub,
		const double PositionXyz[3], const double OrientationXyzw[4],
		const double LinearBody[3], const double AngularBody[3], int64_t SimTimeNs);
	void UrlabRcl_DestroyOdometryPub(struct UrlabRclOdometryPub* Pub);

	// geometry_msgs/PoseWithCovarianceStamped, the ground-truth base pose in the map
	// frame (amcl_pose shape). FrameId is fixed at create ("map"); covariance is the
	// same fixed ground-truth diagonal.
	struct UrlabRclPoseWithCovariancePub* UrlabRcl_CreatePoseWithCovariancePub(
		struct UrlabRclContext* Ctx, const char* Topic, const char* FrameId);
	int UrlabRcl_PublishPoseWithCovariance(struct UrlabRclPoseWithCovariancePub* Pub,
		const double PositionXyz[3], const double OrientationXyzw[4], int64_t SimTimeNs);
	void UrlabRcl_DestroyPoseWithCovariancePub(struct UrlabRclPoseWithCovariancePub* Pub);

	// sensor_msgs/CameraInfo. Intrinsics are constant per camera, so the K matrix
	// (row-major 3x3), width/height, frame id, a zero plumb_bob distortion model, the
	// identity rectification R, and the projection matrix P (K with a zero 4th column)
	// are all filled at create; publish only restamps and sends. K carries fx,fy,cx,cy
	// at the standard pinhole slots (K[0]=fx, K[2]=cx, K[4]=fy, K[5]=cy, K[8]=1).
	struct UrlabRclCameraInfoPub* UrlabRcl_CreateCameraInfoPub(struct UrlabRclContext* Ctx,
		const char* Topic, const char* FrameId, int32_t Width, int32_t Height,
		const double K9[9]);
	int UrlabRcl_PublishCameraInfo(struct UrlabRclCameraInfoPub* Pub, int64_t SimTimeNs);
	void UrlabRcl_DestroyCameraInfoPub(struct UrlabRclCameraInfoPub* Pub);

	// --- Typed user-channel publishers -----------------------------------------
	// One triple per rosidl type the user-channel routing maps kinds to:
	// Bool -> std_msgs/Bool, Int/Scalar -> std_msgs/Float64, Vec3 ->
	// geometry_msgs/Vector3, Quat/Transform -> geometry_msgs/PoseStamped. Array /
	// String / Struct reuse the Float64MultiArray / String triples above.

	struct UrlabRclBoolPub* UrlabRcl_CreateBoolPub(struct UrlabRclContext* Ctx, const char* Topic);
	int UrlabRcl_PublishBool(struct UrlabRclBoolPub* Pub, int32_t bValue); // bValue != 0
	void UrlabRcl_DestroyBoolPub(struct UrlabRclBoolPub* Pub);

	struct UrlabRclFloat64Pub* UrlabRcl_CreateFloat64Pub(struct UrlabRclContext* Ctx, const char* Topic);
	int UrlabRcl_PublishFloat64(struct UrlabRclFloat64Pub* Pub, double Value);
	void UrlabRcl_DestroyFloat64Pub(struct UrlabRclFloat64Pub* Pub);

	struct UrlabRclVector3Pub* UrlabRcl_CreateVector3Pub(struct UrlabRclContext* Ctx, const char* Topic);
	int UrlabRcl_PublishVector3(struct UrlabRclVector3Pub* Pub, const double Xyz[3]);
	void UrlabRcl_DestroyVector3Pub(struct UrlabRclVector3Pub* Pub);

	// geometry_msgs/PoseStamped. FrameId is fixed at create; publish sets position
	// + orientation (xyzw; the provider reorders MuJoCo wxyz) and the stamp.
	struct UrlabRclPoseStampedPub* UrlabRcl_CreatePoseStampedPub(struct UrlabRclContext* Ctx,
		const char* Topic, const char* FrameId);
	int UrlabRcl_PublishPoseStamped(struct UrlabRclPoseStampedPub* Pub,
		const double PositionXyz[3], const double OrientationXyzw[4], int64_t SimTimeNs);
	void UrlabRcl_DestroyPoseStampedPub(struct UrlabRclPoseStampedPub* Pub);

	// --- Subscriptions ---------------------------------------------------------
	// Callbacks fire inside UrlabRcl_SpinSome on its caller's thread; the core does
	// no queuing beyond what the rmw layer holds.
	typedef void (*UrlabRclCtrlCallback)(const double* Values, int32_t Count,
		void* User);
	struct UrlabRclCtrlSub* UrlabRcl_CreateCtrlSub(struct UrlabRclContext* Ctx,
		const char* Topic, UrlabRclCtrlCallback Callback, void* User);
	// std_msgs/Float64MultiArray, the /<art>/cmd_ctrl shape
	void UrlabRcl_DestroyCtrlSub(struct UrlabRclCtrlSub* Sub);

	typedef void (*UrlabRclTwistCallback)(const double Linear[3],
		const double Angular[3], void* User);
	struct UrlabRclTwistSub* UrlabRcl_CreateTwistSub(struct UrlabRclContext* Ctx,
		const char* Topic, UrlabRclTwistCallback Callback, void* User);
	// geometry_msgs/Twist, the /<art>/cmd_vel shape
	void UrlabRcl_DestroyTwistSub(struct UrlabRclTwistSub* Sub);

	// sensor_msgs/JointState, the /<art>/joint_command jog shape. Names and the
	// paired position slice are handed to the callback; velocity / effort are
	// ignored. Names point into the taken message and are valid only for the
	// duration of the callback.
	typedef void (*UrlabRclJointStateCallback)(const char** Names,
		const double* Positions, int32_t Count, void* User);
	struct UrlabRclJointStateSub* UrlabRcl_CreateJointStateSub(struct UrlabRclContext* Ctx,
		const char* Topic, UrlabRclJointStateCallback Callback, void* User);
	void UrlabRcl_DestroyJointStateSub(struct UrlabRclJointStateSub* Sub);

	int UrlabRcl_SpinSome(struct UrlabRclContext* Ctx, int64_t TimeoutNs);

	// --- Services --------------------------------------------------------------
	// A std_srvs/Trigger service (empty request; response {bool success, string
	// message}), the standard type the claim_control / release_control services use
	// so no custom .srv package is needed. The callback fills success + message on
	// each request; the core sends the response. Callbacks fire inside
	// UrlabRcl_SpinSome on its caller's thread, like subscriptions. This service
	// area is kept separate from the message-publisher area of the seam.
	typedef void (*UrlabRclTriggerCallback)(void* User, int32_t* OutSuccess,
		char* OutMessage, int32_t OutMessageCap);
	struct UrlabRclTriggerService* UrlabRcl_CreateTriggerService(struct UrlabRclContext* Ctx,
		const char* ServiceName, UrlabRclTriggerCallback Callback, void* User);
	void UrlabRcl_DestroyTriggerService(struct UrlabRclTriggerService* Srv);

	// --- Zero-copy (only Clock is loanable in our message set) -----------------
	// CanLoan wraps rcl_publisher_can_loan_messages. The loaned publish borrows,
	// fills in place, publishes, and falls back to the plain publish when loaning
	// is unavailable.
	int UrlabRcl_ClockCanLoan(struct UrlabRclClockPub* Pub); // 1 = loanable, 0 = not
	int UrlabRcl_PublishClockLoaned(struct UrlabRclClockPub* Pub, int64_t SimTimeNs);

#ifdef __cplusplus
} // extern "C"
#endif
