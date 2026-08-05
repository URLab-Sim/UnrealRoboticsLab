// Shared includes for RPC handler translation units. Each RpcHandlers_*.cpp
// previously copy-pasted ~30 includes; this header carries the common set.
// Handler-specific headers (e.g. Async/Async.h for Camera) stay in the
// individual .cpp files.
#pragma once

#include "Bridge/RpcDispatcher.h"
#include "Bridge/OpRegistry.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/MsgpackHelpers.h"
#include "State/MjStateTypes.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/Sensors/MjSensor.h"
#include "MuJoCo/Components/Sensors/MjCamera.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Controllers/MjArticulationController.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "Transport/NetworkManager.h"
#include "Utils/URLabLogging.h"
