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
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "Transport/NetworkManager.h"
#include "Utils/URLabLogging.h"
