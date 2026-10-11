#include "Bridge/PoseStreamServiceBinding.h"

#include "MuJoCo/Runtime/MjSimulationInteractionOwner.h"

void FURLabPoseStreamServiceBinding::Deactivate()
{
	if (!bActive.exchange(false, std::memory_order_acq_rel)) return;
	if (InteractionOwner) InteractionOwner->RevokeSource();
}
