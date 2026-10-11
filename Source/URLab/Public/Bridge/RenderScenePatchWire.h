#pragma once

#include "CoreMinimal.h"
#include "Wire/RequestEnvelope.h"
#include "Workflows/RenderScenePatch.h"

namespace URLabRenderScenePatchWire
{
URLAB_API bool Parse(const FURLabWireValue& Value, FURLabRenderScenePatch& Out,
	FString& OutError);
}
