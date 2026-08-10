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

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FURLabEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
	FDelegateHandle ViewportContextMenuExtenderHandle;
	FDelegateHandle OnObjectModifiedHandle;
	FDelegateHandle OnActorMovedHandle;
	FDelegateHandle PostEngineInitHandle;
	bool bIsAutoParenting = false;

	static TSharedRef<FExtender> OnExtendActorContextMenu(const TSharedRef<FUICommandList> CommandList, const TArray<AActor*> SelectedActors);
	static void BuildQuickConvertSubMenu(FMenuBuilder& MenuBuilder, TArray<AActor*> SelectedActors);
	static void ApplyQuickConvert(TArray<TWeakObjectPtr<AActor>> Actors, bool bStatic, bool bComplex);

	void OnObjectModified(UObject* Object);

	/**
	 * Deliver the move hook to a model whose root the engine skips.
	 *
	 * `AActor::PostEditMove` calls `PostEditComponentMove` on the root component
	 * only when the construction script did NOT create it
	 * (`ActorEditor.cpp:323-327`), and a URLab model's root is exactly such a
	 * component. So dragging a placed model -- the ordinary level-editor gesture
	 * -- reached no element hook at all: the per-type scale refusal never ran, and
	 * a scaled actor drew every geom under it stretched while MuJoCo went on using
	 * the `size` the gesture never touched.
	 *
	 * `USceneComponent::PostEditComponentMove` recurses into every attached child,
	 * so one call on the root reaches every element. Broadcast once per drag, not
	 * per delta: `AActor::PostEditMove` fires this only when the move finishes.
	 */
	void OnActorMoved(AActor* Actor);
	static bool AutoParentSCSNode(class USCS_Node* Node, class USimpleConstructionScript* SCS);
};
