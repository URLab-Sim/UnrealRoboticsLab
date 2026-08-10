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

#include "IDetailCustomization.h"
#include "MjEffectiveDetails.h"

/**
 * @class FMjGeomDetailCustomization
 * @brief Adds CoACD decomposition buttons to the UMjGeom Details panel.
 *
 * All other URLab components used to have parallel customizations that
 * existed only to ``HideProperty`` on a single internal UPROPERTY
 * (DefaultClass / ClassName / Name etc). Those were removed in the
 * MjComponentDetailCustomizations cleanup — the same hiding now lives on
 * the UPROPERTY decls themselves via
 * ``meta=(EditCondition="false", EditConditionHides)``. This file
 * keeps the one customization with real Slate-button logic.
 */
class FMjGeomDetailCustomization : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();

	/**
	 * The decomposition buttons, and only those.
	 *
	 * The editor runs EVERY layout registered along the class chain, not just
	 * the most derived one, so the inherited-value layout registered against
	 * `UMjNodeComponent` has already run for this geom by the time this does.
	 * This must not run it again: customising the same row twice resets it
	 * between passes and the first pass's widgets are lost.
	 */
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
};
