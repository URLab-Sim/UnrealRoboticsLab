// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"

/**
 * A name-keyed runtime override for one light, the lighting twin of
 * FMjGeomAppearance: only the fields the client actually named override, so an
 * unmentioned field keeps whatever the model import (or the level) authored.
 *
 * Intensity has two knobs deliberately. `Intensity` is absolute in the light's
 * own units -- lux for a directional light, candelas for a point/spot -- which a
 * client that read those units back off `list_lights` can set exactly.
 * `IntensityScale` multiplies the AUTHORED value instead, which is what domain
 * randomization actually wants: "half as bright as this model imported" is
 * portable across models, an absolute lux figure is not. Setting both applies
 * the scale to the absolute value.
 *
 * Position and direction are carried in the MuJoCo frame (metres, right-handed)
 * because that is the frame the client thinks in; the store converts, and adds
 * back the scene origin the light was imported at.
 */
struct URLAB_API FMjLightOverride
{
	/** Light on/off. Applied as component visibility, so it survives without
	 *  destroying the light or losing its authored settings. */
	TOptional<bool> bEnabled;

	/** Absolute intensity in the light's own units (see class comment). */
	TOptional<float> Intensity;

	/** Multiplier on the authored intensity (see class comment). */
	TOptional<float> IntensityScale;

	TOptional<FLinearColor> Colour;

	/** MuJoCo-frame direction; need not be normalised. Ignored by a sky light. */
	TOptional<FVector> MjDirection;

	/** MuJoCo-frame position in metres. Ignored by a directional or sky light,
	 *  whose position does not affect the render. */
	TOptional<FVector> MjPosition;

	TOptional<bool> bCastShadow;

	bool IsEmpty() const
	{
		return !bEnabled.IsSet() && !Intensity.IsSet() && !IntensityScale.IsSet()
			&& !Colour.IsSet() && !MjDirection.IsSet() && !MjPosition.IsSet()
			&& !bCastShadow.IsSet();
	}
};
