// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Entity/MjAppearance.h"

#include "Materials/MaterialInstanceDynamic.h"

#include "MuJoCo/Entity/MjGeomAppearance.h"
#include "MuJoCo/Spec/MjAssetResolve.h"

namespace MjAppearance
{
	void Apply(UMaterialInstanceDynamic* Mid, const FMjGeomAppearance& Override,
		TFunctionRef<UTexture*(FName Key)> ResolveTexture)
	{
		if (!Mid)
		{
			return;
		}

		if (Override.BaseColor.IsSet())
		{
			Mid->SetVectorParameterValue(TEXT("BaseColor"), Override.BaseColor.GetValue());
		}
		if (Override.Metallic.IsSet())
		{
			Mid->SetScalarParameterValue(TEXT("Metallic"), Override.Metallic.GetValue());
		}
		if (Override.Roughness.IsSet())
		{
			Mid->SetScalarParameterValue(TEXT("Roughness"), Override.Roughness.GetValue());
		}
		if (Override.Specular.IsSet())
		{
			Mid->SetScalarParameterValue(TEXT("Specular"), Override.Specular.GetValue());
		}
		if (Override.Reflectance.IsSet())
		{
			Mid->SetScalarParameterValue(TEXT("Reflectance"), Override.Reflectance.GetValue());
		}
		if (Override.Emission.IsSet())
		{
			Mid->SetScalarParameterValue(TEXT("Emission"), Override.Emission.GetValue());
		}
		if (Override.TexRepeat.IsSet())
		{
			const FVector2D& Repeat = Override.TexRepeat.GetValue();
			Mid->SetScalarParameterValue(TEXT("TexRepeatU"), static_cast<float>(Repeat.X));
			Mid->SetScalarParameterValue(TEXT("TexRepeatV"), static_cast<float>(Repeat.Y));
		}

		for (const TPair<EMjMaterialRole, FName>& Binding : Override.TextureBindings)
		{
			if (UTexture* Texture = ResolveTexture(Binding.Value))
			{
				Mid->SetTextureParameterValue(MjMaterialRoleParameter(Binding.Key), Texture);
			}
		}
	}
}
