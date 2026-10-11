// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "MuJoCo/Elements/MjModelAsset.h"

#if WITH_EDITOR
bool UMjModelAsset::CanEditChange(const FProperty* InProperty) const
{
	if (InProperty != nullptr && InProperty->GetFName() == GET_MEMBER_NAME_CHECKED(UMjModelAssetBase, File))
	{
		return false;
	}
	return Super::CanEditChange(InProperty);
}
#endif
