// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"

struct FMjDocParseOptions;

/**
 * Ask the user how a model should be read, before reading it.
 *
 * The one option here is a security boundary rather than a preference, which
 * is why it needs a place to be seen: an `<include>` that escapes the model's
 * own directory tree is exfiltration-shaped, so the reader refuses it unless
 * the person opening the file says otherwise. There was nowhere to say so.
 *
 * `PreparationStatus` is stated, not decided, here: it tells the user whether
 * meshes will be converted for this import, which is the other thing they
 * cannot otherwise find out until it is over.
 *
 * Returns false when the user cancelled the import.
 */
bool ShowMjImportOptionsDialog(const FString& SourceXmlPath, const FText& PreparationStatus,
	FMjDocParseOptions& InOutOptions);
