// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "urlab_mjshim.h"

#include <map>
#include <string>

#include <mujoco/mujoco.h>

namespace
{

// The type MuJoCo's plugin attribute API expects behind its `void*`
// (`user_objects.h`, `mjCPlugin::config_attribs`). Getting this wrong is not a
// compile error, so it is written once and used by everything here.
using FPluginAttributes = std::map<std::string, std::string, std::less<>>;

}  // namespace

void urlab_mjs_setPluginAttribute(mjsPlugin* plugin, const char* key, const char* value)
{
	if (plugin == nullptr || key == nullptr)
	{
		return;
	}

	// Read, merge, write. MuJoCo's setter replaces rather than merges, so the
	// existing configuration has to be carried forward explicitly or each key
	// would erase the ones before it.
	FPluginAttributes Merged;
	if (const void* const Existing = mjs_getPluginAttributes(plugin))
	{
		Merged = *static_cast<const FPluginAttributes*>(Existing);
	}
	Merged[key] = (value != nullptr) ? value : "";

	// Every allocation and free above and below this line happens in this
	// library and in MuJoCo, which share a heap. That is the entire reason this
	// function exists; see urlab_mjshim.h.
	mjs_setPluginAttributes(plugin, &Merged);
}
