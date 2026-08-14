// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef URLAB_MJSHIM_H
#define URLAB_MJSHIM_H

// MuJoCo entry points that cannot be called across an allocator boundary,
// restated so they can be.
//
// A few of MuJoCo's `mjs_` functions are C in name only: they take a C++
// standard container by `void*` and take ownership of it. `mjs_setPluginAttributes`
// is the one that matters here -- its `void*` must be a
// `std::map<std::string, std::string, std::less<>>*`, and MuJoCo MOVES from it.
//
// That is unusable from an Unreal module. Unreal replaces global operator new
// and delete in every module it builds (`REPLACEMENT_OPERATOR_NEW_AND_DELETE`),
// so a container built there allocates from Unreal's allocator while MuJoCo
// frees it against the CRT heap. It does not fail at the call. It fails later,
// when the spec is deleted, as a heap corruption with no MuJoCo frame on the
// stack. Moving the container back out first does not help either: MSVC's map
// leaves each side holding a sentinel node allocated by the other.
//
// This library is built the way MuJoCo is and replaces nothing, so its
// allocator IS MuJoCo's. The container is created, filled and handed over
// entirely on this side of the boundary, and only C strings cross.
//
// Nothing here belongs to MuJoCo. If MuJoCo ever grows a C-string entry point
// for plugin configuration, every function in this file becomes a forwarding
// call and the library can be deleted.

struct mjsPlugin_;
typedef struct mjsPlugin_ mjsPlugin;

#if defined(_WIN32)
	#if defined(URLAB_MJSHIM_BUILD)
		#define URLAB_MJSHIM_API __declspec(dllexport)
	#else
		#define URLAB_MJSHIM_API __declspec(dllimport)
	#endif
#else
	#define URLAB_MJSHIM_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Set one plugin configuration attribute, replacing any earlier value for the
 * same key and leaving the rest of the plugin's configuration alone.
 *
 * The merge is what makes this per-key rather than per-instance: MuJoCo's own
 * setter replaces the whole map, so a caller writing one `<config>` at a time
 * through it would keep only the last.
 *
 * A null `value` is written as the empty string, which is what an authored
 * `value=""` means. A null `plugin` or `key` does nothing.
 */
URLAB_MJSHIM_API void urlab_mjs_setPluginAttribute(
	mjsPlugin* plugin, const char* key, const char* value);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // URLAB_MJSHIM_H
