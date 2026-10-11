// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

/**
 * Convex decomposition on the Blueprint editor's component tree.
 *
 * Private because the menu is the interface: nothing else in the editor calls
 * a decomposition, and a caller that wants one has `UMjGeom::DecomposeMeshInto`.
 */
struct FMjDecompositionMenu
{
	/** Ask for the extension. Safe to call from StartupModule; defers the work. */
	static void Register();

private:
	/** Do the extending, once the menu system is up. */
	static void Extend();
};
