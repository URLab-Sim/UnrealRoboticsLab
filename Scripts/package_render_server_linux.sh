#!/usr/bin/env bash
# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# package_render_server_linux.sh: cook, stage and pak a Linux render server
# from a host project that has this plugin under Plugins/.
#
# Usage: Scripts/package_render_server_linux.sh /abs/Host.uproject [out dir] [config]
#   out dir  archive the staged build there (default: Saved/StagedBuilds only)
#   config   Development (default), Test or Shipping
# Environment:
#   UE_ROOT    engine root (default ~/Documents/UE/UnrealEngine)
#   URLAB_MAP  render map to create and cook (default /Game/FastPath/FastPathRender)
#   URLAB_DDC  derived-data cache graph for the cook, passed as -ddc= (default: the
#              engine's normal graph). Use Cold to cook against a new, empty cache
#              (the engine's [Cold] graph: a flushed Zen sandbox), which proves a
#              fresh machine compiles every shader and derived asset.
#
# Steps: build the host editor target, create the render map if the project has
# none (an empty level with AGameModeBase, matching URLabLevelOps::CreateLevelSync),
# make sure the map and the plugin materials are cooked, then RunUAT BuildCookRun.

set -euo pipefail

UPROJECT="${1:-}"
OUT_DIR="${2:-}"
CONFIG="${3:-Development}"
[[ -n "$UPROJECT" ]] || { echo "usage: $0 /abs/Host.uproject [out dir] [config]" >&2; exit 2; }
[[ -f "$UPROJECT" ]] || { echo "project not found: $UPROJECT" >&2; exit 2; }
UPROJECT="$(cd "$(dirname "$UPROJECT")" && pwd)/$(basename "$UPROJECT")"
PROJECT_DIR="$(dirname "$UPROJECT")"
PROJECT_NAME="$(basename "$UPROJECT" .uproject)"
UE_ROOT="${UE_ROOT:-$HOME/Documents/UE/UnrealEngine}"
MAP="${URLAB_MAP:-/Game/FastPath/FastPathRender}"
DDC="${URLAB_DDC:-}"
PLUGIN_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

case "$CONFIG" in
	Development|Test|Shipping) ;;
	*) echo "config must be Development, Test or Shipping: $CONFIG" >&2; exit 2 ;;
esac
[[ "$MAP" == /Game/* ]] || { echo "URLAB_MAP must be a /Game/... package path: $MAP" >&2; exit 2; }
[[ -x "$UE_ROOT/Engine/Build/BatchFiles/RunUAT.sh" ]] || { echo "engine not found at UE_ROOT=$UE_ROOT" >&2; exit 2; }

EDITOR_CMD="$UE_ROOT/Engine/Binaries/Linux/UnrealEditor-Cmd"
MAP_FILE="$PROJECT_DIR/Content/${MAP#/Game/}.umap"

echo "== building ${PROJECT_NAME}Editor"
if [[ -f "$PROJECT_DIR/Source/${PROJECT_NAME}Editor.Target.cs" ]]; then
	"$UE_ROOT/Engine/Build/BatchFiles/Linux/Build.sh" "${PROJECT_NAME}Editor" Linux Development \
		-project="$UPROJECT" -waitmutex
else
	"$UE_ROOT/Engine/Build/BatchFiles/Linux/Build.sh" UnrealEditor Linux Development \
		-project="$UPROJECT" -waitmutex
fi
bash "$PLUGIN_ROOT/Scripts/setup_runtime_linux.sh"

if [[ -f "$MAP_FILE" ]]; then
	echo "== render map present: $MAP"
else
	echo "== creating render map $MAP"
	MAP_SCRIPT="$(mktemp --suffix=.py)"
	trap 'rm -f "$MAP_SCRIPT"' EXIT
	cat > "$MAP_SCRIPT" <<PYEOF
import unreal
path = "$MAP"
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
if not levels.new_level(path):
    raise RuntimeError("new_level failed for " + path)
world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
if world is not None:
    ws = world.get_world_settings()
    if ws is not None:
        ws.set_editor_property("default_game_mode", unreal.GameModeBase)
if not levels.save_current_level():
    raise RuntimeError("save_current_level failed for " + path)
unreal.log("created render map " + path)
PYEOF
	"$EDITOR_CMD" "$UPROJECT" -run=pythonscript -script="$MAP_SCRIPT" \
		-EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities \
		-unattended -nullrhi -nosplash -stdout -NoLiveCoding
	[[ -f "$MAP_FILE" ]] || { echo "render map was not created: $MAP_FILE" >&2; exit 1; }
fi

echo "== cook rules in Config/DefaultGame.ini"
python3 - "$PROJECT_DIR/Config/DefaultGame.ini" "$MAP" <<'PYEOF'
import os, sys
ini, map_path = sys.argv[1], sys.argv[2]
section = "[/Script/UnrealEd.ProjectPackagingSettings]"
wanted = [
    '+MapsToCook=(FilePath="%s")' % map_path,
    '+DirectoriesToAlwaysCook=(Path="/UnrealRoboticsLab/Materials")',
]
lines = open(ini).read().splitlines() if os.path.exists(ini) else []
present = {l.strip() for l in lines}
missing = [w for w in wanted if w not in present]
if missing:
    if section not in present:
        if lines and lines[-1].strip():
            lines.append("")
        lines.append(section)
    at = next(i for i, l in enumerate(lines) if l.strip() == section) + 1
    lines[at:at] = missing
    os.makedirs(os.path.dirname(ini), exist_ok=True)
    with open(ini, "w") as f:
        f.write("\n".join(lines) + "\n")
for w in wanted:
    print(("added   " if w in missing else "present ") + w)
PYEOF

echo "== BuildCookRun ($CONFIG)"
ARCHIVE_ARGS=()
if [[ -n "$OUT_DIR" ]]; then
	mkdir -p "$OUT_DIR"
	OUT_DIR="$(cd "$OUT_DIR" && pwd)"
	ARCHIVE_ARGS=(-archive -archivedirectory="$OUT_DIR")
fi
DDC_ARGS=()
[[ -n "$DDC" ]] && DDC_ARGS=(-ddc="$DDC")
"$UE_ROOT/Engine/Build/BatchFiles/RunUAT.sh" BuildCookRun \
	-project="$UPROJECT" -platform=Linux -clientconfig="$CONFIG" \
	-build -cook -stage -pak -iostore -map="$MAP" \
	-noP4 -unattended -utf8output -nocompileeditor -skipbuildeditor -WaitForUATMutex \
	"${ARCHIVE_ARGS[@]}" "${DDC_ARGS[@]}"

STAGED="$PROJECT_DIR/Saved/StagedBuilds/Linux"
[[ -n "$OUT_DIR" ]] && STAGED="$OUT_DIR/Linux"
LAUNCHER="$STAGED/$PROJECT_NAME.sh"
[[ -x "$LAUNCHER" ]] || { echo "staged launcher missing: $LAUNCHER" >&2; exit 1; }
echo "== packaged: $LAUNCHER"
echo "   smoke: $PLUGIN_ROOT/Scripts/smoke_packaged_linux.sh $LAUNCHER"
