#!/usr/bin/env bash
# Build a matched FlexBE engine/WebUI source overlay for ROS 2 Jazzy.
set -euo pipefail

usage() {
  cat <<'HELP'
Usage: ./scripts/install_flexbe_source.sh [options]
  --workspace PATH  Separate overlay workspace (default: ~/flexbe_ws)
  --engine-ref REF  Engine branch, tag or commit (default: ros2-devel)
  --webui-ref REF   WebUI branch, tag or commit (default: main)
  --dry-run         Print the plan without modifying or downloading anything
  -h, --help        Show this help

Requires Ubuntu 24.04 and /opt/ros/jazzy. Run as your normal user; sudo is used
only for apt and initial rosdep configuration. Sources are fetched at the chosen
refs on every run; existing checkouts must be clean and have the expected origin.
Existing apt FlexBE packages are retained and overridden by the source overlay.
Includes the desktop PySide6 client and browser-capable WebUI operator station.
HELP
}

workspace="${HOME}/flexbe_ws"
engine_ref=ros2-devel
webui_ref=main
dry_run=false
while (($#)); do
  case "$1" in
    --workspace|--engine-ref|--webui-ref)
      option=$1
      if (($# < 2)) || [[ -z $2 || $2 == --* ]]; then
        printf 'Missing value for %s\n' "$option" >&2; exit 2
      fi
      case "$option" in
        --workspace) workspace=$2 ;;
        --engine-ref) engine_ref=$2 ;;
        --webui-ref) webui_ref=$2 ;;
      esac
      shift 2 ;;
    --dry-run) dry_run=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done
for ref in "$engine_ref" "$webui_ref"; do
  if [[ $ref == -* || $ref == *[[:space:]]* ]]; then
    printf 'Invalid Git ref: %s\n' "$ref" >&2; exit 2
  fi
done
workspace=$(realpath -m -- "$workspace")
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_workspace=$(dirname -- "$script_dir")
if [[ $workspace == / || $workspace == "$HOME" || $workspace == "$repo_workspace" ]]; then
  printf 'Choose a dedicated FlexBE workspace, not /, home or the navigation workspace.\n' >&2
  exit 2
fi
if "$dry_run"; then
  printf 'Workspace: %s\nEngine: https://github.com/FlexBE/flexbe_behavior_engine.git @ %s\nWebUI: https://github.com/FlexBE/flexbe_webui.git @ %s\n' "$workspace" "$engine_ref" "$webui_ref"
  printf '%s\n' 'Steps: install apt build/GUI prerequisites; fetch clean source checkouts;' \
    'verify engine/WebUI >=4.1; create a system-site-packages venv;' \
    'install upstream WebUI Python requirements; resolve source dependencies with rosdep;' \
    'build all source packages with colcon; verify imports and overlay package paths;' \
    'record commits/Python packages and write setup_flexbe.bash.'
  exit 0
fi
if [[ $EUID -eq 0 ]]; then
  printf 'Run as your normal user, not sudo; the script requests sudo when needed.\n' >&2
  exit 1
fi
source /etc/os-release
if [[ ${ID:-} != ubuntu || ${VERSION_ID:-} != 24.04 || ! -r /opt/ros/jazzy/setup.bash ]]; then
  printf 'Ubuntu 24.04 and ROS 2 Jazzy are required.\n' >&2; exit 1
fi
command -v sudo >/dev/null || { printf 'sudo is required.\n' >&2; exit 1; }

sudo apt-get update
sudo apt-get install -y git build-essential cmake python3-venv python3-pip \
  python3-colcon-common-extensions python3-rosdep python3-numpy \
  libxcb-cursor0 libegl1 libopengl0 libxkbcommon-x11-0

# Start with the Jazzy underlay only; do not inherit an ArduPilot venv or another overlay.
unset VIRTUAL_ENV PYTHONHOME PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH \
  COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source /opt/ros/jazzy/setup.bash
set -u
mkdir -p "$workspace/src"
# Prevent nested discovery if the overlay is placed beneath another colcon workspace.
touch "$workspace/COLCON_IGNORE"

fetch_source() {
  local name=$1 url=$2 ref=$3 destination="$workspace/src/$1"
  if [[ -e $destination ]]; then
    [[ -d $destination/.git ]] || { printf 'Not a Git checkout: %s\n' "$destination" >&2; exit 1; }
    [[ $(git -C "$destination" remote get-url origin) == "$url" ]] || {
      printf 'Unexpected origin in %s\n' "$destination" >&2; exit 1;
    }
    [[ -z $(git -C "$destination" status --porcelain) ]] || {
      printf 'Refusing to update modified checkout: %s\n' "$destination" >&2; exit 1;
    }
  else
    git clone --no-checkout "$url" "$destination"
  fi
  git -C "$destination" fetch origin "$ref"
  git -C "$destination" switch --detach FETCH_HEAD
  printf '%s %s\n' "$name" "$(git -C "$destination" rev-parse HEAD)"
}
fetch_source flexbe_behavior_engine https://github.com/FlexBE/flexbe_behavior_engine.git "$engine_ref"
fetch_source flexbe_webui https://github.com/FlexBE/flexbe_webui.git "$webui_ref"

/usr/bin/python3 - "$workspace" <<'PYTHON'
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
root = Path(sys.argv[1])
versions = []
for relative in ('src/flexbe_behavior_engine/flexbe_core/package.xml',
                 'src/flexbe_webui/package.xml'):
    version = ET.parse(root / relative).findtext('version')
    parts = tuple(int(part) for part in version.split('.')[:2])
    if parts < (4, 1):
        raise SystemExit(f'{relative}: expected version >=4.1, got {version}')
    versions.append(parts)
    print(f'{relative}: {version}')
if versions[0][0] != versions[1][0]:
    raise SystemExit('Engine and WebUI major versions differ; choose matching refs.')
PYTHON

venv_path="$workspace/venv"
/usr/bin/python3 -m venv --system-site-packages "$venv_path"
touch "$venv_path/COLCON_IGNORE"
"$venv_path/bin/python" -m pip install -r "$workspace/src/flexbe_webui/requires.txt"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo /usr/bin/rosdep init
fi
/usr/bin/rosdep update --rosdistro jazzy
/usr/bin/rosdep install --from-paths "$workspace/src" --ignore-src --rosdistro jazzy -y

export PATH="$venv_path/bin:$PATH"
cd "$workspace"
# Explicit paths bypass the workspace COLCON_IGNORE and exclude the venv.
mapfile -t source_packages < <("$venv_path/bin/python" /usr/bin/colcon list --base-paths src --names-only)
if ((${#source_packages[@]} == 0)); then
  printf 'No source packages discovered.\n' >&2; exit 1
fi
"$venv_path/bin/python" /usr/bin/colcon build --base-paths src --symlink-install \
  --allow-overriding "${source_packages[@]}" \
  --cmake-args "-DPython3_EXECUTABLE=$venv_path/bin/python" -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Resolve this path at source time, so quoting/spaces cannot corrupt generated commands.
cat > "$workspace/setup_flexbe.bash" <<'SETUP'
# Source this in a fresh bash terminal before sourcing the navigation workspace.
_flexbe_workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/jazzy/setup.bash
source "$_flexbe_workspace/venv/bin/activate"
source "$_flexbe_workspace/install/local_setup.bash"
unset _flexbe_workspace
SETUP
set +u
source "$workspace/setup_flexbe.bash"
set -u
"$venv_path/bin/python" - "$workspace" <<'PYTHON'
import sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from flexbe_core import Behavior, EventState, OperatableStateMachine
from PySide6 import QtWebEngineWidgets
import fastapi
root = Path(sys.argv[1]).resolve()
for package in ('flexbe_core', 'flexbe_msgs', 'flexbe_onboard', 'flexbe_mirror',
                'flexbe_widget', 'flexbe_webui'):
    prefix = Path(get_package_prefix(package)).resolve()
    if root not in prefix.parents:
        raise SystemExit(f'{package} incorrectly resolves to {prefix}')
    print(f'{package}: {prefix}')
print('Engine, operator station and QtWebEngine imports verified.')
PYTHON
{
  printf 'engine_ref %s\nengine_commit %s\n' "$engine_ref" "$(git -C src/flexbe_behavior_engine rev-parse HEAD)"
  printf 'webui_ref %s\nwebui_commit %s\n' "$webui_ref" "$(git -C src/flexbe_webui rev-parse HEAD)"
} > "$workspace/SOURCE_REVISIONS.txt"
"$venv_path/bin/python" -m pip freeze > "$workspace/PYTHON_PACKAGES.txt"
printf '\nBuild complete. In each fresh terminal, run:\n  source %q\n' "$workspace/setup_flexbe.bash"
printf '%s\n' 'Then launch both engine and operator station:' \
  '  ros2 launch flexbe_webui flexbe_full.launch.py' \
  'Or start them separately:' \
  '  ros2 launch flexbe_onboard behavior_onboard.launch.py' \
  '  ros2 launch flexbe_webui flexbe_ocs.launch.py' \
  'Browser-only OCS: ros2 launch flexbe_webui flexbe_ocs.launch.py headless:=true' \
  'Then open http://127.0.0.1:8000 in your browser.'
