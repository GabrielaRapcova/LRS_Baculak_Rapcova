#!/usr/bin/env bash
# Install the released FlexBE engine for this workspace's ROS 2 Jazzy environment.
set -euo pipefail

usage() {
  cat <<'HELP'
Usage: ./scripts/install_flexbe.sh [--with-tools] [--dry-run]

Installs ROS 2 Jazzy FlexBE core and onboard engine from the configured ROS apt
repository. Requires Ubuntu 24.04 and an existing /opt/ros/jazzy installation.

  --with-tools  Also install released states, mirror and widget tooling.
                This does not install the FlexBE WebUI.
  --dry-run     Print the apt commands without updating or installing anything.
  -h, --help    Show this help.

The installed release follows the ROS Jazzy package repository. No upstream
source checkout, ROS repository configuration or workspace build is performed.
HELP
}

with_tools=false
dry_run=false
for argument in "$@"; do
  case "$argument" in
    --with-tools) with_tools=true ;;
    --dry-run) dry_run=true ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$argument" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -r /etc/os-release ]]; then
  printf 'Cannot identify the operating system. Ubuntu 24.04 is required.\n' >&2
  exit 1
fi
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
  printf 'This installer targets Ubuntu 24.04 with ROS 2 Jazzy. Detected: %s\n' "${PRETTY_NAME:-unknown}" >&2
  exit 1
fi
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  printf 'ROS 2 Jazzy is missing. Install ROS and configure its apt repository first.\n' >&2
  exit 1
fi
for executable in apt-get apt-cache /usr/bin/python3; do
  if ! command -v "$executable" >/dev/null; then
    printf 'Required executable is missing: %s\n' "$executable" >&2
    exit 1
  fi
done

packages=(ros-jazzy-flexbe-core ros-jazzy-flexbe-onboard)
if "$with_tools"; then
  packages+=(ros-jazzy-flexbe-states ros-jazzy-flexbe-mirror ros-jazzy-flexbe-widget)
fi
privilege=()
if [[ $EUID -ne 0 ]]; then
  if ! command -v sudo >/dev/null; then
    printf 'sudo is required, or run this script as root.\n' >&2
    exit 1
  fi
  privilege=(sudo)
fi

if "$dry_run"; then
  printf 'Commands that would run:\n'
  printf '%q ' "${privilege[@]}" apt-get update; printf '\n'
  printf '%q ' "${privilege[@]}" apt-get install -y "${packages[@]}"; printf '\n'
  printf 'Then verify FlexBE Python imports in the ROS Jazzy environment.\n'
  exit 0
fi

"${privilege[@]}" apt-get update
for package in "${packages[@]}"; do
  candidate=$(apt-cache policy "$package")
  if ! [[ "$candidate" =~ Candidate:[[:space:]]+[^\([:space:]] ]]; then
    printf 'No apt candidate for %s. Check that the ROS 2 Jazzy apt repository is configured.\n' "$package" >&2
    exit 1
  fi
done
"${privilege[@]}" apt-get install -y "${packages[@]}"

# ROS setup scripts are not guaranteed to be compatible with nounset.
set +u
source /opt/ros/jazzy/setup.bash
set -u
/usr/bin/python3 - <<'PYTHON'
import importlib.util
for name in ('flexbe_core', 'flexbe_onboard'):
    spec = importlib.util.find_spec(name)
    if spec is None:
        raise SystemExit(f'FlexBE verification failed: {name} is unavailable')
    print(f'{name}: {spec.origin}')
from flexbe_core import Behavior, EventState, OperatableStateMachine
print('FlexBE behavior and state-machine imports passed.')
PYTHON
printf '\nFlexBE installed. In each new terminal, run:\n  source /opt/ros/jazzy/setup.bash\n'
printf 'Onboard engine: ros2 launch flexbe_onboard behavior_onboard.launch.py\n'
