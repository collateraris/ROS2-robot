#!/usr/bin/env bash
# Ubuntu 24.04 only. Run as the ordinary user: bash scripts/install_jazzy.sh pi|pc
set -euo pipefail
ROBOT_ROLE="${1:-pi}"
case "$ROBOT_ROLE" in pi|pc) ;; *) printf 'Usage: %s pi|pc\n' "$0"; exit 2;; esac
if [[ "$EUID" -eq 0 ]]; then
    printf 'Run as the ordinary user; individual apt commands use sudo.\n' >&2
    exit 1
fi
ROBOT_OS_INFO="$(python3 -c 'import platform; x=platform.freedesktop_os_release(); print(x["ID"]+" "+x["VERSION_ID"])')"
if [[ "$ROBOT_OS_INFO" != 'ubuntu 24.04' ]]; then
    printf 'This installer requires Ubuntu 24.04, found: %s\n' "$ROBOT_OS_INFO" >&2
    exit 1
fi
sudo apt-get update
sudo apt-get install -y locales curl ca-certificates software-properties-common
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
ROBOT_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
if [[ ! "$ROBOT_APT_SOURCE_VERSION" =~ ^[A-Za-z0-9._-]+$ ]]; then
    printf 'Unexpected ros-apt-source version\n' >&2
    exit 1
fi
ROBOT_APT_DEB="$(mktemp --suffix=.deb)"
trap 'rm -f -- "$ROBOT_APT_DEB"' EXIT
curl -fL -o "$ROBOT_APT_DEB" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROBOT_APT_SOURCE_VERSION}/ros2-apt-source_${ROBOT_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i "$ROBOT_APT_DEB"
sudo apt-get update
ROBOT_ROS_META=ros-jazzy-ros-base
if [[ "$ROBOT_ROLE" == pc ]]; then ROBOT_ROS_META=ros-jazzy-desktop; fi
sudo apt-get install -y "$ROBOT_ROS_META" ros-dev-tools ros-jazzy-cv-bridge \
    python3-opencv python3-aiohttp python3-serial python3-yaml v4l-utils unzip tmux
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update
if [[ "$ROBOT_ROLE" == pi ]]; then
    sudo usermod -aG dialout,video "$(id -un)"
fi
printf 'Installed. Re-login for device group permissions, then source /opt/ros/jazzy/setup.bash\n'
