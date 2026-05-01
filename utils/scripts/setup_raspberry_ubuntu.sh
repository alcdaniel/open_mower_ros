#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-noetic}"
ENV_FILE="${MOWER_ENV_FILE:-${REPO_ROOT}/.env}"
SKIP_ROS_INSTALL=0
SKIP_ROSDEP_INIT=0
SKIP_BUILD=0
SKIP_SWAP=0
COPY_CONFIG=1
ALLOW_UNSUPPORTED_OS=0
WITH_RVIZ=0
INSTALL_WIFI_SERVICE=1

usage() {
  cat <<'EOF'
Usage: ./utils/scripts/setup_raspberry_ubuntu.sh [options]

Prepares an Ubuntu Raspberry Pi for this OpenMower ROS workspace:
  - installs ROS Noetic apt repository and packages
  - initializes rosdep when needed
  - fetches git submodules
  - installs workspace dependencies with rosdep
  - builds the catkin workspace
  - creates mower_config.sh from the example when missing
  - creates a local .env for Wi-Fi/NTRIP settings when missing
  - installs a systemd Wi-Fi auto-connect service

Options:
  --skip-ros-install       Do not add/install ROS apt packages
  --skip-rosdep-init       Do not run sudo rosdep init
  --skip-build             Install dependencies but do not run catkin_make
  --skip-swap              Do not auto-create /swapfile before building
  --no-copy-config         Do not create mower_config.sh
  --no-wifi-service        Do not install the Wi-Fi systemd service
  --allow-unsupported-os   Continue even if Ubuntu codename is not focal
  --with-rviz              Install rviz for local debugging on the Raspberry
  -h, --help               Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-ros-install)
      SKIP_ROS_INSTALL=1
      ;;
    --skip-rosdep-init)
      SKIP_ROSDEP_INIT=1
      ;;
    --skip-build)
      SKIP_BUILD=1
      ;;
    --skip-swap)
      SKIP_SWAP=1
      ;;
    --no-copy-config)
      COPY_CONFIG=0
      ;;
    --no-wifi-service)
      INSTALL_WIFI_SERVICE=0
      ;;
    --allow-unsupported-os)
      ALLOW_UNSUPPORTED_OS=1
      ;;
    --with-rviz)
      WITH_RVIZ=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "Missing required command: ${command_name}" >&2
    exit 1
  fi
}

remove_conflicting_python_ros_packages() {
  local legacy_packages=(
    python3-catkin-pkg
    python3-rospkg
    python3-rosdistro
  )
  local present=()

  for pkg in "${legacy_packages[@]}"; do
    if dpkg-query -W -f='${db:Status-Abbrev}' "${pkg}" 2>/dev/null | grep -q '^[ih]'; then
      present+=("${pkg}")
    fi
  done

  if [[ "${#present[@]}" -eq 0 ]]; then
    return
  fi

  echo "Force-removing Ubuntu ROS Python packages that conflict with packages.ros.org: ${present[*]}"
  sudo dpkg --remove --force-depends "${present[@]}" || true
}

ensure_swap() {
  local desired_total_mb="${1:-3072}"
  local swapfile="/swapfile"
  local mem_total_mb swap_total_mb need_mb

  mem_total_mb=$(awk '/MemTotal/ {printf "%d", $2/1024}' /proc/meminfo)
  swap_total_mb=$(awk '/SwapTotal/ {printf "%d", $2/1024}' /proc/meminfo)

  if (( mem_total_mb + swap_total_mb >= desired_total_mb )); then
    echo "Memory + swap already >= ${desired_total_mb} MB (mem=${mem_total_mb} swap=${swap_total_mb}). Skipping swap setup."
    return
  fi

  need_mb=$(( desired_total_mb - mem_total_mb - swap_total_mb ))
  echo "Adding ${need_mb} MB of swap at ${swapfile} so the catkin build does not OOM..."

  if [[ -e "${swapfile}" ]]; then
    sudo swapoff "${swapfile}" 2>/dev/null || true
    sudo rm -f "${swapfile}"
  fi

  if ! sudo fallocate -l "${need_mb}M" "${swapfile}"; then
    sudo dd if=/dev/zero of="${swapfile}" bs=1M count="${need_mb}" status=progress
  fi
  sudo chmod 600 "${swapfile}"
  sudo mkswap "${swapfile}"
  sudo swapon "${swapfile}"

  if ! grep -qF "${swapfile}" /etc/fstab; then
    echo "${swapfile} none swap sw 0 0" | sudo tee -a /etc/fstab >/dev/null
  fi
}

pick_build_jobs() {
  local mem_total_mb
  mem_total_mb=$(awk '/MemTotal/ {printf "%d", $2/1024}' /proc/meminfo)

  if (( mem_total_mb < 2048 )); then
    echo 1
  elif (( mem_total_mb < 4096 )); then
    echo 2
  else
    echo 0
  fi
}

repair_apt_state() {
  echo "Checking apt/dpkg state..."
  remove_conflicting_python_ros_packages

  if ! sudo apt-get --fix-broken install --yes; then
    echo "apt fix-broken failed once; removing known conflicting Python ROS packages and retrying..."
    remove_conflicting_python_ros_packages
    sudo apt-get clean
    sudo apt-get --fix-broken install --yes
  fi

  sudo dpkg --configure -a
}

write_default_env() {
  if [[ -f "${ENV_FILE}" ]]; then
    return
  fi

  echo "Creating local .env at ${ENV_FILE}. Edit it before expecting Wi-Fi/NTRIP to work."
  umask 077
  cat > "${ENV_FILE}" <<'EOF'
# Local Raspberry/OpenMower settings.
# This file is intentionally ignored by git because it can contain credentials.

# Wi-Fi used by the Raspberry to reach the internet/NTRIP caster.
MOWER_WIFI_SSID="CHANGE_ME_WIFI_SSID"
MOWER_WIFI_PASSWORD="CHANGE_ME_WIFI_PASSWORD"
MOWER_WIFI_CONNECTION_NAME="openmower-wifi"

# Optional NTRIP placeholders for the future rover GPS integration.
# The current setup script only uses the Wi-Fi values above.
NTRIP_CASTER_HOST=""
NTRIP_CASTER_PORT="2101"
NTRIP_MOUNTPOINT=""
NTRIP_USERNAME=""
NTRIP_PASSWORD=""

# iOS app bridge. The app can use the discovered host, or the manual URL:
# http://<raspberry-ip>:8080
OM_IOS_BRIDGE_ENABLE="True"
OM_IOS_BRIDGE_HOST="0.0.0.0"
OM_IOS_BRIDGE_PORT="8080"
OM_IOS_BRIDGE_NAME="lawnmower"
OM_IOS_BRIDGE_TOKEN=""
OM_IOS_UDP_BEACON_ENABLE="True"
EOF
}

warn_if_env_needs_editing() {
  if [[ ! -f "${ENV_FILE}" ]]; then
    echo "Wi-Fi .env file is missing: ${ENV_FILE}"
    return
  fi

  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a

  if [[ -z "${MOWER_WIFI_SSID:-}" || -z "${MOWER_WIFI_PASSWORD:-}" || "${MOWER_WIFI_SSID:-}" == "CHANGE_ME_WIFI_SSID" || "${MOWER_WIFI_PASSWORD:-}" == "CHANGE_ME_WIFI_PASSWORD" ]]; then
    cat <<EOF

Wi-Fi is not configured yet.
Edit ${ENV_FILE} and fill:
  MOWER_WIFI_SSID
  MOWER_WIFI_PASSWORD

Without this, the Raspberry will not automatically connect to Wi-Fi and will not
have internet access for the rover NTRIP corrections.
EOF
  fi
}

install_wifi_service() {
  if [[ "${INSTALL_WIFI_SERVICE}" -ne 1 ]]; then
    return
  fi

  local wifi_script="${REPO_ROOT}/utils/scripts/connect_wifi_from_env.sh"
  local service_path="/etc/systemd/system/openmower-wifi.service"

  chmod +x "${wifi_script}"

  echo "Installing openmower-wifi systemd service..."
  sudo tee "${service_path}" >/dev/null <<EOF
[Unit]
Description=OpenMower Wi-Fi auto-connect from ${ENV_FILE}
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
WorkingDirectory=${REPO_ROOT}
Environment=MOWER_ENV_FILE=${ENV_FILE}
ExecStart=${wifi_script}
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

  sudo systemctl daemon-reload
  sudo systemctl enable openmower-wifi.service
}

if [[ "${EUID}" -eq 0 ]]; then
  echo "Run this script as a normal user with sudo access, not as root." >&2
  exit 1
fi

require_command sudo
require_command git

if [[ ! -r /etc/os-release ]]; then
  echo "Cannot detect OS because /etc/os-release is missing." >&2
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
UBUNTU_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"

if [[ "${ID:-}" != "ubuntu" ]]; then
  echo "This script is intended for Ubuntu. Detected: ${PRETTY_NAME:-unknown}." >&2
  exit 1
fi

if [[ "${UBUNTU_CODENAME}" != "focal" && "${ALLOW_UNSUPPORTED_OS}" -ne 1 ]]; then
  cat >&2 <<EOF
This workspace is based on ROS Noetic/Focal images, but this system is:
  ${PRETTY_NAME:-unknown}

Use Ubuntu 20.04/Focal for the native setup, or rerun with
--allow-unsupported-os if you know this machine already has compatible ROS
Noetic packages available.
EOF
  exit 1
fi

sudo -v

repair_apt_state

if [[ "${SKIP_ROS_INSTALL}" -ne 1 ]]; then
  echo "Installing apt repository tools..."
  sudo apt-get update
  sudo apt-get install --yes \
    ca-certificates \
    curl \
    gnupg2 \
    git \
    lsb-release \
    software-properties-common

  echo "Ensuring Ubuntu universe repository is enabled..."
  sudo add-apt-repository --yes universe

  echo "Installing base apt packages..."
  sudo apt-get update
  sudo apt-get install --yes \
    build-essential \
    libasio-dev \
    libgeographic-dev \
    libopencv-dev \
    libwebsocketpp-dev \
    network-manager \
    python3-opencv \
    python3-pip \
    python3.8-venv

  if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
    echo "Installing ROS apt repository key..."
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
  fi

  ROS_APT_SOURCE="/etc/apt/sources.list.d/ros1-latest.list"
  ROS_APT_LINE="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu ${UBUNTU_CODENAME} main"
  if [[ ! -f "${ROS_APT_SOURCE}" ]] || ! grep -Fxq "${ROS_APT_LINE}" "${ROS_APT_SOURCE}"; then
    echo "Adding ROS apt repository..."
    echo "${ROS_APT_LINE}" | sudo tee "${ROS_APT_SOURCE}" >/dev/null
  fi

  echo "Installing ROS ${ROS_DISTRO}..."
  sudo apt-get update
  ros_packages=(
    "ros-${ROS_DISTRO}-actionlib"
    "ros-${ROS_DISTRO}-catkin"
    "ros-${ROS_DISTRO}-costmap-2d"
    "ros-${ROS_DISTRO}-dynamic-reconfigure"
    "ros-${ROS_DISTRO}-geometry-msgs"
    "ros-${ROS_DISTRO}-grid-map"
    "ros-${ROS_DISTRO}-grid-map-msgs"
    "ros-${ROS_DISTRO}-imu-tools"
    "ros-${ROS_DISTRO}-mbf-msgs"
    "ros-${ROS_DISTRO}-message-generation"
    "ros-${ROS_DISTRO}-move-base-flex"
    "ros-${ROS_DISTRO}-nav-core"
    "ros-${ROS_DISTRO}-nav-msgs"
    "ros-${ROS_DISTRO}-nmea-msgs"
    "ros-${ROS_DISTRO}-paho-mqtt-c"
    "ros-${ROS_DISTRO}-paho-mqtt-cpp"
    "ros-${ROS_DISTRO}-pcl-conversions"
    "ros-${ROS_DISTRO}-pluginlib"
    "ros-${ROS_DISTRO}-robot-localization"
    "ros-${ROS_DISTRO}-rosbag"
    "ros-${ROS_DISTRO}-ros-base"
    "ros-${ROS_DISTRO}-rosbridge-server"
    "ros-${ROS_DISTRO}-rtcm-msgs"
    "ros-${ROS_DISTRO}-sensor-msgs"
    "ros-${ROS_DISTRO}-serial"
    "ros-${ROS_DISTRO}-std-msgs"
    "ros-${ROS_DISTRO}-tf"
    "ros-${ROS_DISTRO}-tf2"
    "ros-${ROS_DISTRO}-tf2-eigen"
    "ros-${ROS_DISTRO}-tf2-geometry-msgs"
    "ros-${ROS_DISTRO}-tf2-ros"
    "ros-${ROS_DISTRO}-twist-mux-msgs"
    python3-rosdep
  )
  if [[ "${WITH_RVIZ}" -eq 1 ]]; then
    ros_packages+=("ros-${ROS_DISTRO}-rviz")
  fi
  sudo apt-get install --yes "${ros_packages[@]}"
fi

if [[ "${SKIP_ROSDEP_INIT}" -ne 1 && ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  echo "Initializing rosdep..."
  sudo rosdep init
fi

echo "Updating rosdep..."
rosdep update --include-eol-distros

cd "${REPO_ROOT}"

echo "Fetching git submodules..."
git submodule update --init --recursive

echo "Installing workspace dependencies with rosdep..."
ROSDEP_SKIP_KEYS="xesc_msgs xbot_msgs xbot_rpc xesc xesc_interface xesc_2040_driver xesc_yfr4_driver xesc_driver xbot_framework mower_msgs"
rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" --default-yes \
  --skip-keys="${ROSDEP_SKIP_KEYS}" || {
  echo "rosdep could not resolve every dependency. Continuing because core ROS dependencies were installed explicitly."
}

if [[ "${COPY_CONFIG}" -eq 1 && ! -f "${REPO_ROOT}/mower_config.sh" ]]; then
  echo "Creating mower_config.sh from example. Edit it before running the mower."
  cp "${REPO_ROOT}/src/open_mower/config/mower_config.sh.example" "${REPO_ROOT}/mower_config.sh"
fi

write_default_env
install_wifi_service
warn_if_env_needs_editing

if [[ "${SKIP_BUILD}" -ne 1 ]]; then
  if [[ "${SKIP_SWAP}" -ne 1 ]]; then
    ensure_swap 3072
  fi

  build_jobs="$(pick_build_jobs)"
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"

  if [[ "${build_jobs}" -gt 0 ]]; then
    echo "Building catkin workspace with -j${build_jobs} (chosen for available RAM)..."
    catkin_make -j"${build_jobs}" -l"${build_jobs}"
  else
    echo "Building catkin workspace with default parallelism..."
    catkin_make
  fi
fi

cat <<EOF

Setup finished.

Before launching OpenMower in a new shell:
  cd ${REPO_ROOT}
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source devel/setup.bash
  source mower_config.sh
  set -a
  source .env
  set +a

Then launch:
  roslaunch open_mower open_mower.launch

Wi-Fi auto-connect:
  Edit ${ENV_FILE} with MOWER_WIFI_SSID and MOWER_WIFI_PASSWORD.
  The openmower-wifi.service will try to connect on boot.
EOF
