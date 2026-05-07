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
CONFIGURE_UART=1

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
  - configures Raspberry UART for the Mega bridge (or run
    ./utils/scripts/configure_raspberry_uart.sh standalone)

Options:
  --skip-ros-install       Do not add/install ROS apt packages
  --skip-rosdep-init       Do not run sudo rosdep init
  --skip-build             Install dependencies but do not run catkin_make
  --skip-swap              Do not auto-create /swapfile before building
  --no-copy-config         Do not create mower_config.sh
  --no-wifi-service        Do not install the Wi-Fi systemd service
  --no-uart-config         Do not configure Raspberry UART for Mega bridge
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
    --no-uart-config)
      CONFIGURE_UART=0
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

configure_uart_for_mega_bridge() {
  if [[ "${CONFIGURE_UART}" -ne 1 ]]; then
    return
  fi

  local uart_script="${SCRIPT_DIR}/configure_raspberry_uart.sh"
  if [[ ! -f "${uart_script}" ]]; then
    echo "Missing UART setup script: ${uart_script}" >&2
    exit 1
  fi
  if [[ ! -x "${uart_script}" ]]; then
    chmod +x "${uart_script}"
  fi
  echo "Running UART-only setup script: ${uart_script}"
  "${uart_script}"
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

  # Create service with absolute path expansion
  sudo bash -c "cat > '${service_path}' << 'WIFI_SERVICE_EOF'
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
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
WIFI_SERVICE_EOF
"

  sudo systemctl daemon-reload
  if sudo systemctl enable openmower-wifi.service; then
    echo "✓ WiFi service enabled for auto-start"
  else
    echo "✗ Failed to enable WiFi service" >&2
    return 1
  fi

  # Verify service is installed
  if sudo systemctl is-enabled openmower-wifi.service >/dev/null 2>&1; then
    echo "✓ Service verified as enabled"
  else
    echo "⚠ Warning: Service may not be properly installed" >&2
  fi
}

setup_shell_env() {
  local marker="OpenMower ROS environment (managed by setup_raspberry_ubuntu.sh)"
  local bashrc="$HOME/.bashrc"
  local profiled="/etc/profile.d/openmower-ros.sh"

  # /etc/profile.d/ — loaded for all interactive login shells (SSH, TTY).
  # More robust than ~/.bashrc which is skipped by login-shell SSH sessions.
  sudo tee "${profiled}" > /dev/null << PROFILED
export ROS_DISTRO=${ROS_DISTRO}
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "${REPO_ROOT}/devel/setup.bash" ]] && source "${REPO_ROOT}/devel/setup.bash"
[[ -f "${REPO_ROOT}/mower_config.sh"  ]] && source "${REPO_ROOT}/mower_config.sh"
PROFILED
  sudo chmod 644 "${profiled}"

  # Also write to ~/.bashrc for interactive non-login shells (local terminals).
  if grep -q "${marker}" "${bashrc}" 2>/dev/null; then
    sed -i "/# >>> ${marker}/,/# <<< ${marker}/d" "${bashrc}"
  fi

  cat >> "${bashrc}" <<BASHRC_BLOCK

# >>> ${marker} >>>
# Sourced automatically — do not edit this block by hand; re-run setup to update.
export ROS_DISTRO=${ROS_DISTRO}
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "${REPO_ROOT}/devel/setup.bash"  ]] && source "${REPO_ROOT}/devel/setup.bash"
[[ -f "${REPO_ROOT}/mower_config.sh"   ]] && source "${REPO_ROOT}/mower_config.sh"
# <<< ${marker} <<<
BASHRC_BLOCK

  echo "Shell environment written to ${bashrc} and ${profiled}."
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
configure_uart_for_mega_bridge

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
    "ros-${ROS_DISTRO}-rosbash"
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
    "ros-${ROS_DISTRO}-twist-mux"
    "ros-${ROS_DISTRO}-joy"
    "ros-${ROS_DISTRO}-teleop-twist-joy"
    psmisc
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

setup_mower_config() {
  local config="${REPO_ROOT}/mower_config.sh"

  if [[ ! -f "${config}" ]]; then
    echo "Creating mower_config.sh from example."
    cp "${REPO_ROOT}/config/mower_config.sh.example" "${config}"
  fi

  # Ensure OM_MOWER is set (uncomment if commented, default to CUSTOM)
  if grep -qE '^#?\s*export OM_MOWER=' "${config}"; then
    sed -i 's/^#\s*export OM_MOWER=.*/export OM_MOWER="CUSTOM"/' "${config}"
  else
    echo 'export OM_MOWER="CUSTOM"' >> "${config}"
  fi

  # Ensure OM_MOWER_ESC_TYPE is set
  if grep -qE '^#?\s*export OM_MOWER_ESC_TYPE=' "${config}"; then
    sed -i 's/^#\s*export OM_MOWER_ESC_TYPE=.*/export OM_MOWER_ESC_TYPE="xesc_mini"/' "${config}"
  else
    echo 'export OM_MOWER_ESC_TYPE="xesc_mini"' >> "${config}"
  fi

  # ── NTRIP: ERGNSS (IGN Spain) ──────────────────────────────────────────────
  # Server and mountpoint are fixed; user/password come from free registration
  # at https://ergnss.ign.es/gnuserportal/
  # Only overwrite if still set to the old placeholder values from the example.
  _ntrip_sed() {
    local key="$1" val="$2"
    if grep -qE "^export ${key}=" "${config}"; then
      sed -i "s|^export ${key}=.*|export ${key}=${val}|" "${config}"
    else
      echo "export ${key}=${val}" >> "${config}"
    fi
  }
  _ntrip_sed OM_USE_NTRIP       True
  _ntrip_sed OM_NTRIP_HOSTNAME  192.148.213.42
  _ntrip_sed OM_NTRIP_PORT      2101
  _ntrip_sed OM_NTRIP_ENDPOINT  VRS3M
  _ntrip_sed OM_NTRIP_RECONNECT_WAIT_SECONDS 5
  _ntrip_sed OM_NTRIP_RECONNECT_MAX          99999
  # Only set user/password if they still hold the old default placeholder values
  if grep -qE "^export OM_NTRIP_USER=(gps|CHANGE_ME)$" "${config}"; then
    sed -i "s|^export OM_NTRIP_USER=.*|export OM_NTRIP_USER=CHANGE_ME|" "${config}"
  fi
  if grep -qE "^export OM_NTRIP_PASSWORD=(gps|CHANGE_ME)$" "${config}"; then
    sed -i "s|^export OM_NTRIP_PASSWORD=.*|export OM_NTRIP_PASSWORD=CHANGE_ME|" "${config}"
  fi

  # ── DATUM: default to Gijón, Asturias ──────────────────────────────────────
  if grep -qE "^export OM_DATUM_LAT=.*CHANGEME" "${config}"; then
    sed -i "s|^export OM_DATUM_LAT=.*|export OM_DATUM_LAT=43.5350|" "${config}"
  fi
  if grep -qE "^export OM_DATUM_LONG=.*CHANGEME" "${config}"; then
    sed -i "s|^export OM_DATUM_LONG=.*|export OM_DATUM_LONG=-5.6615|" "${config}"
  fi

  # Remove any previous platform block appended by this script
  grep -v 'Platform config (auto)\|^export HARDWARE_PLATFORM=\|^export OM_V2=\|^export MOWER=\$OM_MOWER\|^export ESC_TYPE=\$OM_MOWER_ESC_TYPE\|^export PARAMS_PATH=\|^export RECORDINGS_PATH=' \
    "${config}" > /tmp/_mower_config_clean.sh
  cp /tmp/_mower_config_clean.sh "${config}"

  cat >> "${config}" << 'PLATFORM_BLOCK'

# Platform config (auto) — added by setup_raspberry_ubuntu.sh
export HARDWARE_PLATFORM=2
export OM_V2=True
export MOWER=$OM_MOWER
export ESC_TYPE=$OM_MOWER_ESC_TYPE
export PARAMS_PATH=$HOME
export RECORDINGS_PATH=$HOME
PLATFORM_BLOCK

  # CUSTOM mower: create ~/mower_params/default_environment.sh if missing
  mkdir -p "${HOME}/mower_params"
  if [[ ! -f "${HOME}/mower_params/default_environment.sh" ]]; then
    cat > "${HOME}/mower_params/default_environment.sh" << 'MOWER_ENV'
# Custom mower hardware defaults — adjust to match your robot
export OM_ANTENNA_OFFSET_X=${OM_ANTENNA_OFFSET_X:-0.3}
export OM_ANTENNA_OFFSET_Y=${OM_ANTENNA_OFFSET_Y:-0.0}
export OM_WHEEL_DISTANCE_M=${OM_WHEEL_DISTANCE_M:-0.325}
export OM_WHEEL_TICKS_PER_M=${OM_WHEEL_TICKS_PER_M:-1600.0}
MOWER_ENV
    echo "Created ~/mower_params/default_environment.sh — adjust wheel/antenna values for your hardware."
  fi

  # Create ~/custom_params.yaml if missing (required for MOWER=CUSTOM)
  if [[ ! -f "${HOME}/custom_params.yaml" ]]; then
    cat > "${HOME}/custom_params.yaml" << 'CUSTOM_YAML'
# Custom mower params — adjust to match your robot
ll:
  services:
    sound:
      language: en
      volume: -1
    gps:
      baud_rate: 921600
      protocol: UBX
      datum_height: 0
      absolute_coords: true
    imu:
      axis_config: +X-Y-Z
    power:
      battery_critical_high_voltage: -1
      charge_critical_high_voltage: -1
      charge_critical_high_current: -1

xbot_positioning:
  max_gps_accuracy: 0.2
  debug: false

mower_logic:
  automatic_mode: 0
  docking_approach_distance: 1.5
  docking_extra_time: 0
  docking_retry_count: 4
  docking_redock: false
  outline_overlap_count: 0
  mow_angle_offset: 0
  mow_angle_offset_is_absolute: false
  mow_angle_increment: 0
  gps_wait_time: 10.0
  gps_timeout: 10.0
  rain_mode: 0
  outline_count: 4
  outline_offset: 0.05
  tool_width: 0.13
  battery_full_voltage: 28.5
  battery_empty_voltage: 24.0
  battery_critical_voltage: 23.0
  mow_motor_temp_high: 80.0
  mow_motor_temp_low: 40.0
  undock_distance: 2.0
  docking_distance: 1.0
CUSTOM_YAML
    echo "Created ~/custom_params.yaml — adjust battery/tool/dock values for your hardware."
  fi

  echo "mower_config.sh configured."
}

if [[ "${COPY_CONFIG}" -eq 1 ]]; then
  setup_mower_config
fi

write_default_env
install_wifi_service
setup_shell_env
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

The shell environment is now configured automatically in ~/.bashrc.
Open a new terminal (or run: source ~/.bashrc) and launch directly:

  cd ${REPO_ROOT}
  roslaunch open_mower open_mower.launch

If you ever need to reload the environment manually in the current shell:
  source ~/.bashrc

Wi-Fi auto-connect:
  Edit ${ENV_FILE} with MOWER_WIFI_SSID and MOWER_WIFI_PASSWORD.
  The openmower-wifi.service will try to connect on boot.
EOF
