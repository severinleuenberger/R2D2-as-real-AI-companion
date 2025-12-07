#!/usr/bin/env bash
#
# r2d2_setup.sh
# 
# Purpose: Complete setup script for a freshly flashed Jetson AGX Orin running
# Ubuntu 22.04 (L4T) + JetPack 6.x. Installs all dependencies, ROS 2, clones
# the repository, and configures system-level components.
#
# Usage:
#   bash r2d2_setup.sh
#
# This script is idempotent and safe to run multiple times.
#

set -euo pipefail

# ============================================================================
# CONFIGURATION
# ============================================================================

# Target user (typically 'severin' on the Jetson)
TARGET_USER="${TARGET_USER:-severin}"
TARGET_HOME="/home/${TARGET_USER}"

# Project paths
PROJECT_ROOT="${TARGET_HOME}/dev/r2d2"
REPO_URL="https://github.com/severinleuenberger/R2D2-as-real-AI-companion.git"
ROS2_WS="${PROJECT_ROOT}/ros2_ws"

# System-level config directories
UDEV_RULES_DIR="/etc/udev/rules.d"
SYSTEMD_SERVICES_DIR="/etc/systemd/system"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ============================================================================
# LOGGING FUNCTIONS
# ============================================================================

log_info() {
    echo -e "${GREEN}[INFO]${NC} $*"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $*"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $*"
}

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This script must be run as root (or with sudo)"
        exit 1
    fi
}

ensure_user_exists() {
    if ! id "${TARGET_USER}" &>/dev/null; then
        log_error "User '${TARGET_USER}' does not exist"
        exit 1
    fi
    log_info "User '${TARGET_USER}' exists"
}

apt_update() {
    log_info "Updating package lists..."
    apt-get update
}

apt_install() {
    local packages=("$@")
    log_info "Installing packages: ${packages[*]}"
    apt-get install -y "${packages[@]}" || {
        log_error "Failed to install packages"
        exit 1
    }
}

# ============================================================================
# MAIN SETUP
# ============================================================================

main() {
    log_info "Starting R2D2 Jetson setup..."
    log_info "Target user: ${TARGET_USER}"
    log_info "Target home: ${TARGET_HOME}"
    log_info "Project root: ${PROJECT_ROOT}"

    check_root
    ensure_user_exists

    # Step 1: System updates and essential tools
    log_info "Step 1: System update and essential packages"
    apt_update
    apt_install \
        build-essential \
        cmake \
        git \
        curl \
        wget \
        python3-dev \
        python3-pip \
        python3-venv \
        nano \
        vim \
        htop \
        tmux

    # Step 2: Install ROS 2 Humble
    log_info "Step 2: Installing ROS 2 Humble"
    if command -v ros2 &>/dev/null; then
        log_warn "ROS 2 is already installed, skipping installation"
    else
        # Add ROS 2 repository
        curl -sSL https://repo.ros2.org/ros.key | apt-key add -
        
        # Add ROS 2 apt sources
        sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list'
        
        apt_update
        apt_install ros-humble-desktop
        
        log_info "ROS 2 Humble installed successfully"
    fi

    # Step 3: Install ROS 2 build tools
    log_info "Step 3: Installing ROS 2 build tools"
    apt_install \
        ros-humble-ros-core \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool

    # Initialize rosdep (only once)
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        rosdep init || true
    fi
    rosdep update || true

    # Step 4: Set up project directories
    log_info "Step 4: Setting up project directories"
    if [[ ! -d ${PROJECT_ROOT} ]]; then
        mkdir -p "${PROJECT_ROOT}"
        chown "${TARGET_USER}:${TARGET_USER}" "${PROJECT_ROOT}"
        log_info "Created ${PROJECT_ROOT}"
    fi

    # Step 5: Clone or update the repository
    log_info "Step 5: Clone/update R2D2 repository"
    if [[ -d ${PROJECT_ROOT}/.git ]]; then
        log_info "Repository already exists, pulling latest..."
        cd "${PROJECT_ROOT}"
        sudo -u "${TARGET_USER}" git pull origin master || log_warn "Git pull encountered an issue"
    else
        log_info "Cloning repository from ${REPO_URL}..."
        cd "${TARGET_HOME}/dev"
        sudo -u "${TARGET_USER}" git clone "${REPO_URL}" r2d2
    fi

    chown -R "${TARGET_USER}:${TARGET_USER}" "${PROJECT_ROOT}"

    # Step 6: Create ROS 2 workspace (if not already present)
    log_info "Step 6: Setting up ROS 2 workspace"
    if [[ ! -d ${ROS2_WS}/src ]]; then
        log_info "Creating ROS 2 workspace at ${ROS2_WS}"
        mkdir -p "${ROS2_WS}/src"
        chown -R "${TARGET_USER}:${TARGET_USER}" "${ROS2_WS}"
    fi

    # Step 7: Build ROS 2 workspace
    log_info "Step 7: Building ROS 2 workspace"
    cd "${ROS2_WS}"
    
    # Install dependencies first
    if command -v rosdep &>/dev/null; then
        sudo -u "${TARGET_USER}" bash -c "
            source /opt/ros/humble/setup.bash
            rosdep install --from-paths src --ignore-src -y --rosdistro humble
        " || log_warn "rosdep install had some issues (non-critical)"
    fi

    # Build workspace
    sudo -u "${TARGET_USER}" bash -c "
        source /opt/ros/humble/setup.bash
        export OPENBLAS_CORETYPE=ARMV8
        colcon build --packages-select r2d2_hello r2d2_bringup 2>&1 | tail -20
    " || log_warn "colcon build had some issues"

    # Step 8: Configure bash for the user
    log_info "Step 8: Configuring bash environment"
    setup_user_bashrc

    # Step 9: Install DepthAI environment for camera
    log_info "Step 9: Setting up DepthAI Python environment"
    setup_depthai_env

    # Step 10: Link system-level configs (if present in repo)
    log_info "Step 10: Linking system-level configurations"
    link_system_configs

    log_info "=====================================================​"
    log_info "R2D2 Jetson setup completed successfully!"
    log_info "=====================================================​"
    log_info ""
    log_info "Next steps:"
    log_info "1. Source the ROS 2 environment:"
    log_info "   source ~/.bashrc"
    log_info ""
    log_info "2. Start the R2D2 system:"
    log_info "   ros2 launch r2d2_bringup bringup.launch.py"
    log_info ""
    log_info "3. Monitor the heartbeat topic:"
    log_info "   ros2 topic echo /r2d2/heartbeat"
    log_info ""
}

# ============================================================================
# BASH CONFIGURATION
# ============================================================================

setup_user_bashrc() {
    local bashrc_file="${TARGET_HOME}/.bashrc"
    local setup_block="# R2D2 ROS 2 Configuration"
    
    # Check if configuration already exists
    if grep -q "R2D2 ROS 2 Configuration" "${bashrc_file}" 2>/dev/null; then
        log_warn "R2D2 configuration already in ${bashrc_file}"
        return 0
    fi

    log_info "Adding R2D2 configuration to ${bashrc_file}"

    cat >> "${bashrc_file}" << 'EOF'

# R2D2 ROS 2 Configuration
export OPENBLAS_CORETYPE=ARMV8
if [[ -f ~/depthai_env/bin/activate ]]; then
    source ~/depthai_env/bin/activate
fi
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi
if [[ -f ~/dev/r2d2/ros2_ws/install/setup.bash ]]; then
    source ~/dev/r2d2/ros2_ws/install/setup.bash
fi

# R2D2 Aliases
alias r2d2_build='cd ~/dev/r2d2/ros2_ws && colcon build --packages-select'
alias r2d2_run='ros2 launch r2d2_bringup bringup.launch.py'
alias r2d2_status='ros2 topic echo /r2d2/heartbeat -n 3'

EOF

    chown "${TARGET_USER}:${TARGET_USER}" "${bashrc_file}"
    log_info "Bash configuration updated"
}

# ============================================================================
# DEPTHAI SETUP
# ============================================================================

setup_depthai_env() {
    local venv_path="${TARGET_HOME}/depthai_env"

    if [[ -d ${venv_path} ]]; then
        log_warn "DepthAI virtual environment already exists at ${venv_path}"
        return 0
    fi

    log_info "Creating DepthAI virtual environment at ${venv_path}"
    
    sudo -u "${TARGET_USER}" python3 -m venv "${venv_path}"
    
    log_info "Installing DepthAI and dependencies..."
    sudo -u "${TARGET_USER}" bash -c "
        source ${venv_path}/bin/activate
        pip install --upgrade pip setuptools wheel
        pip install depthai opencv-python numpy
    " || log_warn "DepthAI installation had some issues"

    log_info "DepthAI environment ready"
}

# ============================================================================
# SYSTEM CONFIGURATION LINKING
# ============================================================================

link_system_configs() {
    log_info "Checking for system-level configurations to link..."

    # Check for udev rules in the repository
    local repo_udev="${PROJECT_ROOT}/etc/udev/rules.d"
    if [[ -d ${repo_udev} ]]; then
        log_info "Found udev rules in repository"
        for rule_file in "${repo_udev}"/*.rules; do
            if [[ -f ${rule_file} ]]; then
                local rule_name=$(basename "${rule_file}")
                log_info "Linking udev rule: ${rule_name}"
                ln -sf "${rule_file}" "${UDEV_RULES_DIR}/${rule_name}" || \
                    log_warn "Failed to link ${rule_name}"
            fi
        done
        # Reload udev rules
        udevadm control --reload-rules
        udevadm trigger
    fi

    # Check for systemd services in the repository
    local repo_systemd="${PROJECT_ROOT}/etc/systemd/system"
    if [[ -d ${repo_systemd} ]]; then
        log_info "Found systemd services in repository"
        for service_file in "${repo_systemd}"/*.service; do
            if [[ -f ${service_file} ]]; then
                local service_name=$(basename "${service_file}")
                log_info "Linking systemd service: ${service_name}"
                ln -sf "${service_file}" "${SYSTEMD_SERVICES_DIR}/${service_name}" || \
                    log_warn "Failed to link ${service_name}"
            fi
        done
        # Reload systemd daemon
        systemctl daemon-reload
    fi

    log_info "System configuration linking complete"
}

# ============================================================================
# ENTRY POINT
# ============================================================================

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
