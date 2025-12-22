#!/usr/bin/env bash
#
# r2d2_restore.sh
#
# Purpose: Restore a backup from USB stick to a fresh Jetson that has already been
# set up with r2d2_setup.sh.
#
# Usage:
#   bash r2d2_restore.sh [backup_folder_or_archive]
#   bash r2d2_restore.sh --yes [backup_folder_or_archive]  (skip confirmation)
#
# If no path is provided, uses the latest backup from USB stick.
#

set -euo pipefail

# ============================================================================
# CONFIGURATION
# ============================================================================

TARGET_USER="${TARGET_USER:-severin}"
TARGET_HOME="/home/${TARGET_USER}"

# USB stick label (must be labeled R2D2_BACKUP)
USB_LABEL="R2D2_BACKUP"
USB_MOUNT_BASE="/media/${TARGET_USER}"

PROJECT_ROOT="${TARGET_HOME}/dev/r2d2"
ROS2_WS="${PROJECT_ROOT}/ros2_ws"

UDEV_RULES_DIR="/etc/udev/rules.d"
SYSTEMD_SERVICES_DIR="/etc/systemd/system"

# Flags
SKIP_CONFIRMATION=false
BACKUP_PATH=""
BACKUP_ARCHIVE=""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# ============================================================================
# LOGGING
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

log_section() {
    echo -e "${BLUE}=== $* ===${NC}"
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
}

find_usb_stick() {
    log_info "Looking for USB stick labeled '${USB_LABEL}'..." >&2
    
    # Check if USB is mounted at expected location
    local usb_path="${USB_MOUNT_BASE}/${USB_LABEL}"
    
    if [[ -d "${usb_path}" ]]; then
        log_info "Found USB stick at: ${usb_path}" >&2
        echo "${usb_path}"
        return 0
    fi
    
    # Alternative: search all mounted volumes
    for mount_point in "${USB_MOUNT_BASE}"/*; do
        if [[ -d "${mount_point}" ]]; then
            local label=$(basename "${mount_point}")
            if [[ "${label}" == "${USB_LABEL}" ]]; then
                log_info "Found USB stick at: ${mount_point}" >&2
                echo "${mount_point}"
                return 0
            fi
        fi
    done
    
    log_error "USB stick '${USB_LABEL}' not found!" >&2
    log_error "Please ensure:" >&2
    log_error "  1. USB stick is plugged in" >&2
    log_error "  2. USB stick is labeled '${USB_LABEL}'" >&2
    exit 1
}

find_latest_backup() {
    local usb_path=$(find_usb_stick)
    
    # Find most recent backup folder
    local latest_folder=$(find "${usb_path}" -maxdepth 1 -type d -name "r2d2_backup_*" -printf "%T@ %p\n" 2>/dev/null | \
        sort -rn | head -1 | cut -d' ' -f2-)
    
    if [[ -z "${latest_folder}" ]]; then
        log_error "No backups found on USB stick!"
        log_error "Searched in: ${usb_path}"
        exit 1
    fi
    
    local archive="${latest_folder}/r2d2_backup.tar.gz"
    
    if [[ ! -f "${archive}" ]]; then
        log_error "Backup folder found but archive missing: ${archive}"
        exit 1
    fi
    
    echo "${archive}"
}

resolve_backup_path() {
    local input_path="$1"
    
    # If it's a directory, look for the archive inside
    if [[ -d "${input_path}" ]]; then
        local archive="${input_path}/r2d2_backup.tar.gz"
        if [[ -f "${archive}" ]]; then
            echo "${archive}"
            return 0
        else
            log_error "Backup folder found but archive missing: ${archive}"
            exit 1
        fi
    fi
    
    # If it's a file, use it directly
    if [[ -f "${input_path}" ]]; then
        echo "${input_path}"
        return 0
    fi
    
    log_error "Backup not found: ${input_path}"
    exit 1
}

parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --yes)
                SKIP_CONFIRMATION=true
                shift
                ;;
            *)
                BACKUP_PATH="$1"
                shift
                ;;
        esac
    done
}

confirm_restore() {
    if [[ ${SKIP_CONFIRMATION} == true ]]; then
        return 0
    fi

    echo ""
    log_warn "RESTORE WARNING"
    echo -e "${RED}This will restore files from the backup to your home directory and system.${NC}"
    echo -e "${RED}Existing files will be overwritten.${NC}"
    echo ""
    echo "Backup archive: ${BACKUP_ARCHIVE}"
    echo "Target user: ${TARGET_USER}"
    echo "Target home: ${TARGET_HOME}"
    echo ""
    read -p "Do you want to continue? (yes/no): " confirm
    
    if [[ ${confirm} != "yes" ]]; then
        log_info "Restore cancelled"
        exit 0
    fi
}

# ============================================================================
# MAIN RESTORE LOGIC
# ============================================================================

main() {
    log_section "R2D2 Jetson USB Restore"
    log_info "Starting restore at $(date)"

    check_root
    ensure_user_exists

    # Parse arguments
    parse_arguments "$@"

    # Determine backup archive to use
    if [[ -z ${BACKUP_PATH} ]]; then
        log_info "No backup specified, finding latest on USB stick..."
        BACKUP_ARCHIVE=$(find_latest_backup)
        log_info "Using latest backup: ${BACKUP_ARCHIVE}"
    else
        log_info "Resolving backup path: ${BACKUP_PATH}"
        BACKUP_ARCHIVE=$(resolve_backup_path "${BACKUP_PATH}")
    fi

    # Validate backup file exists
    if [[ ! -f ${BACKUP_ARCHIVE} ]]; then
        log_error "Backup archive not found: ${BACKUP_ARCHIVE}"
        exit 1
    fi

    log_info "Backup archive: ${BACKUP_ARCHIVE}"
    log_info "Backup size: $(du -h ${BACKUP_ARCHIVE} | cut -f1)"

    # Ask for confirmation
    confirm_restore

    # Create temporary extraction directory
    local temp_extract=$(mktemp -d)
    trap "rm -rf ${temp_extract}" EXIT

    log_section "Extracting Backup"
    log_info "Extracting to temporary directory..."
    
    tar -xzf "${BACKUP_ARCHIVE}" -C "${temp_extract}"
    log_info "Extraction complete"

    # Display backup metadata
    if [[ -f ${temp_extract}/BACKUP_METADATA.txt ]]; then
        echo ""
        log_info "Backup metadata:"
        cat "${temp_extract}/BACKUP_METADATA.txt" | sed 's/^/  /'
        echo ""
    fi

    # Restore home directory files
    log_section "Restoring Home Directory Files"
    
    if [[ -d ${temp_extract}/home_staging ]]; then
        log_info "Restoring files to ${TARGET_HOME}..."
        
        # Copy with verbose output
        cd "${temp_extract}/home_staging"
        
        # Restore project root
        if [[ -d dev/r2d2 ]]; then
            log_info "Restoring project root..."
            mkdir -p "${PROJECT_ROOT}"
            cp -r dev/r2d2/* "${PROJECT_ROOT}/" 2>&1 | head -10
            chown -R "${TARGET_USER}:${TARGET_USER}" "${PROJECT_ROOT}"
            log_info "Project root restored"
        fi

        # Restore bashrc
        if [[ -f .bashrc ]]; then
            log_info "Restoring .bashrc..."
            cp .bashrc "${TARGET_HOME}/.bashrc"
            chown "${TARGET_USER}:${TARGET_USER}" "${TARGET_HOME}/.bashrc"
        fi

        # Restore bash_aliases
        if [[ -f .bash_aliases ]]; then
            log_info "Restoring .bash_aliases..."
            cp .bash_aliases "${TARGET_HOME}/.bash_aliases"
            chown "${TARGET_USER}:${TARGET_USER}" "${TARGET_HOME}/.bash_aliases"
        fi

        # Restore config
        if [[ -d .config/r2d2 ]]; then
            log_info "Restoring ~/.config/r2d2..."
            mkdir -p "${TARGET_HOME}/.config"
            cp -r .config/r2d2 "${TARGET_HOME}/.config/"
            chown -R "${TARGET_USER}:${TARGET_USER}" "${TARGET_HOME}/.config"
        fi
    fi

    # Restore system files
    log_section "Restoring System Files"

    if [[ -d ${temp_extract}/etc_staging ]]; then
        log_info "Restoring system configuration..."

        # Restore udev rules
        if [[ -d ${temp_extract}/etc_staging/udev/rules.d ]]; then
            log_info "Restoring udev rules..."
            for rule_file in "${temp_extract}"/etc_staging/udev/rules.d/*; do
                if [[ -f ${rule_file} ]]; then
                    rule_name=$(basename "${rule_file}")
                    log_info "  Restoring: ${rule_name}"
                    cp "${rule_file}" "${UDEV_RULES_DIR}/${rule_name}"
                fi
            done
            # Reload udev
            udevadm control --reload-rules
            udevadm trigger
            log_info "Udev rules reloaded"
        fi

        # Restore systemd services
        if [[ -d ${temp_extract}/etc_staging/systemd/system ]]; then
            log_info "Restoring systemd services..."
            for service_file in "${temp_extract}"/etc_staging/systemd/system/*.service; do
                if [[ -f ${service_file} ]]; then
                    service_name=$(basename "${service_file}")
                    log_info "  Restoring: ${service_name}"
                    cp "${service_file}" "${SYSTEMD_SERVICES_DIR}/${service_name}"
                fi
            done
            # Reload systemd
            systemctl daemon-reload
            log_info "Systemd daemon reloaded"
        fi
    fi

    # Rebuild ROS 2 workspace (install and build artifacts are excluded from backup)
    log_section "Rebuilding ROS 2 Workspace"
    log_info "Rebuilding ${ROS2_WS}..."

    if [[ -d ${ROS2_WS}/src ]]; then
        cd "${ROS2_WS}"
        
        log_info "Building workspace (this may take a few minutes)..."
        sudo -u "${TARGET_USER}" bash -c "
            source /opt/ros/humble/setup.bash
            export OPENBLAS_CORETYPE=ARMV8
            colcon build 2>&1 | tail -20
        " || log_warn "Workspace build had some issues (non-critical)"
        
        log_info "Workspace rebuild complete"
    fi

    # Final status
    log_section "Restore Complete"
    log_info "Restore finished at $(date)"
    echo ""
    log_info "Next steps:"
    echo "  1. Source the environment: source ${TARGET_HOME}/.bashrc"
    echo "  2. Verify system health:"
    echo "     - Check heartbeat: ros2 topic echo /r2d2/heartbeat -n 3"
    echo "     - Check topics: ros2 topic list"
    echo "  3. Test camera (if available):"
    echo "     - ros2 launch r2d2_camera oak_d_camera.launch.py"
    echo "  4. Test face recognition (if trained):"
    echo "     - ros2 launch face_recognition face_recognition.launch.py"
    echo ""
}

# ============================================================================
# ENTRY POINT
# ============================================================================

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
