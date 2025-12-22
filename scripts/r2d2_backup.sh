#!/usr/bin/env bash
#
# r2d2_backup.sh
#
# Purpose: Create a date-versioned backup of the R2D2 Jetson system to USB stick,
# including project code, data, configuration, and training artifacts.
#
# Usage:
#   bash r2d2_backup.sh
#
# Output:
#   Backup written to /media/severin/R2D2_BACKUP/r2d2_backup_YYYYmmdd/
#   Retains the last 5 backups automatically.
#

set -euo pipefail

# ============================================================================
# CONFIGURATION
# ============================================================================

# USB stick label (must be labeled R2D2_BACKUP for auto-detection)
USB_LABEL="R2D2_BACKUP"
USB_MOUNT_BASE="/media/${USER}"

# Retention count (how many backups to keep)
RETENTION_COUNT=5

# Timestamp for this backup (date only, one backup per day)
BACKUP_DATE=$(date +%Y%m%d)
BACKUP_FOLDER_NAME="r2d2_backup_${BACKUP_DATE}"

# Project paths to back up
PROJECT_ROOT="${HOME}/dev/r2d2"

# System paths (backup from root, will be restored relative to root)
SYSTEM_UDEV_RULES="/etc/udev/rules.d/r2d2_*"
SYSTEM_SYSTEMD_SERVICES="/etc/systemd/system/r2d2_*.service"

# User config paths
HOME_BASHRC="${HOME}/.bashrc"
HOME_BASH_ALIASES="${HOME}/.bash_aliases"
HOME_CONFIG_DIR="${HOME}/.config/r2d2"

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
# USB DETECTION
# ============================================================================

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
    log_error "" >&2
    log_error "To relabel your USB stick:" >&2
    log_error "  sudo fatlabel /dev/sda1 ${USB_LABEL}  # For FAT32" >&2
    log_error "  sudo e2label /dev/sda1 ${USB_LABEL}   # For ext4" >&2
    exit 1
}

# ============================================================================
# MAIN BACKUP LOGIC
# ============================================================================

main() {
    log_section "R2D2 Jetson USB Backup"
    log_info "Starting backup at $(date)"
    
    # Find USB stick
    local usb_path=$(find_usb_stick)
    local backup_root="${usb_path}"
    local backup_folder="${backup_root}/${BACKUP_FOLDER_NAME}"
    
    # Check if backup for today already exists
    if [[ -d "${backup_folder}" ]]; then
        log_warn "Backup folder for today already exists: ${BACKUP_FOLDER_NAME}"
        read -p "Overwrite existing backup? (yes/no): " confirm
        if [[ "${confirm}" != "yes" ]]; then
            log_info "Backup cancelled"
            exit 0
        fi
        log_info "Removing existing backup folder..."
        rm -rf "${backup_folder}"
    fi
    
    # Create backup folder
    log_info "Creating backup folder: ${BACKUP_FOLDER_NAME}"
    mkdir -p "${backup_folder}"
    
    local backup_archive="${backup_folder}/r2d2_backup.tar.gz"
    log_info "Backup destination: ${backup_archive}"

    # Create temporary directory for staging
    local temp_staging=$(mktemp -d)
    trap "rm -rf ${temp_staging}" EXIT

    log_info "Staging files for backup..."

    # Stage project root (excluding build/install/log, __pycache__, .git)
    if [[ -d ${PROJECT_ROOT} ]]; then
        log_info "Staging project root: ${PROJECT_ROOT}"
        mkdir -p "${temp_staging}/home_staging/dev/r2d2"
        tar --exclude='build' \
            --exclude='install' \
            --exclude='log' \
            --exclude='__pycache__' \
            --exclude='.git' \
            --exclude='*.pyc' \
            --exclude='*.pyo' \
            -C "${PROJECT_ROOT}" \
            -cf - . | tar -xf - -C "${temp_staging}/home_staging/dev/r2d2"
    else
        log_warn "Project root not found: ${PROJECT_ROOT}"
    fi

    # Stage user config files
    log_info "Staging user configuration files..."
    mkdir -p "${temp_staging}/home_staging"

    if [[ -f ${HOME_BASHRC} ]]; then
        cp "${HOME_BASHRC}" "${temp_staging}/home_staging/.bashrc"
    fi

    if [[ -f ${HOME_BASH_ALIASES} ]]; then
        cp "${HOME_BASH_ALIASES}" "${temp_staging}/home_staging/.bash_aliases"
    fi

    if [[ -d ${HOME_CONFIG_DIR} ]]; then
        log_info "Staging ~/.config/r2d2"
        mkdir -p "${temp_staging}/home_staging/.config"
        cp -r "${HOME_CONFIG_DIR}" "${temp_staging}/home_staging/.config/"
    fi

    # Stage system configuration files (if they exist)
    log_info "Staging system configuration files..."
    mkdir -p "${temp_staging}/etc_staging/udev/rules.d"
    mkdir -p "${temp_staging}/etc_staging/systemd/system"

    # Backup udev rules
    if compgen -G "${SYSTEM_UDEV_RULES}" > /dev/null 2>&1; then
        log_info "Found udev rules, backing up..."
        for rule_file in ${SYSTEM_UDEV_RULES}; do
            if [[ -f ${rule_file} ]]; then
                cp "${rule_file}" "${temp_staging}/etc_staging/udev/rules.d/"
                log_info "  Backed up: $(basename ${rule_file})"
            fi
        done
    else
        log_warn "No R2D2 udev rules found"
    fi

    # Backup systemd services
    if compgen -G "${SYSTEM_SYSTEMD_SERVICES}" > /dev/null 2>&1; then
        log_info "Found systemd services, backing up..."
        for service_file in ${SYSTEM_SYSTEMD_SERVICES}; do
            if [[ -f ${service_file} ]]; then
                cp "${service_file}" "${temp_staging}/etc_staging/systemd/system/"
                log_info "  Backed up: $(basename ${service_file})"
            fi
        done
    else
        log_warn "No R2D2 systemd services found"
    fi

    # Create metadata file
    log_info "Creating backup metadata..."
    cat > "${temp_staging}/BACKUP_METADATA.txt" << EOF
R2D2 Backup Information
=======================
Created: $(date)
Hostname: $(hostname)
Kernel: $(uname -r)
ROS 2: $(. /opt/ros/humble/setup.bash 2>/dev/null && ros2 --version 2>/dev/null || echo "Not installed")

Included:
- Project files from ~/dev/r2d2 (excluding build/, install/, log/, __pycache__, .git)
- User bashrc and bash_aliases from ~
- User config from ~/.config/r2d2
- System udev rules from /etc/udev/rules.d/r2d2_*
- System systemd services from /etc/systemd/system/r2d2_*.service

Excluded:
- Build artifacts (build/, install/, log/)
- Python cache (__pycache__, *.pyc)
- Git metadata (.git/)
- ROS 2 workspace build artifacts

Usage:
See 004_BACKUP_AND_RESTORE.md for restore instructions.
EOF

    # Create the backup archive
    log_info "Creating backup archive..."
    log_info "This may take a few minutes..."

    cd "${temp_staging}"
    tar -czf "${backup_archive}" \
        home_staging/ \
        etc_staging/ \
        BACKUP_METADATA.txt \
        2>&1 | head -20

    if [[ ! -f ${backup_archive} ]]; then
        log_error "Backup file was not created!"
        exit 1
    fi

    # Copy metadata alongside archive
    cp BACKUP_METADATA.txt "${backup_folder}/BACKUP_METADATA.txt"

    local backup_size=$(du -h "${backup_archive}" | cut -f1)
    log_info "Backup created successfully: ${backup_size}"
    log_info "Location: ${backup_folder}"

    # Cleanup old backups
    cleanup_old_backups "${backup_root}"

    log_section "Backup Complete"
    log_info "Backup finished at $(date)"
    log_info "Folder: ${BACKUP_FOLDER_NAME}"
    log_info "Archive: r2d2_backup.tar.gz (${backup_size})"
    log_info "USB Location: ${usb_path}"
    echo ""
    log_info "You can now safely eject the USB stick"
}

# ============================================================================
# CLEANUP OLD BACKUPS
# ============================================================================

cleanup_old_backups() {
    local backup_root="$1"
    log_info "Cleaning up old backups (keeping last ${RETENTION_COUNT})..."

    local backup_count=$(find "${backup_root}" -maxdepth 1 -type d -name "r2d2_backup_*" 2>/dev/null | wc -l)
    
    if [[ ${backup_count} -le ${RETENTION_COUNT} ]]; then
        log_info "Backup count (${backup_count}) is within retention limit"
        return 0
    fi

    local excess=$((backup_count - RETENTION_COUNT))
    log_info "Found ${backup_count} backups, removing oldest ${excess}..."

    # Sort by modification time (oldest first) and remove excess
    find "${backup_root}" -maxdepth 1 -type d -name "r2d2_backup_*" -printf "%T@ %p\n" | \
        sort -n | head -n "${excess}" | cut -d' ' -f2- | while read -r old_backup; do
        log_info "Removing old backup: $(basename ${old_backup})"
        rm -rf "${old_backup}"
    done

    log_info "Cleanup complete"
}

# ============================================================================
# ENTRY POINT
# ============================================================================

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
