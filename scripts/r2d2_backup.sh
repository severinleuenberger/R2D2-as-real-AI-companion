#!/usr/bin/env bash
#
# r2d2_backup.sh
#
# Purpose: Create a timestamped backup of the R2D2 Jetson system, including
# project code, data, configuration, and training artifacts.
#
# Usage:
#   bash r2d2_backup.sh
#
# Output:
#   Backup archive written to ~/backups/r2d2_backup_YYYYmmdd_HHMMSS.tar.gz
#   Retains the last 10 backups automatically.
#

set -euo pipefail

# ============================================================================
# CONFIGURATION
# ============================================================================

# Backup destination directory
BACKUP_DIR="${HOME}/backups"

# Retention count (how many backups to keep)
RETENTION_COUNT=10

# Timestamp for this backup
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BACKUP_FILE="${BACKUP_DIR}/r2d2_backup_${TIMESTAMP}.tar.gz"

# Project paths to back up
PROJECT_ROOT="${HOME}/dev/r2d2"

# System paths (backup from root, will be restored relative to root)
# These are specified as absolute paths but tar will store them with root prefix stripped
SYSTEM_UDEV_RULES="/etc/udev/rules.d/r2d2_*"
SYSTEM_SYSTEMD_SERVICES="/etc/systemd/system/r2d2_*.service"

# User config paths
HOME_BASHRC="${HOME}/.bashrc"
HOME_BASH_ALIASES="${HOME}/.bash_aliases"
HOME_CONFIG_DIR="${HOME}/.config/r2d2"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
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

log_section() {
    echo -e "${BLUE}=== $* ===${NC}"
}

# ============================================================================
# MAIN BACKUP LOGIC
# ============================================================================

main() {
    log_section "R2D2 Jetson Backup"
    log_info "Starting backup at $(date)"
    log_info "Backup destination: ${BACKUP_FILE}"

    # Create backup directory if it doesn't exist
    if [[ ! -d ${BACKUP_DIR} ]]; then
        log_info "Creating backup directory: ${BACKUP_DIR}"
        mkdir -p "${BACKUP_DIR}"
    fi

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
See 07_BACKUP_AND_RESTORE_SETUP.md for restore instructions.
EOF

    # Create the backup archive
    log_info "Creating backup archive..."
    log_info "This may take a few minutes..."

    cd "${temp_staging}"
    tar -czf "${BACKUP_FILE}" \
        home_staging/ \
        etc_staging/ \
        BACKUP_METADATA.txt \
        2>&1 | head -20

    if [[ ! -f ${BACKUP_FILE} ]]; then
        log_warn "ERROR: Backup file was not created!"
        exit 1
    fi

    local backup_size=$(du -h "${BACKUP_FILE}" | cut -f1)
    log_info "Backup created successfully: ${backup_size}"
    log_info "File: ${BACKUP_FILE}"

    # Cleanup old backups
    cleanup_old_backups

    log_section "Backup Complete"
    log_info "Backup finished at $(date)"
    log_info "Archive: ${BACKUP_FILE}"
    log_info "Size: ${backup_size}"
    echo ""
}

# ============================================================================
# CLEANUP OLD BACKUPS
# ============================================================================

cleanup_old_backups() {
    log_info "Cleaning up old backups (keeping last ${RETENTION_COUNT})..."

    local backup_count=$(ls -1 "${BACKUP_DIR}"/r2d2_backup_*.tar.gz 2>/dev/null | wc -l)
    
    if [[ ${backup_count} -le ${RETENTION_COUNT} ]]; then
        log_info "Backup count (${backup_count}) is within retention limit"
        return 0
    fi

    local excess=$((backup_count - RETENTION_COUNT))
    log_info "Found ${backup_count} backups, removing oldest ${excess}..."

    # Sort by modification time (oldest first) and remove excess
    ls -1t "${BACKUP_DIR}"/r2d2_backup_*.tar.gz | tail -n "${excess}" | while read -r old_backup; do
        log_info "Removing old backup: $(basename ${old_backup})"
        rm -f "${old_backup}"
    done

    log_info "Cleanup complete"
}

# ============================================================================
# ENTRY POINT
# ============================================================================

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
