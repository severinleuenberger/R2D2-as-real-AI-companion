# r2d2_backup.ps1
#
# Purpose: Windows PowerShell script to trigger R2D2 Jetson backup via SSH
# and copy the backup archive to OneDrive.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File r2d2_backup.ps1
#
# Configuration: Edit the parameters below before running.
#

# ============================================================================
# CONFIGURATION
# ============================================================================

# Jetson connection details
$JetsonUser = "severin"
$JetsonHost = "192.168.55.1"
$JetsonSSHKey = "$env:USERPROFILE\.ssh\id_rsa"

# Jetson backup script and directory
$JetsonBackupScript = "~/dev/r2d2/scripts/r2d2_backup.sh"
$JetsonBackupDir = "~/backups"

# Windows backup destination (OneDrive)
$WindowsBackupPath = "C:\Users\SeverinLeuenberger\OneDrive\Daten Severin\_Dev_Git_Projekte\R2D2"

# Optional: Keep local copies on Jetson (yes/no)
$KeepJetsonBackups = "yes"

# Optional: Remove local copies after transfer (yes/no)
$RemoveAfterTransfer = "no"

# Log file
$LogFile = "$env:USERPROFILE\Documents\r2d2_backup_$(Get-Date -Format 'yyyyMMdd_HHmmss').log"

# ============================================================================
# LOGGING FUNCTIONS
# ============================================================================

function Write-Log {
    param(
        [string]$Message,
        [string]$Level = "INFO"
    )
    
    $timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    $logMessage = "[$timestamp] [$Level] $Message"
    
    Write-Host $logMessage
    Add-Content -Path $LogFile -Value $logMessage
}

function Write-Info {
    Write-Log -Message $args[0] -Level "INFO"
}

function Write-Warn {
    Write-Host "WARNING: $($args[0])" -ForegroundColor Yellow
    Write-Log -Message $args[0] -Level "WARN"
}

function Write-Error-Log {
    Write-Host "ERROR: $($args[0])" -ForegroundColor Red
    Write-Log -Message $args[0] -Level "ERROR"
}

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

function Test-SSHConnection {
    Write-Info "Testing SSH connection to ${JetsonUser}@${JetsonHost}..."
    
    try {
        $result = ssh -i $JetsonSSHKey -o ConnectTimeout=5 -o StrictHostKeyChecking=no `
            "${JetsonUser}@${JetsonHost}" "echo 'SSH connection OK'"
        
        if ($LASTEXITCODE -eq 0) {
            Write-Info "SSH connection successful"
            return $true
        } else {
            Write-Error-Log "SSH connection failed"
            return $false
        }
    } catch {
        Write-Error-Log "SSH connection error: $_"
        return $false
    }
}

function Invoke-JetsonBackup {
    Write-Info "Triggering backup on Jetson..."
    
    try {
        $output = ssh -i $JetsonSSHKey -o ConnectTimeout=10 `
            "${JetsonUser}@${JetsonHost}" "bash ${JetsonBackupScript}"
        
        Write-Info "Backup script output:"
        $output | ForEach-Object { Write-Info "  $_" }
        
        if ($LASTEXITCODE -eq 0) {
            Write-Info "Backup completed successfully on Jetson"
            return $true
        } else {
            Write-Warn "Backup may have encountered issues"
            return $true  # Continue anyway to try copying
        }
    } catch {
        Write-Error-Log "Failed to run backup on Jetson: $_"
        return $false
    }
}

function Get-LatestBackupFile {
    Write-Info "Finding latest backup on Jetson..."
    
    try {
        $latestBackup = ssh -i $JetsonSSHKey -o ConnectTimeout=10 `
            "${JetsonUser}@${JetsonHost}" "ls -1t ${JetsonBackupDir}/r2d2_backup_*.tar.gz 2>/dev/null | head -1"
        
        if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($latestBackup)) {
            Write-Info "Latest backup found: $latestBackup"
            return $latestBackup.Trim()
        } else {
            Write-Error-Log "No backup files found on Jetson"
            return $null
        }
    } catch {
        Write-Error-Log "Failed to find latest backup: $_"
        return $null
    }
}

function Copy-BackupToWindows {
    param(
        [string]$RemoteBackupFile
    )
    
    if ([string]::IsNullOrWhiteSpace($RemoteBackupFile)) {
        Write-Error-Log "Backup file path is empty"
        return $false
    }

    Write-Info "Copying backup to Windows..."
    Write-Info "Source: ${JetsonUser}@${JetsonHost}:${RemoteBackupFile}"
    Write-Info "Destination: ${WindowsBackupPath}"

    # Ensure destination directory exists
    if (-not (Test-Path -Path $WindowsBackupPath)) {
        Write-Info "Creating backup directory: ${WindowsBackupPath}"
        New-Item -ItemType Directory -Force -Path $WindowsBackupPath | Out-Null
    }

    try {
        $backupFilename = Split-Path -Leaf $RemoteBackupFile
        $destinationFile = Join-Path -Path $WindowsBackupPath -ChildPath $backupFilename
        
        Write-Info "Copying file (this may take several minutes)..."
        Write-Info "Size check on Jetson:"
        ssh -i $JetsonSSHKey "${JetsonUser}@${JetsonHost}" "du -h ${RemoteBackupFile}" | ForEach-Object { Write-Info "  $_" }
        
        # Use scp to copy the file
        scp -i $JetsonSSHKey -o ConnectTimeout=10 `
            "${JetsonUser}@${JetsonHost}:${RemoteBackupFile}" `
            "$destinationFile"
        
        if ($LASTEXITCODE -eq 0) {
            $fileSize = (Get-Item -Path $destinationFile).Length / 1GB
            Write-Info "Backup copied successfully"
            Write-Info "File size: {0:F2} GB" -f $fileSize
            Write-Info "Destination: ${destinationFile}"
            
            return $true
        } else {
            Write-Error-Log "Copy operation failed"
            return $false
        }
    } catch {
        Write-Error-Log "Error during copy: $_"
        return $false
    }
}

function Cleanup-RemoteBackup {
    param(
        [string]$RemoteBackupFile
    )
    
    if ($RemoveAfterTransfer -ne "yes") {
        Write-Info "Keeping remote backup on Jetson (RemoveAfterTransfer = ${RemoveAfterTransfer})"
        return $true
    }

    Write-Info "Removing backup from Jetson to save space..."
    
    try {
        ssh -i $JetsonSSHKey "${JetsonUser}@${JetsonHost}" "rm -f ${RemoteBackupFile}"
        
        if ($LASTEXITCODE -eq 0) {
            Write-Info "Remote backup removed"
            return $true
        } else {
            Write-Warn "Failed to remove remote backup"
            return $true  # Non-critical
        }
    } catch {
        Write-Warn "Error removing remote backup: $_"
        return $true  # Non-critical
    }
}

# ============================================================================
# MAIN BACKUP WORKFLOW
# ============================================================================

function Main {
    Write-Info "=========================================="
    Write-Info "R2D2 Jetson Backup (Windows)"
    Write-Info "=========================================="
    Write-Info "Start time: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
    Write-Info "Log file: ${LogFile}"
    Write-Info ""

    # Step 1: Test SSH connection
    Write-Info "Step 1: Testing SSH connection"
    if (-not (Test-SSHConnection)) {
        Write-Error-Log "Cannot proceed without SSH connection"
        Read-Host "Press Enter to exit"
        exit 1
    }

    # Step 2: Trigger backup on Jetson
    Write-Info ""
    Write-Info "Step 2: Triggering backup on Jetson"
    if (-not (Invoke-JetsonBackup)) {
        Write-Error-Log "Backup failed on Jetson"
        Read-Host "Press Enter to exit"
        exit 1
    }

    # Step 3: Find latest backup
    Write-Info ""
    Write-Info "Step 3: Finding latest backup"
    $latestBackup = Get-LatestBackupFile
    if ($null -eq $latestBackup) {
        Write-Error-Log "Could not find backup file"
        Read-Host "Press Enter to exit"
        exit 1
    }

    # Step 4: Copy to Windows
    Write-Info ""
    Write-Info "Step 4: Copying backup to Windows"
    if (-not (Copy-BackupToWindows -RemoteBackupFile $latestBackup)) {
        Write-Error-Log "Failed to copy backup to Windows"
        Read-Host "Press Enter to exit"
        exit 1
    }

    # Step 5: Optional cleanup
    Write-Info ""
    Write-Info "Step 5: Cleanup"
    Cleanup-RemoteBackup -RemoteBackupFile $latestBackup

    # Final summary
    Write-Info ""
    Write-Info "=========================================="
    Write-Info "Backup Completed Successfully"
    Write-Info "=========================================="
    Write-Info "End time: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
    Write-Info "Backup location: ${WindowsBackupPath}"
    Write-Info ""
    Write-Info "Log saved to: ${LogFile}"
    Write-Info ""
    
    Read-Host "Press Enter to exit"
}

# ============================================================================
# ENTRY POINT
# ============================================================================

Main
