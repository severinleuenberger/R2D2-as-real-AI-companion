# R2D2 Basic System Setup & Foundational Findings
**Project:** R2D2 as a Real AI Companion  
**Hardware Platform:** NVIDIA Jetson AGX Orin 64 GB Developer Kit  
**Host:** Windows 11 Laptop (BitLocker & Secure Boot enabled)

This document complements the more specialized guides:

- `CAMERA_SETUP_DOCUMENTATION.md` – OAK‑D Lite camera + DepthAI + ROS 2 camera node
- `PERCEPTION_SETUP_DOCUMENTATION.md` – `r2d2_perception` node and full perception pipeline

While those focus on camera and perception, **this file documents the underlying system setup, development workflow, networking, display quirks, and Git strategy** that make the whole R2D2 project reliable and reproducible.

---

## 1. High‑Level Architecture Overview

### 1.1 Roles of Each Machine

- **Windows 11 Laptop**
  - Primary “owner” machine.
  - Must *never* be modified in a way that risks BitLocker or boot configuration.
  - Used for:
    - Running Ubuntu from USB (for flashing / SDK Manager).
    - Remote development via VS Code + SSH.
    - Simple file backup (no Git operations).

- **Ubuntu-on-USB Host (64 GB stick)**
  - Persistent Ubuntu 24.04 Desktop installed directly onto a 64 GB USB stick.
  - Used as the **NVIDIA SDK Manager host** for flashing the Jetson.
  - Isolated from Windows: bootloader and partitions live *only* on the USB stick.

- **Jetson AGX Orin 64 GB (“R2D2”)**
  - Target system, running:
    - Ubuntu 22.04 (L4T) + JetPack 6.2.1.
    - ROS 2 Humble.
    - Camera and perception nodes (see the other docs).
  - Hosts the **only active Git repository** for this project.
  - Primary runtime and development environment.

### 1.2 Software Stacks (by layer)

1. **System Layer (Jetson)**
   - Ubuntu 22.04 (L4T) + JetPack 6.2.1.
   - NVIDIA GPU drivers (`nvidia-smi` OK), CUDA, cuDNN.

2. **Robot Stack (Jetson)**
   - ROS 2 Humble (`ros-humble-desktop`).
   - Workspace: `~/dev/r2d2/ros2_ws`.
   - Base nodes: `r2d2_hello`, `r2d2_bringup`.
   - Camera and perception nodes: `r2d2_camera`, `r2d2_perception`.

3. **Tooling (Host + Jetson)**
   - VS Code Remote‑SSH from Windows to Jetson.
   - Git (only on Jetson).
   - DepthAI environment (`~/depthai_env`) for OAK‑D integration (documented elsewhere).

---

## 2. Flashing & Host Setup (Ubuntu on USB)

### 2.1 Motivation

Initial attempts used:

- A VirtualBox VM for flashing → **APX/USB passthrough unreliable**.
- Live Ubuntu (`Try Ubuntu`) → root filesystem (`/cow`) only ~11–12 GB → **too small** for JetPack 6.x + images.

Result: SDK Manager repeatedly failed due to insufficient space and unstable USB/APX connectivity.

### 2.2 Final Working Solution

**Install Ubuntu 24.04 Desktop directly onto a 64 GB USB stick**

Partition layout on the USB stick (`/dev/sda` on the host):

- `sda1` – EFI System Partition (FAT32, ~1 GB, mounted at `/boot/efi`).
- `sda2` – Root Partition (ext4, ~56 GB, mounted at `/`).

Important constraints and checks:

- The **Windows NVMe drive (BitLocker)** remains untouched.
- Before installation, use tools like `lsblk`, `wmic` or `diskpart` on Windows to carefully verify:
  - Disk 0 = internal Windows NVMe.
  - Disk 1 = USB stick for Ubuntu.
- In the Ubuntu installer, explicitly select the USB stick as:
  - Installation target.
  - Bootloader target.

After installation:

- Booting from the USB yields a full Ubuntu system with ~40 GB free on `/`.
- SDK Manager and NGC can operate safely with plenty of disk space.

### 2.3 SDK Manager & NGC Configuration

Key takeaways for using SDK Manager on this USB‑Ubuntu host:

1. **SDK Manager installation**
   - Install the `.deb` locally (e.g. `sdkmanager_2.3.0-xxxx_amd64.deb`) via:
     ```bash
     sudo apt install ./sdkmanager_xxx.deb
     ```

2. **NGC CLI installation & fix for 401 errors**
   - Install NGC CLI into a dedicated directory (e.g. `/opt/ngc-cli`) and symlink it into `PATH`:
     ```bash
     sudo ln -s /opt/ngc-cli/ngc /usr/local/bin/ngc
     ```
   - Run `ngc config set` and configure:
     - API key.
     - Org / team if applicable.
   - This fixes 401/“Invalid password” issues for `JETSON_DOCKER_IMAGE_COMP` pulls inside SDK Manager.

3. **Flash only what’s needed**
   - Use SDK Manager to flash:
     - Jetson Linux / L4T.
   - Avoid installing large, unnecessary host components to keep the environment lean.

### 2.4 Flashing Outcome

After successful flashing:

- Jetson boots into:
  - Ubuntu 22.04 L4T.
  - JetPack 6.2.1 kernel.
- Basic health checks:
  - `uname -a` → shows the expected Jetson kernel.
  - `nvidia-smi` → confirms GPU driver, CUDA, and device visibility.
  - `tegrastats` → confirms stable CPU/GPU and memory behavior.

---

## 3. Networking & Headless Access

### 3.1 USB Networking (Preferred Dev Path)

The Jetson is connected to the Windows laptop via USB‑C device port. Under JetPack:

- Jetson provides a **USB network interface** via a bridge (`l4tbr0`).
- Jetson default IP: `192.168.55.1`.
- Windows sees this as a new RNDIS network adapter and gets an IP in `192.168.55.x`.

This path allows **stable, headless development** even without a display attached to the Jetson.

### 3.2 SSH Key‑Based Login

#### On Windows (client):

1. Generate a key pair (only once):
   ```powershell
   ssh-keygen -t ed25519 -C "r2d2-key"
   ```

2. Public key path:
   ```powershell
   C:\Users\SeverinLeuenberger\.ssh\id_ed25519.pub
   ```

3. Create SSH config at:
   ```text
   C:\Users\SeverinLeuenberger\.ssh\config
   ```
   with content:
   ```text
   Host r2d2
       HostName 192.168.55.1
       User severin
       IdentityFile C:\Users\SeverinLeuenberger\.ssh\id_ed25519
   ```

> Note: The file must be named exactly `config` (no `.txt`).

#### On Jetson (server):

1. Ensure the SSH daemon is running:
   ```bash
   systemctl status ssh
   ```

2. Install the public key:
   ```bash
   mkdir -p ~/.ssh
   nano ~/.ssh/authorized_keys
   ```
   - Paste the content of `id_ed25519.pub` from Windows.

3. Set permissions:
   ```bash
   chmod 700 ~/.ssh
   chmod 600 ~/.ssh/authorized_keys
   ```

#### Result

From Windows, SSH works with a simple:

```bash
ssh r2d2
```

No password prompts, stable connection over USB.

### 3.3 Debugging SSH Problems

If SSH misbehaves:

1. Check connectivity:
   ```bash
   ping 192.168.55.1
   ```

2. On Windows, see what SSH *thinks* it will do:
   ```powershell
   ssh -G r2d2
   ```
   - Verify that `HostName`, `User`, and `IdentityFile` are correct.

3. On Jetson, verify server state and keys:
   ```bash
   systemctl status ssh
   ls -la ~/.ssh
   cat ~/.ssh/authorized_keys
   ```

### 3.4 VS Code Remote‑SSH

With the SSH setup above, VS Code on Windows can connect to the Jetson:

- Install the **“Remote – SSH”** extension.
- Add host `r2d2` to the SSH targets.
- Connect using “SSH: r2d2”.

Behavior:

- VS Code deploys a VS Code Server onto the Jetson (in `~/.vscode-server`).
- File Explorer in VS Code shows `/home/severin` (Jetson filesystem).
- Integrated terminal prompt becomes:
  ```bash
  severin@R2D2:~$
  ```

This makes the **Jetson the primary development machine**, while using Windows only as a thin client UI.

---

## 4. Display Setup & Known Issues (LG 5K vs. Dell)

### 4.1 Observed Behavior

The project uses:

- A Dell 4K monitor (works reliably under all conditions).
- An LG 5K (UltraWide / HDR) monitor (more challenging).

Problems observed with the LG 5K:

- Sometimes 5120×2160 @ 30 Hz, sometimes 3440×1440 @ 60 Hz.
- Occasionally: black screen and **no network** after boot.
- In problematic cases, the Jetson appeared to hang during very early boot (likely cboot / DisplayPort training issues).

### 4.2 Root Cause Hypothesis

- The NVIDIA bootloader (`cboot`) reads EDID and initializes DisplayPort before Linux boots.
- When the LG monitor is connected but still “waking up”, it may present incomplete or invalid EDID.
- This can cause cboot/DP link training to fail in a way that prevents a clean boot — the system doesn’t reach the state where network comes up (`l4tbr0` stays down).

### 4.3 Workable Operational Strategies

Two reliable strategies emerged:

1. **Safe Boot without LG attached**
   - Boot the Jetson with no LG connected (or with the Dell instead).
   - After boot completes and the system is reachable (e.g., via SSH), connect the LG.
   - Use `xrandr` or GNOME settings to set a stable resolution (see below).

2. **Use Dell as “safe boot monitor”**
   - Connect the Dell for all “critical” operations (flashing, firmware updates, bootloader changes).
   - Treat the LG as a high‑end user display for normal development, not as a requirement for system boot.

### 4.4 GNOME & Resolution Persistence

Within the running Linux system:

- Once a stable resolution is chosen (e.g., 3440×1440 @ 60 Hz), GNOME stores it in:
  - `~/.config/monitors.xml`
- After that, GNOME will automatically restore this resolution after login, making everyday use acceptable.

A small helper script and desktop launcher can enforce 3440×1440 explicitly:

```bash
#!/usr/bin/env bash
xrandr --output DP-0 --mode 3440x1440 --rate 60
```

This script is wrapped in a `.desktop` file placed on the desktop, allowing a **one‑click resolution fix**.

### 4.5 Practical Recommendation

- For stable operations, assume:
  - **Dell monitor or SSH** for sensitive work.
  - LG only for normal interactive sessions.
- Always keep SSH access over USB ready as a “rescue path” if the display misbehaves.

---

## 5. ROS 2 Base Workspace & Initial Nodes

### 5.1 Workspace Layout

Main project directory on the Jetson:

```text
~/dev/r2d2
├── ros2_ws/          # ROS 2 workspace
│   ├── src/
│   │   ├── r2d2_hello/      # Simple demo nodes
│   │   ├── r2d2_bringup/    # Launch files for base bringup
│   │   ├── r2d2_camera/     # OAK‑D camera package (see other doc)
│   │   └── r2d2_perception/ # Perception package (see other doc)
│   ├── build/
│   ├── install/
│   └── log/
├── tests/            # Non‑ROS tests (GPU, audio, camera, etc.)
├── docs/             # Documentation & photos
└── scripts/          # Helper scripts (e.g. resolution script)
```

ROS 2 environment basics:

```bash
# One‑time per shell
source /opt/ros/humble/setup.bash
cd ~/dev/r2d2/ros2_ws
colcon build
source install/setup.bash
```

### 5.2 `r2d2_hello` – First Custom Nodes

This package is used as a “hello world” for the robot:

- `beep_node.py`
  - Simple timer node.
  - Periodically logs: “Beep! R2D2 is alive.”
- `heartbeat_node.py`
  - Publishes `std_msgs/String` messages on `/r2d2/heartbeat`.
  - Can be monitored with:
    ```bash
    ros2 topic echo /r2d2/heartbeat
    ```

These nodes validate that:

- ROS 2 installation works.
- Custom packages build correctly with `colcon`.
- Launching nodes from installed packages works.

### 5.3 `r2d2_bringup` – Base Launch Infrastructure

This package contains launch files to simplify starting the system:

- `bringup.launch.py`
  - Starts `beep_node` and `heartbeat_node` together.
- Additional launch files (e.g. `r2d2_camera_perception.launch.py`) are defined in other docs and extend the same pattern.

Typical usage:

```bash
cd ~/dev/r2d2/ros2_ws
source install/setup.bash
ros2 launch r2d2_bringup bringup.launch.py
```

This establishes a consistent convention:
- The **bringup package** orchestrates which nodes form the “robot baseline” at startup.

### 5.4 Development Workflow Pattern

For any new ROS 2 package:

1. Create the package in `ros2_ws/src/` (e.g., `ros2 pkg create ...`).
2. Implement nodes (Python or C++) and register them via `setup.py`/CMake `console_scripts`/executables.
3. Build selectively during development:
   ```bash
   colcon build --packages-select <package_name>
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```
5. Run and test:
   ```bash
   ros2 run <package_name> <executable>
   ros2 launch <package_name> <launch_file>.launch.py
   ```

When stable, commit & push (see Git section).

---

## 6. Git & Backup Strategy

### 6.1 Design Principles

- **Single active local Git repo** → avoid divergence and confusion.
- **Jetson is the source of truth** for development.
- **GitHub is the remote of truth** for sharing and backup.
- **Windows only keeps file copies**, no commits or branches.

### 6.2 Concrete Setup

- **Jetson (local repo)**:
  - Repository root: `~/dev/r2d2`
  - Contains:
    - `ros2_ws/`
    - `tests/`
    - `docs/`
    - `.gitignore`
    - README etc.

- **GitHub (remote repo)**:
  - URL: `git@github.com:severinleuenberger/R2D2-as-real-AI-companion.git`
  - Branch: `main` (synced from Jetson).

- **Windows (backup directory)**:
  - Path:
    - `C:\Users\SeverinLeuenberger\OneDrive\Daten Severin\_Dev_Git_Projekte\R2D2`
  - Contains copied files from GitHub (or zipped exports).
  - **No `.git` directory**, no Git operations.

### 6.3 Standard Git Workflow (Jetson Only)

On Jetson, in `~/dev/r2d2`:

```bash
git status
git add <files>
git commit -m "Describe the change"
git push origin master:main
```

Notes:

- The branch name mapping `master:main` reflects a local `master` branch pushed to remote `main`.
- Initial repo reset was done via force‑push when re‑aligning the project; now the normal workflow is standard `push` without `--force`.

### 6.4 `.gitignore` Highlights

Key exclusions:

- ROS build artifacts:
  - `build/`, `install/`, `log/`
- Local tooling:
  - `.continue/` (Continue.dev config and secrets)
- Large / transient data:
  - `tests/audio/*.wav`
  - Potentially other large binary files as needed.

Images from camera tests are selectively allowed to be committed where they help documentation and validation.

---

## 7. “Touch‑the‑Ground” Demo Concept

Beyond camera and perception, the project defines a set of **minimal, end‑to‑end demos** that validate critical subsystems:

- **Compute / GPU**
  - CUDA sample or small GPU workload verifying performance and stability.

- **Audio Output**
  - Simple test (“R2D2 is online”) via speakers or buzzer.

- **Audio Input**
  - Microphone recording to `.wav`.
  - Later: possibly STT integration.

- **Camera (OAK‑D Lite)**
  - Covered in detail in `CAMERA_SETUP_DOCUMENTATION.md`.

- **Perception Pipeline**
  - Covered in detail in `PERCEPTION_SETUP_DOCUMENTATION.md`.

Each demo is designed to be:
- Small, self‑contained.
- Easy to run and verify.
- A building block for more complex behaviors (navigation, interaction, etc.).

---

## 8. Quick Reference Cheat Sheet

### 8.1 Bring Jetson Online (Headless)

1. Power the Jetson.
2. Connect USB‑C from Jetson to Windows.
3. On Windows, open terminal / PowerShell:
   ```bash
   ssh r2d2
   ```

### 8.2 Start Base ROS 2 System

```bash
cd ~/dev/r2d2/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch r2d2_bringup bringup.launch.py
```

### 8.3 Start Full Camera + Perception Pipeline

(Details in PERCEPTION and CAMERA docs, but for quick reference)

```bash
cd ~/dev/r2d2/ros2_ws
source ~/depthai_env/bin/activate
source ~/.bashrc
source install/setup.bash
ros2 launch r2d2_bringup r2d2_camera_perception.launch.py
```

### 8.4 Git Operations (Jetson)

```bash
cd ~/dev/r2d2
git status
git add .
git commit -m "Describe what changed"
git push origin master:main
```

### 8.5 Resolution Fix for LG Monitor

```bash
xrandr --output DP-0 --mode 3440x1440 --rate 60
```

(Optionally via desktop icon/script).

---

## 9. How This Document Fits With the Other Docs

- **This file**: captures **core system setup and workflow**:
  - Windows / Ubuntu‑USB / Jetson roles.
  - Flashing strategy.
  - Networking + SSH + VS Code.
  - Display quirks and mitigation.
  - ROS 2 base workspace and Git workflow.

- **`CAMERA_SETUP_DOCUMENTATION.md`**:
  - Goes deep on OAK‑D Lite integration, DepthAI environment, camera scripts, and ROS 2 camera package.

- **`PERCEPTION_SETUP_DOCUMENTATION.md`**:
  - Describes the `r2d2_perception` package and end‑to‑end perception pipeline (from `/oak/rgb/image_raw` to perception metrics).

Together, these three documents form a **complete picture** of the current R2D2 project state: from bare‑metal flashing and environment setup to a working vision and perception stack running on the Jetson AGX Orin.
