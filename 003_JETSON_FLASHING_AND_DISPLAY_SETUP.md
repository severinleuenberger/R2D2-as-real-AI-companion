
# Jetson AGX Orin Flashing & Display Setup – Lessons Learned

**Date:** December 2, 2025  
**Project:** R2D2 as a Real AI Companion  
**Host:** HP Windows 11 laptop + Ubuntu 24.04.3 LTS on 64 GB USB stick  
**Target:** Jetson AGX Orin 64 GB Dev Kit, JetPack 6.2.1 (L4T R36.4)

---

## 1. Overall Goal

Establish a **robust, repeatable setup** to:

- Safely flash **JetPack 6.2.1 (L4T R36.4)** onto the Jetson AGX Orin internal storage.
- Keep the **Windows 11 / BitLocker SSD completely untouched**.
- End up with a **usable desktop** on the Jetson (including the LG 5K monitor) and a **stable USB-based SSH connection** from the laptop.

---

## 2. Final Working Setup – High Level

### 2.1 Host side

- **HP laptop, Windows 11**, internal NVMe:
  - `WD PC SN740 SDDPNQD-512G-2006, 512 GB, BitLocker & Secure Boot enabled`
  - Not modified during Jetson setup.
- **Dedicated Ubuntu host for flashing:**
  - Ubuntu **24.04.3 Desktop (amd64)** installed as a **full OS** on a  
    **64 GB SanDisk USB stick**.
  - Partitioning on USB:
    - `sda1` – ~1 GB, `vfat`, mounted as `/boot/efi` (EFI System Partition)
    - `sda2` – rest of stick, `ext4`, mounted as `/`
  - Bootloader explicitly installed to the **USB device**, not the internal SSD.
- Available disk space on Ubuntu USB host (after updates & tools): ≈ **39 GB free** on `/`.

### 2.2 Tools on the Ubuntu USB host

- **NVIDIA SDK Manager** `2.3.0-12626` installed via:
  ```bash
  cd ~/Downloads
  sudo apt install ./sdkmanager_2.3.0-12626_amd64.deb
  ```
- **NGC CLI** `4.9.17` installed manually from NVIDIA zip:
  ```bash
  cd ~/Downloads
  unzip ngc_cli_4.9.17.zip
  cd ngc_cli_4.9.17
  unzip ngccli_linux.zip
  sudo mkdir -p /opt/ngc-cli
  sudo cp -r ngc-cli/* /opt/ngc-cli/
  sudo ln -s /opt/ngc-cli/ngc /usr/local/bin/ngc
  sudo chmod +x /opt/ngc-cli/ngc
  ngc --version   # NGC CLI 4.9.17
  ```

- NGC CLI configured with:
  ```bash
  ngc config set
  # API key  : <personal key from NGC>
  # Format   : ascii
  # Org      : Leuenberger (0846568256634454)
  # Team     : no-team
  # Ace      : no-ace
  ```
  → Config stored in `~/.ngc/config`, successfully validated.

### 2.3 Jetson side (after successful flash)

- **OS:** Ubuntu 22.04.5 LTS (Jetson Linux under JetPack 6.2.1).
- **Kernel:**
  ```bash
  uname -a
  # Linux R2D2 5.15.148-tegra #1 SMP PREEMPT Mon Jun 16 08:24:48 PDT 2025 aarch64 ...
  ```
- **Driver / CUDA:**
  ```bash
  nvidia-smi
  # NVIDIA-SMI 540.4.0, Driver Version: 540.4.0, CUDA Version: 12.6
  ```
- **USB networking bridge (for SSH over USB-C):**
  ```bash
  ip a
  # l4tbr0: 192.168.55.1/24
  # usb0, usb1 enslaved under l4tbr0
  ```

- **tegra stats & GPU monitoring** working:
  ```bash
  tegrastats     # runs continuously; Ctrl+C to stop
  ```

---

## 3. What Did *Not* Work Initially

### 3.1 Flashing via VirtualBox

Attempt: use an **Ubuntu VM in Oracle VirtualBox** on Windows 11 to flash the Jetson.

Issues:

- APX (recovery) device passthrough to the VM was **unstable**.
- USB connection to Jetson dropped during flashing.
- Conclusion: **VirtualBox is not suitable** for the actual flash step, because the APX USB path is too fragile.

Use VirtualBox later for development if you want, but **not for flashing**.

### 3.2 Flashing from a Live-Ubuntu (`/cow` too small)

Attempt: boot from a **Live Ubuntu 24.04** USB (`Try Ubuntu`) and run SDK Manager there.

Facts:

- Root filesystem lives on an overlay: `/cow` with only ≈ 11–12 GB free.
- SDK Manager complained:
  > There is not enough space on required partitions (1GB disk usage threshold on /cow is needed).  
  > Need additional 8xxx / 11xxx MB on /cow.

Even when:

- Using an external HDD (`/dev/sdb1`, ~1.8 TB, NTFS) mounted under `/mnt/usb`, and
- Running SDK Manager with:
  ```bash
  sdkmanager --cli     --action install     --login-type devzone     --product Jetson     --target-os Linux     --version 6.2.1     --show-all-versions     --target JETSON_AGX_ORIN_TARGETS     --flash     --accept     --download-folder /mnt/usb     --data-folder /mnt/usb
  ```

…the **overlay filesystem `/cow` still filled up**, because SDKM writes temporary data and additional content into `$HOME` (`~/.nvsdkm`, `~/Downloads/nvidia/sdkm_downloads`) and other locations.

Result: `Creating OS image…` sometimes started, but later steps failed.

> **Lesson:** Live-Ubuntu with a small `/cow` is **not** a reliable host for flashing Jetson with SDK Manager.

### 3.3 NGC / Docker: `Invalid password` / 401 Unauthorized

During installation, the SDK Manager's component **`JETSON_DOCKER_IMAGE_COMP`** tried to pull a Jetson Docker image from `nvcr.io`.

Symptoms:

- Terminal showed an HTTP 401:
  ```text
  HTTP/1.1 401 Unauthorized
  ...
  info: Password:
  info: Invalid password.
  error: ... JETSON_DOCKER_IMAGE_COMP.sh; [error]: Invalid password.
  ```
- The `Password:` prompt appeared and was rejected **without giving a chance to type**.
- The component failed with `error code: 11`, causing dependent steps (`File System and OS`, `Drivers for Jetson`) to fail.

Root cause:

- NGC CLI was not correctly installed / configured, so SDK Manager had **no valid credentials** to access `nvcr.io`.

Fix:

- Install NGC CLI manually from NVIDIA’s zip.
- Run `ngc config set` with a valid API key and select the correct organization.
- After that, SDK Manager can authenticate correctly.

---

## 4. Final Recommended Flashing Workflow

*(This section documents what worked; it is descriptive, not prescriptive “next steps” for the project.)*

### 4.1 Install Ubuntu on the 64 GB USB stick (host OS)

1. Boot the HP laptop from the **Ubuntu 24.04.3 Desktop ISO** (existing small USB stick).
2. Choose **“Install Ubuntu”** and at partitioning time select **“Something else”**.
3. Identify the **USB stick** (e.g. `sda`, ~57–60 GB) vs the internal SSD (`nvme0n1`, 476 GB, BitLocker).
4. On the USB stick:
   - Create **`sda1`**:
     - Size: ~1 GB
     - Type: `EFI System Partition`
     - Filesystem: FAT32
     - Mount point: `/boot/efi`
   - Create **`sda2`**:
     - Use the rest of the space
     - Type: `ext4`
     - Mount point: `/`
5. At the bottom (“Device for boot loader installation”), choose **the USB device** (e.g. `/dev/sda`).
6. Ensure the internal SSD **has no mountpoints set** and is **never chosen** as the bootloader device.
7. Finish installation → after reboot, you can choose to boot from the USB stick whenever you need the Jetson flashing environment.

This gives you a **persistent**, **fully writable** Ubuntu with enough space for SDK Manager and JetPack, while **keeping Windows/BitLocker untouched**.

### 4.2 Prepare the Ubuntu USB host

After first boot into this USB-Ubuntu:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git curl wget python3-pip usbutils unzip nano
```

(Optional) Install `pipx` to manage Python-based CLIs cleanly:

```bash
sudo apt install -y pipx
pipx ensurepath
source ~/.bashrc
```

Then install **SDK Manager** and **NGC CLI** as described in section 2.2.

### 4.3 SDK Manager configuration for minimal JetPack 6.2.1 flash

Using the GUI:

1. Start GUI SDKM (`sdkmanager`) and log in with NVIDIA Developer account.
2. Select:
   - **Product:** Jetson
   - **Hardware Config:** Target Hardware
   - **Target:** Jetson AGX Orin modules
   - **Target OS:** Linux
   - **JetPack Version:** 6.2.1 (rev.1)
3. In component selection:
   - **Enabled** under *Jetson Linux*:
     - `Jetson Linux image`
     - `Flash Jetson Linux`
   - **Disabled**:
     - Jetson Runtime Components
     - Jetson SDK Components
     - Jetson Platform Services
4. Ensure you *do not* choose “Download Only” – the working run used **Download & Install** so the flash actually happened.
5. Proceed through license acceptance etc.

### 4.4 Recovery (APX) mode steps that worked

Connections during flashing:

- Jetson → **Monitor (DisplayPort)** (optional for flashing itself, but helpful later).
- Jetson → **Host Ubuntu USB** via USB-C (device port).
- Jetson with its power supply connected.

Recovery procedure:

1. Connect USB-C cable between Jetson and Ubuntu host.
2. Press and hold the **Force Recovery** button.
3. While holding, briefly press the **Power** button.
4. Release both.
5. On the Ubuntu host:
   ```bash
   lsusb | grep -i nvidia
   # 0955:7023 NVIDIA Corp. APX
   ```

### 4.5 Flash run and successful result

With SDKM configured and Jetson in APX:

- Start the install/flash step in SDK Manager.
- Flash progressed to 100% without error.

Afterwards:

1. Jetson was powered off.
2. USB-C connection was removed.
3. On next power-on, Jetson booted into **Ubuntu 22.04.5 LTS**, with the standard NVIDIA first-boot wizard (user creation, WiFi, etc.).

Verification (on Jetson):

```bash
uname -a        # kernel 5.15.148-tegra
nvidia-smi      # driver 540.4.0, CUDA 12.6
ip a            # wifi + l4tbr0 present
tegrastats      # works, Ctrl+C to stop
```

---

## 5. USB Networking & SSH from Windows – Working Pattern

The Jetson presents a USB network gadget. On Jetson:

```bash
ip a
# ...
# 6: l4tbr0: 192.168.55.1/24 ...
# 7: usb0: ... master l4tbr0
# 8: usb1: ... master l4tbr0
```

On the Windows 11 laptop, once Jetson is up and USB-C is connected:

- Windows assigns an IP like `192.168.55.100` to the USB network adapter.
- Jetson is reachable at **192.168.55.1**.

Working SSH sequence:

```bat
ping 192.168.55.1

ssh severin@192.168.55.1
# Accept fingerprint
# Enter Jetson password
```

This is the **confirmed-good way** the console was used to debug Jetson even when the LG display showed a black screen.

---

## 6. LG 5K Monitor (34WK95U) – Behaviour & Workarounds

### 6.1 What the Jetson sees

When LG is connected and things work:

```bash
xrandr --current
# Screen 0: min 8 x 8, current 5120 x 2160, max 32767 x 32767
# DP-0 connected primary 5120x2160+0+0 800mm x 330mm
#   5120x2160 30.00
#   3840x2160 60.00 29.98
#   3440x1440 59.97 49.99
#   1920x1080 60.00 59.94
#   ...
```

Using `xrandr --props` showed a valid EDID block that clearly lists the LG HDR 5K.

### 6.2 Problems observed

- **Black screen** when booting with LG attached and powered from the start.
- Occasional message about an incorrect DisplayPort version.
- In bad cases:
  - No picture on LG.
  - No ping to 192.168.55.1.
  - No SSH from Windows.

Meanwhile:

- **Dell U2720Q** always worked fine and showed a stable picture with all boots.
- EDID for Dell:

  ```bash
  sudo cat /sys/class/drm/*/edid | hexdump -C
  # ... .DELL U2720Q ...
  ```

This made the Dell a safe “debug monitor”.

### 6.3 Kernel video parameter that helped

In `/boot/extlinux/extlinux.conf` the `APPEND` line was modified to include a preferred mode for DP-0:

```text
APPEND ${cbootargs} root=PARTUUID=2ee7... rw rootwait rootfstype=ext4 video=DP-0:3440x1440@60
```

This made the system **prefer 3440×1440@60** for DP-0 instead of full 5K.

### 6.4 xrandr script & autostart

To force the LG into 3440×1440 immediately after login, a small script was used:

`~/set-lg-3440x1440.sh`:

```bash
#!/usr/bin/env bash
sleep 5
xrandr --output DP-0 --mode 3440x1440 --rate 60
```

Make executable:

```bash
chmod +x ~/set-lg-3440x1440.sh
```

Autostart entry `~/.config/autostart/set-lg-3440x1440.desktop`:

```ini
[Desktop Entry]
Type=Application
Name=Set LG 5K to 3440x1440
Exec=/home/severin/set-lg-3440x1440.sh
X-GNOME-Autostart-enabled=true
```

Observed behaviour:

- After logoff/login, the desktop reliably switched to **3440×1440@60**.
- GNOME also appears to cache monitor configuration internally (e.g. `~/.config/monitors.xml`), so over time the system “learns” this resolution as default.

### 6.5 Reliable boot sequence for LG

Empirically the most reliable pattern was:

1. Boot Jetson **headless** (without LG connected).
2. Wait until SSH from Windows works:
   ```bat
   ssh severin@192.168.55.1
   ```
3. Only then power on LG and connect DP cable.
4. LG shows the desktop; xrandr/autostart switches to 3440×1440.

If Jetson was booted with LG attached and powered from the very start, the system sometimes fell back into the “black screen, no ping” state.

### 6.6 Why prefer 3440×1440 over full 5K

- Full 5K (`5120×2160`) = more pixels, more GPU/desktop compositor load and more boot quirks.
- `3440×1440`:
  - Still a very wide, comfortable workspace.
  - Lower GPU load.
  - Proven to be stable with the current JetPack 6.2.1 / driver stack and this LG model.

---

## 7. Lessons Learned

1. **Host stability is key.**  
   The biggest early problems came from VirtualBox and Live-Ubuntu `/cow`, not from the Jetson board itself.

2. **Full Ubuntu on USB is a great “flash station”.**  
   It provides persistence and enough space, while completely isolating the critical Windows/BitLocker SSD.

3. **NGC credentials must be correctly wired.**  
   Without a correctly installed and configured NGC CLI (`ngc config set`), SDK Manager fails on `JETSON_DOCKER_IMAGE_COMP` with HTTP 401 and “Invalid password”.

4. **Minimal JetPack selection is sufficient to flash.**  
   For a clean base image, it’s enough to install:
   - Jetson Linux image
   - Flash Jetson Linux  
   All other SDK components can be added later on top.

5. **USB SSH is a lifesaver.**  
   The `l4tbr0` interface and `192.168.55.1` SSH access allowed recovery and debugging even when the display was completely black.

6. **Ultrawide 5K monitors can stress the boot stack.**  
   EDID quirks and DP-negotiation issues can cause hangs. Kernel video hints plus an xrandr autostart script are an effective workaround.

7. **Keep a known-good 4K monitor around.**  
   The Dell U2720Q served as a stable fallback to verify that the Jetson itself was functioning correctly whenever the LG had issues.

8. **Iterative, single-variable changes avoid getting lost.**  
   Whenever something regressed, rolling back to the last known-good state and changing only one thing at a time (monitor, cable, kernel arg, xorg tweak) was crucial to converging on a stable configuration.

This document captures the **actually working path** from the initial broken attempts (VM, Live-Ubuntu, NGC errors) to a stable state with JetPack 6.2.1 on the Jetson AGX Orin, a safe flashing host environment on USB Ubuntu, reliable USB SSH, and a workable configuration for the LG 5K display.
