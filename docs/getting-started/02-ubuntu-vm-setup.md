\# 02 – Ubuntu VM Setup – Get Maximum Speed on Your Laptop



Even if you don’t have a high-end machine, a normal laptop from the last 5–6 years is \*\*more than enough\*\* and still \*\*10–20× faster\*\* than the Jetson during development.



\### Minimum laptop specs that work perfectly (2020 or newer)



| Component       | Minimum (still very fast)          | Recommended (feels like native)       | What happens if you’re below |

|-----------------|------------------------------------|---------------------------------------|------------------------------|

| CPU             | Intel i5 / AMD Ryzen 5 (6+ cores)  | i7 / Ryzen 7 (8+ cores)               | Builds just take 8–12 min instead of 3 min → still fine |

| RAM             | 16 GB total                        | 24–32 GB total                        | Give VM only 10–12 GB → still works |

| Storage         | Any SSD (even SATA)                | NVMe SSD                              | Slightly slower package installs |



\*\*Real-world examples that work great\*\*  

\- Any 2020+ office laptop (ThinkPad, Dell Latitude, HP EliteBook/ProBook)  

\- MacBook Air/Pro M1/M2/M3 (use UTM or Parallels instead of VirtualBox)  

\- Gaming laptops (obviously overkill but perfect)



\### VirtualBox settings – the sweet spot for almost everyone



| Setting                | Value (safe \& fast)           | Why |

|------------------------|-------------------------------|-----|

| CPU cores              | 6–8 cores (leave 2 for Windows) | ROS2 loves many cores when building |

| RAM                    | 12–24 GB (leave ~6–8 GB for Windows) | 16 GB works perfectly |

| Video Memory           | 256 MB                        | Needed for smooth Gazebo |

| Enable 3D Acceleration | Checked                       | Without it Gazebo runs at 5 FPS |

| Shared Clipboard       | Bidirectional                 | Copy-paste between Windows ↔ Ubuntu |

| Drag and Drop          | Bidirectional                 | Drag files into the VM |



\### Exact VirtualBox steps (works the same on every Windows 11 machine)



1\. Shut down the VM completely  

2\. Select your Ubuntu VM → \*\*Settings\*\* → \*\*System\*\* → \*\*Processor\*\*  

&nbsp;  → Move slider to \*\*6–8 cores\*\* (green/orange zone)  

3\. \*\*System\*\* → \*\*Motherboard\*\* → \*\*Base Memory\*\*  

&nbsp;  → Set to \*\*16384 MB\*\* (16 GB) or \*\*20480 MB\*\* (20 GB) or \*\*24576 MB\*\* (24 GB)  

4\. \*\*Display\*\* → \*\*Video Memory\*\* → \*\*256 MB\*\*  

&nbsp;  → Check \*\*Enable 3D Acceleration\*\*  

5\. \*\*General\*\* → \*\*Advanced\*\*  

&nbsp;  → Shared Clipboard → \*\*Bidirectional\*\*  

&nbsp;  → Drag and Drop → \*\*Bidirectional\*\*  

6\. OK → start the VM



\### Install Guest Additions inside Ubuntu (one-time, 2 minutes)



Open a terminal in Ubuntu and run:



```bash

sudo apt update \&\& sudo apt upgrade -y

sudo apt install -y build-essential dkms linux-headers-$(uname -r)

sudo apt install -y virtualbox-guest-dkms virtualbox-guest-x11 virtualbox-guest-utils

sudo reboot
````
After reboot, test it:


````Bash

sudo apt install -y mesa-utils

glxinfo | grep "OpenGL renderer"
````

You should NOT see llvmpipe. Anything else (Iris Xe, UHD, VBoxSVGA…) = perfect.



\##Result



Your VM now compiles the entire R2D2 workspace in under 5 minutes (often 2–3 min) and runs Gazebo at 100+ FPS – exactly what you want for fast, frustration-free development.

Next → 03 – Fast Local Development (build \& test everything now)

textPaste this into `02-ubuntu-vm-setup.md`, commit \& push – ready!  

Want 03 and 04 right now too? Just say go. 🚀



