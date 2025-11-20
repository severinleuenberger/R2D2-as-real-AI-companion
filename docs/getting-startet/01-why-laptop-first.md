\# 01 – Why Develop on Your Laptop First (NOT on the Jetson!)



\*\*TL;DR:\*\*  

The Jetson AGX Orin is amazing when everything works – but it is \*\*10–50× slower\*\* and full of beginner traps during development.  

Do 90 % of the work on your normal laptop → you stay motivated, learn fast, and when the Jetson finally arrives, everything “just works”.



\### Real comparison (your HP Elite x360 with 32 GB RAM vs. Jetson AGX Orin)



| Task                            | Laptop (i7-1355U, 32 GB) | Jetson AGX Orin         | Speed difference |

|---------------------------------|--------------------------|-------------------------|------------------|

| Build the whole ROS2 workspace (`colcon build`) | 2–5 minutes             | 30–90 minutes           | \*\*10–30×\*\*       |

| Install Python packages         | Seconds                  | Minutes → hours (ARM)   | Many fail first try |

| Fix a bug → rebuild → test      | 30 seconds               | 10–20 minutes           | Kills your flow  |

| Run Gazebo simulation           | 100+ FPS                 | Possible, but slower    | Smoother on laptop |

| Ollama + Llama3 inference       | 1–3 s per answer         | 0.5–2 s (wins here)     | Only at the very end |



\*\*Result:\*\*  

90 % of all bugs are \*\*software bugs\*\*, not hardware bugs.  

You find and fix them 20× faster on the laptop.



\### What most beginners do (and regret) .



1\. Jetson arrives → spend 3–7 days flashing, installing CUDA, ROS2, drivers  

2\. First `colcon build` → wait 1.5 hours → compilation error because of ARM  

3\. Reflash Jetson 3 times → give up for weeks



\### What professionals (and now you) do – “Simulation-first”



1\. Build \& test everything on laptop (ROS2 nodes, LLM bridge, memory DB, navigation in Gazebo, speech with your laptop mic)  

2\. When Jetson arrives → copy the folder → `colcon build` once (already tested!) → it works on the first or second try  

3\. You only wire motors, cameras, etc. → R2D2 is alive in 1–2 days instead of 1–2 months



\### What you can already have running TODAY on your laptop

\- Ollama + Llama3 talking to you  

\- Full speech-to-text → LLM → text-to-speech (using your laptop microphone/speakers)  

\- Memory database with embeddings  

\- Navigation stack running in Gazebo (virtual R2D2 driving around)  

\- Person detection with your webcam instead of OAK-D  



→ When the Jetson arrives, you literally just move the brain inside the droid.



\*\*Bottom line:\*\*  

Your laptop is the fast, forgiving playground.  

The Jetson is the final boss – and you will beat it on day 1 because everything is already tested.



\*\*Next →\*\* \[02 – Ubuntu VM Setup (get maximum speed)](./02-ubuntu-vm-setup.md)

