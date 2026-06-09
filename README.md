# Soft Drone Simulation

This repository contains MuJoCo simulation environments for soft drones, currently featuring the HoloArm drone.

## 📁 Project Structure

```
soft_drone_ws/
├── HoloArm_drone/
│   ├── assets/
│   │   ├── holoarm.xml                  # HoloArm core model
│   │   └── holoarm_narrow_passage.xml   # HoloArm in a narrow passage environment
│   └── scripts/
│       └── test_collision_narrow.py     # Script to test HoloArm collision in a narrow passage
├── X2_drone/
│   ├── main.py                 # Main flight script with Keyboard Teleop and PID Controller
│   └── mujoco_menagerie-main/  # Drone models from DeepMind's mujoco_menagerie
├── indoor/
│   └── indoor_env.xml          # The house environment with walls, furniture, lighting
└── README.md                   # This file
```

## 🛠️ Requirements

Make sure you have Python installed along with the following dependencies:

```bash
pip install mujoco pynput simple-pid numpy
```

## 🎮 How to Run

To run the automated collision test for the HoloArm drone navigating through a narrow passage:

```bash
python HoloArm_drone/scripts/test_collision_narrow.py
```
