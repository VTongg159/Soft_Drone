
## 📁 Project Structure

```
soft_drone_ws/
├── MuJoCo/
│   ├── main.py                 # Main flight script with Keyboard Teleop and PID Controller
│   └── mujoco_menagerie-main/  # Drone models from DeepMind's mujoco_menagerie
├── indoor/
│   ├── indoor_env.xml          # The house environment with walls, furniture, lighting
│   └── indoor_test.xml         # Additional test environment
├── test_collision.py           # Script to launch the drone into a wall and test recovery
├── X2_soft_tpu95.xml           # The core Soft Drone model with TPU95 properties
└── README.md                   # This file
```

## 🛠️ Requirements

Make sure you have Python installed along with the following dependencies:

```bash
pip install mujoco pynput simple-pid numpy
```

## 🎮 How to Run

### 1. Interactive Flight (Teleoperation)
To fly the drone manually inside the indoor environment, run:

```bash
python MuJoCo/main.py
```

**Keyboard Controls:**
- `i` / `k` or `↑` / `↓` : Move Forward / Backward (World Y)
- `j` / `l` or `←` / `→` : Move Left / Right (World X)
- `u` / `o` : Move Up / Down (World Z)
- `ESC` : Exit the simulation

### 2. Collision & Resilience Test
To run the automated collision test where the drone is pushed forward into a wall at `2.0 m/s` to evaluate its recovery:

```bash
python test_collision.py
```


