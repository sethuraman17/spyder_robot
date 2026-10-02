# Quadruped MuJoCo Simulation (8-DOF)

Python + MuJoCo simulation of the same 8-servo quadruped controlled by an ESP32.
No ROS, no Gazebo — MuJoCo only.

## 1. Install Python on Windows

1. Download Python 3.10+ from https://www.python.org/downloads/
2. During setup, enable **Add python.exe to PATH**
3. Open a new PowerShell or Command Prompt and check:

```bat
python --version
```

## 2. Create a virtual environment

From this folder (`quadruped_mujoco`):

```bat
cd C:\Users\sethu\Downloads\spider_robot\quadruped_mujoco
python -m venv .venv
```

## 3. Activate the virtual environment

```bat
.venv\Scripts\activate
```

## 4. Install dependencies

```bat
pip install -r requirements.txt
```

## 5. Launch the simulation

```bat
python main.py
```

A MuJoCo viewer window opens. The robot starts **SITTING** and waits for commands.

## 6. Keyboard controls

| Key | Command | Notes |
|-----|---------|--------|
| `↑` Up arrow | STAND | Knee joints 0 → 90 deg, then hold |
| `↓` Down arrow | SIT | Knee joints 90 → 0 deg, then hold |
| `←` Left arrow | MOVE LEFT LEG | Rear-left sequence; only while STANDING |
| `→` Right arrow | MOVE RIGHT LEG | Rear-right sequence; only while STANDING |
| `R` | RESET | Return to sitting pose |

Letter aliases: `S` stand, `D` sit, `L` left leg.

Invalid commands are ignored (e.g. leg move while sitting, STAND while already standing).

### Visible hardware in the model

- **8 hobby servos** mounted on each hip/knee joint (blue body + silver horn)
- **Arduino Nano** (blue PCB + USB) on top of the chassis
- **Extension shield** (green board) stacked on the Nano
- **External wire harness** via MuJoCo spatial tendons (black leads + orange signal + red VCC), routed out through side cable exits

## 7. Where robot dimensions are defined

File: `quadruped.xml`

- Body size: torso `geom` `size="0.225 0.15 0.06"` → **0.45 × 0.30 × 0.12 m**
- Upper leg length: `fromto="0 0 0 0 0 -0.18"` → **0.18 m**
- Lower leg length: same `fromto` on knee bodies → **0.18 m**
- Hip attachment positions: `fl_hip` / `fr_hip` / `rl_hip` / `rr_hip` `pos=...`
- Masses: `inertial` / `mass` attributes on torso and leg geoms

## 8. Where joint limits are defined

File: `quadruped.xml` → `<default>` classes:

- Hip / joint-1 (`class="hip_joint"`): `range="0 7.0"` radians ≈ **0° … 401°**
  - Wide on purpose so `leftdown1 = 360` is **not** clamped to 180
- Knee / joint-2 (`class="knee_joint"`): `range="-0.2 2.8"` radians ≈ **-11° … 160°**

## 9. Where actuator strength / damping is defined

File: `quadruped.xml`:

- Joint damping / armature: `hip_joint` and `knee_joint` defaults
- Position actuator gains:
  - `class="servo_hip"` → `kp`, `kv`, `forcerange`
  - `class="servo_knee"` → `kp`, `kv`, `forcerange`

File: `main.py` (motion timing):

- `STAND_DURATION`
- `SIT_DURATION`
- `LEFT_LEG_STEP_DURATION`

## 10. ESP32 ↔ MuJoCo joint mapping

| Logical name | Role | GPIO (ID only) | MuJoCo joint |
|--------------|------|----------------|--------------|
| leftup1 | Front Left hip | 14 | `leftup1` |
| leftup2 | Front Left knee | 16 | `leftup2` |
| rightup1 | Front Right hip | 12 | `rightup1` |
| rightup2 | Front Right knee | 26 | `rightup2` |
| leftdown1 | Rear Left hip | 17 | `leftdown1` |
| leftdown2 | Rear Left knee | 4 | `leftdown2` |
| rightdown1 | Rear Right hip | 13 | `rightdown1` |
| rightdown2 | Rear Right knee | 2 | `rightdown2` |

GPIO numbers are identifiers in simulation only. No real GPIO is accessed.

High-level Python API (degrees, same idea as ESP32):

- `stand_robot()`
- `sit_robot()`
- `move_left_leg()`
- `reset_robot()`

Later you can swap `RobotController.apply_to_mujoco()` for Wi-Fi / ESP32 output without rewriting the motion sequences.

## Initial pose

Sitting:

- All joint-1 = **145°**
- All joint-2 = **0°**

Standing:

- All joint-1 = **145°**
- All joint-2 = **90°**

## MOVE LEFT LEG sequence

Requires STANDING first.

1. `leftdown2`: 90 → 125  
2. `leftdown1`: 145 → 360 *(not normalized to 0)*  
3. `leftdown2`: 125 → 90  
4. `leftdown1`: 360 → 145  
5. Stop, remain STANDING  

## Project layout

```
quadruped_mujoco/
├── main.py
├── quadruped.xml
├── requirements.txt
└── README.md
```

## Angle units

- Commands, targets, and terminal output: **degrees**
- MuJoCo actuators / physics: **radians**
- Helpers in `main.py`: `deg_to_rad()`, `rad_to_deg()`
