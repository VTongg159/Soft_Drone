import time
import math
import mujoco
import mujoco.viewer
import numpy as np

# configure
MODEL_PATH       = "/home/admin1/soft_drone_ws/X2_soft_tpu95.xml"
SIM_DURATION     =10.0
INITIAL_VEL_X    = 2.0   
HOVER_THRUST     = 3.15   
MAX_THRUST       = 6.0    
TARGET_ALT       = 0.5   

# PD Gains
ALT_KP, ALT_KD     = 10.0, 4.0
PITCH_KP, PITCH_KD =  8.0, 2.0
ROLL_KP, ROLL_KD   =  8.0, 2.0

# import model
print("[INFO] import model:", MODEL_PATH)
try:
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
except Exception as e:
    print(f"[ERROR] {e}")
    raise SystemExit(1)

print(f"[INFO] nq={m.nq}, nv={m.nv}, nu={m.nu}, nflex={m.nflex}")

# Reset to keyframe

key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "hover_collision_start")
if key_id >= 0:
    mujoco.mj_resetDataKeyframe(m, d, key_id)
else:
    mujoco.mj_resetData(m, d)

# Set initial velocity (qvel[0] = vx của freejoint x2)
d.qvel[0] = INITIAL_VEL_X
d.ctrl[:] = HOVER_THRUST

# Helpers
collision_logged = [False]
survival_result  = [None]
prev_vz          = [0.0]

def check_survival(d):
    pos  = d.qpos[0:3].copy()
    quat = d.qpos[3:7].copy()
    w    = quat[0]
    tilt = math.degrees(math.acos(min(abs(w), 1.0))) * 2
    alive = (abs(w) > 0.5) and (pos[2] > 0.05)
    return {"pos": pos, "quat": quat, "w": w, "tilt": tilt, "alive": alive}

def attitude_controller(d, prev_vz):
    # PID for Z, Roll, Pitch → Calculate 4 signals for 4 motors
    # Altitude hold
    z   = d.qpos[2]
    vz  = d.qvel[2]
    prev_vz[0] = vz
    T = HOVER_THRUST + ALT_KP * (TARGET_ALT - z) - ALT_KD * vz

    # Read quaternion and convert to Euler
    w, x, y, z_quat = d.qpos[3:7]
    roll_rad  = math.atan2(2*(w*x + y*z_quat), 1 - 2*(x*x + y*y))
    # math.asin domain error fix
    val = np.clip(2*(w*y - z_quat*x), -1.0, 1.0)
    pitch_rad = math.asin(val)

    # Angular velocity (local frame) in d.qvel[3:6]
    wx = d.qvel[3]
    wy = d.qvel[4]

    # PD
    pitch_cmd = -PITCH_KP * pitch_rad - PITCH_KD * wy
    roll_cmd  = -ROLL_KP * roll_rad  - ROLL_KD * wx

    m1 = T + pitch_cmd - roll_cmd
    m2 = T + pitch_cmd + roll_cmd
    m3 = T - pitch_cmd + roll_cmd
    m4 = T - pitch_cmd - roll_cmd

    return [
        float(np.clip(m1, 0, MAX_THRUST)),
        float(np.clip(m2, 0, MAX_THRUST)),
        float(np.clip(m3, 0, MAX_THRUST)),
        float(np.clip(m4, 0, MAX_THRUST))
    ]

# Simulation loop
print(f"\n{'='*70}")
print(f"{'Time':>5}  {'X':>6}  {'VX':>6}  {'Z':>6}  {'Tilt°':>7}  {'Status':>8}")
print(f"{'='*70}")

with mujoco.viewer.launch_passive(m, d) as viewer:
    last_log = -0.1

    while viewer.is_running() and d.time < SIM_DURATION:
        step_start = time.time()

        sv = check_survival(d)
        
        # Apply controller if not completely flipped (> 75°)
        if abs(sv["w"]) > 0.26:
            d.ctrl[:] = attitude_controller(d, prev_vz)
        else:
            d.ctrl[:] = 1.0    # Cut thrust if flipped

        mujoco.mj_step(m, d)

        if not collision_logged[0] and d.qpos[0] >= 1.25: # Collision
            collision_logged[0] = True
            print(f"\n  [!] Collision at t={d.time:.3f}s | x={d.qpos[0]:.3f}m | vx={d.qvel[0]:.3f}m/s\n")

        if d.time - last_log >= 0.25:
            last_log = d.time
            sv = check_survival(d)
            status = "ALIVE" if sv["alive"] else "DEAD"
            print(f"{d.time:5.2f}  {sv['pos'][0]:6.3f}  {d.qvel[0]:6.3f}  {sv['pos'][2]:6.3f}"
                  f"  {sv['tilt']:7.1f}  {status}")
            survival_result[0] = sv

        viewer.sync()
        elapsed = time.time() - step_start
        if m.opt.timestep - elapsed > 0:
            time.sleep(m.opt.timestep - elapsed)

# Result
print(f"\n{'='*70}")
if survival_result[0]:
    sv = survival_result[0]
    verdict = "RE-BOUNCE SUCCESS " if (sv["alive"] and sv['pos'][0] < 1.0) else "FAIL "
    print(f"\n[FINAL RESULT]")
    print(f"  Position:     x={sv['pos'][0]:.3f}  y={sv['pos'][1]:.3f}  z={sv['pos'][2]:.3f}")
    print(f"  Velocity:    vx={d.qvel[0]:.3f} m/s")
    print(f"  Tilt: {sv['tilt']:.1f}°")
    print(f"  Verdict:   {verdict}")
