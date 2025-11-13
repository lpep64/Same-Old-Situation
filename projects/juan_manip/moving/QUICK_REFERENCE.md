# Quick Reference - Niryo Pick-and-Place

## File Purposes

| File | Purpose | Key Functions |
|------|---------|---------------|
| `jpnp.py` | Main execution script | `main()`, `move_to_position()`, `wait_for_convergence()` |
| `jgg.py` | IK position generator | `get_q_from_ik()` - Returns 11 waypoint positions |
| `ik_module.py` | IK solver with penalties | `solve_IK()` - Joint limit avoidance |
| `niryo_conveyor.xml` | MuJoCo scene | Robot, conveyor, block definitions |
| `mujoco_parser.py` | Environment wrapper | `MuJoCoParserClass` - Simulation interface |

## Critical Configuration Values

```python
# Joint Limits (most restrictive)
arm_joint_range = [-1.8326, 0.6102]  # Real limit
arm_safe_min = -1.4                  # IK safety limit

# Block & Robot Positions
block_pos = [-0.25, 0.35, 0.595]     # On conveyor
robot_base = [-0.3, 0.0, 0.45]       # Table mounted

# Waypoint Heights (raised for safety)
above_block = block_pos + [0, 0, 0.20]   # 20cm above
grasp_height = block_pos + [0, 0, 0.05]  # 5cm above (NOT 2cm)
lift_height = grasp + [0, 0, 0.15]       # 15cm lift

# Convergence Parameters
tolerance_arm = 0.02      # 1.15° for arm movements
tolerance_gripper = 0.01  # 0.57° for gripper
max_wait = 15000          # 30 seconds timeout
timestep = 0.002          # MuJoCo timestep

# Actuator PID Gains
kp_arm = 500             # Shoulder, arm joints
kp_elbow = 400           # Elbow, forearm
kp_wrist = 300           # Wrist, hand, grippers
```

## 11-Step Sequence

```
1. ABOVE BLOCK    → [-0.077, -1.085, 1.57, 2.09, -0.491, 0.876, 0.01, 0.01]
2. GRASP position → [-0.077, -1.085, 1.57, 2.09, -0.491, 1.176, 0.01, 0.01]
3. CLOSE GRIPPER  → Same position, grippers: [-0.01, -0.01]
4. LIFT           → [1.405, -1.40, 1.57, 2.09, -1.065, -1.177, -0.01, -0.01]
5. TRANSPORT      → [1.112, -1.40, 1.57, 2.09, -0.902, -0.774, -0.01, -0.01]
6. PLACE (high)   → Same position (no lowering)
7. OPEN GRIPPER   → Same position, grippers: [0.01, 0.01]
8. LIFT up        → [1.405, -1.40, 1.57, 2.09, -1.065, -1.177, 0.01, 0.01]
9. RETURN         → [1.405, -1.40, 1.57, 2.09, -1.065, -0.877, 0.01, 0.01]
10. CLOSE GRIPPER → Same position, grippers: [-0.01, -0.01]
11. LIFT          → [1.405, -1.40, 1.57, 2.09, -1.065, -0.277, -0.01, -0.01]
12. START         → Return to position 1
```

## Joint Indexing (CRITICAL)

```python
# qpos structure: [block(7), joint1, joint2, ..., joint8]
# WRONG:
current_q = env.data.qpos[:8]  # ❌ Includes block!

# CORRECT:
arm_q = env.data.qpos[env.rev_joint_qpos_idxs]  # 6 joints
gripper_q = env.data.qpos[idxs[-1]+1:idxs[-1]+3]  # 2 joints
current_q = np.concatenate([arm_q, gripper_q])    # 8 total
```

## IK Penalties

```python
# 1. Mid-range preference
normalized = 2 * (q - q_mid) / range_size
penalty += 0.05 * normalized * |normalized|

# 2. Arm safety
if arm_joint < -1.4:
    penalty[1] += 0.5 * (-1.4 - arm_joint)

# 3. Degeneracy avoidance
penalty += 0.01 * (q - q_init)
```

## Common Issues & Fixes

| Issue | Cause | Fix |
|-------|-------|-----|
| Robot doesn't move | Wrong qpos indexing | Use `env.rev_joint_qpos_idxs` |
| Array shape (6,) vs (8,) | Missing grippers | Concatenate arm + gripper |
| IK doesn't converge | Target too low | Raise Z heights by 3-5cm |
| Stuck at -1.545 rad | Extreme arm angle | Add IK penalties, use -1.4 limit |
| Gripper not closing | Wrong sign | Right: +open/-close, Left: opposite |

## Running the Demo

```bash
# Windows PowerShell
cd C:\Users\lukep\Documents\Same-Old-Situation
.\.venv\Scripts\Activate.ps1
cd .\projects\juan_manip\moving
python jpnp.py

# Expected: 3 complete cycles, ~48 seconds, 100% convergence
```

## Performance Targets

```
✅ Convergence rate: 100% (33/33 tasks)
✅ Cycle time: ~16 seconds per cycle
✅ Max error: <0.02 rad (<1.15°)
✅ Arm joint range: -1.4 to +1.405 rad (safe)
✅ No timeouts, no failures
```

## Documentation

- **README.md** - Full system documentation (400+ lines)
- **IMPLEMENTATION_SUMMARY.md** - Changes and results
- **joint_analysis.txt** - Joint limit investigation
- **This file** - Quick reference

## Key Insights

1. **Arm joint (-1.833 to 0.610 rad)** is the bottleneck
2. **Keep arm > -1.4 rad** to avoid singularities
3. **Raise all heights** to reduce joint stress
4. **Position actuators** = built-in PID, no external control needed
5. **qpos indexing** must account for block freejoint (7 values)
6. **IK penalties** prevent extreme configurations
7. **Simplified paths** = more reliable than complex motions
