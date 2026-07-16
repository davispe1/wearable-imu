# Inverse Kinematics — `shoulder_arm_ik.m`

Design document for the 5-DOF analytical inverse kinematics solver.

---

## 1. Goal

Given a desired end-effector **position** (x, y, z) and **orientation** (ZYX Euler angles: yaw, pitch, roll), find all valid joint angle configurations `[q1, q2, q3, q4, q5]` that reach that target. Since the arm has only 5 DOF, it cannot independently satisfy all 6 constraints (3 position + 3 orientation). The solver uses position + 2 orientation components as hard constraints, then ranks solutions by how closely the remaining orientation component matches the input.

---

## 2. DH Table (current, from `shoulder_arm_fk.m`)

| Joint | theta_offset (deg) | alpha (deg) | a (mm) | d (mm)  | Description                        |
|------:|--------------------|-------------|--------|---------|------------------------------------|
| 1     | 0                  | 90          | 0      | 55.4    | Shoulder flexion / extension       |
| 2     | 90                 | 90          | 0      | 0       | Shoulder abduction / adduction     |
| 3     | -90                | -90         | 0      | 283.5   | Shoulder internal / external rot.  |
| 4     | 0                  | 90          | 0      | 0       | Elbow flexion / extension          |
| 5     | 90                 | 0           | 30     | 257     | Forearm pronation / supination     |

**Key dimensions:**
- `d1 = 55.4 mm` — shoulder offset (lateral, along initial Z)
- `d3 = 283.5 mm` — upper arm length (shoulder to elbow)
- `d5 = 257 mm` — forearm length (elbow to wrist)
- `a5 = 30 mm` — wrist lateral offset ("watch" side)

---

## 3. Kinematic Structure

Joints 1–3 form a **spherical shoulder** — three revolute axes intersecting at a point offset `d1 = 55.4 mm` from the base along Z. This allows **wrist-point decoupling**: the position problem (joints 1, 2, 4) can be separated from the orientation problem (joints 3, 5).

### 3.1 Decoupling Strategy

```
[Position]    → Solve for wrist center location → q1, q2, q4
[Orientation] → Solve for rotation around arm axis → q3, q5
```

**Step A — Wrist center:**
The wrist center `p_wc` is the origin of frame 4 (elbow-to-wrist intersection). Given the desired EE position `p_ee` and orientation `R_ee`:

```
p_wc = p_ee - d5 * R_ee(:,3) - a5 * R_ee(:,1)
```

This removes the forearm length and wrist offset from the target, leaving the point that joints 1–2–4 must reach.

**Step B — Shoulder angles (q1, q2) and elbow (q4):**
With `p_wc` known, project onto the shoulder geometry:

1. **q1 (shoulder flex/ext):** from the projection of `p_wc` onto the XZ plane (after removing the `d1` offset)
2. **q2 (shoulder abd/add):** from the elevation angle of `p_wc` relative to the shoulder axis
3. **q4 (elbow flex/ext):** from the triangle formed by `d3` (upper arm), the distance from shoulder to `p_wc`, using the law of cosines. This gives **two solutions**: elbow-up and elbow-down.

**Step C — Rotation angles (q3, q5):**
With q1, q2, q4 known, compute `R_0_to_3` (rotation from base to frame 3) and `R_4_to_5` (frame 4 to 5). Then:

```
R_3_to_4 = R_0_to_3' * R_ee * R_5_to_ee'
```

Extract q3 and q5 from this intermediate rotation matrix.

---

## 4. Solution Pipeline

The MATLAB file will execute the following steps in order:

### Step 1 — Input & Workspace Check
- User enters: `[x, y, z]` (mm) and `[yaw, pitch, roll]` (deg, ZYX Euler)
- Convert Euler angles to rotation matrix `R_ee`
- Compute wrist center `p_wc`
- Check reachability: `norm(p_wc - p_shoulder) <= d3 + d5` (max reach) and `>= |d3 - d5|` (min reach)
- If unreachable → display error, stop

### Step 2 — Analytical IK (enumerate all solutions)
- Solve q4 from law of cosines → 2 solutions (elbow-up / elbow-down)
- For each q4, solve q1, q2 → may produce sign ambiguities (2 branches)
- For each (q1, q2, q4), solve q3, q5 from orientation
- Total: up to **4 candidate solutions**

### Step 3 — Display all raw solutions
- Show a table of all candidate `[q1, q2, q3, q4, q5]` (degrees)
- Label each with its configuration (e.g., "elbow-up", "elbow-down")

### Step 4 — Joint limit filter
- Remove any solution where a joint exceeds its physiological limit
- Mark filtered-out solutions in the table (grayed out or red)
- If all filtered → display warning

### Step 5 — Rank by orientation match
- Position will already be accurate (3 joints dedicated to it), so ranking is purely by **orientation fit**
- For each surviving solution, compute FK to get the actual EE orientation `R_actual`
- Compare `R_actual` vs. the full input `R_ee`:

```
ori_error = norm(logm(R_actual' * R_ee))   % geodesic distance on SO(3)
```

- Rank solutions by smallest orientation error only
- The best match is the configuration whose orientation (driven by q3, q5) aligns most closely with the full 3-axis IMU orientation input

### Step 6 — Visualize best solution
- Draw the arm in the selected configuration (reuse FK + drawing code from `shoulder_arm_fk.m`)
- Draw **target reference arrows** at the desired `[x, y, z]`: RGB arrows (X=red, Y=green, Z=blue) showing the desired orientation frame
- Draw the actual EE frame on the arm for visual comparison
- Display: joint angles, position error (mm), orientation error (deg)

---

## 5. GUI Layout

```
+---------------------------+--------------------------------------+
|  INPUT                    |                                      |
|  Position [x, y, z] mm   |                                      |
|  Orientation [Y, P, R] ° |           3D VISUALIZATION           |
|  [Solve] button           |                                      |
|                           |   - Arm in solved configuration      |
|  RESULTS TABLE            |   - Target frame (RGB arrows)        |
|  Solution | q1..q5 | err  |   - Body context (torso + FRONT)     |
|  #1  ✓    | ...    | 0.3° |   - Shoulder disc                    |
|  #2  ✗    | ...    | lim  |                                      |
|  #3  ✓    | ...    | 4.1° |                                      |
|                           |                                      |
|  SELECTED: #1             |                                      |
|  Pos error: 0.01 mm       |                                      |
|  Ori error: 0.3°          |                                      |
+---------------------------+--------------------------------------+
```

- No joint sliders (IK drives the angles, not the user)
- Clicking a row in the results table switches the visualization to that solution
- Solutions that violate joint limits are shown but grayed out

---

## 6. File Structure

Single file: `simulation/scripts/shoulder_arm_ik.m`

```
shoulder_arm_ik()
├── CONFIG (DH table, joint limits, mount_angle_deg — copied from FK file)
├── GUI setup (input fields, results table, 3D axes)
├── solve_ik(p_target, R_target)
│   ├── workspace_check()
│   ├── solve_position()      → q1, q2, q4 candidates
│   ├── solve_orientation()   → q3, q5 for each candidate
│   └── returns all solutions
├── filter_and_rank(solutions, R_target)
│   ├── joint_limit_filter()
│   ├── orientation_ranking() → geodesic distance
│   └── returns ranked list
├── forward_kinematics(q)     → reused from FK file
├── draw_robot(q)             → reused from FK file
├── draw_target_frame(p, R)   → RGB arrows at target
└── drawing helpers           → reused from FK file
```

---

## 7. Edge Cases to Handle

- **Singular configurations:** wrist center on the shoulder Z-axis (q1 undefined) — use atan2 fallback
- **At reach limit:** elbow fully extended, only 1 solution instead of 2
- **No valid solutions:** all candidates violate joint limits — show warning, display closest invalid solution dimmed
- **Orientation mismatch:** large gap between best solution's orientation and input — show warning if > 15°

---

## 8. Future Extensions (not implemented now)

- Quaternion input (replace Euler → rotation matrix conversion)
- Real-time IMU stream: solve IK each frame, use continuity filter to pick smoothest solution
- Animation: interpolate between current and solved configuration
- Left/right arm toggle with mirrored DH
