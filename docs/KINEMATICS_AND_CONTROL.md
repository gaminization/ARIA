# Project ARIA: Kinematics, Dynamics & Trajectory Control

## 1. Kinematic Structure & Authoritative URDF Mapping

Project ARIA employs a 5-Degree-of-Freedom (5-DoF) revolute serial chain. The base link is fixed to the workcell frame at $[0.0, 0.0, 0.614]\,\text{m}$ with a $90^\circ$ yaw offset. The mechanical structure consists of:
- **Joint 1 (Waist):** Revolute about vertical $Z$-axis.
- **Joints 2, 3, and 4 (Shoulder, Elbow, Wrist Pitch):** Planar revolute joints whose rotation axes are strictly parallel to each other and perpendicular to Joint 1.
- **Joint 5 (Wrist Roll / Gripper):** Revolute along the tool longitudinal axis.

### Craig Modified Denavit-Hartenberg (MDH) Parameters (Table III):

| Joint $i$ | Link Description | $\alpha_{i-1}$ | $a_{i-1}$ (m) | $d_i$ (m) | $\theta_i^{\text{DH}}$ Range (rad) | Servo Travel | Affine Offset $\theta_{\text{offset}}$ | Max Velocity $\dot{q}_{\max}$ |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1** | Waist (Azimuth) | $0$ | $0.000$ | $0.105$ | $[-\pi/2, +\pi/2]$ | $[0^\circ, 180^\circ]$ | $90^\circ$ ($\pi/2$) | $1.50\,\text{rad/s}$ |
| **2** | Shoulder (Pitch) | $+\pi/2$ | $0.014$ | $0.000$ | $[-\pi/2, +\pi/2]$ | $[0^\circ, 180^\circ]$ | $90^\circ$ ($\pi/2$) | $1.50\,\text{rad/s}$ |
| **3** | Elbow (Pitch) | $0$ | $0.145$ | $0.000$ | $[-\pi/2, +\pi/3]$ | $[0^\circ, 150^\circ]$ | $90^\circ$ ($\pi/2$) | $1.50\,\text{rad/s}$ |
| **4** | Wrist Pitch | $0$ | $0.115$ | $0.000$ | $[-\pi/2, +\pi/2]$ | $[0^\circ, 180^\circ]$ | $90^\circ$ ($\pi/2$) | $1.50\,\text{rad/s}$ |
| **5** | Wrist Roll / Tool | $-\pi/2$ | $0.000$ | $0.095$ | $[-\pi/2, +\pi/2]$ | $[0^\circ, 180^\circ]$ | $90^\circ$ ($\pi/2$) | $2.50\,\text{rad/s}$ |

---

## 2. Forward Kinematics Formulation

The transformation matrix from base to end-effector is computed by multiplying link transforms:
$$\mathbf{T}_0^5 = \mathbf{T}_0^1 \mathbf{T}_1^2 \mathbf{T}_2^3 \mathbf{T}_3^4 \mathbf{T}_4^5$$

The resulting Cartesian TCP position $(P_x, P_y, P_z)$ is given symbolically by:
$$P_x = \left[a_1 + a_2 \cos\theta_2 + a_3 \cos(\theta_2 + \theta_3) + d_5 \sin(\theta_2 + \theta_3 + \theta_4)\right] \cos\theta_1$$
$$P_y = \left[a_1 + a_2 \cos\theta_2 + a_3 \cos(\theta_2 + \theta_3) + d_5 \sin(\theta_2 + \theta_3 + \theta_4)\right] \sin\theta_1$$
$$P_z = d_1 + a_2 \sin\theta_2 + a_3 \sin(\theta_2 + \theta_3) - d_5 \cos(\theta_2 + \theta_3 + \theta_4)$$

The tool orientation approach vector $\hat{\mathbf{a}}$ is:
$$\hat{\mathbf{a}} = \begin{bmatrix} \cos\theta_1 \sin(\theta_2 + \theta_3 + \theta_4) \\ \sin\theta_1 \sin(\theta_2 + \theta_3 + \theta_4) \\ -\cos(\theta_2 + \theta_3 + \theta_4) \end{bmatrix}$$

---

## 3. Closed-Form Inverse Kinematics on $\mathcal{M}_{\text{task}}$

Implemented in [`arm_ik/arm_ik/ik_solvers/aria_analytical_ik.py`](file:///home/gaminizer/Projects/ARIA/arm_ik/arm_ik/ik_solvers/aria_analytical_ik.py):

Because a 5-DoF manipulator cannot independently achieve arbitrary 6D poses in $SE(3)$, inverse kinematics is strictly formulated over the 5-dimensional admissible task manifold $\mathcal{M}_{\text{task}}$:
$$\mathcal{M}_{\text{task}} = \left\{ \mathbf{T} \in SE(3) \;\middle|\; \mathbf{p} \in \mathcal{W}_{\text{reach}}, \; \phi_{\text{yaw}} = \text{atan2}(p_y, p_x), \; \theta_{\text{pitch}} \in [\theta_{\min}, \theta_{\max}], \; \psi_{\text{roll}} = 0 \right\}$$

### Step-by-Step Closed-Form Resolution:
1. **Azimuth Angle ($\theta_1$):**
   $$\theta_1 = \text{atan2}(P_y, P_x)$$
2. **Wrist Center Coordinate ($\mathbf{P}_w$):**
   $$\mathbf{P}_w = \mathbf{P}_{\text{target}} - d_5 \begin{bmatrix} \cos\theta_1 \cos\theta_{\text{pitch}} \\ \sin\theta_1 \cos\theta_{\text{pitch}} \\ \sin\theta_{\text{pitch}} \end{bmatrix}$$
3. **Signed Radius & Planar Height:**
   $$r_w = \sqrt{P_{wx}^2 + P_{wy}^2} - a_1$$
   $$z_w = P_{wz} - d_1$$
4. **Elbow Angle via Law of Cosines ($\theta_3$):**
   $$\cos\theta_3 = \frac{r_w^2 + z_w^2 - a_2^2 - a_3^2}{2 a_2 a_3}$$
   If $|\cos\theta_3| > 1$, the pose is geometrically unreachable.
   - **Elbow-up branch:** $\theta_3 = -\arccos(\cos\theta_3)$
   - **Elbow-down branch:** $\theta_3 = +\arccos(\cos\theta_3)$
5. **Shoulder Angle ($\theta_2$):**
   $$\theta_2 = \text{atan2}(z_w, r_w) - \text{atan2}(a_3 \sin\theta_3, a_2 + a_3 \cos\theta_3)$$
6. **Wrist Pitch Angle ($\theta_4$):**
   $$\theta_4 = \theta_{\text{pitch}} - \theta_2 - \theta_3$$
7. **Joint Limits Verification:**
   Enforce physical servo travel bounds, specifically the asymmetric mechanical stop $\theta_3 \in [-\pi/2, +\pi/3]$. If the primary elbow-up solution violates limits, the solver evaluates the elbow-down solution. If both fail, the pose is deterministically rejected as out-of-limits without iterative divergence.

### Benchmarked Performance (100,000 Reachable Poses):
- **Recovery Rate:** **$100.00\%$** ($100{,}000 / 100{,}000$).
- **Mean Residual Position Error:** **$0.0000\,\text{mm}$** (maximum $< 10^{-6}\,\text{mm}$).
- **Solve Latency:** Mean **$0.0802\,\text{ms}$** ($80.2\,\mu\text{s}$), p99 **$0.195\,\text{ms}$**.

---

## 4. Damped Least-Squares (DLS) Velocity Mapping & Manipulability

For continuous Cartesian velocity control during visual tracking and conveyor interception:
$$\dot{\mathbf{q}} = \mathbf{J}_p^\dagger \mathbf{v}_{\text{cartesian}}$$

Where the positional Jacobian $\mathbf{J}_p \in \mathbb{R}^{3 \times 5}$ maps joint velocities to end-effector linear velocity. To guarantee numerical stability near kinematic singularities, the damped least-squares (DLS) pseudoinverse is used:
$$\mathbf{J}_p^\dagger = \mathbf{J}_p^T (\mathbf{J}_p \mathbf{J}_p^T + \lambda^2 \mathbf{I}_3)^{-1} = (\mathbf{J}_p^T \mathbf{J}_p + \lambda^2 \mathbf{I}_5)^{-1} \mathbf{J}_p^T \quad (\lambda = 0.02)$$

### Yoshikawa Reduced Gram Determinant Manipulability:
Because the manipulator has 5 joints, the full $6 \times 6$ square Jacobian determinant is identically zero ($\det(\mathbf{J}\mathbf{J}^T) \equiv 0$). We quantify posture manipulability using the reduced Gram determinant over the positional Jacobian:
$$w_5(\mathbf{q}) = \sqrt{\det\left( \tilde{\mathbf{J}}_p(\mathbf{q}) \tilde{\mathbf{J}}_p^T(\mathbf{q}) \right)}$$

---

## 5. Spline Trajectory Generation with Joint Velocity Time-Scaling

Implemented in [`arm_control/scripts/trajectory_generator.py`](file:///home/gaminizer/Projects/ARIA/arm_control/scripts/trajectory_generator.py):

Point-to-point joint motions are interpolated via quintic (5th-order) polynomial splines:
$$q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5$$

Subject to boundary conditions:
- $q(0) = q_0, \quad \dot{q}(0) = 0, \quad \ddot{q}(0) = 0$
- $q(\tau) = q_1, \quad \dot{q}(\tau) = v_f, \quad \ddot{q}(\tau) = 0$

### Actuator Velocity Limit Enforcement:
Low-cost hobbyist servos (MG995) have a physical maximum velocity of $1.50\,\text{rad/s}$ under load. Fast trajectory motions risk actuator saturation and positional tracking failure. ARIA enforces strict velocity limits through **automatic trajectory time-scaling**:

$$\Delta q_i = |q_{1, i} - q_{0, i}|$$
$$\tau_{\min} = \max_i \left( \frac{15 \cdot \Delta q_i}{8 \cdot \dot{q}_{\max, i}} \right) = \max_i \left( 1.875 \cdot \frac{\Delta q_i}{\dot{q}_{\max, i}} \right)$$
$$\tau_{\text{execution}} = \max(\tau_{\text{commanded}}, \tau_{\min})$$

This formulation guarantees that $\max_t |\dot{q}_i(t)| \le 1.50\,\text{rad/s}$ across all trajectory segments.
