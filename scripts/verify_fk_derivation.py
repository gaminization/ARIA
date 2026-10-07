#!/usr/bin/env python3
"""
scripts/verify_fk_derivation.py

Rigorous symbolic and numerical verification of Craig's Modified Denavit-Hartenberg (MDH)
kinematic formulation for the 5-DoF Project ARIA manipulator.

Verifies:
1. Matrix product of Craig MDH matrices from Table III.
2. Exact analytical match for Px, Py, Pz (Equations 3-5 in main.tex).
3. Exact analytical match for approach vector a (Equation 6 in main.tex).
4. Tool orientation matrix R and proof of theta5 = theta1 - psi for vertical grasps (phi = 0).
5. Monte-Carlo numerical evaluation across 10,000 random joint configurations.
"""

import math
import numpy as np
import sympy as sp

def main():
    print("=" * 70)
    print("PROJECT ARIA: SYMBOLIC & NUMERICAL FORWARD KINEMATICS VERIFICATION")
    print("=" * 70)

    # 1. Define symbolic variables
    t1, t2, t3, t4, t5 = sp.symbols('theta_1 theta_2 theta_3 theta_4 theta_5', real=True)
    a1, a2, a3, d1, d5 = sp.symbols('a_1 a_2 a_3 d_1 d_5', real=True)

    # Craig's Modified DH transformation matrix from frame {i-1} to frame {i}:
    # T_{i-1, i} = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) * Rot_z(theta_i) * Trans_z(d_i)
    # [ cos(theta_i)              -sin(theta_i)             0              a_{i-1}            ]
    # [ sin(theta_i)*cos(alpha)   cos(theta_i)*cos(alpha)   -sin(alpha)   -d_i * sin(alpha)   ]
    # [ sin(theta_i)*sin(alpha)   cos(theta_i)*sin(alpha)   cos(alpha)     d_i * cos(alpha)   ]
    # [ 0                         0                         0              1                  ]
    def craig_mdh(alpha, a, d, theta):
        ca, sa = sp.cos(alpha), sp.sin(alpha)
        ct, st = sp.cos(theta), sp.sin(theta)
        return sp.Matrix([
            [ct,    -st,     0,    a],
            [st*ca,  ct*ca, -sa,  -d*sa],
            [st*sa,  ct*sa,  ca,   d*ca],
            [0,      0,      0,    1]
        ])

    print("\n[Step 1] Constructing Craig Modified DH transformation matrices from Table III:")
    # Link 1: alpha0 = 0,      a0 = 0,   d1 = d1, theta1 = t1
    T01 = craig_mdh(0, 0, d1, t1)
    # Link 2: alpha1 = +pi/2,   a1 = a1,  d2 = 0,  theta2 = t2
    T12 = craig_mdh(sp.pi/2, a1, 0, t2)
    # Link 3: alpha2 = 0,      a2 = a2,  d3 = 0,  theta3 = t3
    T23 = craig_mdh(0, a2, 0, t3)
    # Link 4: alpha3 = 0,      a3 = a3,  d4 = 0,  theta4 = t4
    T34 = craig_mdh(0, a3, 0, t4)
    # Link 5: alpha4 = +pi/2,   a4 = 0,   d5 = d5, theta5 = t5
    T45 = craig_mdh(sp.pi/2, 0, d5, t5)

    print("Computing symbolic product T05 = T01 * T12 * T23 * T34 * T45 ...")
    T05 = sp.trigsimp(T01 * T12 * T23 * T34 * T45)

    P_x_sym = sp.trigsimp(T05[0, 3])
    P_y_sym = sp.trigsimp(T05[1, 3])
    P_z_sym = sp.trigsimp(T05[2, 3])
    R_sym = sp.trigsimp(T05[:3, :3])

    print("\nSymbolic Cartesian Coordinates from Craig MDH Product:")
    print(f"  P_x = {P_x_sym}")
    print(f"  P_y = {P_y_sym}")
    print(f"  P_z = {P_z_sym}")

    # 2. Compare with paper equations (3)-(5)
    # Px = c1*(a1 + a2*c2 + a3*c23 + d5*s234)
    # Py = s1*(a1 + a2*c2 + a3*c23 + d5*s234)
    # Pz = d1 + a2*s2 + a3*s23 - d5*c234
    c1, s1 = sp.cos(t1), sp.sin(t1)
    c2, s2 = sp.cos(t2), sp.sin(t2)
    c23, s23 = sp.cos(t2 + t3), sp.sin(t2 + t3)
    c234, s234 = sp.cos(t2 + t3 + t4), sp.sin(t2 + t3 + t4)

    Px_paper = c1 * (a1 + a2*c2 + a3*c23 + d5*s234)
    Py_paper = s1 * (a1 + a2*c2 + a3*c23 + d5*s234)
    Pz_paper = d1 + a2*s2 + a3*s23 - d5*c234

    diff_x = sp.trigsimp(P_x_sym - Px_paper)
    diff_y = sp.trigsimp(P_y_sym - Py_paper)
    diff_z = sp.trigsimp(P_z_sym - Pz_paper)

    print("\n[Step 2] Analytical Difference between Matrix Product and Equations (3)-(5):")
    print(f"  Delta P_x = {diff_x}")
    print(f"  Delta P_y = {diff_y}")
    print(f"  Delta P_z = {diff_z}")
    assert diff_x == 0 and diff_y == 0 and diff_z == 0, "Forward kinematics does not match paper equations!"
    print("  >>> PROOF SUCCESSFUL: Equations (3)-(5) are identically zero in symbolic difference.")

    # 3. Check Approach Vector (Column 3 of R)
    a_sym = T05[:3, 2]
    a_paper = sp.Matrix([c1 * s234, s1 * s234, -c234])
    diff_a = sp.trigsimp(a_sym - a_paper)
    print("\n[Step 3] Approach Vector Verification (Column 3 of R):")
    print(f"  Symbolic a = [{a_sym[0]}, {a_sym[1]}, {a_sym[2]}]^T")
    print(f"  Delta a    = [{diff_a[0]}, {diff_a[1]}, {diff_a[2]}]^T")
    assert diff_a[0] == 0 and diff_a[1] == 0 and diff_a[2] == 0, "Approach vector mismatch!"
    print("  >>> PROOF SUCCESSFUL: Approach vector identically matches Equation (6).")

    # 4. Tool Orientation for Vertical Downward Grasp (phi = 0)
    # When phi = t2 + t3 + t4 = 0: c234 = 1, s234 = 0.
    print("\n[Step 4] Tool Orientation for Vertical Grasps (phi = t2 + t3 + t4 = 0):")
    R_phi0 = sp.trigsimp(R_sym.subs(t2 + t3 + t4, 0))
    print("  R(phi = 0):")
    print(f"    n = [{R_phi0[0,0]}, {R_phi0[1,0]}, {R_phi0[2,0]}]^T")
    print(f"    s = [{R_phi0[0,1]}, {R_phi0[1,1]}, {R_phi0[2,1]}]^T")
    print(f"    a = [{R_phi0[0,2]}, {R_phi0[1,2]}, {R_phi0[2,2]}]^T")

    # Notice R_phi0[0,0] = sin(t1)*sin(t5) + cos(t1)*cos(t5) = cos(t1 - t5)
    # R_phi0[1,0] = sin(t1)*cos(t5) - sin(t5)*cos(t1) = sin(t1 - t5)
    psi = sp.symbols('psi', real=True)
    # If theta5 = theta1 - psi, then theta1 - theta5 = psi:
    R_phi0_sub = sp.trigsimp(R_phi0.subs(t5, t1 - psi))
    print("\n  Substituting theta5 = theta1 - psi:")
    print(f"    n_x = {sp.trigsimp(R_phi0_sub[0,0])}  (Expected: cos(psi))")
    print(f"    n_y = {sp.trigsimp(R_phi0_sub[1,0])}  (Expected: sin(psi))")
    print(f"    n_z = {sp.trigsimp(R_phi0_sub[2,0])}  (Expected: 0)")
    assert sp.trigsimp(R_phi0_sub[0,0] - sp.cos(psi)) == 0
    assert sp.trigsimp(R_phi0_sub[1,0] - sp.sin(psi)) == 0
    assert R_phi0_sub[2,0] == 0
    print("  >>> PROOF SUCCESSFUL: Tool orientation vector n exactly aligns with grasp angle psi in horizontal plane.")

    # 5. Numerical Monte-Carlo Verification
    print("\n[Step 5] Numerical Monte-Carlo Stress Test (N = 10,000 random poses)...")
    np.random.seed(42)
    a1_val, a2_val, a3_val, d1_val, d5_val = 0.030, 0.145, 0.115, 0.105, 0.095

    def np_craig_mdh(alpha, a, d, theta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        return np.array([
            [ct,    -st,     0,    a],
            [st*ca,  ct*ca, -sa,  -d*sa],
            [st*sa,  ct*sa,  ca,   d*ca],
            [0,      0,      0,    1]
        ])

    def np_fk_matrix(joints):
        q1, q2, q3, q4, q5 = joints
        T1 = np_craig_mdh(0, 0, d1_val, q1)
        T2 = np_craig_mdh(np.pi/2, a1_val, 0, q2)
        T3 = np_craig_mdh(0, a2_val, 0, q3)
        T4 = np_craig_mdh(0, a3_val, 0, q4)
        T5 = np_craig_mdh(np.pi/2, 0, d5_val, q5)
        return T1 @ T2 @ T3 @ T4 @ T5

    def np_fk_closed_form(joints):
        q1, q2, q3, q4, q5 = joints
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)
        c234, s234 = np.cos(q2 + q3 + q4), np.sin(q2 + q3 + q4)
        px = c1 * (a1_val + a2_val * c2 + a3_val * c23 + d5_val * s234)
        py = s1 * (a1_val + a2_val * c2 + a3_val * c23 + d5_val * s234)
        pz = d1_val + a2_val * s2 + a3_val * s23 - d5_val * c234
        return np.array([px, py, pz])

    N_TRIALS = 10000
    q_samples = np.random.uniform(-np.pi/2, np.pi/2, size=(N_TRIALS, 5))
    q_samples[:, 2] = np.random.uniform(-np.pi/2, np.pi/3, size=N_TRIALS) # Joint 3 limits

    max_pos_err = 0.0
    for i in range(N_TRIALS):
        q = q_samples[i]
        T = np_fk_matrix(q)
        P_mat = T[:3, 3]
        P_ana = np_fk_closed_form(q)
        err = np.linalg.norm(P_mat - P_ana)
        if err > max_pos_err:
            max_pos_err = err

    print(f"  Evaluated {N_TRIALS:,} random joint configurations.")
    print(f"  Maximum Cartesian discrepancy: {max_pos_err:.2e} m")
    assert max_pos_err < 1e-14, f"Numerical discrepancy exceeds tolerance: {max_pos_err}"
    print("  >>> NUMERICAL STRESS TEST PASSED: Identical to 10^-15 m precision (machine epsilon).")
    print("=" * 70)

if __name__ == "__main__":
    main()
