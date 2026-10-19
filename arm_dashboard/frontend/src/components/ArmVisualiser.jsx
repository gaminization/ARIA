import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { Compass, RotateCw } from 'lucide-react';

const JOINT_NAMES = [
  { name: 'J1 Waist', limit: [-180, 180] },
  { name: 'J2 Shoulder', limit: [-90, 90] },
  { name: 'J3 Elbow', limit: [-90, 90] },
  { name: 'J4 Wrist Pitch', limit: [-90, 90] },
  { name: 'J5 Gripper', limit: [0, 60] },
];

/*
 * Exact forward kinematics taken from arm_description/urdf/aria_arm.urdf.xacro.
 * Every joint in the real robot is revolute about its own local +Z axis; each
 * entry below is the URDF <origin xyz rpy> of that joint relative to its parent
 * link. Building the same chain in Three.js makes the twin track the physical
 * arm exactly (instead of the old illustrative 0.12/0.12/0.10 stick figure).
 */
const CHAIN = [
  { name: 'waist_joint', xyz: [-0.00345, -0.00001, 0.04449], rpy: [0, 0, 0] },
  { name: 'shoulder_joint', xyz: [0.00396, 0.01369, 0.03521], rpy: [1.5708, 0.03778, 1.5708] },
  { name: 'elbow_joint', xyz: [-0.00007, 0.11689, -0.00792], rpy: [-0.0013, -3.14159, 0.03778] },
  { name: 'wrist_pitch_joint', xyz: [-0.0088, 0.12752, -0.00487], rpy: [-0.01458, 3.14159, 0.0] },
  { name: 'gripper_joint', xyz: [-0.02735, 0.03009, -0.02013], rpy: [-1.27431, -1.58408, 1.5708] },
];
// Right finger mimics the left gripper joint (URDF <mimic>).
const MIMIC_JOINT = { name: 'finger_mimic_joint', xyz: [-0.02492, 0.03048, 0.00564], rpy: [-1.25673, -1.55752, -1.57079] };

// URDF fixed-axis (extrinsic XYZ) RPY -> quaternion, i.e. R = Rz(yaw)·Ry(pitch)·Rx(roll).
function rpyToQuaternion(rpy) {
  const [rr, rp, ry] = rpy;
  const cr = Math.cos(rr), sr = Math.sin(rr);
  const cp = Math.cos(rp), sp = Math.sin(rp);
  const cy = Math.cos(ry), sy = Math.sin(ry);
  const m = new THREE.Matrix4().set(
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0,
    -sp, cp * sr, cp * cr, 0,
    0, 0, 0, 1
  );
  return new THREE.Quaternion().setFromRotationMatrix(m);
}

function makeFrame({ xyz, rpy }) {
  const g = new THREE.Group();
  g.position.set(xyz[0], xyz[1], xyz[2]);
  g.quaternion.copy(rpyToQuaternion(rpy));
  return g;
}

// A box mesh placed with the given URDF local origin/rpy (used to approximate the STL links).
function makeBox(size, position, rpy, material) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(size[0], size[1], size[2]), material);
  mesh.position.set(position[0], position[1], position[2]);
  mesh.quaternion.copy(rpyToQuaternion(rpy || [0, 0, 0]));
  return mesh;
}

// A cylinder whose axis runs along the link's local +Z (URDF convention).
function makeCylinder(radius, length, position, material) {
  const geo = new THREE.CylinderGeometry(radius, radius, length, 24);
  geo.rotateX(Math.PI / 2); // three.js cylinder is Y-up; rotate so its axis is +Z
  const mesh = new THREE.Mesh(geo, material);
  mesh.position.set(position[0], position[1], position[2]);
  return mesh;
}

export default function ArmVisualiser({ jointState }) {
  const mountRef = useRef(null);
  const pivotRef = useRef(null);
  const rotGroupsRef = useRef([]); // [waist, shoulder, elbow, wrist, gripperL, gripperR]
  const tcpRef = useRef(null);

  const currentAngles = jointState?.current_angles || [0, 0, 0, 0, 0];
  const targetAngles = jointState?.target_angles || [0, 0, 0, 0, 0];

  const [ee, setEe] = useState({ x: '0.000', y: '0.000', z: '0.000' });

  useEffect(() => {
    const container = mountRef.current;
    if (!container) return;

    const width = container.clientWidth || 360;
    const height = container.clientHeight || 240;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0c11);

    const camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 20);
    camera.position.set(0.5, 0.42, -0.52); // URDF +Y maps to three -Z, so view from the front (-Z side)
    camera.lookAt(0, 0.2, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.replaceChildren(renderer.domElement);

    // ── Lighting & ground grid ─────────────────────────────────
    scene.add(new THREE.AmbientLight(0xffffff, 0.85));
    const dirLight = new THREE.DirectionalLight(0x9fb4ff, 1.3);
    dirLight.position.set(1.2, 2.4, 1.2);
    scene.add(dirLight);
    const fill = new THREE.DirectionalLight(0x8b5cf6, 0.5);
    fill.position.set(-1.2, 1.0, -1.5);
    scene.add(fill);

    const grid = new THREE.GridHelper(0.9, 18, 0x4f8ef7, 0x1f2433);
    grid.position.y = 0;
    scene.add(grid);

    // Workspace reach boundary (base offset in URDF z).
    const reachSphere = new THREE.Mesh(
      new THREE.SphereGeometry(0.36, 24, 24),
      new THREE.MeshBasicMaterial({ color: 0x06b6d4, wireframe: true, transparent: true, opacity: 0.10 })
    );
    reachSphere.position.set(0, 0.15, 0);
    scene.add(reachSphere);

    // ── Robot assembly ─────────────────────────────────────────
    // Everything is built in URDF coordinates (Z-up). The pivot lets the user
    // orbit; robotGroup rotates URDF Z-up onto the three.js Y-up world.
    const pivot = new THREE.Group();
    scene.add(pivot);
    pivotRef.current = pivot;

    const robotGroup = new THREE.Group();
    robotGroup.rotation.x = -Math.PI / 2; // URDF +Z -> three +Y, URDF +Y -> three -Z
    pivot.add(robotGroup);

    const materials = {
      base: new THREE.MeshStandardMaterial({ color: 0x2a3040, metalness: 0.4, roughness: 0.5 }),
      hollow: new THREE.MeshStandardMaterial({ color: 0x3a2f22, metalness: 0.55, roughness: 0.4 }),
      orangeA: new THREE.MeshStandardMaterial({ color: 0xf59e0b, metalness: 0.35, roughness: 0.4 }),
      orangeB: new THREE.MeshStandardMaterial({ color: 0xf97316, metalness: 0.35, roughness: 0.4 }),
      orangeC: new THREE.MeshStandardMaterial({ color: 0xfb923c, metalness: 0.3, roughness: 0.45 }),
      blue: new THREE.MeshStandardMaterial({ color: 0x3b82f6, metalness: 0.4, roughness: 0.35 }),
      joint: new THREE.MeshStandardMaterial({ color: 0x111827, metalness: 0.7, roughness: 0.3 }),
    };

    // base_link: white cylindrical base housing (URDF collision proxy)
    robotGroup.add(makeCylinder(0.0592, 0.0547, [-0.0029, 0.00016, 0.027], materials.base));

    const rotGroups = [];

    // waist_joint frame -> rotation -> waist_link visual
    const waistFrame = makeFrame(CHAIN[0]);
    robotGroup.add(waistFrame);
    const waistRot = new THREE.Group();
    waistFrame.add(waistRot);
    rotGroups.push(waistRot);
    // waist_link spans from its own origin up to the shoulder joint (z = 0.03521).
    waistRot.add(makeBox([0.078, 0.056, 0.036], [0, 0, 0.018], [0, 0, 0], materials.orangeA));
    waistRot.add(makeCylinder(0.022, 0.036, [0, 0, 0.036], materials.joint));

    // shoulder_joint frame -> rotation -> upper_arm_link visual
    const shoulderFrame = makeFrame(CHAIN[1]);
    waistRot.add(shoulderFrame);
    const shoulderRot = new THREE.Group();
    shoulderFrame.add(shoulderRot);
    rotGroups.push(shoulderRot);
    // upper_arm_link spans from the shoulder origin (y = 0) to the elbow joint (y = 0.11689).
    shoulderRot.add(makeBox([0.038, 0.118, 0.028], [-0.00004, 0.0584, -0.00396], [0, 0, 0], materials.orangeB));

    // elbow_joint frame -> rotation -> forearm_link visual
    const elbowFrame = makeFrame(CHAIN[2]);
    shoulderRot.add(elbowFrame);
    const elbowRot = new THREE.Group();
    elbowFrame.add(elbowRot);
    rotGroups.push(elbowRot);
    // forearm_link spans from the elbow origin (y = 0) to the wrist joint (y = 0.12752).
    elbowRot.add(makeBox([0.030, 0.128, 0.030], [-0.0044, 0.0638, -0.0024], [0, 0, 0], materials.orangeC));

    // wrist_pitch_joint frame -> rotation -> wrist_link visual
    const wristFrame = makeFrame(CHAIN[3]);
    elbowRot.add(wristFrame);
    const wristRot = new THREE.Group();
    wristFrame.add(wristRot);
    rotGroups.push(wristRot);
    // wrist_link spans from the wrist origin to the gripper mount (y ≈ 0.030, x ≈ -0.026).
    wristRot.add(makeBox([0.042, 0.034, 0.032], [-0.014, 0.015, -0.007], [0, 0, 0], materials.orangeA));

    // Left finger (gripper_joint)
    const gripFrameL = makeFrame(CHAIN[4]);
    wristRot.add(gripFrameL);
    const gripRotL = new THREE.Group();
    gripFrameL.add(gripRotL);
    rotGroups.push(gripRotL);
    gripRotL.add(makeBox([0.016, 0.014, 0.052], [0, 0, 0.03], [0, 0, 0], materials.blue));

    // Right finger (mimic joint)
    const gripFrameR = makeFrame(MIMIC_JOINT);
    wristRot.add(gripFrameR);
    const gripRotR = new THREE.Group();
    gripFrameR.add(gripRotR);
    rotGroups.push(gripRotR);
    gripRotR.add(makeBox([0.016, 0.014, 0.052], [0, 0, 0.03], [0, 0, 0], materials.blue));

    // Tool centre point (grasp point) marker attached to the wrist link.
    const tcp = new THREE.Object3D();
    tcp.position.set(0, 0.07, 0);
    wristRot.add(tcp);
    tcpRef.current = tcp;
    const tcpVis = new THREE.Mesh(
      new THREE.SphereGeometry(0.006, 12, 12),
      new THREE.MeshBasicMaterial({ color: 0x22d3ee })
    );
    tcp.add(tcpVis);

    rotGroupsRef.current = rotGroups;

    // ── Orbit / rotate interaction ─────────────────────────────
    let isDragging = false;
    let prevMouseX = 0;
    let prevMouseY = 0;

    const onMouseDown = (e) => { isDragging = true; prevMouseX = e.clientX; prevMouseY = e.clientY; };
    const onMouseMove = (e) => {
      if (!isDragging) return;
      const dx = e.clientX - prevMouseX;
      const dy = e.clientY - prevMouseY;
      pivot.rotation.y += dx * 0.01;
      camera.position.y = Math.max(0.08, camera.position.y - dy * 0.005);
      camera.lookAt(0, 0.2, 0);
      prevMouseX = e.clientX;
      prevMouseY = e.clientY;
    };
    const onMouseUp = () => { isDragging = false; };

    container.addEventListener('mousedown', onMouseDown);
    window.addEventListener('mousemove', onMouseMove);
    window.addEventListener('mouseup', onMouseUp);

    let animId;
    const animate = () => {
      animId = requestAnimationFrame(animate);
      renderer.render(scene, camera);
    };
    animate();

    const handleResize = () => {
      if (!container) return;
      const w = container.clientWidth;
      const h = container.clientHeight;
      if (!w || !h) return;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    window.addEventListener('resize', handleResize);
    requestAnimationFrame(handleResize);

    return () => {
      cancelAnimationFrame(animId);
      container.removeEventListener('mousedown', onMouseDown);
      window.removeEventListener('mousemove', onMouseMove);
      window.removeEventListener('mouseup', onMouseUp);
      window.removeEventListener('resize', handleResize);
      renderer.dispose();
    };
  }, []);

  // Drive the URDF joint groups from the live joint angles.
  useEffect(() => {
    const rotGroups = rotGroupsRef.current;
    if (!rotGroups || rotGroups.length < 6) return;

    const setRot = (group, deg) => { group.rotation.z = (deg * Math.PI) / 180; };
    setRot(rotGroups[0], currentAngles[0] ?? 0); // waist
    setRot(rotGroups[1], currentAngles[1] ?? 0); // shoulder
    setRot(rotGroups[2], currentAngles[2] ?? 0); // elbow
    setRot(rotGroups[3], currentAngles[3] ?? 0); // wrist pitch
    const grip = currentAngles[4] ?? 0;
    setRot(rotGroups[4], grip); // left finger
    setRot(rotGroups[5], grip); // right finger (mimic)

    // Report the true end-effector position in the robot base frame (URDF axes).
    const pivot = pivotRef.current;
    const tcp = tcpRef.current;
    if (pivot && tcp && pivot.parent) {
      pivot.updateMatrixWorld(true);
      const world = new THREE.Vector3();
      tcp.getWorldPosition(world);
      // robotGroup = pivot.children[0] built purely in URDF coords.
      const robotGroup = pivot.children[0];
      const local = robotGroup.worldToLocal(world.clone());
      setEe({ x: local.x.toFixed(3), y: local.y.toFixed(3), z: local.z.toFixed(3) });
    }
  }, [currentAngles]);

  const getJointColor = (val, [min, max]) => {
    const margin = 10;
    if (val <= min + margin || val >= max - margin) return 'text-amber-400 bg-amber-500';
    return 'text-emerald-400 bg-emerald-500';
  };

  return (
    <div className="panel-card h-full flex flex-col bg-[#13161e] border border-[#252a38] rounded-md overflow-hidden select-none">
      {/* Top Header */}
      <div className="h-9 px-3 border-b border-[#252a38] flex items-center justify-between bg-[#0e1017]">
        <div className="flex items-center gap-2">
          <Compass className="w-3.5 h-3.5 text-purple-400" />
          <span className="text-xs font-semibold tracking-wider text-slate-200 uppercase">
            5-DOF Arm + Gripper — URDF Kinematic Twin
          </span>
        </div>
        <div className="flex items-center gap-1.5 text-[10px] font-mono text-emerald-400 bg-emerald-950/40 px-2 py-0.5 rounded border border-emerald-800/40">
          <RotateCw className="w-2.5 h-2.5" />
          <span>LIVE JOINT-STATE MIRROR</span>
        </div>
      </div>

      {/* 3D Viewport */}
      <div className="relative flex-1 min-h-[140px] bg-[#090b0f] overflow-hidden" ref={mountRef}>
        {/* End-Effector Telemetry Overlay */}
        <div className="absolute top-2 left-2 z-10 bg-black/80 border border-white/10 px-2 py-1 rounded text-[10px] font-mono text-slate-300">
          <div className="text-cyan-400 font-semibold mb-0.5">END-EFFECTOR POSITION (TCP)</div>
          <div>X: <span className="text-white font-bold">{ee.x}m</span></div>
          <div>Y: <span className="text-white font-bold">{ee.y}m</span></div>
          <div>Z: <span className="text-white font-bold">{ee.z}m</span></div>
        </div>

        <div className="absolute bottom-2 right-2 z-10 text-[9px] font-mono text-slate-500 bg-black/60 px-1.5 py-0.5 rounded pointer-events-none">
          DRAG TO ORBIT · URDF-EXACT KINEMATICS
        </div>
      </div>

      {/* Joint Angle Bars */}
      <div className="p-2 border-t border-[#252a38] bg-[#0e1017] flex flex-col gap-1.5">
        <div className="grid grid-cols-2 gap-x-3 gap-y-1">
          {JOINT_NAMES.map((j, idx) => {
            const cur = currentAngles[idx] ?? 0;
            const tgt = targetAngles[idx] ?? cur;
            const [min, max] = j.limit;
            const pct = Math.min(100, Math.max(0, ((cur - min) / (max - min)) * 100));
            const colorClass = getJointColor(cur, [min, max]);

            return (
              <div key={idx} className="flex flex-col text-[10px] font-mono">
                <div className="flex justify-between items-center text-slate-300">
                  <span className="font-semibold">{j.name}</span>
                  <span>
                    <span className="text-white font-bold">{cur.toFixed(1)}°</span>
                    <span className="text-slate-500 text-[9px] ml-1">(tgt: {tgt.toFixed(0)}°)</span>
                  </span>
                </div>
                <div className="w-full bg-[#1a1f2c] h-1.5 rounded-full overflow-hidden mt-0.5">
                  <div
                    className={`h-full rounded-full transition-all duration-150 ${colorClass.split(' ')[1]}`}
                    style={{ width: `${pct}%` }}
                  />
                </div>
              </div>
            );
          })}
        </div>
      </div>
    </div>
  );
}
