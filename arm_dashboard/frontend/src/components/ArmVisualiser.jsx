import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { Compass, RotateCw } from 'lucide-react';

const JOINT_NAMES = [
  { name: 'J1 Waist', limit: [-180, 180] },
  { name: 'J2 Shoulder', limit: [-90, 90] },
  { name: 'J3 Elbow', limit: [-90, 90] },
  { name: 'J4 Wrist Pitch', limit: [-90, 90] },
  { name: 'J5 Wrist Roll', limit: [-90, 90] },
  { name: 'J6 Gripper', limit: [0, 60] },
];

export default function ArmVisualiser({ jointState }) {
  const mountRef = useRef(null);
  const armMeshRef = useRef(null);
  const linksRef = useRef([]);

  const currentAngles = jointState?.current_angles || [0, 0, 0, 0, 0, 0];
  const targetAngles = jointState?.target_angles || [0, 0, 0, 0, 0, 0];

  // Calculate Forward Kinematics End-Effector XYZ estimate (in meters)
  const q = currentAngles.map((a) => (a * Math.PI) / 180);
  const l1 = 0.12, l2 = 0.12, l3 = 0.10, l4 = 0.08;
  const eeX = (Math.sin(q[0]) * (l2 * Math.sin(q[1]) + l3 * Math.sin(q[1] + q[2]))).toFixed(3);
  const eeY = (Math.cos(q[0]) * (l2 * Math.sin(q[1]) + l3 * Math.sin(q[1] + q[2]))).toFixed(3);
  const eeZ = (0.10 + l2 * Math.cos(q[1]) + l3 * Math.cos(q[1] + q[2])).toFixed(3);

  useEffect(() => {
    const container = mountRef.current;
    if (!container) return;

    const width = container.clientWidth || 360;
    const height = container.clientHeight || 240;

    // Three.js Scene Setup
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0c11);

    const camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 10);
    camera.position.set(0.6, 0.5, 0.7);
    camera.lookAt(0, 0.15, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.replaceChildren(renderer.domElement);

    // Grid and Lighting
    const grid = new THREE.GridHelper(0.8, 16, 0x4f8ef7, 0x1f2433);
    grid.position.y = 0;
    scene.add(grid);

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.8);
    scene.add(ambientLight);
    const dirLight = new THREE.DirectionalLight(0x8b5cf6, 1.2);
    dirLight.position.set(1, 2, 1);
    scene.add(dirLight);

    // Workspace reach boundary sphere (transparent, R=0.35m)
    const sphereGeo = new THREE.SphereGeometry(0.35, 24, 24);
    const sphereMat = new THREE.MeshBasicMaterial({
      color: 0x06b6d4,
      wireframe: true,
      transparent: true,
      opacity: 0.12,
    });
    const reachSphere = new THREE.Mesh(sphereGeo, sphereMat);
    reachSphere.position.set(0, 0.12, 0);
    scene.add(reachSphere);

    // Robot Kinematic Group
    const robotGroup = new THREE.Group();
    scene.add(robotGroup);

    // 1. Base pedestal
    const baseGeo = new THREE.CylinderGeometry(0.06, 0.07, 0.04, 20);
    const baseMat = new THREE.MeshStandardMaterial({ color: 0x252a38, roughness: 0.5 });
    const baseMesh = new THREE.Mesh(baseGeo, baseMat);
    baseMesh.position.y = 0.02;
    robotGroup.add(baseMesh);

    // Arm Link Segments
    const materials = [
      new THREE.MeshStandardMaterial({ color: 0x4f8ef7, metalness: 0.3, roughness: 0.4 }),
      new THREE.MeshStandardMaterial({ color: 0x8b5cf6, metalness: 0.3, roughness: 0.4 }),
      new THREE.MeshStandardMaterial({ color: 0x06b6d4, metalness: 0.3, roughness: 0.4 }),
      new THREE.MeshStandardMaterial({ color: 0x4fc9a4, metalness: 0.3, roughness: 0.4 }),
      new THREE.MeshStandardMaterial({ color: 0xf7a84f, metalness: 0.3, roughness: 0.4 }),
    ];

    // J1 Group (Waist rotation around Y)
    const j1Group = new THREE.Group();
    j1Group.position.set(0, 0.04, 0);
    robotGroup.add(j1Group);

    const j1Mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.06, 16), materials[0]);
    j1Mesh.position.y = 0.03;
    j1Group.add(j1Mesh);

    // J2 Group (Shoulder pitch)
    const j2Group = new THREE.Group();
    j2Group.position.set(0, 0.06, 0);
    j1Group.add(j2Group);

    const j2Mesh = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.12, 0.04), materials[1]);
    j2Mesh.position.y = 0.06;
    j2Group.add(j2Mesh);

    // J3 Group (Elbow pitch)
    const j3Group = new THREE.Group();
    j3Group.position.set(0, 0.12, 0);
    j2Group.add(j3Group);

    const j3Mesh = new THREE.Mesh(new THREE.BoxGeometry(0.032, 0.10, 0.032), materials[2]);
    j3Mesh.position.y = 0.05;
    j3Group.add(j3Mesh);

    // J4 Group (Wrist pitch)
    const j4Group = new THREE.Group();
    j4Group.position.set(0, 0.10, 0);
    j3Group.add(j4Group);

    const j4Mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.05, 16), materials[3]);
    j4Mesh.position.y = 0.025;
    j4Group.add(j4Mesh);

    // J5 Gripper & End Effector
    const j5Group = new THREE.Group();
    j5Group.position.set(0, 0.05, 0);
    j4Group.add(j5Group);

    const gripperPalm = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.02, 0.04), materials[4]);
    j5Group.add(gripperPalm);

    // Gripper fingers
    const fingerGeo = new THREE.BoxGeometry(0.008, 0.035, 0.008);
    const leftFinger = new THREE.Mesh(fingerGeo, baseMat);
    leftFinger.position.set(-0.012, 0.02, 0);
    const rightFinger = new THREE.Mesh(fingerGeo, baseMat);
    rightFinger.position.set(0.012, 0.02, 0);
    j5Group.add(leftFinger);
    j5Group.add(rightFinger);

    // End-Effector 3-axis helper (RGB = XYZ)
    const axesHelper = new THREE.AxesHelper(0.06);
    axesHelper.position.set(0, 0.035, 0);
    j5Group.add(axesHelper);

    linksRef.current = [j1Group, j2Group, j3Group, j4Group, j5Group];
    armMeshRef.current = { robotGroup, scene, camera, renderer };

    // Orbit mouse interaction
    let isDragging = false;
    let prevMouseX = 0;
    let prevMouseY = 0;

    const onMouseDown = (e) => {
      isDragging = true;
      prevMouseX = e.clientX;
      prevMouseY = e.clientY;
    };

    const onMouseMove = (e) => {
      if (!isDragging) return;
      const dx = e.clientX - prevMouseX;
      const dy = e.clientY - prevMouseY;
      robotGroup.rotation.y += dx * 0.01;
      camera.position.y = Math.max(0.1, camera.position.y - dy * 0.005);
      camera.lookAt(0, 0.15, 0);
      prevMouseX = e.clientX;
      prevMouseY = e.clientY;
    };

    const onMouseUp = () => {
      isDragging = false;
    };

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
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    window.addEventListener('resize', handleResize);

    return () => {
      cancelAnimationFrame(animId);
      container.removeEventListener('mousedown', onMouseDown);
      window.removeEventListener('mousemove', onMouseMove);
      window.removeEventListener('mouseup', onMouseUp);
      window.removeEventListener('resize', handleResize);
      renderer.dispose();
    };
  }, []);

  // Drive Three.js joint groups from real jointState angles
  useEffect(() => {
    if (!linksRef.current || linksRef.current.length < 5) return;
    const [j1, j2, j3, j4, j5] = linksRef.current;
    if (j1 && currentAngles[0] !== undefined) {
      j1.rotation.y = (currentAngles[0] * Math.PI) / 180;
    }
    if (j2 && currentAngles[1] !== undefined) {
      j2.rotation.z = (currentAngles[1] * Math.PI) / 180;
    }
    if (j3 && currentAngles[2] !== undefined) {
      j3.rotation.z = (currentAngles[2] * Math.PI) / 180;
    }
    if (j4 && currentAngles[3] !== undefined) {
      j4.rotation.z = (currentAngles[3] * Math.PI) / 180;
    }
    if (j5 && currentAngles[4] !== undefined) {
      j5.rotation.y = (currentAngles[4] * Math.PI) / 180;
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
            Robot Digital Twin (5-DOF + Gripper)
          </span>
        </div>
        <div className="flex items-center gap-2 text-[10px] font-mono text-purple-400 bg-purple-950/40 px-2 py-0.5 rounded border border-purple-800/40">
          <span>SPHERE REACH: 0.35m</span>
        </div>
      </div>

      {/* 3D Viewport */}
      <div className="relative flex-1 min-h-[140px] bg-[#090b0f] overflow-hidden" ref={mountRef}>
        {/* End-Effector Telemetry Overlay */}
        <div className="absolute top-2 left-2 z-10 bg-black/80 border border-white/10 px-2 py-1 rounded text-[10px] font-mono text-slate-300">
          <div className="text-cyan-400 font-semibold mb-0.5">END EFFECTOR (BASE FRAME)</div>
          <div>X: <span className="text-white font-bold">{eeX}m</span></div>
          <div>Y: <span className="text-white font-bold">{eeY}m</span></div>
          <div>Z: <span className="text-white font-bold">{eeZ}m</span></div>
        </div>

        <div className="absolute bottom-2 right-2 z-10 text-[9px] font-mono text-slate-500 bg-black/60 px-1.5 py-0.5 rounded pointer-events-none">
          DRAG TO ROTATE SCENE
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
