// demo_bounce.js — hop physics + Three.js visualization
// A box, sphere, and two capsules bounce around inside a 6x6x6 room.

import * as THREE from 'three';

async function main() {
	const Module = await createHopModule();
	const sim = new Module.HopSimulator();
	sim.setGravity(0, 0, -9.81);

	// --- Room walls (6x6x6, floor at z=0, ceiling at z=6) ---
	const HS = 3;   // half-size
	const S = 6;    // full size
	const WT = 1;   // wall thickness

	function addWall(hx, hy, hz, px, py, pz) {
		const id = sim.addBox(1, hx, hy, hz);
		sim.setInfiniteMass(id);
		sim.setCoefficientOfGravity(id, 0);
		sim.setCoefficientOfRestitution(id, 1.0);
		sim.setCoefficientOfRestitutionOverride(id, true);
		sim.setFriction(id, 0, 0);
		sim.setPosition(id, px, py, pz);
		return id;
	}

	// Floor and ceiling
	addWall(HS, HS, WT / 2, 0, 0, -WT / 2);   // floor: z = -WT to 0
	addWall(HS, HS, WT / 2, 0, 0, S + WT / 2); // ceiling: z = S to S+WT

	// Walls -X, +X
	addWall(WT / 2, HS, S / 2, -HS - WT / 2, 0, S / 2);
	addWall(WT / 2, HS, S / 2, HS + WT / 2, 0, S / 2);

	// Walls -Y, +Y
	addWall(HS, WT / 2, S / 2, 0, -HS - WT / 2, S / 2);
	addWall(HS, WT / 2, S / 2, 0, HS + WT / 2, S / 2);

	// --- Dynamic objects ---
	const COR = 1.0;

	// Box: 1x1x1, starts at (1, 0, 4)
	const boxId = sim.addBox(1, 0.5, 0.5, 0.5);
	sim.setCoefficientOfRestitution(boxId, COR);
	sim.setCoefficientOfRestitutionOverride(boxId, true);
	sim.setFriction(boxId, 0, 0);
	sim.setPosition(boxId, 1, 0, 4);
	sim.setVelocity(boxId, 3, -2, 0);

	// Sphere: radius 0.5, starts at (-1, 1, 5)
	const sphereId = sim.addSphere(1, 0.5);
	sim.setCoefficientOfRestitution(sphereId, COR);
	sim.setCoefficientOfRestitutionOverride(sphereId, true);
	sim.setFriction(sphereId, 0, 0);
	sim.setPosition(sphereId, -1, 1, 5);
	sim.setVelocity(sphereId, -1, 3, 2);

	// Capsule: radius 0.4, direction (0,0,1.5), starts at (0, -1, 3)
	const capId = sim.addCapsule(1, 0.4, 0, 0, 1.5);
	sim.setCoefficientOfRestitution(capId, COR);
	sim.setCoefficientOfRestitutionOverride(capId, true);
	sim.setFriction(capId, 0, 0);
	sim.setPosition(capId, 0, -1, 3);
	sim.setVelocity(capId, 2, 1, -3);

	// Capsule 2: radius 0.3, direction (2,0,0) — horizontal along X, starts at (1, 1, 2)
	const cap2Id = sim.addCapsule(1, 0.3, 2, 0, 0);
	sim.setCoefficientOfRestitution(cap2Id, COR);
	sim.setCoefficientOfRestitutionOverride(cap2Id, true);
	sim.setFriction(cap2Id, 0, 0);
	sim.setPosition(cap2Id, 1, 1, 2);
	sim.setVelocity(cap2Id, -1, 2, 3);

	// --- Three.js setup ---
	const scene = new THREE.Scene();
	scene.background = new THREE.Color(0x1e1e28);

	const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 100);
	const renderer = new THREE.WebGLRenderer({ antialias: true });
	renderer.setSize(window.innerWidth, window.innerHeight);
	renderer.setPixelRatio(window.devicePixelRatio);
	document.body.appendChild(renderer.domElement);

	// Lighting
	scene.add(new THREE.AmbientLight(0x404050));
	const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
	dirLight.position.set(5, 10, 7);
	scene.add(dirLight);

	// Room wireframe (Y-up: room spans x[-3,3], y[0,6], z[-3,3])
	const roomGeo = new THREE.BoxGeometry(S, S, S);
	const roomWire = new THREE.LineSegments(
		new THREE.EdgesGeometry(roomGeo),
		new THREE.LineBasicMaterial({ color: 0x555555 })
	);
	// Room center in Three.js coords: (0, 3, 0) since floor at y=0, ceiling at y=6
	roomWire.position.set(0, S / 2, 0);
	scene.add(roomWire);

	// Semi-transparent floor
	const floorGeo = new THREE.PlaneGeometry(S, S);
	const floorMat = new THREE.MeshStandardMaterial({
		color: 0x3c3c50, transparent: true, opacity: 0.4, side: THREE.DoubleSide
	});
	const floor = new THREE.Mesh(floorGeo, floorMat);
	floor.rotation.x = -Math.PI / 2;
	scene.add(floor);

	// Box mesh
	const boxMat = new THREE.MeshStandardMaterial({ color: 0xcc3333 });
	const boxMesh = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), boxMat);
	scene.add(boxMesh);

	// Sphere mesh
	const sphereMat = new THREE.MeshStandardMaterial({ color: 0x3366cc });
	const sphereMesh = new THREE.Mesh(new THREE.SphereGeometry(0.5, 24, 16), sphereMat);
	scene.add(sphereMesh);

	// Capsule mesh — Three.js CapsuleGeometry(radius, length_of_cylinder, ...)
	// hop capsule: direction (0,0,1.5) means total length = 1.5 (center-to-center of hemispheres)
	const capMat = new THREE.MeshStandardMaterial({ color: 0x33aa44 });
	const capMesh = new THREE.Mesh(new THREE.CapsuleGeometry(0.4, 1.5, 8, 16), capMat);
	scene.add(capMesh);

	// Capsule 2 mesh — direction (2,0,0), horizontal along X
	// Three.js CapsuleGeometry creates a Y-aligned capsule, so rotate it to lie along X (Z in Three.js coords after hop→three swap)
	const cap2Mat = new THREE.MeshStandardMaterial({ color: 0xdd8833 });
	const cap2Mesh = new THREE.Mesh(new THREE.CapsuleGeometry(0.3, 2.0, 8, 16), cap2Mat);
	// Hop X maps to Three.js X, so rotate capsule from Y-axis to X-axis: rotate 90° around Z
	cap2Mesh.rotation.z = -Math.PI / 2;
	scene.add(cap2Mesh);

	// HUD
	const hud = document.getElementById('hud');

	// Hop is Z-up, Three.js is Y-up — swap Y and Z
	function hopToThree(x, y, z) {
		return [x, z, y];
	}

	let camAngle = 0;
	const CAM_DIST = 18;
	const CAM_HEIGHT = 8;

	function animate() {
		requestAnimationFrame(animate);

		sim.update(16);

		// Update mesh positions (Z-up → Y-up)
		boxMesh.position.set(...hopToThree(sim.getX(boxId), sim.getY(boxId), sim.getZ(boxId)));
		sphereMesh.position.set(...hopToThree(sim.getX(sphereId), sim.getY(sphereId), sim.getZ(sphereId)));

		// Capsule: position is the base; offset to center for Three.js mesh
		const cx = sim.getX(capId), cy = sim.getY(capId), cz = sim.getZ(capId);
		// hop capsule direction is (0,0,1.5), so center = position + (0,0,0.75)
		// In Three.js Y-up: center.y = cz + 0.75
		capMesh.position.set(...hopToThree(cx, cy, cz + 0.75));

		// Capsule 2: direction is (2,0,0), so center = position + (1,0,0)
		const c2x = sim.getX(cap2Id), c2y = sim.getY(cap2Id), c2z = sim.getZ(cap2Id);
		cap2Mesh.position.set(...hopToThree(c2x + 1, c2y, c2z));

		// Orbit camera
		camAngle += 0.005;
		camera.position.set(
			CAM_DIST * Math.cos(camAngle),
			CAM_HEIGHT,
			CAM_DIST * Math.sin(camAngle)
		);
		camera.lookAt(0, S / 2, 0);

		// HUD
		if (hud) {
			hud.textContent =
				`box:      z=${sim.getZ(boxId).toFixed(2)}\n` +
				`sphere:   z=${sim.getZ(sphereId).toFixed(2)}\n` +
				`capsule:  z=${sim.getZ(capId).toFixed(2)}\n` +
				`capsule2: z=${sim.getZ(cap2Id).toFixed(2)}`;
		}

		renderer.render(scene, camera);
	}

	animate();

	// Handle resize
	window.addEventListener('resize', () => {
		camera.aspect = window.innerWidth / window.innerHeight;
		camera.updateProjectionMatrix();
		renderer.setSize(window.innerWidth, window.innerHeight);
	});
}

main();
