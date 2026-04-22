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
		const wall = sim.addSolid();
		wall.setInfiniteMass();
		wall.addShape(Module.HopShape.box(hx, hy, hz));
		wall.setCoefficientOfGravity(0);
		wall.setCoefficientOfRestitution(1.0);
		wall.setCoefficientOfRestitutionOverride(true);
		wall.setFriction(0, 0);
		wall.setPosition(px, py, pz);
		return wall;
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

	function addDynamic(shape) {
		const solid = sim.addSolid();
		solid.setMass(1);
		solid.addShape(shape);
		solid.setCoefficientOfRestitution(COR);
		solid.setCoefficientOfRestitutionOverride(true);
		solid.setFriction(0, 0);
		return solid;
	}

	// Box: 1x1x1, starts at (1, 0, 4)
	const box = addDynamic(Module.HopShape.box(0.5, 0.5, 0.5));
	box.setPosition(1, 0, 4);
	box.setVelocity(3, -2, 0);

	// Sphere: radius 0.5, starts at (-1, 1, 5)
	const sphere = addDynamic(Module.HopShape.sphere(0.5));
	sphere.setPosition(-1, 1, 5);
	sphere.setVelocity(-1, 3, 2);

	// Capsule: radius 0.4, direction (0,0,1.5), hangs from the ceiling via a spring constraint
	const cap = addDynamic(Module.HopShape.capsule(0.4, 0, 0, 1.5));
	cap.setPosition(2, 0, 4);
	cap.setVelocity(-3, 2, 0);
	const anchorPoint = {x: 0, y: 0, z: S}; // ceiling
	const rope = sim.addConstraint(cap, anchorPoint.x, anchorPoint.y, anchorPoint.z);
	rope.setSpringConstant(5);
	rope.setDampingConstant(0.5);
	rope.setDistanceThreshold(2);

	// Capsule 2: radius 0.3, direction (2,0,0) — horizontal along X, starts at (1, 1, 2)
	const cap2 = addDynamic(Module.HopShape.capsule(0.3, 2, 0, 0));
	cap2.setPosition(1, 1, 2);
	cap2.setVelocity(-1, 2, 3);

	// Compound dumbbell: two spheres on one solid, offset via shape local_position.
	// Demonstrates compound colliders (Phase 2: shape local_position).
	const dumbbell = sim.addSolid();
	dumbbell.setMass(1);
	dumbbell.setCoefficientOfRestitution(COR);
	dumbbell.setCoefficientOfRestitutionOverride(true);
	dumbbell.setFriction(0, 0);
	const dbLeft = Module.HopShape.sphere(0.35);
	dbLeft.setLocalPosition(-0.7, 0, 0);
	dumbbell.addShape(dbLeft);
	const dbRight = Module.HopShape.sphere(0.35);
	dbRight.setLocalPosition(0.7, 0, 0);
	dumbbell.addShape(dbRight);
	dumbbell.setPosition(-1, -1, 4);
	dumbbell.setVelocity(2, 1, -1);

	// --- Collision sparks ---
	const sparks = [];

	function onCollision(c) {
		const speed = Math.sqrt(c.velocity.x ** 2 + c.velocity.y ** 2 + c.velocity.z ** 2);
		const count = Math.min(16, 4 + Math.floor(speed * 1.5));
		for (let i = 0; i < count; i++) {
			const angle = (i / count) * Math.PI * 2 + Math.random() * 0.5;
			const pitch = (Math.random() - 0.5) * 2 * Math.PI / 3;
			const spd = 1.5 + Math.random() * 2;
			const maxLife = 0.3 + Math.random() * 0.25;
			sparks.push({
				// Store in hop coords (Z-up), convert when rendering
				x: c.impact.x, y: c.impact.y, z: c.impact.z,
				vx: Math.cos(pitch) * Math.cos(angle) * spd,
				vy: Math.cos(pitch) * Math.sin(angle) * spd,
				vz: Math.sin(pitch) * spd,
				life: maxLife,
				maxLife,
			});
		}
	}

	box.setCollisionCallback(onCollision);
	sphere.setCollisionCallback(onCollision);
	cap.setCollisionCallback(onCollision);
	cap2.setCollisionCallback(onCollision);
	dumbbell.setCollisionCallback(onCollision);

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

	// Dumbbell meshes — two spheres + a connecting rod (visual only)
	const dumbbellMat = new THREE.MeshStandardMaterial({ color: 0xa060c0 });
	const dbLeftMesh = new THREE.Mesh(new THREE.SphereGeometry(0.35, 20, 12), dumbbellMat);
	const dbRightMesh = new THREE.Mesh(new THREE.SphereGeometry(0.35, 20, 12), dumbbellMat);
	scene.add(dbLeftMesh);
	scene.add(dbRightMesh);
	const dbRodGeo = new THREE.BufferGeometry().setFromPoints([
		new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0)
	]);
	const dbRod = new THREE.Line(dbRodGeo, new THREE.LineBasicMaterial({ color: 0xb478c8 }));
	scene.add(dbRod);

	// Spark particles (pre-allocate buffer, render as points)
	const MAX_SPARKS = 256;
	const sparkGeo = new THREE.BufferGeometry();
	const sparkPositions = new Float32Array(MAX_SPARKS * 3);
	const sparkColors = new Float32Array(MAX_SPARKS * 3);
	sparkGeo.setAttribute('position', new THREE.BufferAttribute(sparkPositions, 3));
	sparkGeo.setAttribute('color', new THREE.BufferAttribute(sparkColors, 3));
	sparkGeo.setDrawRange(0, 0);
	const sparkMat = new THREE.PointsMaterial({
		size: 0.08,
		vertexColors: true,
		transparent: true,
		depthWrite: false,
		sizeAttenuation: true,
	});
	const sparkPoints = new THREE.Points(sparkGeo, sparkMat);
	scene.add(sparkPoints);

	// Rope line from ceiling anchor to capsule
	const ropeGeo = new THREE.BufferGeometry().setFromPoints([
		new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0)
	]);
	const ropeLine = new THREE.Line(ropeGeo, new THREE.LineBasicMaterial({ color: 0xaaaaaa }));
	scene.add(ropeLine);

	// HUD
	const hud = document.getElementById('hud');

	// Hop is Z-up, Three.js is Y-up — swap Y and Z
	function hopToThree(pos) {
		return [pos.x, pos.z, pos.y];
	}

	let camAngle = 0;
	const CAM_DIST = 18;
	const CAM_HEIGHT = 8;
	let lastTime = performance.now();
	let frameCount = 0;
	let fps = 0;

	function animate() {
		requestAnimationFrame(animate);

		sim.update(16);

		// Update mesh positions (Z-up → Y-up)
		boxMesh.position.set(...hopToThree(box.getPosition()));
		sphereMesh.position.set(...hopToThree(sphere.getPosition()));

		// Capsule: position is the base; offset to center for Three.js mesh
		const cp = cap.getPosition();
		// hop capsule direction is (0,0,1.5), so center = position + (0,0,0.75)
		// In Three.js Y-up: center.y = cz + 0.75
		capMesh.position.set(...hopToThree({x: cp.x, y: cp.y, z: cp.z + 0.75}));

		// Update rope line from anchor to capsule
		const ropePositions = ropeLine.geometry.attributes.position.array;
		const anchorThree = hopToThree(anchorPoint);
		ropePositions[0] = anchorThree[0]; ropePositions[1] = anchorThree[1]; ropePositions[2] = anchorThree[2];
		const capThree = hopToThree(cp);
		ropePositions[3] = capThree[0]; ropePositions[4] = capThree[1]; ropePositions[5] = capThree[2];
		ropeLine.geometry.attributes.position.needsUpdate = true;

		// Capsule 2: direction is (2,0,0), so center = position + (1,0,0)
		const c2p = cap2.getPosition();
		cap2Mesh.position.set(...hopToThree({x: c2p.x + 1, y: c2p.y, z: c2p.z}));

		// Dumbbell: each sphere at solid.position + shape.local_position (hop frame)
		const dbp = dumbbell.getPosition();
		const dbL = {x: dbp.x - 0.7, y: dbp.y, z: dbp.z};
		const dbR = {x: dbp.x + 0.7, y: dbp.y, z: dbp.z};
		dbLeftMesh.position.set(...hopToThree(dbL));
		dbRightMesh.position.set(...hopToThree(dbR));
		// Connecting rod
		const dbRodPositions = dbRod.geometry.attributes.position.array;
		const dbLt = hopToThree(dbL);
		const dbRt = hopToThree(dbR);
		dbRodPositions[0] = dbLt[0]; dbRodPositions[1] = dbLt[1]; dbRodPositions[2] = dbLt[2];
		dbRodPositions[3] = dbRt[0]; dbRodPositions[4] = dbRt[1]; dbRodPositions[5] = dbRt[2];
		dbRod.geometry.attributes.position.needsUpdate = true;

		// Update sparks
		const dt = 1 / 60;
		for (let i = sparks.length - 1; i >= 0; i--) {
			const s = sparks[i];
			s.x += s.vx * dt;
			s.y += s.vy * dt;
			s.z += s.vz * dt;
			s.vz -= 9.8 * dt;
			s.life -= dt;
			if (s.life <= 0) {
				sparks[i] = sparks[sparks.length - 1];
				sparks.pop();
			}
		}
		const n = Math.min(sparks.length, MAX_SPARKS);
		for (let i = 0; i < n; i++) {
			const s = sparks[i];
			const t = s.life / s.maxLife;
			// Hop Z-up to Three.js Y-up
			sparkPositions[i * 3] = s.x;
			sparkPositions[i * 3 + 1] = s.z;
			sparkPositions[i * 3 + 2] = s.y;
			// Yellow-orange fade
			sparkColors[i * 3] = 1.0;
			sparkColors[i * 3 + 1] = 0.78 * t + 0.22;
			sparkColors[i * 3 + 2] = 0.2 * t;
		}
		sparkGeo.attributes.position.needsUpdate = true;
		sparkGeo.attributes.color.needsUpdate = true;
		sparkGeo.setDrawRange(0, n);

		// Orbit camera
		camAngle += 0.005;
		camera.position.set(
			CAM_DIST * Math.cos(camAngle),
			CAM_HEIGHT,
			CAM_DIST * Math.sin(camAngle)
		);
		camera.lookAt(0, S / 2, 0);

		// FPS counter
		frameCount++;
		const now = performance.now();
		if (now - lastTime >= 1000) {
			fps = frameCount;
			frameCount = 0;
			lastTime = now;
		}

		// HUD
		if (hud) {
			hud.innerHTML =
				`<span style="color:#ccc">float</span>\n` +
				`<span style="color:#ccc">${fps} FPS</span>\n\n` +
				`<span style="color:#cc3333">box:      z=${box.getPosition().z.toFixed(2)}</span>\n` +
				`<span style="color:#3366cc">sphere:   z=${sphere.getPosition().z.toFixed(2)}</span>\n` +
				`<span style="color:#33aa44">capsule:  z=${cap.getPosition().z.toFixed(2)}</span>\n` +
				`<span style="color:#dd8833">capsule2: z=${cap2.getPosition().z.toFixed(2)}</span>\n` +
				`<span style="color:#a060c0">dumbbell: z=${dumbbell.getPosition().z.toFixed(2)}</span>`;
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
