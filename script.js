// Drone Simulator - Main JavaScript File
let scene, camera, renderer, drone, controls;
let clock = new THREE.Clock();

// Drone properties
const droneMass = 0.8; // kg
const armLength = 0.22; // meters (distance from center to motor)
const motorThrustConstant = 2.8e-6; // N/(rad/s)^2
const motorTorqueConstant = 1.1e-7; // Nm/(rad/s)^2
const inertia = {
    xx: 0.0014, // kg.m^2
    yy: 0.0014,
    zz: 0.0023
};

// Drone state
let droneState = {
    position: new THREE.Vector3(0, 1, 0), // Start 1m above ground
    velocity: new THREE.Vector3(0, 0, 0),
    acceleration: new THREE.Vector3(0, 0, 0),
    rotation: new THREE.Euler(0, 0, 0, 'XYZ'), // roll, pitch, yaw
    angularVelocity: new THREE.Vector3(0, 0, 0),
    angularAcceleration: new THREE.Vector3(0, 0, 0),
    motorSpeeds: [0, 0, 0, 0] // RPM for 4 motors
};

// Input state
const keys = {
    ArrowUp: false,
    ArrowDown: false,
    ArrowLeft: false,
    ArrowRight: false,
    Space: false,
    ShiftLeft: false,
    KeyW: false,
    KeyS: false,
    KeyA: false,
    KeyD: false,
    KeyQ: false,
    KeyE: false
};

// Initialize the scene
function init() {
    // Create scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x87ceeb);

    // Create camera
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(0, 3, 5);
    camera.lookAt(0, 1, 0);

    // Create renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(renderer.domElement);

    // Add controls for camera
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.enablePan = false;
    controls.minDistance = 2;
    controls.maxDistance = 10;
    controls.target.set(0, 1, 0);

    // Add lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 7);
    scene.add(directionalLight);

    // Create ground
    const groundGeometry = new THREE.PlaneGeometry(100, 100);
    const groundMaterial = new THREE.MeshStandardMaterial({
        color: 0x228B22,
        roughness: 0.8
    });
    const ground = new THREE.Mesh(groundGeometry, groundMaterial);
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = 0;
    scene.add(ground);

    // Create drone
    createDrone();

    // Set up event listeners
    window.addEventListener('resize', onWindowResize);
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    renderer.domElement.addEventListener('pointerdown', () => {
        renderer.domElement.requestPointerLock();
    });
    document.addEventListener('pointerlockchange', () => {
        if (document.pointerLockElement === renderer.domElement) {
            document.addEventListener('mousemove', onMouseMove);
        } else {
            document.removeEventListener('mousemove', onMouseMove);
        }
    });

    // Start animation loop
    animate();
}

// Create the drone model
function createDrone() {
    // Create a group for the drone
    drone = new THREE.Group();

    // Create central body
    const bodyGeometry = new THREE.CylinderGeometry(0.1, 0.1, 0.05, 16);
    const bodyMaterial = new THREE.MeshStandardMaterial({ color: 0x333333 });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    body.position.y = 0.025;
    drone.add(body);

    // Create arms and motors
    const motorPositions = [
        new THREE.Vector3(armLength, 0, armLength),   // Front right
        new THREE.Vector3(-armLength, 0, armLength),  // Front left
        new THREE.Vector3(-armLength, 0, -armLength), // Back left
        new THREE.Vector3(armLength, 0, -armLength)   // Back right
    ];

    const motorGeometry = new THREE.CylinderGeometry(0.02, 0.02, 0.03, 8);
    const motorMaterial = new THREE.MeshStandardMaterial({ color: 0xff0000 });

    for (let i = 0; i < 4; i++) {
        // Arm
        const armGeometry = new THREE.BoxGeometry(armLength*2, 0.005, 0.01);
        const armMaterial = new THREE.MeshStandardMaterial({ color: 0x555555 });
        const arm = new THREE.Mesh(armGeometry, armMaterial);

        // Position arm
        if (i % 2 === 0) { // Front motors (0, 2) - X axis aligned
            arm.rotation.z = Math.PI/2;
            arm.position.x = 0;
            arm.position.z = (i < 2) ? armLength : -armLength;
        } else { // Side motors (1, 3) - Z axis aligned
            arm.rotation.x = Math.PI/2;
            arm.position.x = (i === 1) ? -armLength : armLength;
            arm.position.z = 0;
        }
        arm.position.y = 0.025;
        drone.add(arm);

        // Motor
        const motor = new THREE.Mesh(motorGeometry, motorMaterial);
        motor.position.copy(motorPositions[i]);
        motor.position.y = 0.04;
        drone.add(motor);

        // Propeller (simple disc)
        const propGeometry = new THREE.CylinderGeometry(0.05, 0.05, 0.005, 16);
        const propMaterial = new THREE.MeshStandardMaterial({
            color: 0xffffff,
            opacity: 0.7,
            transparent: true
        });
        const prop = new THREE.Mesh(propGeometry, propMaterial);
        prop.position.copy(motorPositions[i]);
        prop.position.y = 0.055;
        prop.rotation.x = Math.PI/2;
        drone.add(prop);
    }

    drone.position.copy(droneState.position);
    scene.add(drone);

    // Initialize drone pose
    updateDronePose();
}

// Update drone physics
function updatePhysics(deltaTime) {
    // Forces and torques
    const forces = new THREE.Vector3(0, 0, 0);
    const torques = new THREE.Vector3(0, 0, 0);

    // Calculate thrust from each motor
    const thrusts = [];
    const torquesFromThrust = [];

    for (let i = 0; i < 4; i++) {
        const omega = droneState.motorSpeeds[i] * (2 * Math.PI / 60); // Convert RPM to rad/s
        const thrust = motorThrustConstant * omega * omega;
        const torque = motorTorqueConstant * omega * omega * (i % 2 === 0 ? 1 : -1); // Alternating torque

        thrusts.push(thrust);
        torquesFromThrust.push(torque);

        // Add force in drone's local up direction
        const localUp = new THREE.Vector3(0, 1, 0);
        localUp.applyEuler(droneState.rotation);
        forces.add(localUp.multiplyScalar(thrust));

        // Calculate torque from thrust (arm length cross force)
        const motorPos = [
            new THREE.Vector3(armLength, 0, armLength),
            new THREE.Vector3(-armLength, 0, armLength),
            new THREE.Vector3(-armLength, 0, -armLength),
            new THREE.Vector3(armLength, 0, -armLength)
        ][i];

        const torqueVector = motorPos.clone().cross(localUp.multiplyScalar(thrust));
        torques.add(torqueVector);
    }

    // Add torque from motor reactions
    torques.z += torquesFromThrust[0] + torquesFromThrust[1] + torquesFromThrust[2] + torquesFromThrust[3];

    // Add gravity
    forces.y -= droneMass * 9.81;

    // Add air resistance (simplified)
    forces.add(droneState.velocity.clone().multiplyScalar(-0.1));
    torques.add(droneState.angularVelocity.clone().multiplyScalar(-0.01));

    // Update accelerations (F=ma, τ=Iα)
    droneState.acceleration.copy(forces).divideScalar(droneMass);
    droneState.angularAcceleration.set(
        torques.x / inertia.xx,
        torques.y / inertia.yy,
        torques.z / inertia.zz
    );

    // Integrate motion (semi-implicit Euler)
    droneState.velocity.add(droneState.acceleration.clone().multiplyScalar(deltaTime));
    droneState.position.add(droneState.velocity.clone().multiplyScalar(deltaTime));

    droneState.angularVelocity.add(droneState.angularAcceleration.clone().multiplyScalar(deltaTime));
    droneState.rotation.x += droneState.angularVelocity.x * deltaTime;
    droneState.rotation.y += droneState.angularVelocity.y * deltaTime;
    droneState.rotation.z += droneState.angularVelocity.z * deltaTime;

    // Keep drone above ground
    if (droneState.position.y < 0.1) {
        droneState.position.y = 0.1;
        droneState.velocity.y = Math.max(0, droneState.velocity.y);
    }

    // Update motor speeds based on input
    updateMotorSpeeds();

    // Update visualization
    updateDronePose();
}

// Update motor speeds based on user input
function updateMotorSpeeds() {
    // Base hover throttle (adjust for drone mass and gravity)
    const hoverThrust = droneMass * 9.81 / 4; // Total thrust / 4 motors
    const hoverOmega = Math.sqrt(hoverThrust / motorThrustConstant);
    const baseSpeed = hoverOmega * (60 / (2 * Math.PI)); // Convert to RPM

    // Control inputs
    const throttle = (keys.KeyW || keys.ArrowUp ? 1 : 0) -
                    (keys.KeyS || keys.ArrowDown ? 1 : 0);
    const roll = (keys.KeyD || keys.ArrowRight ? 1 : 0) -
                (keys.KeyA || keys.ArrowLeft ? 1 : 0);
    const pitch = (keys.KeyS || keys.ArrowDown ? 1 : 0) -
                 (keys.KeyW || keys.ArrowUp ? 1 : 0);
    const yaw = (keys.KeyE ? 1 : 0) - (keys.KeyQ ? 1 : 0);
    const vertical = (keys.Space ? 1 : 0) - (keys.ShiftLeft ? 1 : 0);

    // Mixing matrix for quadcopter (X configuration)
    // Motor order: [front right, front left, back left, back right]
    const motorMix = [
        [ 1, -1, -1, -1], // Motor 0: front right
        [ 1,  1,  1, -1], // Motor 1: front left
        [ 1,  1, -1,  1], // Motor 2: back left
        [ 1, -1,  1,  1]  // Motor 3: back right
    ];

    // Calculate base motor speeds
    const baseSpeeds = [baseSpeed, baseSpeed, baseSpeed, baseSpeed];

    // Add control inputs (scaled appropriately)
    const rollFactor = 200;   // RPM per unit roll
    const pitchFactor = 200;  // RPM per unit pitch
    const yawFactor = 150;    // RPM per unit yaw
    const throttleFactor = 300; // RPM per unit throttle
    const verticalFactor = 200; // RPM per unit vertical

    for (let i = 0; i < 4; i++) {
        let adjustment =
            throttle * throttleFactor * motorMix[i][0] +
            roll * rollFactor * motorMix[i][1] +
            pitch * pitchFactor * motorMix[i][2] +
            yaw * yawFactor * motorMix[i][3] +
            vertical * verticalFactor;

        droneState.motorSpeeds[i] = Math.max(0, baseSpeeds[i] + adjustment);
    }
}

// Update drone visualization
function updateDronePose() {
    if (drone) {
        drone.position.copy(droneState.position);
        drone.rotation.copy(droneState.rotation);

        // Update motor visual speed (simple rotation)
        const propellers = drone.children.filter(child =>
            child.geometry &&
            child.geometry.parameters &&
            child.geometry.parameters.radiusTop === 0.05
        );

        propellers.forEach((propeller, index) => {
            propeller.rotation.z += droneState.motorSpeeds[index] * (2 * Math.PI / 60) * 0.016;
        });
    }
}

// Update info display
function updateInfoDisplay() {
    document.getElementById('altitude').textContent =
        `Độ cao: ${droneState.position.y.toFixed(2)} m`;

    document.getElementById('velocity').textContent =
        `Vận tốc: ${droneState.velocity.x.toFixed(2)}, ${droneState.velocity.y.toFixed(2)}, ${droneState.velocity.z.toFixed(2)} m/s`;

    document.getElementById('attitude').textContent =
        `Góc nghiêng: ${(droneState.rotation.x * 180/Math.PI).toFixed(2)}°, ${(droneState.rotation.y * 180/Math.PI).toFixed(2)}°, ${(droneState.rotation.z * 180/Math.PI).toFixed(2)}° (roll, pitch, yaw)`;

    document.getElementById('motor-speed').textContent =
        `Tốc độ motore: [${droneState.motorSpeeds[0].toFixed(0)}, ${droneState.motorSpeeds[1].toFixed(0)}, ${droneState.motorSpeeds[2].toFixed(0)}, ${droneState.motorSpeeds[3].toFixed(0)}] RPM`;

    // Simple battery drain simulation
    const totalThrottle = droneState.motorSpeeds.reduce((sum, speed) => sum + speed, 0) / 4000; // Normalize
    const batteryDrain = totalThrottle * 0.001; // Adjust as needed
    // In a real app, we would track actual battery level
}

// Event handlers
function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

function onKeyDown(event) {
    if (keys[event.code] !== undefined) {
        keys[event.code] = true;
    }
}

function onKeyUp(event) {
    if (keys[event.code] !== undefined) {
        keys[event.code] = false;
    }

    // Reset drone on 'R' key
    if (event.code === 'KeyR') {
        resetDrone();
    }
}

function onMouseMove(event) {
    if (document.pointerLockElement === renderer.domElement) {
        const moveX = event.movementX;
        const moveY = event.movementY;

        // Rotate camera based on mouse movement
        const rotationSpeed = 0.002;
        camera.rotation.y -= moveX * rotationSpeed;
        camera.rotation.x -= moveY * rotationSpeed;

        // Limit vertical look angle
        camera.rotation.x = Math.max(-Math.PI/2, Math.min(Math.PI/2, camera.rotation.x));
    }
}

// Reset drone to initial state
function resetDrone() {
    droneState.position.set(0, 1, 0);
    droneState.velocity.set(0, 0, 0);
    droneState.acceleration.set(0, 0, 0);
    droneState.rotation.set(0, 0, 0);
    droneState.angularVelocity.set(0, 0, 0);
    droneState.angularAcceleration.set(0, 0, 0);
    droneState.motorSpeeds = [0, 0, 0, 0];

    if (drone) {
        drone.position.copy(droneState.position);
        drone.rotation.copy(droneState.rotation);
    }
}

// Animation loop
function animate() {
    requestAnimationFrame(animate);

    const deltaTime = clock.getDelta();

    // Update physics
    updatePhysics(deltaTime);

    // Update controls
    controls.update();

    // Render
    renderer.render(scene, camera);

    // Update UI
    updateInfoDisplay();
}

// Initialize when page loads
window.addEventListener('load', init);