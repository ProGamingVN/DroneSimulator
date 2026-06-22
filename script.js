// Drone Simulator using Three.js
// Educational simulation only - not for real flight

// Global variables for Three.js
let scene, camera, renderer;
let droneGroup; // Group to hold all drone parts
let ground;

// Physics variables
let mass = 1.0; // kg
let g = 9.81; // m/s^2
let max_thrust = 2 * mass * g; // N (so 50% throttle gives hover)

let position = new THREE.Vector3(0, 0, 0); // world position (y is up)
let velocity = new THREE.Vector3(0, 0, 0); // world velocity
let quaternion = new THREE.Vector3(0, 0, 0, 1).set(0, 0, 0, 1); // [x, y, z, w]
let angularVelocity = new THREE.Vector3(0, 0, 0); // body frame angular velocity (rad/s)

// Inertia (symmetric)
let Ixx = 0.1, Iyy = 0.1, Izz = 0.1; // kg*m^2

// Motor constants
let armLength = 0.5; // meters
let motorRadius = 0.05;
let bodySize = 0.3;

// PID controller class
class PID {
    constructor(Kp, Ki, Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.integral = 0;
        this.previousError = 0;
    }

    compute(error, dt) {
        if (dt <= 0) return 0;
        this.integral += error * dt;
        const derivative = (error - this.previousError) / dt;
        const output = this.Kp * error + this.Ki * this.integral + this.Kd * derivative;
        this.previousError = error;
        return output;
    }

    reset() {
        this.integral = 0;
        this.previousError = 0;
    }
}

// PID controllers for attitude (roll, pitch, yaw)
let pidRoll = new PID(4.0, 0.2, 0.8); // tune these gains
let pidPitch = new PID(4.0, 0.2, 0.8);
let pidYaw = new PID(2.0, 0.0, 0.5); // yaw PID often needs less integral

// User input state
let keys = {
    ArrowUp: false,
    ArrowDown: false,
    ArrowLeft: false,
    ArrowRight: false,
    KeyW: false,
    KeyS: false,
    Comma: false,
    Period: false
};

// Desired setpoints (from user input)
let desiredRoll = 0; // degrees
let desiredPitch = 0; // degrees
let desiredYaw = 0; // degrees
let desiredThrottle = 0.0; // 0 to 1

// Input step sizes
const rollStep = 2; // degrees per key press
const pitchStep = 2;
const yawStep = 2;
const throttleStep = 0.02;

// Motor thrust coefficients (simplified)
let thrustConstant = 1.0; // N/(rad/s)^2 - actual value depends on motor/prop
let torqueConstant = 0.02; // N*m/(rad/s)^2 - yaw torque per motor

// Motor forces (will be computed each frame)
let motorForces = [0, 0, 0, 0]; // F1, F2, F3, F4

// Initialize the simulation
function init() {
    // Scene setup
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x87ceeb); // sky blue

    // Camera
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(0, 5, 10);
    camera.lookAt(0, 0, 0);

    // Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true, canvas: document.getElementById('droneCanvas') });
    renderer.setSize(window.innerWidth, window.innerHeight);

    // Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 7);
    scene.add(directionalLight);

    // Ground plane
    const groundGeometry = new THREE.PlaneGeometry(100, 100);
    const groundMaterial = new THREE.MeshStandardMaterial({ color: 0x555555 });
    ground = new THREE.Mesh(groundGeometry, groundMaterial);
    ground.rotation.x = -Math.PI / 2; // rotate to be horizontal
    ground.position.y = 0;
    scene.add(ground);

    // Create drone model
    createDroneModel();

    // Set initial drone position slightly above ground
    position.y = 1.0;

    // Event listeners for keyboard input
    window.addEventListener('keydown', (e) => {
        if (keys.hasOwnProperty(e.code)) keys[e.code] = true;
    });
    window.addEventListener('keyup', (e) => {
        if (keys.hasOwnProperty(e.code)) keys[e.code] = false;
    });

    // Handle window resize
    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });

    // Start animation loop
    animate();
}

// Create the drone visual model
function createDroneModel() {
    droneGroup = new THREE.Group();

    // Central body (cube)
    const bodyGeometry = new THREE.BoxGeometry(bodySize, bodySize, bodySize);
    const bodyMaterial = new THREE.MeshStandardMaterial({ color: 0x606060 });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    body.position.y = 0; // center at origin
    droneGroup.add(body);

    // Arms and motors
    const armColor = 0x009688; // teal
    const motorColor = 0x424242; // dark gray
    const propColor = 0x212121; // very dark gray

    // Motor positions: [front, right, back, left] -> [+y, +x, -y, -x] in drone's body frame
    // Note: In our body frame: x=forward, y=right, z=down
    // But for visualization, we'll place arms along x and z axes?
    // Let's define:
    //   Arm 1 (front): along +z (using three.js z as forward? Actually we want y up, so let's use x for forward, z for right)
    //   To avoid confusion, we'll place:
    //   Motor 1 (front): (0, 0, armLength)  // +z
    //   Motor 2 (right): (armLength, 0, 0)   // +x
    //   Motor 3 (back): (0, 0, -armLength)   // -z
    //   Motor 4 (left): (-armLength, 0, 0)   // -x
    // This means the drone's front is +z, right is +x.
    // Then the body's forward is +z, right is +x, up is +y.

    const armLengthVis = armLength * 0.8; // slightly shorter for visuals
    const armThickness = 0.05;
    const armWidth = 0.15;

    // Arm 1 (front)
    const arm1Geometry = new THREE.BoxGeometry(armThickness, armWidth, armLengthVis);
    const arm1Material = new THREE.MeshStandardMaterial({ color: armColor });
    const arm1 = new THREE.Mesh(arm1Geometry, arm1Material);
    arm1.position.set(0, 0, armLengthVis/2);
    droneGroup.add(arm1);

    // Motor 1 (front)
    const motor1Geometry = new THREE.CylinderGeometry(motorRadius, motorRadius, 0.02, 16);
    const motor1Material = new THREE.MeshStandardMaterial({ color: motorColor });
    const motor1 = new THREE.Mesh(motor1Geometry, motor1Material);
    motor1.position.set(0, 0, armLengthVis);
    droneGroup.add(motor1);

    // Propeller 1 (front) - spinning disc
    const prop1Geometry = new THREE.CylinderGeometry(0.15, 0.15, 0.01, 32);
    const prop1Material = new THREE.MeshStandardMaterial({ color: propColor });
    const prop1 = new THREE.Mesh(prop1Geometry, prop1Material);
    prop1.position.set(0, 0, armLengthVis + 0.015);
    prop1.rotation.x = Math.PI / 2; // make it face forward? Actually propeller should be perpendicular to arm
    // For front arm along z, propeller should be in x-y plane?
    // We'll make it spin around the arm's axis (z-axis) so rotation should be around z.
    // But we set the cylinder's height along z, so to have it perpendicular to arm (along z), we need to rotate it 90 degrees around x or y.
    // Let's set the propeller as a disc in the x-y plane (so normal along z).
    // Then for front arm (along z), the disc is already in x-y plane -> good.
    // So we leave rotation as is (default cylinder along z, we want it in x-y plane -> rotate 90 around x).
    prop1.rotation.x = Math.PI / 2;
    droneGroup.add(prop1);

    // Arm 2 (right)
    const arm2Geometry = new THREE.BoxGeometry(armLengthVis, armWidth, armThickness);
    const arm2Material = new THREE.MeshStandardMaterial({ color: armColor });
    const arm2 = new THREE.Mesh(arm2Geometry, arm2Material);
    arm2.position.set(armLengthVis/2, 0, 0);
    droneGroup.add(arm2);

    // Motor 2 (right)
    const motor2 = motor1.clone();
    motor2.position.set(armLengthVis, 0, 0);
    droneGroup.add(motor2);

    // Propeller 2 (right)
    const prop2 = prop1.clone();
    prop2.position.set(armLengthVis, 0, 0.015);
    prop2.rotation.x = Math.PI / 2;
    // For right arm along x, propeller should be in y-z plane -> rotate 90 around y
    prop2.rotation.y = Math.PI / 2;
    droneGroup.add(prop2);

    // Arm 3 (back)
    const arm3Geometry = new THREE.BoxGeometry(armThickness, armWidth, armLengthVis);
    const arm3Material = new THREE.MeshStandardMaterial({ color: armColor });
    const arm3 = new THREE.Mesh(arm3Geometry, arm3Material);
    arm3.position.set(0, 0, -armLengthVis/2);
    droneGroup.add(arm3);

    // Motor 3 (back)
    const motor3 = motor1.clone();
    motor3.position.set(0, 0, -armLengthVis);
    droneGroup.add(motor3);

    // Propeller 3 (back)
    const prop3 = prop1.clone();
    prop3.position.set(0, 0, -armLengthVis - 0.015);
    prop3.rotation.x = Math.PI / 2;
    droneGroup.add(prop3);

    // Arm 4 (left)
    const arm4Geometry = new THREE.BoxGeometry(armLengthVis, armWidth, armThickness);
    const arm4Material = new THREE.MeshStandardMaterial({ color: armColor });
    const arm4 = new THREE.Mesh(arm4Geometry, arm4Material);
    arm4.position.set(-armLengthVis/2, 0, 0);
    droneGroup.add(arm4);

    // Motor 4 (left)
    const motor4 = motor1.clone();
    motor4.position.set(-armLengthVis, 0, 0);
    droneGroup.add(motor4);

    // Propeller 4 (left)
    const prop4 = prop1.clone();
    prop4.position.set(-armLengthVis, 0, 0.015);
    prop4.rotation.x = Math.PI / 2;
    prop4.rotation.y = Math.PI / 2;
    droneGroup.add(prop4);

    // Store references to propellers for animation
    droneGroup.propellers = [prop1, prop2, prop3, prop4];

    // Add drone to scene
    scene.add(droneGroup);
}

// Update drone visual based on physics state
function updateDroneVisual(dt) {
    // Set position and orientation
    droneGroup.position.copy(position);
    droneGroup.setRotationFromQuaternion(quaternion);

    // Update propeller spins (based on motor forces)
    const spinSpeeds = [0, 0, 0, 0];
    // Convert thrust to angular speed (simplified: omega = sqrt(F / k))
    for (let i = 0; i < 4; i++) {
        const F = Math.max(0, motorForces[i]);
        spinSpeeds[i] = Math.sqrt(F / thrustConstant) * 5; // scale factor for visual
    }

    // Apply rotation to each propeller
    droneGroup.propellers[0].rotation.z += spinSpeeds[0] * dt; // front
    droneGroup.propellers[1].rotation.z += spinSpeeds[1] * dt; // right
    droneGroup.propellers[2].rotation.z += spinSpeeds[2] * dt; // back
    droneGroup.propellers[3].rotation.z += spinSpeeds[3] * dt; // left
}

// Physics update function
function updatePhysics(dt) {
    // 1. Get user input and update desired setpoints
    processInput(dt);

    // 2. Compute current orientation angles (roll, pitch, yaw) from quaternion
    // Convert quaternion to Euler angles (in degrees)
    const euler = new THREE.Euler();
    euler.setFromQuaternion(quaternion, 'YXZ'); // yaw, pitch, roll - but we want roll, pitch, yaw
    // Actually, we'll extract roll (x), pitch (y), yaw (z) from Euler set as 'XYZ'
    euler.setFromQuaternion(quaternion, 'XYZ');
    const roll = THREE.MathUtils.radToDeg(euler.x);
    const pitch = THREE.MathUtils.radToDeg(euler.y);
    const yaw = THREE.MathUtils.radToDeg(euler.z);

    // 3. Compute errors
    const rollError = desiredRoll - roll;
    const pitchError = desiredPitch - pitch;
    const yawError = desiredYaw - yaw;

    // 4. Compute PID outputs (desired torques in body frame, N*m)
    const tauRoll = pidRoll.compute(rollError, dt);
    const tauPitch = pidPitch.compute(pitchError, dt);
    const tauYaw = pidYaw.compute(yawError, dt);

    // 5. Compute collective thrust from throttle
    const Ftotal = desiredThrottle * max_thrust;

    // 6. Compute individual motor forces using mixing matrix
    // Formulas derived earlier:
    // F2 + F3 = (Ftotal - (tauRoll + tauPitch)/armLength) / 2
    // F3 - F2 = (tauYaw/torqueConstant - (pitchError - rollError)/armLength) / 2
    // Note: We're using errors in the yaw formula? Actually we should use the PID output for yaw torque.
    // But note: tauYaw is already the desired yaw torque from PID.
    // So:
    const S = (Ftotal - (tauRoll + tauPitch) / armLength) / 2;
    const D = (tauYaw / torqueConstant - (tauPitch - tauRoll) / armLength) / 2; // Check signs
    // Let me re-derive with signs:
    // tauRoll = L * (-F2 + F4)
    // tauPitch = L * (F1 - F3)
    // tauYaw = b * (F1 - F2 + F3 - F4)
    // We had:
    // F1 = F3 + tauPitch/L
    // F4 = F2 + tauRoll/L
    // Then Ftotal = F1+F2+F3+F4 = 2F3 + tauPitch/L + 2F2 + tauRoll/L
    // => F2 + F3 = (Ftotal - (tauRoll+tauPitch)/L)/2   (same as S)
    // And tauYaw/b = F1 - F2 + F3 - F4 = (F3+tauPitch/L) - F2 + F3 - (F2+tauRoll/L)
    // = 2F3 - 2F2 + tauPitch/L - tauRoll/L
    // => F3 - F2 = (tauYaw/b - (tauPitch - tauRoll)/L)/2
    // So D = (tauYaw/torqueConstant - (tauPitch - tauRoll)/armLength)/2

    motorForces[1] = (S - D) / 2; // F2 (right)
    motorForces[2] = (S + D) / 2; // F3 (back)
    motorForces[0] = motorForces[2] + tauPitch / armLength; // F1 = F3 + tauPitch/L
    motorForces[3] = motorForces[1] - tauRoll / armLength;  // F4 = F2 - tauRoll/L? Wait check:
    // From F4 = F2 + tauRoll/L -> so F4 = F2 + tauRoll/L
    // But above I had F4 = F2 + tauRoll/L, so:
    motorForces[3] = motorForces[1] + tauRoll / armLength; // F4 = F2 + tauRoll/L

    // Double-check F1: F1 = F3 + tauPitch/L
    motorForces[0] = motorForces[2] + tauPitch / armLength;

    // Ensure no negative forces (motors can't pull)
    for (let i = 0; i < 4; i++) {
        motorForces[i] = Math.max(0, motorForces[i]);
    }

    // 7. Compute total force and torque in body frame
    // Thrust is along -z_body (since we defined z_body as down, thrust up is negative z)
    const thrustBody = new THREE.Vector3(0, 0, -Ftotal);
    const torqueBody = new THREE.Vector3(tauRoll, tauPitch, tauYaw);

    // 8. Convert thrust to world frame
    const thrustWorld = thrustBody.clone().applyQuaternion(quaternion);

    // 9. Compute net force (thrust + gravity)
    const gravity = new THREE.Vector3(0, -g, 0); // y is up
    const forceWorld = thrustWorld.clone().add(gravity).multiplyScalar(1 / mass); // F=ma -> a = F/m

    // 10. Update linear motion (Euler integration)
    velocity.add(forceWorld.clone().multiplyScalar(dt));
    position.add(velocity.clone().multiplyScalar(dt));

    // 11. Update angular motion
    // Angular acceleration in body frame: alpha = I^-1 * (torque - omega x (I*omega))
    const Iomega = new THREE.Vector3(
        Ixx * angularVelocity.x,
        Iyy * angularVelocity.y,
        Izz * angularVelocity.z
    );
    const cross = new THREE.Vector3(
        angularVelocity.y * Iomega.z - angularVelocity.z * Iomega.y,
        angularVelocity.z * Iomega.x - angularVelocity.x * Iomega.z,
        angularVelocity.x * Iomega.y - angularVelocity.y * Iomega.x
    );
    const angularAccel = new THREE.Vector3(
        (torqueBody.x - cross.x) / Ixx,
        (torqueBody.y - cross.y) / Iyy,
        (torqueBody.z - cross.z) / Izz
    );

    angularVelocity.add(angularAccel.clone().multiplyScalar(dt));

    // 12. Update orientation (quaternion)
    const omegaQuat = new THREE.Quaternion(0, angularVelocity.x, angularVelocity.y, angularVelocity.z);
    const dq = omegaQuat.multiply(new THREE.Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)).multiplyScalar(0.5);
    quaternion.x += dq.x * dt;
    quaternion.y += dq.y * dt;
    quaternion.z += dq.z * dt;
    quaternion.w += dq.w * dt;
    quaternion.normalize();

    // 13. Update telemetry display
    updateTelemetry(roll, pitch, yaw);
}

// Process keyboard input
function processInput(dt) {
    // Throttle
    if (keys.KeyW) desiredThrottle = Math.min(1.0, desiredThrottle + throttleStep * dt * 5); // W to increase
    if (keys.KeyS) desiredThrottle = Math.max(0.0, desiredThrottle - throttleStep * dt * 5); // S to decrease

    // Roll (A: left wing up -> negative roll, D: right wing up -> positive roll)
    if (keys.ArrowLeft) desiredRoll = Math.max(-30, desiredRoll - rollStep * dt * 5);
    if (keys.ArrowRight) desiredRoll = Math.min(30, desiredRoll + rollStep * dt * 5);

    // Pitch (Up: nose up -> positive pitch, Down: nose down -> negative pitch)
    if (keys.ArrowUp) desiredPitch = Math.min(30, desiredPitch + pitchStep * dt * 5);
    if (keys.ArrowDown) desiredPitch = Math.max(-30, desiredPitch - pitchStep * dt * 5);

    // Yaw (, : yaw left -> positive yaw, . : yaw right -> negative yaw)
    if (keys.Comma) desiredYaw = Math.min(180, desiredYaw + yawStep * dt * 5);
    if (keys.Period) desiredYaw = Math.max(-180, desiredYaw - yawStep * dt * 5);

    // Optional: reset to center when keys released? We'll let it hold last value.
}

// Update telemetry display
function updateTelemetry(roll, pitch, yaw) {
    document.getElementById('altitude').textContent = position.y.toFixed(2);
    document.getElementById('roll').textContent = roll.toFixed(2);
    document.getElementById('pitch').textContent = pitch.toFixed(2);
    document.getElementById('yaw').textContent = yaw.toFixed(2);

    // Motor thrust percentage (0-100%)
    const maxPossibleThrust = max_thrust; // at throttle=1
    document.getElementById('motor1').textContent = Math.round((motorForces[0] / maxPossibleThrust) * 100);
    document.getElementById('motor2').textContent = Math.round((motorForces[1] / maxPossibleThrust) * 100);
    document.getElementById('motor3').textContent = Math.round((motorForces[2] / maxPossibleThrust) * 100);
    document.getElementById('motor4').textContent = Math.round((motorForces[3] / maxPossibleThrust) * 100);
}

// Animation loop
const clock = new THREE.Clock();

function animate() {
    requestAnimationFrame(animate);

    const dt = clock.getDelta();

    // Update physics
    updatePhysics(dt);

    // Update visuals
    updateDroneVisual(dt);

    // Render
    renderer.render(scene, camera);
}

// Initialize when window loads
window.onload = init;