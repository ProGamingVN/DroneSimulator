/* --------------------------------------------------------------
     Drone Simulator – Pro Edition
     Author:  (you can put your name here)
     License: MIT – feel free to fork and improve
     -------------------------------------------------------------- */

  /* ==== CONFIGURATION ==== */
  const CONFIG = {
      // Physics
      mass: 1.2,                 // kg
      g: 9.81,                   // m/s²
      maxThrust: 2.5 * 9.81,     // N (≈2.5× weight → generous thrust)
      armLength: 0.4,            // m (distance from centre to motor)
      inertia: {xx:0.08, yy:0.08, zz:0.12}, // kg·m²
      thrustConstant: 0.0002,   // N/(rad/s)²  (tuned for visual thrust)
      torqueConstant: 0.00002,  // N·m/(rad/s)² (yaw torque)

      // PID gains (tuned for responsive but not aggressive behavior)
      pidRoll:    {Kp:4.5, Ki:0.2, Kd:0.9},
      pidPitch:   {Kp:4.5, Ki:0.2, Kd:0.9},
      pidYaw:     {Kp:2.0, Ki:0.0, Kd:0.4},

      // Input limits (degrees)
      maxAngle: 25,              // roll/pitch/yaw limits from stick
      throttleStep:0.015,        // per second when holding key
      angleStep:1.2,             // degrees per second when holding key

      // Visual
      droneModelPath: "assets/drone.glb", // set to null to force procedural
      groundSize: 200,
      groundTexture: "assets/textures/grass.jpg",
      skyTexture:   "assets/textures/sky.jpg",
      bloomEnabled: true,
      showWireframeFallback: true   // show simple shapes if model fails
  };

  /* ==== GLOBAL THREE.JS OBJECTS ==== */
  let scene, camera, renderer, clock;
  let droneGroup = null;          // will hold the visual model
  let groundMesh;
  let controls;                   // OrbitControls (debug only, can be removed)
  let mixers = [];                // for GLTF animations (unused now)

  /* ==== PHYSICS STATE ==== */
  let position = new THREE.Vector3(0, 0, 0);
  let velocity = new THREE.Vector3(0, 0, 0);
  let quaternion = new THREE.Quaternion(); // body orientation (world)
  let angularVelocity = new THREE.Vector3(); // body frame

  // Desired setpoints from user input
  let desired = {
      throttle: 0,
      roll: 0,
      pitch: 0,
      yaw: 0
  };

  // Motor forces (N) – will be computed each frame
  let motorForces = [0,0,0,0];

  /* ==== INPUT STATE ==== */
  const keys = {
      KeyW:false, KeyS:false,
      ArrowLeft:false, ArrowRight:false,
      ArrowUp:false, ArrowDown:false,
      Comma:false, Period:false
  };

  let touchJoysticks = {left:{x:0,y:0}, right:{x:0,y:0}};

  /* ==== PID CLASS ==== */
  class PID {
      constructor({Kp,Ki,Kd}){
          this.Kp=Kp; this.Ki=Ki; this.Kd=Kd;
          this.integral=0; this.prevError=0;
      }
      compute(error, dt){
          if(dt<=0) return 0;
          this.integral += error*dt;
          const derivative = (error-this.prevError)/dt;
          const out = this.Kp*error + this.Ki*this.integral + this.Kd*derivative;
          this.prevError = error;
          return out;
      }
      reset(){this.integral=0; this.prevError=0;}
  }
  const pidRoll = new PID(CONFIG.pidRoll);
  const pidPitch = new PID(CONFIG.pidPitch);
  const pidYaw = new PID(CONFIG.pidYaw);

  /* ==== INIT ==== */
  function init(){
      // ----- Scene -----
      scene = new THREE.Scene();
      scene.background = new THREE.Color(0x87ceeb); // fallback sky

      // ----- Camera -----
      const aspect = window.innerWidth/window.innerHeight;
      camera = new THREE.PerspectiveCamera(60, aspect, 0.1, 500);
      camera.position.set(0,5,8);
      camera.lookAt(0,0,0);

      // ----- Renderer -----
      const canvas = document.getElementById('droneCanvas');
      renderer = new THREE.WebGLRenderer({canvas, antialias:true, powerPreference:"high-performance"});
      renderer.setSize(window.innerWidth, window.innerHeight);
      renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2)); // cap for mobile
      if(CONFIG.bloomEnabled){
          // Simple bloom via EffectComposer (optional, lightweight)
          const {EffectComposer} = THREE;
          const {RenderPass} = THREE;
          const {UnrealBloomPass} = THREE;
          composer = new EffectComposer(renderer);
          composer.addPass(new RenderPass(scene, camera));
          const bloomPass = new UnrealBloomPass(
              new THREE.Vector2(window.innerWidth, window.innerHeight),
              0.3, 0.4, 0.85
          );
          composer.addPass(bloomPass);
      }

      // ----- Lights -----
      const hemi = new THREE.HemisphereLight(0xffffbb, 0x080820, 1.2);
      scene.add(hemi);
      const dir = new THREE.DirectionalLight(0xffffff, 0.8);
      dir.position.set(5,12,6);
      scene.add(dir);

      // ----- Ground -----
      const groundGeo = new THREE.PlaneGeometry(CONFIG.groundSize, CONFIG.groundSize);
      let groundMat;
      if(CONFIG.groundTexture){
          const tex = new THREE.TextureLoader().load(CONFIG.groundTexture);
          tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
          tex.repeat.set( CONFIG.groundSize/8, CONFIG.groundSize/8 );
          groundMat = new THREE.MeshStandardMaterial({map:tex});
      }else{
          groundMat = new THREE.MeshStandardMaterial({color:0x555555});
      }
      groundMesh = new THREE.Mesh(groundGeo, groundMat);
      groundMesh.rotation.x = -Math.PI/2;
      scene.add(groundMesh);

      // ----- Drone (model or procedural) -----
      loadDroneModel();

      // ----- OrbitControls (debug) -----
      // Uncomment the lines below if you want mouse orbit while developing.
      // controls = new THREE.OrbitControls(camera, renderer.domElement);
      // controls.enableDamping = true;
      // controls.minDistance = 2;
      // controls.maxDistance = 20;

      // ----- Input listeners -----
      window.addEventListener('keydown', e=>{ if(keys.hasOwnProperty(e.code)) keys[e.code]=true; });
      window.addEventListener('keyup',   e=>{ if(keys.hasOwnProperty(e.code)) keys[e.code]=false; });
      window.addEventListener('resize', onWindowResize);
      initTouchJoysticks();

      // ----- Start loop -----
      clock = new THREE.Clock();
      animate();
  }

  /* ==== DRONE LOADING ==== */
  function loadDroneModel(){
      if(!CONFIG.droneModelPath){
          createProceduralDrone();
          return;
      }
      const loader = new THREE.GLTFLoader();
      loader.load(
          CONFIG.droneModelPath,
          gltf=>{
              droneGroup = gltf.scene;
              droneGroup.traverse(o=>{
                  if(o.isMesh){
                      o.castShadow = true;
                      o.receiveShadow = true;
                  }
              });
              // Optional: center model at origin (many GLTFs are already centered)
              const box = new THREE.Box3().setFromObject(droneGroup);
              const center = box.getCenter(new THREE.Vector3());
              droneGroup.position.sub(center);
              scene.add(droneGroup);
          },
          undefined,
          err=>{
              console.warn("GLTF load failed, falling back to procedural drone.", err);
              createProceduralDrone();
          }
      );
  }

  /* ==== PROCEDURAL LOW‑POLY DRONE ==== */
  function createProceduralDrone(){
      droneGroup = new THREE.Group();
      // Central body
      const bodyGeo = new THREE.BoxGeometry(0.3,0.3,0.2);
      const bodyMat = new THREE.MeshStandardMaterial({color:0x606060});
      const body = new THREE.Mesh(bodyGeo,bodyMat);
      body.position.y = 0;
      droneGroup.add(body);

      // Arms + motors (simple cylinders)
      const armColor = 0x009688;
      const motorColor = 0x424242;
      const propColor  = 0x212121;
      const armLength = CONFIG.armLength * 0.9; // a tad shorter for visuals
      const armThick = 0.04;
      const armWidth = 0.12;
      const motorRadius = 0.04;
      const motorHeight = 0.02;
      const propRadius = 0.14;
      const propHeight = 0.01;

      const armData = [
          {pos:[ 0, 0, armLength], rot:[0,0,0]},   // front (+z)
          {pos:[armLength,0, 0   ], rot:[0, Math.PI/2,0]}, // right (+x)
          {pos:[ 0, 0,-armLength], rot:[0,0,0]},   // back (-z)
          {pos:[-armLength,0, 0  ], rot:[0, Math.PI/2,0]} // left (-x)
      ];

      armData.forEach((d,i)=>{
          // arm
          const armGeo = new THREE.BoxGeometry(armThick, armWidth, armLength);
          const armMat = new THREE.MeshStandardMaterial({color:armColor});
          const arm = new THREE.Mesh(armGeo, armMat);
          arm.position.set(...d.pos);
          arm.rotation.set(...d.map((v,idx)=> idx<3?v*Math.PI/180:v));
          droneGroup.add(arm);

          // motor
          const motorGeo = new THREE.CylinderGeometry(motorRadius,motorRadius,motorHeight,12);
          const motorMat = new THREE.MeshStandardMaterial({color:motorColor});
          const motor = new THREE.Mesh(motorGeo, motorMat);
          motor.position.set(d.pos[0], d.pos[1], d.pos[2] + (d[2]===0?0: armLength/2));
          droneGroup.add(motor);

          // propeller (spinning disc)
          const propGeo = new THREE.CylinderGeometry(propRadius,propRadius,propHeight,24);
          const propMat = new THREE.MeshStandardMaterial({color:propColor});
          const prop = new THREE.Mesh(propGeo, propMat);
          // align disc perpendicular to arm
          prop.rotation.set(
              d[2]===0 ? Math.PI/2 : 0,
              d[0]===0 ? 0 : Math.PI/2,
              0
          );
          prop.position.set(d.pos[0], d.pos[1], d.pos[2] + (d[2]===0? armLength/2:0)+propHeight/2);
          droneGroup.add(prop);
          // store reference for spin animation
          if(!droneGroup.propellers) droneGroup.propellers = [];
          droneGroup.propellers.push(prop);
      });

      scene.add(droneGroup);
  }

  /* ==== WINDOW RESIZE ==== */
  function onWindowResize(){
      const w = window.innerWidth, h = window.innerHeight;
      camera.aspect = w/h;
      camera.updateProjectionMatrix();
      renderer.setSize(w,h);
      if(composer) composer.setSize(w,h);
  }

  /* ==== TOUCH JOYSTICKS ==== */
  function initTouchJoysticks(){
      const wrapper = document.getElementById('joystick-wrapper');
      const hint = document.getElementById('touch-hint');
      if('ontouchstart' in window || navigator.maxTouchPoints>0){
          wrapper.style.display = 'flex';
          hint.style.display = 'block';
      }else{
          wrapper.style.display = 'none';
          hint.style.display = 'none';
      }

      const stickIDs = ['joystick-left','joystick-right'];
      stickIDs.forEach((id, idx)=>{
          const stick = document.getElementById(id);
          let active = false;
          const center = {x: stick.offsetWidth/2, y: stick.offsetHeight/2};

          const getTouch = e=>{
              if(!active) return;
              const touch = e.touches[0] || e.changedTouches[0];
              const rect = stick.getBoundingClientRect();
              let dx = touch.clientX - rect.left - center.x;
              let dy = touch.clientY - rect.top  - center.y;
              const dist = Math.hypot(dx,dy);
              const max = stick.offsetWidth/2 * 0.8; // 80% radius
              if(dist>max){ const scale = max/dist; dx*=scale; dy*=scale; }
              // normalize -1..1
              touchJoysticks[idx].x = dx/(stick.offsetWidth/2*0.8);
              touchJoysticks[idx].y = dy/(stick.offsetHeight/2*0.8);
              // move visual nub
              stick.style.setProperty('--nx', touchJoysticks[idx].x);
              stick.style.setProperty('--ny', touchJoysticks[idx].y);
          };
          const start = e=>{ e.preventDefault(); active = true; getTouch(e); };
          const move  = e=>{ e.preventDefault(); if(active) getTouch(e); };
          const end   = e=>{ active = false; touchJoysticks[idx] = {x:0,y:0}; stick.style.removeProperty('--nx');
  stick.style.removeProperty('--ny'); };

          stick.addEventListener('touchstart', start);
          stick.addEventListener('touchmove',  move);
          stick.addEventListener('touchend',   end);
          stick.addEventListener('touchcancel',end);
      });
  }

  /* ==== INPUT PROCESSING ==== */
  function processInput(dt){
      // ----- Throttle (W/S) -----
      if(keys.KeyW) desired.throttle = Math.min(1, desired.throttle + CONFIG.throttleStep*dt);
      if(keys.KeyS) desired.throttle = Math.max(0, desired.throttle - CONFIG.throttleStep*dt);

      // ----- Roll (A/D) -----
      if(keys.ArrowLeft)  desired.roll  = Math.max(-CONFIG.maxAngle, desired.roll  - CONFIG.angleStep*dt);
      if(keys.ArrowRight) desired.roll  = Math.min( CONFIG.maxAngle, desired.roll  + CONFIG.angleStep*dt);

      // ----- Pitch (↑/↓) -----
      if(keys.ArrowUp)    desired.pitch = Math.min( CONFIG.maxAngle, desired.pitch + CONFIG.angleStep*dt);
      if(keys.ArrowDown)  desired.pitch = Math.max(-CONFIG.maxAngle, desired.pitch - CONFIG.angleStep*dt);

      // ----- Yaw (,/.) -----
      if(keys.Comma)      desired.yaw   = Math.min(180, desired.yaw   + CONFIG.angleStep*dt);
      if(keys.Period)     desired.yaw   = Math.max(-180, desired.yaw   - CONFIG.angleStep*dt);

      // ----- Touch joysticks -----
      // Left stick -> throttle (Y) + roll (X)
      desired.throttle = THREE.MathUtils.clamp(
          desired.throttle - touchJoysticks.left.y * 0.5 * dt, 0, 1);
      desired.roll = THREE.MathUtils.clamp(
          touchJoysticks.left.x * CONFIG.maxAngle, -CONFIG.maxAngle, CONFIG.maxAngle);

      // Right stick -> pitch (Y) + yaw (X)
      desired.pitch = THREE.MathUtils.clamp(
          -touchJoysticks.right.y * CONFIG.maxAngle, -CONFIG.maxAngle, CONFIG.maxAngle);
      desired.yaw = THREE.MathUtils.clamp(
          touchJoysticks.right.x * 180, -180, 180);
  }

  /* ==== PHYSICS UPDATE ==== */
  function updatePhysics(dt){
      // 1️⃣Convert quaternion → Euler (in degrees) for PID error
      const euler = new THREE.Euler();
      euler.setFromQuaternion(quaternion, 'XYZ'); // returns rad
      const roll    = THREE.MathUtils.radToDeg(euler.x);
      const pitch   = THREE.MathUtils.radToDeg(euler.y);
      const yaw     = THREE.MathUtils.radToDeg(euler.z);

      // 2️⃣Errors
      const errRoll  = desired.roll  - roll;
      const errPitch = desired.pitch - pitch;
      const errYaw   = desired.yaw   - yaw;

      // 3️⃣PID torques (body frame, N·m)
      const tauRoll  = pidRoll.compute(errRoll,  dt);
      const tauPitch = pidPitch.compute(errPitch,dt);
      const tauYaw   = pidYaw.compute(errYaw,  dt);

      // 4️⃣Collective thrust
      const Ftotal = desired.throttle * CONFIG.maxThrust; // N

      // 5️⃣Mixing matrix (quad‑X layout)
      // Derived from:
      //   tauRoll  = L * (-F2 + F4)
      //   tauPitch = L * ( F1 - F3)
      //   tauYaw   = b * ( F1 - F2 + F3 - F4)
      //   Ftotal   = F1+F2+F3+F4
      const L = CONFIG.armLength;
      const b = CONFIG.torqueConstant;

      // Solve for F2 & F3 first (see derivation in comments)
      const S = (Ftotal - (tauRoll + tauPitch)/L) / 2; // F2+F3
      const D = (tauYaw/b - (tauPitch - tauRoll)/L) / 2; // F3 - F2
      const F2 = (S - D)/2;
      const F3 = (S + D)/2;
      const F1 = F3 + tauPitch/L;
      const F4 = F2 + tauRoll/L;

      motorForces = [F1, F2, F3, F4].map(f=>Math.max(0,f)); // no negative thrust

      // 6️⃣Forces & torques in world frame
      const thrustBody = new THREE.Vector3(0,0,-Ftotal); // -Z_body is up
      const thrustWorld = thrustBody.clone().applyQuaternion(quaternion);
      const gravity = new THREE.Vector3(0, -CONFIG.g, 0);
      const forceWorld = thrustWorld.clone().add(gravity).divideScalar(CONFIG.mass); // a = F/m

      // Torque in body frame
      const torqueBody = new THREE.Vector3(tauRoll, tauPitch, tauYaw);
      // Convert to world (needed for angular acceleration using world inertia? We'll keep body frame)
      // We'll compute angular accel in body frame using body inertia (already diagonal)
      const I = new THREE.Vector3(CONFIG.inertia.xx, CONFIG.inertia.yy, CONFIG.inertia.zz);
      const Iomega = new THREE.Vector3(
          I.x * angularVelocity.x,
          I.y * angularVelocity.y,
          I.z * angularVelocity.z
      );
      const cross = new THREE.Vector3(
          angularVelocity.y * Iomega.z - angularVelocity.z * Iomega.y,
          angularVelocity.z * Iomega.x - angularVelocity.x * Iomega.z,
          angularVelocity.x * Iomega.y - angularVelocity.y * Iomega.x
      );
      const angularAccel = new THREE.Vector3(
          (torqueBody.x - cross.x) / I.x,
          (torqueBody.y - cross.y) / I.y,
          (torqueBody.z - cross.z) / I.z
      );

      // 7️⃣Integrate linear motion (semi‑implicit Euler)
      velocity.add(forceWorld.clone().multiplyScalar(dt));
      position.add(velocity.clone().multiplyScalar(dt));

      // 8️⃣Integrate angular motion
      angularVelocity.add(angularAccel.clone().multiplyScalar(dt));
      // Update orientation via quaternion
      const omegaQuat = new THREE.Quaternion(
          angularVelocity.x * dt * 0.5,
          angularVelocity.y * dt * 0.5,
          angularVelocity.z * dt * 0.5,
          1
      );
      quaternion.multiply(omegaQuat).normalize();

      // 9️⃣Update visual meshes
      if(droneGroup){
          droneGroup.position.copy(position);
          droneGroup.setQuaternion(quaternion);
          // spin propellers proportional to thrust
          if(droneGroup.propellers){
              droneGroup.propellers.forEach((prop, i)=>{
                  const omega = Math.sqrt(Math.max(0, motorForces[i]) / CONFIG.thrustConstant) * 6; // rad/s visual
                  prop.rotation.z += omega * dt;
              });
          }
      }

      // 10️⃣Update telemetry UI
      document.getElementById('altitude').textContent = position.y.toFixed(2);
      document.getElementById('roll').textContent    = roll.toFixed(2);
      document.getElementById('pitch').textContent   = pitch.toFixed(2);
      document.getElementById('yaw').textContent     = yaw.toFixed(2);
      const maxPercent = CONFIG.maxThrust;
      ['motor1','motor2','motor3','motor4'].forEach((id,idx)=>{
          const pct = Math.round((motorForces[idx]/maxPercent)*100);
          document.getElementById(id).textContent = pct;
      });
  }

  /* ==== MAIN LOOP ==== */
  function animate(){
      requestAnimationFrame(animate);
      const dt = clock.getDelta();
      // Clamp dt to avoid huge spikes when tab is hidden
      const clampedDt = Math.min(dt, 0.1);

      processInput(clampedDt);
      updatePhysics(clampedDt);

      // Render
      if(composer){
          composer.render();
      }else{
          renderer.render(scene, camera);
      }
      // Update OrbitControls if enabled
      // if(controls) controls.update();
  }

  /* ==== START ==== */
  window.addEventListener('load', init);
