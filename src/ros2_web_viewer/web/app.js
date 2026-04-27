/**
 * ROS2 Web Viewer — Three.js frontend
 *
 * Features:
 *  • Live URDF parsing with joint-state animation (box/cylinder/sphere geometry + STL/DAE meshes)
 *  • Point cloud with custom GLSL shader (per-point colour, additive glow, distance attenuation)
 *  • Camera image panel
 *  • UnrealBloom post-processing pass for scanner glow
 *  • WebSocket auto-reconnect bridge to FastAPI backend
 *  • TF tree cached for frame transforms
 */

import * as THREE from 'three';
import { OrbitControls }   from 'three/addons/controls/OrbitControls.js';
import { STLLoader }       from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader }   from 'three/addons/loaders/ColladaLoader.js';
import { EffectComposer }  from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass }      from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { OutputPass }      from 'three/addons/postprocessing/OutputPass.js';

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

const MAX_CLOUD_PTS = 60_000;
const URDF_RETRY_MS = 2_000;
const WS_RETRY_MS   = 3_000;
const WS_URL        = `ws://${location.host}/ws`;
const DEFAULT_FPS_THROTTLE_HZ = 5;
const TRIGGER_TIMEOUT_DEFAULT = 2.0;
const TRIGGER_TIMEOUT_MIN = 0.1;
const TRIGGER_TIMEOUT_MAX = 30.0;

const VIEW_LAYERS = {
  BASE: 0,
  URDF: 1,
};

const TOPIC_LAYER_START = 2;
const TOPIC_LAYER_END = 31;
const TOPIC_LAYER_FALLBACK = {
  pointcloud: 2,
  marker_array: 3,
};

const topicLayers = {
  pointcloud: new Map(),
  marker_array: new Map(),
};
let nextTopicLayer = TOPIC_LAYER_START;

const DEFAULT_VIEWER_TOPICS = {
  image: ['/camera/image_raw'],
  pointcloud: ['/points'],
  marker_array: ['/markers'],
};

// ─────────────────────────────────────────────────────────────────────────────
// Widget discovery
// ─────────────────────────────────────────────────────────────────────────────

const threeCanvases = Array.from(document.querySelectorAll('[data-ros-widget="3d"]'));
if (threeCanvases.length === 0) {
  throw new Error('No 3D canvas found. Add a canvas with data-ros-widget="3d".');
}

function getCanvasSize(canvasEl) {
  return {
    width: Math.max(canvasEl.clientWidth, 1),
    height: Math.max(canvasEl.clientHeight, 1),
  };
}

function parseNumberAttr(el, attrName, defaultValue) {
  const raw = el.getAttribute(attrName);
  if (raw == null || raw === '') return defaultValue;
  const parsed = Number.parseFloat(raw);
  return Number.isFinite(parsed) ? parsed : defaultValue;
}

function parseBoolAttr(el, attrName, defaultValue) {
  const raw = el.getAttribute(attrName);
  if (raw == null || raw === '') return defaultValue;
  const normalized = String(raw).trim().toLowerCase();
  if (['1', 'true', 'yes', 'on'].includes(normalized)) return true;
  if (['0', 'false', 'no', 'off'].includes(normalized)) return false;
  return defaultValue;
}

function parseVector3Attr(el, attrName, fallback) {
  const raw = el.getAttribute(attrName);
  const rawTrimmed = String(raw || '').trim();
  if (!rawTrimmed) return [...fallback];
  const parsed = rawTrimmed
    .split(/[,\s]+/)
    .map(v => Number.parseFloat(v))
    .filter(v => Number.isFinite(v));
  if (parsed.length !== 3) return [...fallback];
  return parsed;
}

function parseTopicListFromAttrs(el, attrNames, fallback = []) {
  for (const attrName of attrNames) {
    const raw = String(el.getAttribute(attrName) || '').trim();
    if (!raw) continue;
    const topics = raw
      .split(/[\s,]+/)
      .map((value) => value.trim())
      .filter((value) => value.startsWith('/'));
    if (topics.length > 0) {
      return [...new Set(topics)];
    }
  }
  return [...fallback];
}

function parseDisplaySetAttr(el) {
  const raw = String(el.getAttribute('data-display') || '').trim();
  if (!raw) {
    return new Set(['urdf', 'pointcloud', 'markers']);
  }
  const values = raw
    .split(/[\s,]+/)
    .map((value) => value.trim().toLowerCase())
    .filter(Boolean);

  if (values.includes('all')) {
    return new Set(['urdf', 'pointcloud', 'markers']);
  }
  if (values.includes('none')) {
    return new Set();
  }

  const aliasToKey = {
    robot: 'urdf',
    urdf: 'urdf',
    cloud: 'pointcloud',
    pc: 'pointcloud',
    pointcloud: 'pointcloud',
    point_cloud: 'pointcloud',
    marker: 'markers',
    markers: 'markers',
  };

  const selected = new Set();
  for (const token of values) {
    const key = aliasToKey[token];
    if (key) selected.add(key);
  }
  return selected;
}

function parse3DDisplayConfig(el) {
  const selected = parseDisplaySetAttr(el);
  return {
    showUrdf: parseBoolAttr(el, 'data-show-urdf', selected.has('urdf')),
    showPointCloud: parseBoolAttr(el, 'data-show-pointcloud', selected.has('pointcloud')),
    showMarkers: parseBoolAttr(el, 'data-show-markers', selected.has('markers')),
  };
}

function parse3DTopicConfig(el, displayConfig) {
  return {
    pointCloudTopics: displayConfig.showPointCloud
      ? parseTopicListFromAttrs(
        el,
        ['data-pointcloud-topics', 'data-topic-pointcloud', 'data-topic-cloud'],
        DEFAULT_VIEWER_TOPICS.pointcloud,
      )
      : [],
    markerArrayTopics: displayConfig.showMarkers
      ? parseTopicListFromAttrs(
        el,
        ['data-marker-array-topics', 'data-marker-topics', 'data-topic-markers', 'data-topic-marker'],
        DEFAULT_VIEWER_TOPICS.marker_array,
      )
      : [],
  };
}

function parseUrdfLinkFiltersAttr(el) {
  const whitelist = String(el.getAttribute('data-urdf-link-whitelist') || '').trim();
  const blacklist = String(el.getAttribute('data-urdf-link-blacklist') || '').trim();
  const filters = {
    whitelist: whitelist ? new Set(whitelist.split(/[\s,]+/).filter(Boolean)) : null,
    blacklist: blacklist ? new Set(blacklist.split(/[\s,]+/).filter(Boolean)) : null,
  };

  console.log('[URDF] Link filters:', {
    whitelist: filters.whitelist ? [...filters.whitelist] : null,
    blacklist: filters.blacklist ? [...filters.blacklist] : null,
  });

  return filters;
}

function shouldShowUrdfLink(linkName, filters) {
  if (!filters || (!filters.whitelist && !filters.blacklist)) return true;
  if (filters.whitelist) return filters.whitelist.has(linkName);
  if (filters.blacklist) return !filters.blacklist.has(linkName);
  return true;
}

function getTopicLayer(kind, topicName) {
  const topic = String(topicName || '').trim();
  if (!topic) return TOPIC_LAYER_FALLBACK[kind];
  const existing = topicLayers[kind].get(topic);
  if (existing != null) return existing;

  if (nextTopicLayer > TOPIC_LAYER_END) {
    const fallback = TOPIC_LAYER_FALLBACK[kind];
    console.warn(
      `[Viewer] Layer limit reached; falling back to shared ${kind} layer ${fallback} for topic "${topic}"`,
    );
    topicLayers[kind].set(topic, fallback);
    return fallback;
  }

  const layer = nextTopicLayer;
  nextTopicLayer += 1;
  topicLayers[kind].set(topic, layer);
  return layer;
}

function setObjectLayerRecursive(object3d, layer) {
  object3d.traverse((node) => {
    node.layers.set(layer);
  });
}

function enableLightOnAllLayers(light) {
  light.layers.enableAll();
}

// ─────────────────────────────────────────────────────────────────────────────
// Scene
// ─────────────────────────────────────────────────────────────────────────────

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1e1408);
//scene.fog = new THREE.FogExp2(0x1e1408, 0.05);

// ROS (X forward, Y left, Z up) → Three (X right, Y up, Z out) basis change
const ROS_TO_THREE_QUAT = new THREE.Quaternion()
  .setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0, 'XYZ'));
const rosSceneRoot = new THREE.Group();
rosSceneRoot.name = 'ros_scene_root';
rosSceneRoot.quaternion.copy(ROS_TO_THREE_QUAT);
scene.add(rosSceneRoot);

// ─────────────────────────────────────────────────────────────────────────────
// Camera, renderer and controls per canvas widget
// ─────────────────────────────────────────────────────────────────────────────

const viewerWidgets = threeCanvases.map((canvasEl) => {
  const renderer = new THREE.WebGLRenderer({
    canvas: canvasEl,
    antialias: true,
    powerPreference: 'default',
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.1;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.shadowMap.enabled = false;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const cameraFov = parseNumberAttr(canvasEl, 'data-camera-fov', 55);
  const cameraNear = parseNumberAttr(canvasEl, 'data-camera-near', 0.001);
  const cameraFar = parseNumberAttr(canvasEl, 'data-camera-far', 60);
  const [cameraX, cameraY, cameraZ] = parseVector3Attr(canvasEl, 'data-camera-position', [2.0, 1.6, 2.0]);
  const [lookAtX, lookAtY, lookAtZ] = parseVector3Attr(canvasEl, 'data-camera-look-at', [0, 0.5, 0]);

  const camera = new THREE.PerspectiveCamera(cameraFov, 1.0, cameraNear, cameraFar);
  camera.position.set(cameraX, cameraY, cameraZ);
  camera.lookAt(lookAtX, lookAtY, lookAtZ);

  const displayConfig = parse3DDisplayConfig(canvasEl);
  const topicConfig = parse3DTopicConfig(canvasEl, displayConfig);
  camera.layers.disableAll();
  camera.layers.enable(VIEW_LAYERS.BASE);
  if (displayConfig.showUrdf) camera.layers.enable(VIEW_LAYERS.URDF);
  if (displayConfig.showPointCloud) {
    for (const topic of topicConfig.pointCloudTopics) {
      camera.layers.enable(getTopicLayer('pointcloud', topic));
    }
  }
  if (displayConfig.showMarkers) {
    for (const topic of topicConfig.markerArrayTopics) {
      camera.layers.enable(getTopicLayer('marker_array', topic));
    }
  }

  const controls = new OrbitControls(camera, renderer.domElement);
  const [targetX, targetY, targetZ] = parseVector3Attr(canvasEl, 'data-controls-target', [0, 0.4, 0]);
  controls.target.set(targetX, targetY, targetZ);
  controls.enableDamping = parseBoolAttr(canvasEl, 'data-controls-enable-damping', true);
  controls.dampingFactor = parseNumberAttr(canvasEl, 'data-controls-damping-factor', 0.06);
  controls.minDistance = parseNumberAttr(canvasEl, 'data-controls-min-distance', 0.1);
  controls.maxDistance = parseNumberAttr(canvasEl, 'data-controls-max-distance', 20);
  controls.autoRotate = parseBoolAttr(canvasEl, 'data-controls-auto-rotate', true);
  controls.autoRotateSpeed = parseNumberAttr(canvasEl, 'data-controls-auto-rotate-speed', 0.75);
  controls.update();

  const composer = new EffectComposer(renderer);
  composer.addPass(new RenderPass(scene, camera));

  const initialCanvasSize = getCanvasSize(canvasEl);
  const bloomPass = new UnrealBloomPass(
    new THREE.Vector2(initialCanvasSize.width, initialCanvasSize.height),
    /*strength*/ 0.25, /*radius*/ 0.5, /*threshold*/ 0.88);
  composer.addPass(bloomPass);
  composer.addPass(new OutputPass());

  const urdfLinkFilters = parseUrdfLinkFiltersAttr(canvasEl);
  const fpsThrottleRaw = parseNumberAttr(canvasEl, 'data-fps-throttle', DEFAULT_FPS_THROTTLE_HZ);
  const fpsThrottleHz = Math.max(1, fpsThrottleRaw);
  const frameIntervalMs = 1000 / fpsThrottleHz;

  return {
    canvasEl,
    renderer,
    camera,
    controls,
    composer,
    bloomPass,
    displayConfig,
    topicConfig,
    urdfLinkFilters,
    fpsThrottleHz,
    frameIntervalMs,
  };
});

const activePointCloudTopics = new Set(
  viewerWidgets.flatMap((widget) => widget.topicConfig.pointCloudTopics),
);
const activeMarkerArrayTopics = new Set(
  viewerWidgets.flatMap((widget) => widget.topicConfig.markerArrayTopics),
);

// ─────────────────────────────────────────────────────────────────────────────
// Lighting
// ─────────────────────────────────────────────────────────────────────────────

scene.add(new THREE.AmbientLight(0xd4c090, 0.5));

const keyLight = new THREE.DirectionalLight(0xfff5e0, 2.0);
keyLight.position.set(3, 5, 2);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(1024, 1024);
keyLight.shadow.camera.near = 0.1;
keyLight.shadow.camera.far = 20;
scene.add(keyLight);
enableLightOnAllLayers(keyLight);

const fillLight = new THREE.PointLight(0x8ab870, 1.8, 10);
fillLight.position.set(-2.5, 1.5, -1);
scene.add(fillLight);
enableLightOnAllLayers(fillLight);

const rimLight = new THREE.PointLight(0xe0c060, 1.0, 8);
rimLight.position.set(1, 3, -3);
scene.add(rimLight);
enableLightOnAllLayers(rimLight);

const hemiLight = new THREE.HemisphereLight(0xd4c8a0, 0x2a1e10, 0.5);
scene.add(hemiLight);
enableLightOnAllLayers(hemiLight);

// ─────────────────────────────────────────────────────────────────────────────
// Scene decorations
// ─────────────────────────────────────────────────────────────────────────────

// Grid
const gridHelper = new THREE.GridHelper(10, 30, 0x4a4133, 0x30281e);
gridHelper.material.transparent = true;
gridHelper.material.opacity = 0.5;
scene.add(gridHelper);

// Ground plane (shadow receiver)
const groundMesh = new THREE.Mesh(
  new THREE.PlaneGeometry(10, 10),
  new THREE.ShadowMaterial({ opacity: 0.25 }),
);
groundMesh.rotation.x = -Math.PI / 2;
groundMesh.receiveShadow = true;
scene.add(groundMesh);

// Origin axes marker
{
  const axesMat = (hex) => new THREE.MeshBasicMaterial({ color: hex });
  const axGeo = new THREE.CylinderGeometry(0.005, 0.005, 0.15, 6);
  const mkAxis = (color, rot) => {
    const m = new THREE.Mesh(axGeo, axesMat(color));
    m.rotation.copy(rot);
    m.position.y = 0.075;
    return m;
  };
  const axGroup = new THREE.Group();
  axGroup.add(mkAxis(0xff2244, new THREE.Euler(0, 0, -Math.PI / 2)));  // X red
  axGroup.add(mkAxis(0x22ff44, new THREE.Euler(0, 0, 0)));              // Y green
  axGroup.add(mkAxis(0x2244ff, new THREE.Euler(Math.PI / 2, 0, 0)));   // Z blue
  rosSceneRoot.add(axGroup);
}

// ─────────────────────────────────────────────────────────────────────────────
// Point Cloud — custom GLSL shader
// ─────────────────────────────────────────────────────────────────────────────

const pointCloudByTopic = new Map();

function createPointCloudMaterial() {
  return new THREE.ShaderMaterial({
    vertexShader: /* glsl */`
      attribute vec3 aColor;
      varying   vec3 vColor;
      uniform   float uSize;

      void main() {
        vColor = aColor;
        vec4 mvPos = modelViewMatrix * vec4(position, 1.0);
        gl_PointSize = uSize * (40.0 / -mvPos.z);
        gl_Position  = projectionMatrix * mvPos;
      }
    `,
    fragmentShader: /* glsl */`
      varying vec3 vColor;

      void main() {
        vec2  uv = 2.0 * gl_PointCoord - 1.0;
        float r  = dot(uv, uv);
        if (r > 1.0) discard;

        // Tight glowing disc
        float core  = smoothstep(1.0, 0.0, r);
        float glow  = pow(core, 6.0);
        float alpha = glow * 0.95;

        gl_FragColor = vec4(vColor * (0.7 + 0.3 * glow), alpha);
      }
    `,
    uniforms: { uSize: { value: 2.0 } },
    vertexColors: false,
    transparent: true,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
  });
}

function getOrCreatePointCloudEntry(topicName) {
  const topic = String(topicName || '').trim() || '<unknown>';
  const existing = pointCloudByTopic.get(topic);
  if (existing) return existing;

  const positions = new Float32Array(MAX_CLOUD_PTS * 3);
  const colors = new Float32Array(MAX_CLOUD_PTS * 3);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute('aColor', new THREE.BufferAttribute(colors, 3));
  geometry.setDrawRange(0, 0);

  const points = new THREE.Points(geometry, createPointCloudMaterial());
  points.frustumCulled = false;
  setObjectLayerRecursive(points, getTopicLayer('pointcloud', topic));
  rosSceneRoot.add(points);

  const entry = { topic, points, geometry, positions, colors, frameId: '' };
  pointCloudByTopic.set(topic, entry);
  return entry;
}

// ─────────────────────────────────────────────────────────────────────────────
// Viridis colourmap (JS-side, for any unmapped points)
// ─────────────────────────────────────────────────────────────────────────────

function viridis(t) {
  t = Math.max(0, Math.min(1, t));
  // Polynomial approximation
  const r = 0.267 + t * (1.055  + t * (-2.80  + t * 3.49));
  const g = 0.005 + t * (1.594  + t * (-0.531 + t * -0.068));
  const b = 0.329 + t * (1.015  + t * (-2.758 + t * 1.414));
  return [Math.max(0, Math.min(1, r)), Math.max(0, Math.min(1, g)), Math.max(0, Math.min(1, b))];
}

// Decode base64 → Float32Array, fill buffers
function updatePointCloud(topic, b64, count, frameId) {
  const entry = getOrCreatePointCloudEntry(topic);
  const binary = atob(b64);
  const buf    = new ArrayBuffer(binary.length);
  const bytes  = new Uint8Array(buf);
  for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);

  const f = new Float32Array(buf);
  const n = Math.min(count, MAX_CLOUD_PTS);

  let zMin = Infinity, zMax = -Infinity;
  for (let i = 0; i < n; i++) {
    const z = f[i * 6 + 2];
    if (isFinite(z)) { zMin = Math.min(zMin, z); zMax = Math.max(zMax, z); }
  }
  const zRange = (zMax - zMin) > 1e-6 ? (zMax - zMin) : 1;

  for (let i = 0; i < n; i++) {
    const fi = i * 6;
    entry.positions[i * 3]     = f[fi];
    entry.positions[i * 3 + 1] = f[fi + 1];
    entry.positions[i * 3 + 2] = f[fi + 2];

    let r = f[fi + 3], g = f[fi + 4], b = f[fi + 5];
    if (!isFinite(r) || r < 0) {
      // Fallback height colourmap
      [r, g, b] = viridis((f[fi + 2] - zMin) / zRange);
    }
    entry.colors[i * 3]     = r;
    entry.colors[i * 3 + 1] = g;
    entry.colors[i * 3 + 2] = b;
  }

  entry.geometry.setDrawRange(0, n);
  entry.geometry.attributes.position.needsUpdate = true;
  entry.geometry.attributes.aColor.needsUpdate   = true;

  entry.frameId = normalizeFrameId(frameId);
  updatePointCloudPoseFromTF(entry);
}

function updatePointCloudPoseFromTF(entry) {
  const frameMat = getFrameMatrixInFixedFrame(entry.frameId, `pointcloud:${entry.topic}`);
  if (frameMat) {
    clearTfWarning(`pointcloud-fallback:${entry.topic}:${entry.frameId || 'empty'}`);
    frameMat.decompose(entry.points.position, entry.points.quaternion, entry.points.scale);
  } else {
    warnTfOnce(
      `pointcloud-fallback:${entry.topic}:${entry.frameId || 'empty'}`,
      `[TF] Point cloud topic "${entry.topic}" pose fallback to identity because transform lookup failed for frame "${entry.frameId || '<empty>'}"`,
    );
    entry.points.position.set(0, 0, 0);
    entry.points.quaternion.identity();
    entry.points.scale.set(1, 1, 1);
  }
}

function updateAllPointCloudPosesFromTF() {
  for (const entry of pointCloudByTopic.values()) {
    updatePointCloudPoseFromTF(entry);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// URDF robot — scene graph
// ─────────────────────────────────────────────────────────────────────────────

const robotRoot = new THREE.Group();
robotRoot.name = 'robot_root';
rosSceneRoot.add(robotRoot);
setObjectLayerRecursive(robotRoot, VIEW_LAYERS.URDF);

// Maps populated when URDF is parsed
const jointPivots       = {};   // joint name → Group (the pivot at joint origin)
const jointAxes_        = {};   // joint name → Vector3 (normalised)
const jointTypes_       = {};   // joint name → string
const jointOriginQuats  = {};   // joint name → Quaternion (initial, from origin/rpy)
const jointOriginPos    = {};   // joint name → Vector3   (initial, from origin/xyz)

let   robotLoaded       = false;
let   pendingJointState = null;
let   robotBaseFrame    = 'base_link';

// ── Material factory ──────────────────────────────────────────────────────

function robotMaterial(hexColor, emissiveMult = 0.12) {
  const col = new THREE.Color(hexColor);
  return new THREE.MeshStandardMaterial({
    color: col,
    metalness: 0.72,
    roughness: 0.28,
    emissive: col.clone().multiplyScalar(emissiveMult),
    emissiveIntensity: 1.0,
    envMapIntensity: 0.6,
  });
}

const DEFAULT_LINK_COLOR  = 0x2a6ccc;
const JOINT_SPHERE_COLOR  = 0x00d2aa;

// ── URDF XML parsing ──────────────────────────────────────────────────────

function parseOriginEl(el) {
  if (!el) return { xyz: [0, 0, 0], rpy: [0, 0, 0] };
  const xyz = (el.getAttribute('xyz') || '0 0 0').trim().split(/\s+/).map(Number);
  const rpy = (el.getAttribute('rpy') || '0 0 0').trim().split(/\s+/).map(Number);
  return { xyz, rpy };
}

function applyOrigin(obj, origin) {
  obj.position.set(origin.xyz[0], origin.xyz[1], origin.xyz[2]);
  // URDF rpy is fixed-axis roll(X), pitch(Y), yaw(Z), equivalent to intrinsic ZYX.
  obj.quaternion.setFromEuler(
    new THREE.Euler(origin.rpy[0], origin.rpy[1], origin.rpy[2], 'ZYX'));
}

function parseMaterialColor(visualEl) {
  const matEl = visualEl?.querySelector('material > color');
  if (!matEl) return DEFAULT_LINK_COLOR;
  const rgba = (matEl.getAttribute('rgba') || '0.2 0.4 0.8 1').trim().split(/\s+/).map(Number);
  return new THREE.Color(rgba[0], rgba[1], rgba[2]);
}

function createLinkVisuals(linkEl, linkGroup) {
  for (const visual of linkEl.querySelectorAll('visual')) {
    const origin  = parseOriginEl(visual.querySelector('origin'));
    const geomEl  = visual.querySelector('geometry');
    if (!geomEl) continue;

    const child   = geomEl.firstElementChild;
    if (!child) continue;

    const tag     = child.tagName.toLowerCase();
    const color   = parseMaterialColor(visual);
    const mat     = robotMaterial(color);

    let mesh = null;

    if (tag === 'box') {
      const size = (child.getAttribute('size') || '0.1 0.1 0.1').trim().split(/\s+/).map(Number);
      mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), mat);

    } else if (tag === 'cylinder') {
      const r = parseFloat(child.getAttribute('radius') || 0.04);
      const l = parseFloat(child.getAttribute('length') || 0.1);
      // URDF cylinders are along Z; Three.js CylinderGeometry is along Y.
      // The alignment rotation is composed with the origin quaternion below.
      mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, l, 20), mat);

    } else if (tag === 'sphere') {
      const r = parseFloat(child.getAttribute('radius') || 0.04);
      mesh = new THREE.Mesh(new THREE.SphereGeometry(r, 20, 14), mat);

    } else if (tag === 'mesh') {
      const filename = child.getAttribute('filename') || '';
      const url = filename.replace('package://', '/mesh/');
      const urlLower = url.toLowerCase();

      if (urlLower.endsWith('.stl')) {
        // Placeholder octahedron while STL loads
        const phMat = robotMaterial(color, 0.05);
        phMat.transparent = true; phMat.opacity = 0.35;
        const ph = new THREE.Mesh(new THREE.OctahedronGeometry(0.025), phMat);
        applyOrigin(ph, origin);
        setObjectLayerRecursive(ph, VIEW_LAYERS.URDF);
        linkGroup.add(ph);

        const scaleAttr = child.getAttribute('scale');
        const loader = new STLLoader();
        loader.load(url, (geo) => {
          if (scaleAttr) {
            const s = scaleAttr.trim().split(/\s+/).map(Number);
            geo.scale(s[0] ?? 1, s[1] ?? 1, s[2] ?? 1);
          }
          geo.computeVertexNormals();
          const realMesh = new THREE.Mesh(geo, robotMaterial(color));
          realMesh.castShadow = true;
          applyOrigin(realMesh, origin);
          setObjectLayerRecursive(realMesh, VIEW_LAYERS.URDF);
          linkGroup.remove(ph);
          linkGroup.add(realMesh);
        }, undefined, () => { /* silently keep placeholder */ });
        continue;  // handled async

      } else if (urlLower.endsWith('.dae')) {
        // Placeholder octahedron while Collada loads
        const phMat = robotMaterial(color, 0.05);
        phMat.transparent = true; phMat.opacity = 0.35;
        const ph = new THREE.Mesh(new THREE.OctahedronGeometry(0.025), phMat);
        applyOrigin(ph, origin);
        setObjectLayerRecursive(ph, VIEW_LAYERS.URDF);
        linkGroup.add(ph);

        const scaleAttr = child.getAttribute('scale');
        const loader = new ColladaLoader();
        loader.load(url, (collada) => {
          const daeScene = collada.scene;
          const daeRoot = new THREE.Group();

          if (scaleAttr) {
            const s = scaleAttr.trim().split(/\s+/).map(Number);
            daeScene.scale.set(s[0] ?? 1, s[1] ?? 1, s[2] ?? 1);
          }
          daeScene.traverse(child => {
            if (child.isMesh) {
              child.rotation.x += Math.PI / 2;
              child.castShadow = true;
              child.receiveShadow = true;
              // Keep the DAE's own materials; they carry colour & texture info
            }
          });
          applyOrigin(daeRoot, origin);
          daeRoot.add(daeScene);
          setObjectLayerRecursive(daeRoot, VIEW_LAYERS.URDF);
          linkGroup.remove(ph);
          linkGroup.add(daeRoot);
        }, undefined, () => { /* silently keep placeholder */ });
        continue;  // handled async
      }
      // Unsupported mesh type → box placeholder
      mesh = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 0.05), robotMaterial(color));
    }

    if (mesh) {
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      applyOrigin(mesh, origin);
      if (tag === 'cylinder') {
        // CylinderGeometry axis is along Three.js Y; URDF cylinders are along Z.
        // Compose the alignment (Y→Z) with the origin rotation so it isn't lost:
        //   q_final = q_origin_rpy × q_align  (align first, then origin rotation)
        mesh.quaternion.multiply(
          new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2));
      }
      setObjectLayerRecursive(mesh, VIEW_LAYERS.URDF);
      linkGroup.add(mesh);
    }
  }
}

async function loadURDF(xmlString, filters = null) {
  // Clear previous robot
  while (robotRoot.children.length) robotRoot.remove(robotRoot.children[0]);
  for (const k of Object.keys(jointPivots))      delete jointPivots[k];
  for (const k of Object.keys(jointAxes_))        delete jointAxes_[k];
  for (const k of Object.keys(jointTypes_))       delete jointTypes_[k];
  for (const k of Object.keys(jointOriginQuats))  delete jointOriginQuats[k];
  for (const k of Object.keys(jointOriginPos))    delete jointOriginPos[k];

  const doc = new DOMParser().parseFromString(xmlString, 'text/xml');
  if (doc.querySelector('parseerror')) {
    console.error('URDF parse error');
    return;
  }

  // ── Collect links ─────────────────────────────────────────────────────
  const linkGroups = {};
  for (const linkEl of doc.querySelectorAll('link')) {
    const name  = linkEl.getAttribute('name');
    const group = new THREE.Group();
    group.name  = `link_${name}`;
    if (shouldShowUrdfLink(name, filters)) {
      createLinkVisuals(linkEl, group);
    }
    linkGroups[name] = group;
  }

  // Filtering is applied by selectively creating link visuals. The link/joint
  // hierarchy is always preserved so transforms and children remain valid.

  // ── Collect joints and wire scene graph ───────────────────────────────
  const childLinkNames = new Set();

  for (const jEl of doc.querySelectorAll('joint')) {
    const jName   = jEl.getAttribute('name');
    const jType   = jEl.getAttribute('type') || 'fixed';
    const parent  = jEl.querySelector('parent')?.getAttribute('link');
    const child   = jEl.querySelector('child')?.getAttribute('link');
    if (!parent || !child || !linkGroups[parent] || !linkGroups[child]) continue;

    const origin  = parseOriginEl(jEl.querySelector('origin'));
    const axisEl  = jEl.querySelector('axis');
    const axXYZ   = (axisEl?.getAttribute('xyz') || '0 0 1').trim().split(/\s+/).map(Number);

    // Pivot group placed at joint origin within parent link
    const pivot = new THREE.Group();
    pivot.name  = `joint_${jName}`;
    applyOrigin(pivot, origin);
    pivot.add(linkGroups[child]);
    linkGroups[parent].add(pivot);

    // Store initial transform so joint-state rotation is relative to it
    jointPivots[jName]      = pivot;
    jointAxes_[jName]       = new THREE.Vector3(...axXYZ).normalize();
    jointTypes_[jName]      = jType;
    jointOriginQuats[jName] = pivot.quaternion.clone();
    jointOriginPos[jName]   = pivot.position.clone();

    // // Small teal sphere at each mobile joint origin (visual cue)
    // if (jType !== 'fixed') {
    //   const dot = new THREE.Mesh(
    //     new THREE.SphereGeometry(0.012, 8, 6),
    //     new THREE.MeshStandardMaterial({
    //       color: JOINT_SPHERE_COLOR,
    //       emissive: JOINT_SPHERE_COLOR,
    //       emissiveIntensity: 0.6,
    //       metalness: 0.9,
    //       roughness: 0.1,
    //     }),
    //   );
    //   pivot.add(dot);
    // }

    childLinkNames.add(child);
  }

  // ── Find root link and attach ─────────────────────────────────────────
  const rootName = Object.keys(linkGroups).find(n => !childLinkNames.has(n))
                   ?? Object.keys(linkGroups)[0];
  if (rootName && linkGroups[rootName]) {
    robotRoot.add(linkGroups[rootName]);
    setObjectLayerRecursive(robotRoot, VIEW_LAYERS.URDF);
    robotBaseFrame = normalizeFrameId(rootName) || 'base_link';
    updateRobotPoseFromTF();
  }

  robotLoaded = true;
  console.log('[URDF] Loaded, root link:', rootName,
    '| joints:', Object.keys(jointPivots).length);

  if (pendingJointState) {
    applyJointStates(pendingJointState.name, pendingJointState.position);
    pendingJointState = null;
  }
}

function applyJointStates(names, positions) {
  if (!robotLoaded) {
    pendingJointState = { name: names, position: positions };
    return;
  }
  for (let i = 0; i < names.length; i++) {
    const pivot = jointPivots[names[i]];
    if (!pivot) continue;
    const axis = jointAxes_[names[i]];
    const type = jointTypes_[names[i]];
    const oQ   = jointOriginQuats[names[i]];
    const oP   = jointOriginPos[names[i]];

    if (type === 'prismatic') {
      pivot.position.copy(oP).addScaledVector(axis, positions[i]);
    } else if (type !== 'fixed') {
      const delta = new THREE.Quaternion().setFromAxisAngle(axis, positions[i]);
      pivot.quaternion.multiplyQuaternions(oQ, delta);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// TF tree (stored, not yet visualised but available for extensions)
// ─────────────────────────────────────────────────────────────────────────────

const tfTree = {};  // frame_id → { parent, tx, ty, tz, rx, ry, rz, rw }
let fixedFrame = 'base_link';
let fixedFrameLocked = false;
const tfWarningKeys = new Set();

function warnTfOnce(key, message) {
  if (tfWarningKeys.has(key)) return;
  tfWarningKeys.add(key);
  console.warn(message);
}

function clearTfWarning(key) {
  tfWarningKeys.delete(key);
}

function normalizeFrameId(frameId) {
  return (frameId || '').trim().replace(/^\/+/, '');
}

function applyTF(transforms, _static, fixedFrameFromMsg) {
  if (fixedFrameFromMsg) {
    const requestedFixedFrame = normalizeFrameId(fixedFrameFromMsg) || 'base_link';
    if (!fixedFrameLocked) {
      fixedFrame = requestedFixedFrame;
      fixedFrameLocked = true;
    } else if (requestedFixedFrame !== fixedFrame) {
      console.warn(
        `[TF] Ignoring fixed frame change from "${fixedFrame}" to "${requestedFixedFrame}"`,
      );
    }
  }
  for (const t of transforms) {
    const child = normalizeFrameId(t.child);
    if (!child) continue;
    tfTree[child] = {
      parent: normalizeFrameId(t.parent),
      tx: t.tx, ty: t.ty, tz: t.tz,
      rx: t.rx, ry: t.ry, rz: t.rz, rw: t.rw,
    };
  }
  updateRobotPoseFromTF();
  updateAllPointCloudPosesFromTF();
  updateMarkerPosesFromTF();
}

function getFrameToRootMatrix(frameId) {
  const chain = [];
  let f = normalizeFrameId(frameId);
  const visited = new Set();
  while (f && tfTree[f] && !visited.has(f)) {
    visited.add(f);
    chain.push(tfTree[f]);
    f = tfTree[f].parent;
  }
  const mat = new THREE.Matrix4();
  for (let i = chain.length - 1; i >= 0; i--) {
    const t = chain[i];
    mat.multiply(new THREE.Matrix4().compose(
      new THREE.Vector3(t.tx, t.ty, t.tz),
      new THREE.Quaternion(t.rx, t.ry, t.rz, t.rw),
      new THREE.Vector3(1, 1, 1),
    ));
  }
  return mat;
}

function getRootFrameId(frameId) {
  let f = normalizeFrameId(frameId);
  const visited = new Set();
  while (f && tfTree[f] && !visited.has(f)) {
    visited.add(f);
    f = tfTree[f].parent;
  }
  return f;
}

function getFrameMatrixInFixedFrame(frameId, consumer = 'unknown') {
  const f = normalizeFrameId(frameId);
  if (!f) {
    warnTfOnce(
      `empty:${consumer}`,
      `[TF] Cannot resolve transform for ${consumer}: empty frame_id`,
    );
    return null;
  }
  if (f === fixedFrame) {
    clearTfWarning(`disconnected:${consumer}:${f}:${fixedFrame}`);
    return new THREE.Matrix4();
  }
  const fixedRoot = getRootFrameId(fixedFrame);
  const frameRoot = getRootFrameId(f);
  if (fixedRoot && frameRoot && fixedRoot !== frameRoot) {
    warnTfOnce(
      `disconnected:${consumer}:${f}:${fixedFrame}`,
      `[TF] Cannot resolve transform for ${consumer}: frame "${f}" is in tree rooted at "${frameRoot}", but fixed_frame is "${fixedFrame}" rooted at "${fixedRoot}"`,
    );
    return null;
  }
  clearTfWarning(`disconnected:${consumer}:${f}:${fixedFrame}`);
  const rootToFixed = getFrameToRootMatrix(fixedFrame);
  const rootToFrame = getFrameToRootMatrix(f);
  return rootToFixed.clone().invert().multiply(rootToFrame);
}

function updateRobotPoseFromTF() {
  if (!robotLoaded) return;
  const frameMat = getFrameMatrixInFixedFrame(robotBaseFrame, 'robot');
  if (!frameMat) {
    robotRoot.position.set(0, 0, 0);
    robotRoot.quaternion.identity();
    robotRoot.scale.set(1, 1, 1);
    return;
  }
  frameMat.decompose(robotRoot.position, robotRoot.quaternion, robotRoot.scale);
}

// ─────────────────────────────────────────────────────────────────────────────
// MarkerArray — visualization_msgs/MarkerArray support
// ─────────────────────────────────────────────────────────────────────────────

// Marker type constants (from visualization_msgs/msg/Marker.msg)
const MK = {
  ARROW: 0, CUBE: 1, SPHERE: 2, CYLINDER: 3,
  LINE_STRIP: 4, LINE_LIST: 5, CUBE_LIST: 6, SPHERE_LIST: 7,
  POINTS: 8, TEXT: 9, MESH: 10, TRIANGLE_LIST: 11,
  ADD: 0, MODIFY: 0, DELETE: 2, DELETEALL: 3,
};

const markerRootsByTopic = new Map();

// "<topic>::ns:id" → Object3D
const markerObjects = new Map();
// "<topic>::ns:id" → latest marker payload (for TF-driven pose refresh)
const markerMessages = new Map();
// "<topic>::ns:id" → topic
const markerObjectTopics = new Map();

function getOrCreateMarkerRoot(topicName) {
  const topic = String(topicName || '').trim() || '<unknown>';
  const existing = markerRootsByTopic.get(topic);
  if (existing) return existing;

  const root = new THREE.Group();
  root.name = `markers_${topic}`;
  setObjectLayerRecursive(root, getTopicLayer('marker_array', topic));
  rosSceneRoot.add(root);
  markerRootsByTopic.set(topic, root);
  return root;
}

// Walk TF chain to compute world-space matrix of a named frame
function getFrameWorldMatrix(frameId) {
  return getFrameMatrixInFixedFrame(frameId, 'marker') || new THREE.Matrix4();
}

// Standard MeshStandardMaterial for markers
function mkMat(r, g, b, a, side = THREE.FrontSide) {
  return new THREE.MeshStandardMaterial({
    color: new THREE.Color(r, g, b),
    opacity: a,
    transparent: a < 0.999,
    metalness: 0.3,
    roughness: 0.5,
    side,
  });
}

// Dispose all geometries/materials/textures in an Object3D hierarchy
function disposeMarker(obj) {
  obj.traverse(child => {
    if (child.geometry) child.geometry.dispose();
    if (child.material) {
      const mats = Array.isArray(child.material) ? child.material : [child.material];
      for (const m of mats) { if (m.map) m.map.dispose(); m.dispose(); }
    }
  });
}

// Return a Group with a cylinder+cone arrow pointing along +X from origin
function makeArrowGeom(length, shaftR, headR, r, g, b, a) {
  const headLen  = Math.min(length * 0.23, headR * 3);
  const shaftLen = Math.max(length - headLen, 0.001);
  const group    = new THREE.Group();

  // CylinderGeometry is along Y; rotate -90° around Z → points along +X
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(shaftR, shaftR, shaftLen, 8),
    mkMat(r, g, b, a),
  );
  shaft.rotation.z  = -Math.PI / 2;
  shaft.position.x  = shaftLen / 2;

  const head = new THREE.Mesh(
    new THREE.ConeGeometry(headR, headLen, 8),
    mkMat(r, g, b, a),
  );
  head.rotation.z = -Math.PI / 2;
  head.position.x = shaftLen + headLen / 2;

  group.add(shaft, head);
  return group;
}

// Build a Three.js Object3D for a single marker (geometry in marker-local space)
function createMarkerObject(m) {
  const [r, g, b, a] = [m.r, m.g, m.b, m.a];

  switch (m.type) {

    case MK.ARROW: {
      const length = m.sx > 0 ? m.sx : 0.5;
      const shaftR = m.sy > 0 ? m.sy / 2 : length * 0.04;
      const headR  = m.sz > 0 ? m.sz / 2 : shaftR * 2.5;

      if (m.points && m.points.length >= 2) {
        // points[0] → points[1] in marker reference frame (override pose)
        const pStart = new THREE.Vector3(...m.points[0]);
        const pEnd   = new THREE.Vector3(...m.points[1]);
        const dir    = pEnd.clone().sub(pStart);
        const len2   = dir.length();
        if (len2 < 1e-6) return null;
        dir.normalize();

        const sh2R = m.sy > 0 ? m.sy / 2 : len2 * 0.04;
        const he2R = m.sz > 0 ? m.sz / 2 : sh2R * 2.5;
        const grp  = makeArrowGeom(len2, sh2R, he2R, r, g, b, a);
        grp.position.copy(pStart);
        grp.quaternion.setFromUnitVectors(new THREE.Vector3(1, 0, 0), dir);
        return grp;
      }
      return makeArrowGeom(length, shaftR, headR, r, g, b, a);
    }

    case MK.CUBE:
      return new THREE.Mesh(
        new THREE.BoxGeometry(m.sx || 0.1, m.sy || 0.1, m.sz || 0.1),
        mkMat(r, g, b, a),
      );

    case MK.SPHERE: {
      const obj = new THREE.Mesh(new THREE.SphereGeometry(0.5, 16, 12), mkMat(r, g, b, a));
      obj.scale.set(m.sx || 0.1, m.sy || 0.1, m.sz || 0.1);
      return obj;
    }

    case MK.CYLINDER: {
      // Cylinder axis along Z in ROS; wrap in a Group so local rotation is not
      // overwritten when applyMarkerPose sets the parent quaternion
      const wrapper = new THREE.Group();
      const cyl = new THREE.Mesh(
        new THREE.CylinderGeometry((m.sx || 0.1) / 2, (m.sy || 0.1) / 2, m.sz || 0.2, 16),
        mkMat(r, g, b, a),
      );
      cyl.rotation.x = Math.PI / 2;  // Y-axis cylinder → Z-axis
      wrapper.add(cyl);
      return wrapper;
    }

    case MK.LINE_STRIP:
    case MK.LINE_LIST: {
      if (!m.points || m.points.length < 2) return null;
      const geo = new THREE.BufferGeometry().setFromPoints(
        m.points.map(p => new THREE.Vector3(...p)));
      const hasCol = m.colors && m.colors.length >= m.points.length;
      const mat = new THREE.LineBasicMaterial({
        color: hasCol ? 0xffffff : new THREE.Color(r, g, b),
        opacity: a, transparent: a < 0.999, vertexColors: hasCol,
      });
      if (hasCol) {
        const ca = new Float32Array(m.points.length * 3);
        for (let i = 0; i < m.points.length; i++) {
          ca[i * 3] = m.colors[i][0]; ca[i * 3 + 1] = m.colors[i][1]; ca[i * 3 + 2] = m.colors[i][2];
        }
        geo.setAttribute('color', new THREE.BufferAttribute(ca, 3));
      }
      return m.type === MK.LINE_LIST
        ? new THREE.LineSegments(geo, mat)
        : new THREE.Line(geo, mat);
    }

    case MK.CUBE_LIST: {
      if (!m.points || m.points.length === 0) return null;
      const group = new THREE.Group();
      const geo   = new THREE.BoxGeometry(m.sx || 0.05, m.sy || 0.05, m.sz || 0.05);
      for (let i = 0; i < m.points.length; i++) {
        const c   = (m.colors && m.colors[i]) || [r, g, b, a];
        const msh = new THREE.Mesh(geo, mkMat(c[0], c[1], c[2], c[3] ?? a));
        msh.position.set(...m.points[i]);
        group.add(msh);
      }
      return group;
    }

    case MK.SPHERE_LIST: {
      if (!m.points || m.points.length === 0) return null;
      const group = new THREE.Group();
      const geo   = new THREE.SphereGeometry(0.5, 8, 6);
      for (let i = 0; i < m.points.length; i++) {
        const c   = (m.colors && m.colors[i]) || [r, g, b, a];
        const msh = new THREE.Mesh(geo, mkMat(c[0], c[1], c[2], c[3] ?? a));
        msh.position.set(...m.points[i]);
        msh.scale.set(m.sx || 0.05, m.sy || 0.05, m.sz || 0.05);
        group.add(msh);
      }
      return group;
    }

    case MK.POINTS: {
      if (!m.points || m.points.length === 0) return null;
      const n      = m.points.length;
      const posArr = new Float32Array(n * 3);
      const colArr = new Float32Array(n * 3);
      for (let i = 0; i < n; i++) {
        posArr[i * 3]     = m.points[i][0];
        posArr[i * 3 + 1] = m.points[i][1];
        posArr[i * 3 + 2] = m.points[i][2];
        const c = (m.colors && m.colors[i]) || [r, g, b];
        colArr[i * 3]     = c[0];
        colArr[i * 3 + 1] = c[1];
        colArr[i * 3 + 2] = c[2];
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(posArr, 3));
      geo.setAttribute('color',    new THREE.BufferAttribute(colArr, 3));
      return new THREE.Points(geo, new THREE.PointsMaterial({
        size: m.sx || 0.05, vertexColors: true,
        opacity: a, transparent: a < 0.999,
      }));
    }

    case MK.TEXT: {
      // Billboard sprite rendered from an off-screen canvas
      const cvs = document.createElement('canvas');
      cvs.width = 512; cvs.height = 128;
      const ctx = cvs.getContext('2d');
      ctx.font        = 'bold 68px monospace';
      ctx.fillStyle   = `rgba(${Math.round(r * 255)},${Math.round(g * 255)},${Math.round(b * 255)},${a})`;
      ctx.textAlign   = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText((m.text || '').substring(0, 28), cvs.width / 2, cvs.height / 2);
      const tex    = new THREE.CanvasTexture(cvs);
      const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true }));
      const h      = m.sz > 0 ? m.sz : 0.2;
      sprite.scale.set(h * 4, h, 1);
      return sprite;
    }

    case MK.MESH: {
      if (!m.mesh_resource) return null;
      const url = m.mesh_resource.replace('package://', '/mesh/');
      if (!url.toLowerCase().endsWith('.stl')) return null;
      const group = new THREE.Group();
      new STLLoader().load(url, (geo) => {
        geo.computeVertexNormals();
        const msh = new THREE.Mesh(geo, mkMat(r, g, b, a));
        msh.scale.set(m.sx || 1, m.sy || 1, m.sz || 1);
        group.add(msh);
      });
      return group;
    }

    case MK.TRIANGLE_LIST: {
      if (!m.points || m.points.length < 3 || m.points.length % 3 !== 0) return null;
      const posArr = new Float32Array(m.points.length * 3);
      for (let i = 0; i < m.points.length; i++) {
        posArr[i * 3]     = m.points[i][0];
        posArr[i * 3 + 1] = m.points[i][1];
        posArr[i * 3 + 2] = m.points[i][2];
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(posArr, 3));
      if (m.colors && m.colors.length >= m.points.length) {
        const ca = new Float32Array(m.points.length * 3);
        for (let i = 0; i < m.points.length; i++) {
          ca[i * 3] = m.colors[i][0]; ca[i * 3 + 1] = m.colors[i][1]; ca[i * 3 + 2] = m.colors[i][2];
        }
        geo.setAttribute('color', new THREE.BufferAttribute(ca, 3));
        geo.computeVertexNormals();
        return new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
          vertexColors: true, side: THREE.DoubleSide,
          opacity: a, transparent: a < 0.999, metalness: 0.2, roughness: 0.6,
        }));
      }
      geo.computeVertexNormals();
      return new THREE.Mesh(geo, mkMat(r, g, b, a, THREE.DoubleSide));
    }

    default:
      console.warn('[Markers] Unsupported marker type:', m.type);
      return null;
  }
}

// Apply TF-frame + marker pose to the object (unless skipPose is set)
function applyMarkerPose(obj, m) {
  if (obj.userData.skipPose) return;

  const frameMat = getFrameWorldMatrix(m.frame_id);
  const fPos  = new THREE.Vector3();
  const fQuat = new THREE.Quaternion();
  frameMat.decompose(fPos, fQuat, new THREE.Vector3());

  // World pos = frame_pos + frame_rot * local_pos
  const wPos  = new THREE.Vector3(m.px, m.py, m.pz).applyQuaternion(fQuat).add(fPos);
  // World quat = frame_rot * local_rot
  const wQuat = fQuat.clone().multiply(new THREE.Quaternion(m.rx, m.ry, m.rz, m.rw));

  obj.position.copy(wPos);
  obj.quaternion.copy(wQuat);
}

function _removeMarker(key) {
  const obj = markerObjects.get(key);
  if (!obj) return;
  const topic = markerObjectTopics.get(key) || '';
  const topicRoot = markerRootsByTopic.get(topic);
  if (topicRoot) {
    topicRoot.remove(obj);
  }
  disposeMarker(obj);
  markerObjects.delete(key);
  markerMessages.delete(key);
  markerObjectTopics.delete(key);
}

function _removeAllMarkers(topic, ns) {
  for (const key of [...markerObjects.keys()]) {
    const topicPrefix = `${topic}::`;
    if (!key.startsWith(topicPrefix)) continue;
    if (ns === null || key.substring(topicPrefix.length).startsWith(ns + ':')) {
      _removeMarker(key);
    }
  }
}

function updateMarkerArray(msg) {
  const topic = String(msg.topic || '').trim() || '<unknown>';
  const topicRoot = getOrCreateMarkerRoot(topic);
  const layer = getTopicLayer('marker_array', topic);

  for (const m of msg.markers) {
    const key = `${topic}::${m.ns}:${m.id}`;

    if (m.action === MK.DELETE)    { _removeMarker(key); continue; }
    if (m.action === MK.DELETEALL) { _removeAllMarkers(topic, m.ns || null); continue; }

    // ADD / MODIFY (both == 0): recreate geometry
    if (markerObjects.has(key)) _removeMarker(key);

    const obj = createMarkerObject(m);
    if (!obj) continue;
    setObjectLayerRecursive(obj, layer);
    applyMarkerPose(obj, m);
    topicRoot.add(obj);
    markerObjects.set(key, obj);
    markerMessages.set(key, m);
    markerObjectTopics.set(key, topic);
  }
}

function updateMarkerPosesFromTF() {
  for (const [key, obj] of markerObjects.entries()) {
    const msg = markerMessages.get(key);
    if (!msg) continue;
    applyMarkerPose(obj, msg);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// WebSocket — connect / reconnect / dispatch
// ─────────────────────────────────────────────────────────────────────────────

let ws = null;

function connectWS() {
  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    setWsStatus(true);
    console.log('[WS] Connected');
  };

  ws.onclose = () => {
    setWsStatus(false);
    console.log(`[WS] Disconnected — retrying in ${WS_RETRY_MS}ms`);
    setTimeout(connectWS, WS_RETRY_MS);
  };

  ws.onerror = (err) => {
    console.warn('[WS] Error:', err);
  };

  ws.onmessage = (evt) => {
    let msg;
    try { msg = JSON.parse(evt.data); } catch { return; }

    switch (msg.type) {
      case 'joint_states':
        applyJointStates(msg.name, msg.position);
        break;

      case 'tf':
        applyTF(msg.transforms, msg.static, msg.fixed_frame);
        break;

      case 'image':
        if (activeImageTopics.size > 0 && !activeImageTopics.has(msg.topic)) {
          break;
        }
        updateImage(msg.data, msg.topic);
        break;

      case 'pointcloud':
        if (activePointCloudTopics.size > 0 && !activePointCloudTopics.has(msg.topic)) {
          break;
        }
        updatePointCloud(msg.topic, msg.data, msg.count, msg.frame_id);
        break;

      case 'marker_array':
        if (activeMarkerArrayTopics.size > 0 && !activeMarkerArrayTopics.has(msg.topic)) {
          break;
        }
        updateMarkerArray(msg);
        break;

      case 'html_panel':
        updateHtmlPanel(msg.data, msg.topic);
        break;
    }
  };
}

// ─────────────────────────────────────────────────────────────────────────────
// Camera image panel
// ─────────────────────────────────────────────────────────────────────────────

const imagePanelWidgets = Array.from(document.querySelectorAll('[data-ros-widget="image-panel"]'))
  .map((panel) => {
    const imageEl = panel.querySelector('img');
    const placeholderEl = panel.querySelector('#image-placeholder, [data-role="image-placeholder"]');
    const topicEl = panel.querySelector('#image-topic-label, [data-role="image-topic-label"]');
    if (!imageEl || !placeholderEl || !topicEl) return null;
    return {
      panel,
      imageEl,
      placeholderEl,
      topicEl,
      topicFilter: String(panel.getAttribute('data-topic') || '').trim(),
    };
  })
  .filter(Boolean);

const activeImageTopics = new Set(
  imagePanelWidgets
    .map((widget) => widget.topicFilter)
    .filter((topic) => topic && topic.startsWith('/')),
);

const htmlPanelWidgets = Array.from(document.querySelectorAll('[data-ros-widget="html-panel"]'))
  .map((panel) => {
    const contentEl = panel.querySelector('[data-role="html-content"]');
    const topicEl = panel.querySelector('[data-role="html-topic-label"]');
    if (!contentEl || !topicEl) return null;
    return {
      panel,
      contentEl,
      topicEl,
      topicFilter: String(panel.getAttribute('data-topic') || '').trim(),
    };
  })
  .filter(Boolean);

function updateImage(dataUri, topic) {
  for (const widget of imagePanelWidgets) {
    if (widget.topicFilter && widget.topicFilter !== topic) continue;
    widget.imageEl.src = dataUri;
    widget.imageEl.style.display = 'block';
    widget.placeholderEl.style.display = 'none';
    widget.topicEl.textContent = topic.split('/').pop();
    widget.panel.classList.remove('hidden');
  }
}

function isSafeUrl(url) {
  const value = String(url || '').trim();
  if (!value) return false;
  if (value.startsWith('#') || value.startsWith('/') || value.startsWith('./') || value.startsWith('../')) {
    return true;
  }
  try {
    const parsed = new URL(value, window.location.origin);
    const allowedProtocols = new Set(['http:', 'https:', 'mailto:', 'tel:']);
    return allowedProtocols.has(parsed.protocol);
  } catch {
    return false;
  }
}

const panelSanitizerConfig = {
  allowElements: ['div', 'p', 'span', 'strong', 'em', 'b', 'i', 'u',
    'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
    'ul', 'ol', 'li', 'br', 'hr', 'code', 'pre', 'blockquote', 'a', 'button'],
  allowAttributes: {
    class: ['*'],
    id: ['*'],
    title: ['*'],
    role: ['*'],
    href: ['a'],
    target: ['a'],
    rel: ['a'],
    type: ['button'],
    'data-trigger-service': ['button'],
    'data-trigger-timeout': ['button'],
  },
};

function normalizePanelLinks(root) {
  for (const link of root.querySelectorAll('a')) {
    const href = link.getAttribute('href');
    if (!isSafeUrl(href)) {
      link.removeAttribute('href');
      link.removeAttribute('target');
      link.removeAttribute('rel');
      continue;
    }
    link.setAttribute('target', '_blank');
    link.setAttribute('rel', 'noopener noreferrer');
  }
}

function bindTriggerButtons(root) {
  for (const button of root.querySelectorAll('button[data-trigger-service]')) {
    if (button.dataset.triggerBound === 'true') continue;
    button.dataset.triggerBound = 'true';
    button.addEventListener('click', async (evt) => {
      evt.preventDefault();
      const service = String(button.getAttribute('data-trigger-service') || '').trim();
      if (!service) return;

      const timeoutRaw = Number.parseFloat(
        button.getAttribute('data-trigger-timeout') || String(TRIGGER_TIMEOUT_DEFAULT),
      );
      const timeoutSec = Number.isFinite(timeoutRaw)
        ? Math.min(Math.max(timeoutRaw, TRIGGER_TIMEOUT_MIN), TRIGGER_TIMEOUT_MAX)
        : TRIGGER_TIMEOUT_DEFAULT;

      button.disabled = true;
      button.dataset.triggerState = 'pending';
      try {
        const response = await fetch('/api/trigger', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ service, timeout_sec: timeoutSec }),
        });
        const payload = await response.json();
        const ok = Boolean(payload?.ok);
        button.dataset.triggerState = ok ? 'ok' : 'error';
        // Prefer backend error text, then service message, then generic fallback.
        const failureMessage = payload?.error || payload?.message || 'Trigger service call failed';
        button.title = ok ? String(payload.message || 'Trigger service call succeeded')
          : String(failureMessage);
      } catch (err) {
        button.dataset.triggerState = 'error';
        button.title = `Trigger service call failed: ${err?.message || err}`;
      } finally {
        setTimeout(() => {
          button.disabled = false;
          button.dataset.triggerState = '';
        }, 180);
      }
    });
  }
}

function updateHtmlPanel(html, topic) {
  const rawHtml = String(html ?? '');
  for (const widget of htmlPanelWidgets) {
    if (widget.topicFilter && widget.topicFilter !== topic) continue;

    if (typeof window.Sanitizer === 'function' && typeof widget.contentEl.setHTML === 'function') {
      const sanitizer = new window.Sanitizer(panelSanitizerConfig);
      widget.contentEl.setHTML(rawHtml, { sanitizer });
      normalizePanelLinks(widget.contentEl);
      bindTriggerButtons(widget.contentEl);
    } else {
      // Degraded fallback for browsers without Sanitizer API support.
      widget.contentEl.textContent = rawHtml;
    }
    widget.topicEl.textContent = topic.split('/').pop();
  }
}

async function registerHtmlPanelTopics() {
  const topics = [...new Set(htmlPanelWidgets
    .map(widget => widget.topicFilter)
    .filter(topic => topic && topic.startsWith('/')))];
  for (const topic of topics) {
    try {
      const response = await fetch('/api/register_html_panel_topic', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ topic }),
      });
      if (!response.ok) {
        let detail = '';
        try {
          const payload = await response.json();
          detail = payload?.error ? `: ${payload.error}` : '';
        } catch {
          // ignore parse errors for non-JSON responses
        }
        console.warn(`[HTML] Failed to register panel topic "${topic}"${detail}`);
      }
    } catch (err) {
      console.warn(`[HTML] Failed to register panel topic "${topic}":`, err);
    }
  }
}

async function registerViewerTopics() {
  const payload = {
    image: [...activeImageTopics],
    pointcloud: [...activePointCloudTopics],
    markers: [...activeMarkerArrayTopics],
  };

  if (
    payload.image.length === 0
    && payload.pointcloud.length === 0
    && payload.markers.length === 0
  ) {
    return;
  }

  try {
    const response = await fetch('/api/register_viewer_topics', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    if (!response.ok) {
      let detail = '';
      try {
        const result = await response.json();
        detail = result?.error ? `: ${result.error}` : '';
      } catch {
        // ignore parse errors for non-JSON responses
      }
      console.warn(`[Viewer] Failed to register dynamic topics${detail}`);
    }
  } catch (err) {
    console.warn('[Viewer] Failed to register dynamic topics:', err);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// URDF fetching (polls until robot_description arrives)
// ─────────────────────────────────────────────────────────────────────────────

let urdfLoaded = false;

async function fetchURDF() {
  if (urdfLoaded) return;
  try {
    const res = await fetch('/api/urdf');
    if (res.status === 204) {
      // Not yet available
      setTimeout(fetchURDF, URDF_RETRY_MS);
      return;
    }
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const xml = await res.text();
    if (xml.trim().length > 0) {
      urdfLoaded = true;
      // Get filters from first widget that displays URDF
      const filters = viewerWidgets.find(w => w.displayConfig.showUrdf)?.urdfLinkFilters || null;
      await loadURDF(xml, filters);
    } else {
      setTimeout(fetchURDF, URDF_RETRY_MS);
    }
  } catch (e) {
    console.warn('[URDF] Fetch failed:', e.message, '— retrying');
    setTimeout(fetchURDF, URDF_RETRY_MS);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// UI helpers
// ─────────────────────────────────────────────────────────────────────────────

const elWsDot   = document.getElementById('ws-dot');
const elWsLabel = document.getElementById('ws-label');

function setWsStatus(connected) {
  if (!elWsDot || !elWsLabel) return;
  elWsDot.classList.toggle('connected', connected);
  elWsLabel.textContent = connected ? 'LIVE' : 'OFFLINE';
}

// ─────────────────────────────────────────────────────────────────────────────
// Animation loop
// ─────────────────────────────────────────────────────────────────────────────

function startWidgetAnimationLoops() {
  for (const widget of viewerWidgets) {
    let widgetLastRenderTime = performance.now() - widget.frameIntervalMs;
    widget.renderer.setAnimationLoop((now) => {
      const elapsedSinceRender = now - widgetLastRenderTime;
      if (elapsedSinceRender < widget.frameIntervalMs) {
        return;
      }

      const dt = Math.max(0, elapsedSinceRender);
      widgetLastRenderTime = now;

      widget.controls.update(dt * 0.001);
      widget.composer.render();
    });
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Resize handler
// ─────────────────────────────────────────────────────────────────────────────

window.addEventListener('resize', () => {
  for (const widget of viewerWidgets) {
    const { width, height } = getCanvasSize(widget.canvasEl);
    widget.camera.aspect = width / height;
    widget.camera.updateProjectionMatrix();
    widget.renderer.setSize(width, height, false);
    widget.composer.setSize(width, height);
    widget.bloomPass.resolution.set(width, height);
  }
});

// ─────────────────────────────────────────────────────────────────────────────
// Loading screen dismiss
// ─────────────────────────────────────────────────────────────────────────────

const loadingEl = document.getElementById('loading');
if (loadingEl) {
  setTimeout(() => {
    loadingEl.classList.add('fade-out');
    setTimeout(() => loadingEl.remove(), 900);
  }, 2000);
}

// ─────────────────────────────────────────────────────────────────────────────
// Start
// ─────────────────────────────────────────────────────────────────────────────

registerViewerTopics();
connectWS();
registerHtmlPanelTopics();
fetchURDF();
bindTriggerButtons(document);
window.dispatchEvent(new Event('resize'));
startWidgetAnimationLoops();
