<!DOCTYPE html>
<html lang="fr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Drone HE — Démo 3D avec Z variable</title>
  <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.2/dist/css/bootstrap.min.css" rel="stylesheet">
  <style>
    * { box-sizing: border-box; }
    body { background: #f0f4f8; font-family: 'Segoe UI', system-ui, sans-serif; margin: 0; }
    .topbar { background: #fff; border-bottom: 1px solid #ddd; padding: 10px 20px; }
    .demo-container { display: flex; gap: 20px; padding: 20px; flex-wrap: wrap; }
    .canvas-panel { flex: 2; min-width: 500px; background: #1a1a2e; border-radius: 12px; overflow: hidden; }
    #three-canvas { width: 100%; height: 550px; display: block; }
    .controls-panel { flex: 1; min-width: 280px; background: #fff; border-radius: 12px; padding: 15px; box-shadow: 0 2px 8px rgba(0,0,0,0.1); }
    .btn-d { display: inline-block; padding: 8px 16px; margin: 4px; border: none; border-radius: 8px; cursor: pointer; font-weight: bold; }
    .btn-d1 { background: #0ea5e9; color: white; }
    .btn-d2 { background: #6366f1; color: white; }
    .btn-d3 { background: #10b981; color: white; }
    .btn-d.active { outline: 3px solid #f59e0b; }
    .z-control { display: flex; gap: 10px; align-items: center; margin: 15px 0; }
    input[type="range"] { flex: 1; }
    .point-list { max-height: 200px; overflow-y: auto; font-size: 12px; font-family: monospace; }
    .point-item { display: flex; justify-content: space-between; padding: 4px 0; border-bottom: 1px solid #eee; }
    .collision-badge { background: #ef4444; color: white; border-radius: 4px; padding: 2px 6px; font-size: 10px; }
    .verdict { padding: 12px; border-radius: 8px; text-align: center; font-weight: bold; margin: 10px 0; }
    .verdict.col { background: #fee2e2; color: #b91c1c; }
    .verdict.safe { background: #dcfce7; color: #15803d; }
    button:disabled { opacity: 0.5; cursor: not-allowed; }
    footer { text-align: center; padding: 15px; color: #666; font-size: 12px; }
  </style>
</head>
<body>

<div class="topbar">
  <h2 style="margin:0">✈️ Drone HE — Détection de collisions 3D (FHE)</h2>
  <small>Z variable : chaque waypoint a sa propre altitude</small>
</div>

<div class="demo-container">
  <div class="canvas-panel">
    <canvas id="three-canvas"></canvas>
  </div>
  <div class="controls-panel">
    <div>
      <button class="btn-d btn-d1 active" onclick="setDrone(1)" id="btn-d1">🔵 Drone 1</button>
      <button class="btn-d btn-d2" onclick="setDrone(2)" id="btn-d2">🟣 Drone 2</button>
      <button class="btn-d btn-d3" onclick="setDrone(3)" id="btn-d3">🟢 Drone 3</button>
    </div>

    <div class="z-control">
      <span>Altitude Z:</span>
      <input type="range" id="z-slider" min="0" max="200" value="50" oninput="updateZ()">
      <span id="z-val" style="font-weight:bold;width:45px">50</span>
      <button onclick="stepZ(-5)" style="padding:0 8px">-5</button>
      <button onclick="stepZ(5)" style="padding:0 8px">+5</button>
    </div>

    <div style="margin: 10px 0">
      <button onclick="addPointAtCurrentZ()" style="width:100%; padding:8px; background:#0ea5e9; border:none; border-radius:8px; color:white; font-weight:bold">➕ Ajouter waypoint à Z actuel</button>
    </div>

    <div style="display:flex; gap:8px; margin:10px 0">
      <button onclick="undoLast()" style="flex:1; padding:6px">↩ Undo</button>
      <button onclick="clearAll()" style="flex:1; padding:6px">🗑 Reset</button>
      <button onclick="loadExample()" style="flex:1; padding:6px">📋 Exemple</button>
    </div>

    <h6 style="margin:15px 0 5px">Waypoints (X, Y, Z):</h6>
    <div id="point-list" class="point-list">
      <span style="color:#888">Aucun point</span>
    </div>

    <button id="analyze-btn" onclick="analyze()" disabled style="width:100%; padding:12px; margin-top:15px; background:linear-gradient(135deg,#0ea5e9,#6366f1); border:none; border-radius:10px; color:white; font-weight:bold; font-size:16px; cursor:pointer">
      🔍 Analyser les trajectoires (FHE)
    </button>

    <div id="result-area" style="margin-top:20px; display:none">
      <div id="verdict" class="verdict">—</div>
      <table style="width:100%; font-size:13px">
        <tr><td>Paires testées:</td><td id="seg-count">—</td></tr>
        <tr><td>Temps FHE:</td><td id="time-ms">—</td></tr>
        <tr><td>Scheme switches:</td><td id="ss-count">—</td></tr>
        <tr><td>Mode:</td><td id="mode-badge">—</td></tr>
      </table>
      <div id="collision-details" style="font-size:11px; margin-top:8px; color:#b91c1c"></div>
    </div>
  </div>
</div>

<footer>Chaque waypoint a une altitude Z indépendante → segments 3D réels</footer>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script>
// ============================================================
// CONFIGURATION
// ============================================================
const API = window.location.port === "8765" ? "" : "http://localhost:8765";
const GRID_SIZE = 100;
const COLORS = { 1: 0x0ea5e9, 2: 0x6366f1, 3: 0x10b981 };
const ACTIVE_COLORS = { 1: 0x38bdf8, 2: 0x818cf8, 3: 0x34d399 };

// État
let activeDrone = 1;
let paths = { 1: [], 2: [], 3: [] };  // chaque point: [x, y, z]
let currentZ = 50;
let hitSegments = [];  // stocke les segments en collision

// ============================================================
// THREE.JS INIT
// ============================================================
const canvas = document.getElementById("three-canvas");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setClearColor(0x1a1a2e, 1);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 1, 500);
camera.position.set(120, 100, 120);
camera.lookAt(50, 50, 50);

// Groupe pour les objets dynamiques
let sceneGroup = new THREE.Group();
scene.add(sceneGroup);

// Lumières
const ambient = new THREE.AmbientLight(0x404060);
scene.add(ambient);
const dirLight = new THREE.DirectionalLight(0xffffff, 1);
dirLight.position.set(50, 100, 50);
scene.add(dirLight);
const backLight = new THREE.PointLight(0x4466cc, 0.3);
backLight.position.set(0, 50, 0);
scene.add(backLight);

// Grille au sol
const gridHelper = new THREE.GridHelper(GRID_SIZE, 20, 0x888888, 0x444466);
gridHelper.position.y = -0.5;
scene.add(gridHelper);

// Axes
const axesHelper = new THREE.AxesHelper(60);
scene.add(axesHelper);

// Orbite manuelle (clic droit)
let isOrbiting = false;
let lastMouse = { x: 0, y: 0 };
let theta = 0.8, phi = 0.9, radius = 150;
const center = new THREE.Vector3(50, 50, 50);

function updateCamera() {
  camera.position.x = center.x + radius * Math.sin(phi) * Math.sin(theta);
  camera.position.y = center.y + radius * Math.cos(phi);
  camera.position.z = center.z + radius * Math.sin(phi) * Math.cos(theta);
  camera.lookAt(center);
}
updateCamera();

canvas.addEventListener("contextmenu", e => e.preventDefault());
canvas.addEventListener("mousedown", e => { if (e.button === 2) { isOrbiting = true; lastMouse = { x: e.clientX, y: e.clientY }; } });
window.addEventListener("mouseup", e => { if (e.button === 2) isOrbiting = false; });
window.addEventListener("mousemove", e => {
  if (!isOrbiting) return;
  const dx = (e.clientX - lastMouse.x) * 0.008;
  const dy = (e.clientY - lastMouse.y) * 0.008;
  theta -= dx;
  phi = Math.max(0.2, Math.min(Math.PI - 0.2, phi + dy));
  lastMouse = { x: e.clientX, y: e.clientY };
  updateCamera();
});
canvas.addEventListener("wheel", e => {
  radius = Math.max(60, Math.min(300, radius + e.deltaY * 0.3));
  updateCamera();
});

// Raycaster pour le clic
const raycaster = new THREE.Raycaster();
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);

canvas.addEventListener("click", e => {
  if (e.button !== 0) return;
  const rect = canvas.getBoundingClientRect();
  const mouse = new THREE.Vector2(
    ((e.clientX - rect.left) / rect.width) * 2 - 1,
    -((e.clientY - rect.top) / rect.height) * 2 + 1
  );
  raycaster.setFromCamera(mouse, camera);
  const hitPoint = new THREE.Vector3();
  if (raycaster.ray.intersectPlane(groundPlane, hitPoint)) {
    const x = Math.round(Math.min(GRID_SIZE, Math.max(0, hitPoint.x)));
    const z = Math.round(Math.min(GRID_SIZE, Math.max(0, hitPoint.z)));
    // Y est l'altitude (Z dans notre convention drone)
    const y = currentZ;
    paths[activeDrone].push([x, z, y]);
    rebuildScene();
    updateUI();
  }
});

// ============================================================
// FONCTIONS DE RENDU 3D (support Z variable)
// ============================================================
function toV3(p) {
  // p = [x, y, z_altitude] → Three.js Vector3(x, z_altitude, y)
  return new THREE.Vector3(p[0], p[2], p[1]);
}

function createSegmentLine(p1, p2, color, linewidth = 2) {
  const points = [toV3(p1), toV3(p2)];
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  const material = new THREE.LineBasicMaterial({ color, linewidth });
  return new THREE.Line(geometry, material);
}

function createSphere(pos, color, radius = 2) {
  const geometry = new THREE.SphereGeometry(radius, 16, 16);
  const material = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.2 });
  const sphere = new THREE.Mesh(geometry, material);
  sphere.position.copy(toV3(pos));
  return sphere;
}

function createPillar(p, color) {
  const top = toV3(p);
  const bottom = new THREE.Vector3(top.x, 0, top.z);
  const points = [bottom, top];
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  const material = new THREE.LineBasicMaterial({ color: color, transparent: true, opacity: 0.4 });
  return new THREE.Line(geometry, material);
}

function createLabel(text, pos, color) {
  const canvas = document.createElement("canvas");
  canvas.width = 64;
  canvas.height = 32;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = `#${color.toString(16).padStart(6,'0')}`;
  ctx.font = "bold 20px monospace";
  ctx.fillText(text, 4, 24);
  const texture = new THREE.CanvasTexture(canvas);
  const material = new THREE.SpriteMaterial({ map: texture });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(8, 4, 1);
  sprite.position.copy(toV3(pos));
  sprite.position.y += 4;
  return sprite;
}

function rebuildScene() {
  while (sceneGroup.children.length) sceneGroup.remove(sceneGroup.children[0]);

  for (let drone = 1; drone <= 3; drone++) {
    const pts = paths[drone];
    const color = COLORS[drone];
    if (pts.length === 0) continue;

    // Tracer les segments entre points consécutifs
    for (let i = 0; i < pts.length - 1; i++) {
      const line = createSegmentLine(pts[i], pts[i+1], color);
      sceneGroup.add(line);
    }

    // Tracer les piliers verticaux et les sphères
    pts.forEach((p, idx) => {
      sceneGroup.add(createSphere(p, color, idx === 0 ? 3 : 2));
      sceneGroup.add(createPillar(p, color));
    });

    // Label au premier point
    if (pts.length > 0) {
      sceneGroup.add(createLabel(`D${drone}`, pts[0], color));
    }
  }

  // Tracer les collisions en rouge
  hitSegments.forEach(h => {
    const drone1Points = paths[1];
    const drone2Points = paths[2];
    if (drone1Points[h.seg1] && drone1Points[h.seg1+1]) {
      const line = createSegmentLine(drone1Points[h.seg1], drone1Points[h.seg1+1], 0xef4444, 3);
      sceneGroup.add(line);
    }
    if (drone2Points[h.seg2] && drone2Points[h.seg2+1]) {
      const line = createSegmentLine(drone2Points[h.seg2], drone2Points[h.seg2+1], 0xef4444, 3);
      sceneGroup.add(line);
    }
  });
}

// ============================================================
// UI HELPERS
// ============================================================
function setDrone(n) {
  activeDrone = n;
  document.querySelectorAll('.btn-d').forEach(btn => btn.classList.remove('active'));
  document.getElementById(`btn-d${n}`).classList.add('active');
}

function updateZ() {
  currentZ = parseInt(document.getElementById("z-slider").value);
  document.getElementById("z-val").innerText = currentZ;
}

function stepZ(delta) {
  currentZ = Math.min(200, Math.max(0, currentZ + delta));
  document.getElementById("z-slider").value = currentZ;
  updateZ();
}

function addPointAtCurrentZ() {
  // Position par défaut au centre approximatif
  const last = paths[activeDrone][paths[activeDrone].length - 1];
  let x = 50, y = 50;
  if (last) {
    x = last[0] + (Math.random() - 0.5) * 20;
    y = last[1] + (Math.random() - 0.5) * 20;
  }
  x = Math.min(GRID_SIZE, Math.max(0, Math.round(x)));
  y = Math.min(GRID_SIZE, Math.max(0, Math.round(y)));
  paths[activeDrone].push([x, y, currentZ]);
  rebuildScene();
  updateUI();
}

function undoLast() {
  paths[activeDrone].pop();
  hitSegments = [];
  document.getElementById("result-area").style.display = "none";
  rebuildScene();
  updateUI();
}

function clearAll() {
  paths = { 1: [], 2: [], 3: [] };
  hitSegments = [];
  document.getElementById("result-area").style.display = "none";
  rebuildScene();
  updateUI();
}

function loadExample() {
  // Exemple avec Z variables pour montrer des segments 3D
  paths[1] = [
    [10, 30, 20],
    [30, 50, 40],
    [50, 50, 60],
    [70, 40, 50],
    [90, 20, 30]
  ];
  paths[2] = [
    [90, 70, 60],
    [70, 60, 50],
    [50, 50, 45],
    [30, 40, 50],
    [10, 20, 40]
  ];
  paths[3] = [
    [20, 20, 80],
    [50, 20, 85],
    [80, 20, 80]
  ];
  hitSegments = [];
  document.getElementById("result-area").style.display = "none";
  rebuildScene();
  updateUI();
}

function updateUI() {
  const container = document.getElementById("point-list");
  const allPoints = [];
  for (let d = 1; d <= 3; d++) {
    paths[d].forEach((p, i) => {
      allPoints.push(`<div class="point-item"><span style="color:#${COLORS[d].toString(16)}">D${d}[${i}]</span> (${p[0]}, ${p[1]}, ${p[2]})</div>`);
    });
  }
  container.innerHTML = allPoints.length ? allPoints.join("") : '<span style="color:#888">Aucun point</span>';
  
  const btn = document.getElementById("analyze-btn");
  btn.disabled = (paths[1].length < 2 || paths[2].length < 2);
}

function showResult(result) {
  const area = document.getElementById("result-area");
  const verdictDiv = document.getElementById("verdict");
  area.style.display = "block";
  
  if (result.collision) {
    verdictDiv.className = "verdict col";
    verdictDiv.innerHTML = "⚠️ COLLISION DÉTECTÉE ⚠️";
  } else {
    verdictDiv.className = "verdict safe";
    verdictDiv.innerHTML = "✅ PAS DE COLLISION ✅";
  }
  
  document.getElementById("seg-count").innerText = result.pairs_tested || result.segments_tested || "—";
  document.getElementById("time-ms").innerText = result.total_time_ms ? `${result.total_time_ms} ms` : "—";
  document.getElementById("ss-count").innerText = result.scheme_switches ? `${result.scheme_switches} SS` : "—";
  document.getElementById("mode-badge").innerHTML = result.mode === "fhe_reel" ? "<span style='background:#10b981;padding:2px 8px;border-radius:12px'>🔐 FHE réel</span>" : "<span style='background:#f59e0b;padding:2px 8px;border-radius:12px'>📊 Simulation</span>";
  
  if (result.details && result.details.length) {
    document.getElementById("collision-details").innerHTML = result.details.map(d => `🔴 ${d.pair || `D1[${d.seg1}] ∩ D2[${d.seg2}]`}`).join("<br>");
    hitSegments = result.details;
  } else {
    document.getElementById("collision-details").innerHTML = "";
    hitSegments = [];
  }
  rebuildScene();
}

// ============================================================
// ANALYSE FHE
// ============================================================
async function analyze() {
  const btn = document.getElementById("analyze-btn");
  btn.disabled = true;
  const originalText = btn.innerHTML;
  btn.innerHTML = "<span class='spin'></span> Calcul FHE en cours...";
  
  try {
    const response = await fetch(`${API}/api/analyze`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        path1: paths[1],
        path2: paths[2],
        path3: paths[3]
      }),
      signal: AbortSignal.timeout(300000)
    });
    
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || "Erreur serveur");
    showResult(data);
  } catch (err) {
    alert(`Erreur: ${err.message}\nAssurez-vous que python3 server.py est lancé`);
  } finally {
    btn.disabled = false;
    btn.innerHTML = originalText;
  }
}

// Redimensionnement
function resizeCanvas() {
  const container = document.querySelector(".canvas-panel");
  const width = container.clientWidth;
  const height = 550;
  renderer.setSize(width, height);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", resizeCanvas);
resizeCanvas();

// Animation
function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}
animate();

// Initialisation
updateUI();
rebuildScene();
setDrone(1);
</script>
</body>
</html>                if len(path1) < 2 or len(path2) < 2:
                    self._json({"error": "Minimum 2 points par drone"}, 400)
                    return

                paths = [path1, path2]
                if len(path3) >= 2:
                    paths.append(path3)

                result = analyze_paths(paths)
                self._json(result)

            except json.JSONDecodeError:
                self._json({"error": "JSON invalide"}, 400)
            except Exception as e:
                self._json({"error": str(e)}, 500)

        else:
            self._json({"error": "Not found"}, 404)

    def _json(self, data, code=200):
        body = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(code)
        self._cors()
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def log_message(self, fmt, *args):
        print(f"[http] {fmt % args}")


class ThreadedServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads      = True


def run_binary(paths):
    tmp_files = []
    try:
        for i, path in enumerate(paths):
            with tempfile.NamedTemporaryFile(mode='w', suffix='.txt',
                                             delete=False, prefix=f'drone{i+1}_') as f:
                line = "".join(
                    f"({int(p[0])},{int(p[1])},{int(p[2]) if len(p)>2 else 0})"
                    for p in path
                )
                f.write(line + "\n")
                tmp_files.append(f.name)

        cmd = [BINARY_PATH, "--path1", tmp_files[0], "--path2", tmp_files[1]]
        if len(tmp_files) >= 3:
            cmd += ["--path3", tmp_files[2]]

        t0 = time.time()
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        elapsed_ms = (time.time() - t0) * 1000

        if result.returncode != 0:
            return None, result.stderr

        for line in result.stdout.splitlines():
            if line.startswith("JSON_RESULT:"):
                data = json.loads(line[len("JSON_RESULT:"):])
                data["mode"] = "fhe_reel"
                data["total_time_ms"] = round(elapsed_ms, 1)
                return data, None

        return None, "Pas de JSON_RESULT"

    finally:
        for f in tmp_files:
            try:
                os.unlink(f)
            except:
                pass


def analyze_paths(paths):
    total_collisions = 0
    n_seg_total = 0
    all_hits = []

    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            n_seg_total += (len(paths[i]) - 1) * (len(paths[j]) - 1)

    if os.path.isfile(BINARY_PATH):
        data, error = run_binary(paths)
        if data:
            return data

    metrics = simulate_metrics(n_seg_total)
    return {
        "mode": "simulation",
        "collision": False,
        "scheme_switches": metrics["scheme_switches"],
        "ciphertext_size_kb": metrics["ciphertext_size_kb"],
        "time_ms": metrics["time_ms"],
    }


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    print(f"Drone HE server running on http://localhost:{PORT}")

    with ThreadedServer(("", PORT), Handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServeur arrete.")
