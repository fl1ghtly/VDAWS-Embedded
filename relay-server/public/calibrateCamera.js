let cvReady = false;
let currentMode = 'IDLE'; 
let pendingPixel = null;
let pendingImgMarker = null; // Track the half-step image marker
let pointPairs = [];

// Origin for all devices to calibrate to
let mathOrigin = null;
let worldOriginMarker = null; // Track the origin marker so we can undo it

let streetLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 });
let satelliteLayer = L.tileLayer('https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', { maxZoom: 22 });
  
let map = L.map('map', {
    center: [0, 0],
    zoom: 0,
    layers: [streetLayer]
});

// Create a dictionary of your base maps
let baseMaps = {
    "Satellite View": satelliteLayer,
    "Street View": streetLayer
};

// Add the toggle control UI to the top right of the map
L.control.layers(baseMaps).addTo(map);

function onOpenCvReady() {
  cvReady = true;
  document.getElementById('status').innerText = '✅ Ready. Enter Device ID and fetch image.';
  document.getElementById('status').style.color = '#a6e3a1';
}

async function fetchDeviceData() {
  const deviceId = document.getElementById('deviceId').value.trim();
  if (!deviceId) return alert("Enter a Device ID");
  document.getElementById('status').innerText = 'Fetching Image...';

  try {
    // Fetch the image from the device
    const imgRes = await fetch('/' + deviceId + '/capture');
    if (!imgRes.ok) throw new Error("Device offline or camera failed.");
    
    const blob = await imgRes.blob();
    document.getElementById('deviceImage').src = URL.createObjectURL(blob);

    // Update UI State
    document.getElementById('status').innerText = '✅ Click a distinct feature on the image.';
    document.getElementById('status').style.color = '#a6e3a1';
    currentMode = 'WAITING_IMAGE_CLICK';
    
    // Reset our working arrays for a fresh calculation
    pointPairs = [];
    updateTable();

  } catch (err) {
    alert(err.message);
    document.getElementById('status').innerText = '❌ Error fetching image.';
    document.getElementById('status').style.color = '#f38ba8';
  }
}

// Handle Image Clicks (2D Pixels)
function handleImageClick(e) {
  if (currentMode !== 'WAITING_IMAGE_CLICK') return;
  
  const img = document.getElementById('deviceImage');
  const rect = img.getBoundingClientRect();
  const scaleX = img.naturalWidth / rect.width;
  const scaleY = img.naturalHeight / rect.height;

  pendingPixel = {
    u: Math.round((e.clientX - rect.left) * scaleX),
    v: Math.round((e.clientY - rect.top) * scaleY)
  };

  // Create and save a reference to the image marker
  pendingImgMarker = document.createElement('div');
  pendingImgMarker.className = 'img-marker';
  pendingImgMarker.style.left = (e.clientX - rect.left) + 'px';
  pendingImgMarker.style.top = (e.clientY - rect.top) + 'px';
  document.getElementById('imageContainer').appendChild(pendingImgMarker);

  document.getElementById('status').innerText = 'Now click the exact SAME feature on the map.';
  currentMode = 'WAITING_MAP_CLICK';
}

// Handle Map Clicks (3D World Coordinates)
map.on('click', function(e) {
  if (currentMode !== 'WAITING_MAP_CLICK') return;
  
  // Default to 0 for the IPPE method
  let altInput = prompt("Elevation of this specific feature (meters above ground):", "0");
  if (altInput === null) return;

  if (!mathOrigin) {
    mathOrigin = { lat: e.latlng.lat, lon: e.latlng.lng, alt: parseFloat(altInput) };
    // Save the origin marker reference. 
    worldOriginMarker = L.marker([mathOrigin.lat, mathOrigin.lon], { interactive: false })
      .addTo(map).bindPopup('World Origin (0,0,0)').openPopup();
  }

  const R = 6378137; 
  const dLat = (e.latlng.lat - mathOrigin.lat) * Math.PI / 180;
  const dLon = (e.latlng.lng - mathOrigin.lon) * Math.PI / 180;
  
  // Create a click-through map marker
  let newMapMarker = L.marker([e.latlng.lat, e.latlng.lng], { interactive: false }).addTo(map);

  const pair = {
    id: pointPairs.length + 1,
    pixel: pendingPixel,
    cartesian: {
      x: R * dLon * Math.cos(mathOrigin.lat * Math.PI / 180),
      y: R * dLat,
      z: parseFloat(altInput) - mathOrigin.alt
    },
    imgMarker: pendingImgMarker, // Save the visual dot
    mapMarker: newMapMarker      // Save the visual pin
  };
  
  pointPairs.push(pair);
  updateTable();

  document.getElementById('status').innerText = 'Pair saved! Click image again or run solvePnP.';
  currentMode = 'WAITING_IMAGE_CLICK';
  pendingImgMarker = null; // Clear pending for the next pair
  
  if (pointPairs.length >= 4 && cvReady) {
    document.getElementById('runPnpBtn').disabled = false;
  }
});

function undoLastAction() {
  if (currentMode === 'WAITING_MAP_CLICK') {
    // 1. User clicked the image, but hasn't clicked the map yet. Undo the image click.
    if (pendingImgMarker) pendingImgMarker.remove();
    pendingImgMarker = null;
    pendingPixel = null;
    
    document.getElementById('status').innerText = '✅ Undid image point. Click a feature on the image.';
    currentMode = 'WAITING_IMAGE_CLICK';
    
  } else if (currentMode === 'WAITING_IMAGE_CLICK' && pointPairs.length > 0) {
    // 2. User completely saved a pair. Undo both the map pin and image dot.
    const lastPair = pointPairs.pop();
    
    // Remove visuals
    lastPair.imgMarker.remove();
    map.removeLayer(lastPair.mapMarker);
    
    // If we just deleted the very first point, we need to completely wipe the World Origin!
    if (pointPairs.length === 0) {
      mathOrigin = null;
      if (worldOriginMarker) {
        map.removeLayer(worldOriginMarker);
        worldOriginMarker = null;
      }
    }
    
    updateTable();
    document.getElementById('status').innerText = `✅ Point ${lastPair.id} removed. Click image.`;
    
    if (pointPairs.length < 4) {
      document.getElementById('runPnpBtn').disabled = true;
    }
  }
}

// Helper to redraw the UI table
function updateTable() {
  const table = document.getElementById('pointsTable');
  while (table.rows.length > 1) { table.deleteRow(1); }

  pointPairs.forEach(p => {
    const row = table.insertRow();
    row.insertCell(0).innerText = p.id;
    row.insertCell(1).innerText = '(' + p.pixel.u + ', ' + p.pixel.v + ')';
    row.insertCell(2).innerText = '(' + p.cartesian.x.toFixed(2) + 'm, ' + p.cartesian.y.toFixed(2) + 'm, ' + p.cartesian.z.toFixed(2) + 'm)';
  });
}

function runSolvePnP() {
  if (pointPairs.length < 4) return alert("You need at least 4 points!");
  const deviceId = document.getElementById('deviceId').value.trim();
  
  let objPointsArray = [], imgPointsArray = [];
  
  // Prevent NaN values from crashing WebAssembly
  for (let p of pointPairs) {
    if (isNaN(p.cartesian.x) || isNaN(p.cartesian.y) || isNaN(p.cartesian.z) || isNaN(p.pixel.u) || isNaN(p.pixel.v)) {
        return alert(`Error: Point ${p.id} has invalid math data (NaN). You likely left an elevation prompt blank. Please refresh the page and try again.`);
    }
    objPointsArray.push(p.cartesian.x, p.cartesian.y, p.cartesian.z);
    imgPointsArray.push(p.pixel.u, p.pixel.v);
  }

  let objPoints = cv.matFromArray(pointPairs.length, 1, cv.CV_32FC3, objPointsArray);
  let imgPoints = cv.matFromArray(pointPairs.length, 1, cv.CV_32FC2, imgPointsArray);

  // Read dynamic camera parameters from the UI
  const fx = parseFloat(document.getElementById('fx').value) || 720.0;
  const fy = parseFloat(document.getElementById('fy').value) || 720.0;
  const cx = parseFloat(document.getElementById('cx').value) || 160.0;
  const cy = parseFloat(document.getElementById('cy').value) || 120.0;

  const k1 = parseFloat(document.getElementById('k1').value) || 0;
  const k2 = parseFloat(document.getElementById('k2').value) || 0;
  const p1 = parseFloat(document.getElementById('p1').value) || 0;
  const p2 = parseFloat(document.getElementById('p2').value) || 0;
  const k3 = parseFloat(document.getElementById('k3').value) || 0;

  let camMatrix = cv.matFromArray(3, 3, cv.CV_64F, [
    fx,  0, cx,
     0, fy, cy,
     0,  0,  1
  ]);
  
  let distCoeffs = cv.matFromArray(5, 1, cv.CV_64F, [k1, k2, p1, p2, k3]);
  let rvec = new cv.Mat(), tvec = new cv.Mat();

  // Wrap in try/catch to decode C++ Pointers
  try {
    if (cv.solvePnP(objPoints, imgPoints, camMatrix, distCoeffs, rvec, tvec, false, cv.SOLVEPNP_IPPE)) {
      
      let R = new cv.Mat();
      cv.Rodrigues(rvec, R);

      let R_t = new cv.Mat();
      cv.transpose(R, R_t); 

      let cameraPos = new cv.Mat();
      cv.gemm(R_t, tvec, -1, new cv.Mat(), 0, cameraPos); 
      
      let Fx = R_t.data64F[2],  Fy = R_t.data64F[5],  Fz = R_t.data64F[8];  // Forward Vector (+Z cv)
      let Rx = R_t.data64F[0],  Ry = R_t.data64F[3],  Rz = R_t.data64F[6];  // Right Vector (+X cv)
      let Ux = -R_t.data64F[1], Uy = -R_t.data64F[4], Uz = -R_t.data64F[7]; // Up Vector (-Y cv)

      // Calculate Pitch, Roll, Yaw relative to a standard XYZ grid
      // Yaw: Direction Forward vector is pointing on the XY plane (0° = East, 90° = North)
      let yaw = Math.atan2(Fy, Fx) * (180 / Math.PI); 
      
      // Pitch: How far the Forward vector is looking up/down from the horizon
      let pitch = Math.asin(Fz) * (180 / Math.PI);
      pitch = pitch - 90;
      
      // Roll: Rotation around the Forward vector (Bank angle)
      let roll = Math.atan2(-Rz, Uz) * (180 / Math.PI);

      let camX = cameraPos.data64F[0];  
      let camY = cameraPos.data64F[1];
      let camZ = cameraPos.data64F[2];

      const R_EARTH = 6378137; 
      const originLatRad = mathOrigin.lat * (Math.PI / 180); 

      const dLatRad = camY / R_EARTH;
      const dLonRad = camX / (R_EARTH * Math.cos(originLatRad));

      const finalLat = mathOrigin.lat + (dLatRad * (180 / Math.PI));
      const finalLon = mathOrigin.lon + (dLonRad * (180 / Math.PI));
      const finalAlt = mathOrigin.alt + camZ;

      const imgElement = document.getElementById('deviceImage');
      const imgWidth = imgElement ? imgElement.naturalWidth : 320;
      const fovDeg = 2 * Math.atan(imgWidth / (2 * fx)) * (180 / Math.PI);

      const payload = {
          pitch: parseFloat(pitch),
          roll: parseFloat(roll),
          yaw: parseFloat(yaw),
          useHardcodedOrientation: true,
          
          latitude: parseFloat(finalLat),
          longitude: parseFloat(finalLon),
          useHardcodedPosition: true,
          
          altitude: parseFloat(finalAlt), 
          useHardcodedAltitude: true,

          fov: parseFloat(fovDeg)
      };

      fetch('/' + deviceId + '/calibrate', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload)
      })
      .then(res => res.json())
      .then(data => {
          alert(`Calibration Complete!\n\nPos: Lat ${payload.latitude}, Lon ${payload.longitude}, Alt ${payload.altitude}m\nAngles: P ${payload.pitch}°, R ${payload.roll}°, Y ${payload.yaw}°\nFOV: ${payload.fov}°`);
          document.querySelectorAll('.img-marker').forEach(e => e.remove());
      })
      .catch(err => alert("Failed to send to device: " + err));

      R.delete(); R_t.delete(); cameraPos.delete();
    } else {
      alert("solvePnP failed. Try selecting points that are spread further apart.");
    }
  } catch (err) {
    let errorMsg = "Unknown WebAssembly Error";
    if (typeof err === 'number') {
        errorMsg = cv.exceptionFromPtr(err).msg;
    } else {
        errorMsg = err.message || err;
    }
    alert("OpenCV C++ Crash: " + errorMsg);
    console.error("OpenCV Stack Trace:", errorMsg);
  } finally {
    objPoints.delete(); imgPoints.delete(); camMatrix.delete(); distCoeffs.delete(); rvec.delete(); tvec.delete();
  }
}