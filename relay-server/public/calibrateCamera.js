let cvReady = false;
let currentMode = 'IDLE'; 
let pendingPixel = null;
let pointPairs = [];

// Initialize Leaflet Map
let map = L.map('map').setView([0, 0], 0); // Default map view
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);

// Origin for all devices to calibrate to
let mathOrigin = null;

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
  
  // Calculate exact pixel coordinates based on the image's intrinsic size
  const scaleX = img.naturalWidth / rect.width;
  const scaleY = img.naturalHeight / rect.height;

  pendingPixel = {
    u: Math.round((e.clientX - rect.left) * scaleX),
    v: Math.round((e.clientY - rect.top) * scaleY)
  };

  // Draw visual marker on image
  const marker = document.createElement('div');
  marker.className = 'img-marker';
  marker.style.left = (e.clientX - rect.left) + 'px';
  marker.style.top = (e.clientY - rect.top) + 'px';
  document.getElementById('imageContainer').appendChild(marker);

  document.getElementById('status').innerText = 'Now click the exact SAME feature on the map.';
  currentMode = 'WAITING_MAP_CLICK';
}

// Handle Map Clicks (3D World Coordinates)
map.on('click', function(e) {
  if (currentMode !== 'WAITING_MAP_CLICK') return;
  
  let altInput = prompt("Elevation of this specific feature (meters above ground):", "0");
  if (altInput === null) return;

  if (!mathOrigin) {
    mathOrigin = {
      lat: e.latlng.lat,
      lon: e.latlng.lng,
      alt: parseFloat(altInput)
    };
    // Add marker to show where world origin is
    L.marker([mathOrigin.lat, mathOrigin.lon]).addTo(map)
     .bindPopup('World Origin (0,0,0)').openPopup();
  }

  // Convert Map Lat/Lon to Local Cartesian (X,Y in meters) relative to Fleet Origin
  const R = 6378137; 
  const dLat = (e.latlng.lat - mathOrigin.lat) * Math.PI / 180;
  const dLon = (e.latlng.lng - mathOrigin.lon) * Math.PI / 180;
  
  const pair = {
    id: pointPairs.length + 1,
    pixel: pendingPixel,
    cartesian: {
      x: R * dLon * Math.cos(mathOrigin.lat * Math.PI / 180),
      y: R * dLat,
      z: parseFloat(altInput) - mathOrigin.alt
    }
  };
  
  pointPairs.push(pair);
  
  // Draw visual marker on map
  L.marker([e.latlng.lat, e.latlng.lng]).addTo(map).bindPopup('Point ' + pair.id);
  updateTable();

  document.getElementById('status').innerText = 'Pair saved! Click image again or run solvePnP.';
  currentMode = 'WAITING_IMAGE_CLICK';
  
  if (pointPairs.length >= 4 && cvReady) {
    document.getElementById('runPnpBtn').disabled = false;
  }
});

// Helper to redraw the UI table
function updateTable() {
  const table = document.getElementById('pointsTable');
  while (table.rows.length > 1) { table.deleteRow(1); } // Clear old rows

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
  pointPairs.forEach(p => {
    objPointsArray.push(p.cartesian.x, p.cartesian.y, p.cartesian.z);
    imgPointsArray.push(p.pixel.u, p.pixel.v);
  });

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

  // Construct the matrices for OpenCV
  let camMatrix = cv.matFromArray(3, 3, cv.CV_64F, [
    fx,  0, cx,
     0, fy, cy,
     0,  0,  1
  ]);
  
  // Use the 5-parameter distortion array standard for OpenCV checkerboard calibration
  let distCoeffs = cv.matFromArray(5, 1, cv.CV_64F, [k1, k2, p1, p2, k3]);
  
  let rvec = new cv.Mat(), tvec = new cv.Mat();

  // Run the OpenCV Solver
  if (cv.solvePnP(objPoints, imgPoints, camMatrix, distCoeffs, rvec, tvec, false, cv.SOLVEPNP_ITERATIVE)) {
    
    // Calculate Camera Orientation (Pitch, Roll, Yaw)
    let R = new cv.Mat();
    cv.Rodrigues(rvec, R);

    let sy = Math.sqrt(R.data64F[0] * R.data64F[0] + R.data64F[3] * R.data64F[3]);
    let pitch, roll, yaw;
    
    if (sy > 1e-6) {
        roll = Math.atan2(R.data64F[7], R.data64F[8]);
        pitch = Math.atan2(-R.data64F[6], sy);
        yaw = Math.atan2(R.data64F[3], R.data64F[0]);
    } else {
        roll = Math.atan2(-R.data64F[5], R.data64F[4]);
        pitch = Math.atan2(-R.data64F[6], sy);
        yaw = 0;
    }

    // Convert Radians to Degrees
    pitch = pitch * (180 / Math.PI);
    roll = roll * (180 / Math.PI);
    yaw = yaw * (180 / Math.PI);

    // Calculate exact camera position (X, Y, Z meters)
    // Formula: C = -R^T * tvec
    let R_t = new cv.Mat();
    cv.transpose(R, R_t); 
    
    let cameraPos = new cv.Mat();
    cv.gemm(R_t, tvec, -1, new cv.Mat(), 0, cameraPos); 

    let camX = cameraPos.data64F[0];
    let camY = cameraPos.data64F[1];
    let camZ = cameraPos.data64F[2];

    // Convert local (x, y, z) back to (lon, lat, height) 
    const R_EARTH = 6378137; 
    const originLatRad = mathOrigin.lat * (Math.PI / 180); 

    const dLatRad = camY / R_EARTH;
    const dLonRad = camX / (R_EARTH * Math.cos(originLatRad));

    const finalLat = mathOrigin.lat + (dLatRad * (180 / Math.PI));
    const finalLon = mathOrigin.lon + (dLonRad * (180 / Math.PI));
    const finalAlt = mathOrigin.alt + camZ;

    // Send calibrated parameters to device
    const payload = {
        pitch: parseFloat(pitch.toFixed(2)),
        roll: parseFloat(roll.toFixed(2)),
        yaw: parseFloat(yaw.toFixed(2)),
        useHardcodedOrientation: true,
        
        latitude: parseFloat(finalLat.toFixed(6)),
        longitude: parseFloat(finalLon.toFixed(6)),
        useHardcodedPosition: true,
        
        altitude: parseFloat(finalAlt.toFixed(2)), 
        useHardcodedAltitude: true
    };

    fetch('/' + deviceId + '/calibrate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    })
    .then(res => res.json())
    .then(data => {
        alert(`Calibration Complete!\n\nPos: Lat ${payload.latitude}, Lon ${payload.longitude}, Alt ${payload.altitude}m\nAngles: Pitch ${payload.pitch}°, Roll ${payload.roll}°, Yaw ${payload.yaw}°`);
        
        // Clean up the image markers for the next run
        document.querySelectorAll('.img-marker').forEach(e => e.remove());
    })
    .catch(err => alert("Failed to send to device: " + err));

    R.delete(); R_t.delete(); cameraPos.delete();
  } else {
    alert("solvePnP failed. Try selecting points that are spread further apart.");
  }

  // WebAssembly cleanup
  objPoints.delete(); imgPoints.delete(); camMatrix.delete(); distCoeffs.delete(); rvec.delete(); tvec.delete();
}