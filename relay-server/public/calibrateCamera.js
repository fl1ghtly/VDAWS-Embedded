let cvReady = false;
let map = L.map('map').setView([0, 0], 2);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);

let deviceOrigin = null; 
let currentMode = 'IDLE'; 
let pendingPixel = null;
let pointPairs = [];

function onOpenCvReady() {
    cvReady = true;
    document.getElementById('status').innerText = '✅ Ready. Enter Device ID and fetch data.';
    document.getElementById('status').style.color = '#a6e3a1';
}

async function fetchDeviceData() {
    const deviceId = document.getElementById('deviceId').value.trim();
    if (!deviceId) return alert("Enter a Device ID");
    document.getElementById('status').innerText = 'Fetching...';

    try {
    const sensorRes = await fetch('/' + deviceId + '/sensors');
    if (!sensorRes.ok) throw new Error("Device offline.");
    const sensorData = await sensorRes.json();

    deviceOrigin = {
        lat: sensorData.position.latitude,
        lon: sensorData.position.longitude,
        alt: sensorData.position.altitude || 0
    };

    map.setView([deviceOrigin.lat, deviceOrigin.lon], 19);
    L.marker([deviceOrigin.lat, deviceOrigin.lon]).addTo(map).bindPopup('Device (0,0,0)');

    const imgRes = await fetch('/' + deviceId + '/capture');
    const blob = await imgRes.blob();
    document.getElementById('deviceImage').src = URL.createObjectURL(blob);

    document.getElementById('status').innerText = '✅ Click a point on the image.';
    currentMode = 'WAITING_IMAGE_CLICK';
    } catch (err) {
    alert(err.message);
    }
}

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

    const marker = document.createElement('div');
    marker.className = 'img-marker';
    marker.style.left = (e.clientX - rect.left) + 'px';
    marker.style.top = (e.clientY - rect.top) + 'px';
    document.getElementById('imageContainer').appendChild(marker);

    document.getElementById('status').innerText = 'Now click the exact SAME feature on the map.';
    currentMode = 'WAITING_MAP_CLICK';
}

map.on('click', function(e) {
    if (currentMode !== 'WAITING_MAP_CLICK') return;
    
    let altInput = prompt("Elevation of this point (meters from device level):", "0");
    if (altInput === null) return;

    const R = 6378137; 
    const dLat = (e.latlng.lat - deviceOrigin.lat) * Math.PI / 180;
    const dLon = (e.latlng.lng - deviceOrigin.lon) * Math.PI / 180;
    
    const pair = {
    id: pointPairs.length + 1,
    pixel: pendingPixel,
    cartesian: {
        x: R * dLon * Math.cos(deviceOrigin.lat * Math.PI / 180),
        y: R * dLat,
        z: parseFloat(altInput) - deviceOrigin.alt
    }
    };
    pointPairs.push(pair);
    
    L.marker([e.latlng.lat, e.latlng.lng]).addTo(map).bindPopup('P' + pair.id);
    
    const table = document.getElementById('pointsTable');
    const row = table.insertRow();
    row.insertCell(0).innerText = pair.id;
    row.insertCell(1).innerText = '(' + pair.pixel.u + ', ' + pair.pixel.v + ')';
    row.insertCell(2).innerText = '(' + pair.cartesian.x.toFixed(2) + ', ' + pair.cartesian.y.toFixed(2) + ', ' + pair.cartesian.z.toFixed(2) + ')';

    document.getElementById('status').innerText = 'Pair saved. Click image again or run solvePnP.';
    currentMode = 'WAITING_IMAGE_CLICK';
    if (pointPairs.length >= 4) document.getElementById('runPnpBtn').disabled = false;
});

function runSolvePnP() {
    const deviceId = document.getElementById('deviceId').value.trim();
    
    let objPointsArray = [], imgPointsArray = [];
    pointPairs.forEach(p => {
    objPointsArray.push(p.cartesian.x, p.cartesian.y, p.cartesian.z);
    imgPointsArray.push(p.pixel.u, p.pixel.v);
    });

    let objPoints = cv.matFromArray(pointPairs.length, 1, cv.CV_32FC3, objPointsArray);
    let imgPoints = cv.matFromArray(pointPairs.length, 1, cv.CV_32FC2, imgPointsArray);

    // Placeholder Intrinsic Matrix
    let camMatrix = cv.matFromArray(3, 3, cv.CV_64F, [300, 0, 160, 0, 300, 120, 0, 0, 1]);
    let distCoeffs = cv.matFromArray(4, 1, cv.CV_64F, [0, 0, 0, 0]);
    let rvec = new cv.Mat(), tvec = new cv.Mat();

    if (cv.solvePnP(objPoints, imgPoints, camMatrix, distCoeffs, rvec, tvec, false, cv.SOLVEPNP_ITERATIVE)) {
    
    // Convert Rodrigues rotation vector to a 3x3 Rotation Matrix
    let R = new cv.Mat();
    cv.Rodrigues(rvec, R);

    // Extract Euler Angles (Pitch, Roll, Yaw) from Rotation Matrix
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

    // Send payload to existing calibration endpoint
    const payload = {
        pitch: parseFloat(pitch.toFixed(2)),
        roll: parseFloat(roll.toFixed(2)),
        yaw: parseFloat(yaw.toFixed(2)),
        useHardcodedOrientation: true
    };

    fetch('/' + deviceId + '/calibrate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
    })
    .then(res => res.json())
    .then(data => alert("Success! Sent to device:\\nPitch: " + payload.pitch + "\\nRoll: " + payload.roll + "\\nYaw: " + payload.yaw))
    .catch(err => alert("Failed to send to device: " + err));

    R.delete();
    } else {
    alert("solvePnP failed. Pick better points.");
    }

    objPoints.delete(); imgPoints.delete(); camMatrix.delete(); distCoeffs.delete(); rvec.delete(); tvec.delete();
}