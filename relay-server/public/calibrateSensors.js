function submitCalibration(e) {
    e.preventDefault(); 
    
    const deviceId = document.getElementById('deviceId').value.trim();
    if (!deviceId) return alert("Please enter a Target Device ID!");

    // Helper function to safely extract numbers
    const getVal = (id) => {
    const val = document.getElementById(id).value;
    return val !== "" ? parseFloat(val) : undefined;
    };

    const payload = {};
    
    // Only add values if they were actually typed in
    if (getVal('seaLevelhPA') !== undefined) payload.seaLevelhPA = getVal('seaLevelhPA');
    if (getVal('latitude') !== undefined) payload.latitude = getVal('latitude');
    if (getVal('longitude') !== undefined) payload.longitude = getVal('longitude');
    if (getVal('altitude') !== undefined) payload.altitude = getVal('altitude');
    if (getVal('pitch') !== undefined) payload.pitch = getVal('pitch');
    if (getVal('roll') !== undefined) payload.roll = getVal('roll');
    if (getVal('yaw') !== undefined) payload.yaw = getVal('yaw');
    
    payload.useHardcodedPosition = document.getElementById('useHardcodedPosition').checked;
    payload.useHardcodedAltitude = document.getElementById('useHardcodedAltitude').checked;
    payload.useHardcodedOrientation = document.getElementById('useHardcodedOrientation').checked;
    payload.useHardcodeYaw = document.getElementById('useHardcodeYaw').checked;

    // Send POST request to your EC2 endpoint
    fetch('/' + deviceId + '/calibrate', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload)
    })
    .then(response => response.json())
    .then(data => {
    if (data.error) alert('Error: ' + data.error);
    else alert(data.message || 'Calibration sent successfully!');
    })
    .catch(err => {
    console.error('Error:', err);
    alert('Failed to send calibration. Check server connection.');
    });
}

function clearCalibration(e) {
    e.preventDefault();
    
    const deviceId = document.getElementById('deviceId').value.trim();
    if (!deviceId) return alert("Please enter a Target Device ID!");
    
    // Add a safety check so you don't accidentally click it
    if (!confirm('Are you sure you want to wipe the calibration memory on ' + deviceId + '?')) return;

    // Send POST request to the new /clear endpoint
    fetch('/' + deviceId + '/clear', {
    method: 'POST'
    })
    .then(response => response.json())
    .then(data => {
    if (data.error) alert('Error: ' + data.error);
    else alert(data.message || 'Memory cleared successfully!');
    
    // Optional: clear the form inputs
    document.getElementById('calForm').reset(); 
    })
    .catch(err => {
    console.error('Error:', err);
    alert('Failed to send clear command. Check connection.');
    });
}