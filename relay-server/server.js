const express = require('express');
const WebSocket = require('ws');
const http = require('http');

const app = express();
app.use(express.json());
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

const devices = {};

wss.on('connection', (ws) => {
    console.log('New device connected');

    ws.on('message', (message, isBinary) => {
        if (!isBinary) {
            const text = message.toString();
            if (text.startsWith('AUTH:')) {
                const deviceId = text.split(':')[1];
                devices[deviceId] = ws;
                ws.deviceId = deviceId;
                console.log(`Device ${deviceId} registered.`);
            }
        }
    });

    ws.on('close', () => {
        if (ws.deviceId && devices[ws.deviceId]) {
            console.log(`Device ${ws.deviceId} disconnected.`);
            delete devices[ws.deviceId];
        }
    });
});

app.get('/:deviceId/capture', (req, res) => {
    const deviceId = req.params.deviceId;
    const deviceSocket = devices[deviceId];

    if (!deviceSocket || deviceSocket.readyState !== WebSocket.OPEN) {
        return res.status(404).send(`Device ${deviceId} is offline.`);
    }

    deviceSocket.send("CAPTURE");

    const onMessage = (message, isBinary) => {
        if (isBinary) {
            res.set('Content-Type', 'image/jpeg');
            res.send(message);
            deviceSocket.removeListener('message', onMessage);
        }
    };

    deviceSocket.on('message', onMessage);

    setTimeout(() => {
        if (!res.headersSent) {
            res.status(504).send("Camera capture timeout.");
            deviceSocket.removeListener('message', onMessage);
        }
    }, 5000);
});

app.get('/:deviceId/sensors', (req, res) => {
    const deviceId = req.params.deviceId;
    const deviceSocket = devices[deviceId];

    if (!deviceSocket || deviceSocket.readyState !== WebSocket.OPEN) {
        return res.status(404).send(`Device ${deviceId} is offline.`);
    }

    deviceSocket.send("SENSORS");

    const onMessage = (message, isBinary) => {
        if (!isBinary) {
            const text = message.toString();
            if (!text.startsWith('AUTH:')) {
                res.set('Content-Type', 'application/json');
                res.send(text);
                deviceSocket.removeListener('message', onMessage);
            }
        }
    };

    deviceSocket.on('message', onMessage);

    setTimeout(() => {
        if (!res.headersSent) {
            res.status(504).send("Sensor read timeout.");
            deviceSocket.removeListener('message', onMessage);
        }
    }, 5000);
});

app.post('/:deviceId/calibrate', (req, res) => {
    const deviceId = req.params.deviceId;
    const deviceSocket = devices[deviceId];

    // Check if the device is actually connected
    if (!deviceSocket || deviceSocket.readyState !== WebSocket.OPEN) {
        return res.status(404).json({ error: `Device ${deviceId} is offline.` });
    }

    // Convert the incoming JSON body back to a string and attach a "CALIBRATE:" prefix
    const payloadString = "CALIBRATE:" + JSON.stringify(req.body);
    
    // Send it down the WebSocket to the ESP32
    deviceSocket.send(payloadString);

    // Immediately tell your browser/script that the command was sent
    res.json({ status: "success", message: `Calibration sent to ${deviceId}` });
});

// Serve the Calibration Web Dashboard
app.get('/calibrate', (req, res) => {
    res.send(`
    <!DOCTYPE html>
    <html>
    <head>
      <title>Fleet Calibration Dashboard</title>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #1e1e2e; color: #cdd6f4; margin: 0; padding: 20px; display: flex; justify-content: center; }
        .container { background: #313244; padding: 30px; border-radius: 12px; box-shadow: 0 8px 16px rgba(0,0,0,0.4); max-width: 500px; width: 100%; }
        h1, h2, h3 { color: #89b4fa; text-align: center; margin-top: 0; }
        .section { background: #181825; padding: 15px; border-radius: 8px; margin-bottom: 20px; }
        .form-group { margin-bottom: 15px; display: flex; justify-content: space-between; align-items: center; }
        label { font-weight: bold; font-size: 14px; }
        input[type="text"], input[type="number"] { width: 50%; padding: 8px; background: #45475a; border: 1px solid #585b70; color: #cdd6f4; border-radius: 4px; outline: none; }
        input[type="checkbox"] { transform: scale(1.2); }
        button { width: 100%; background: #a6e3a1; color: #11111b; font-weight: bold; border: none; padding: 12px; border-radius: 6px; cursor: pointer; font-size: 16px; transition: background 0.2s; }
        button:hover { background: #94e2d5; }
      </style>
    </head>
    <body>
      <div class="container">
        <h1>⚙️ Device Calibration</h1>
        
        <form id="calForm" onsubmit="submitCalibration(event)">
          <div class="section">
            <div class="form-group">
              <label>Target Device ID:</label>
              <input type="text" id="deviceId" placeholder="Device ID" required>
            </div>
          </div>

          <div class="section">
            <div class="form-group">
              <label>Sea Level Pressure (hPa):</label>
              <input type="number" step="any" id="seaLevelhPA" placeholder="e.g. 1013.25">
            </div>
          </div>
          
          <div class="section">
            <h3 style="font-size: 16px; margin-bottom: 15px;">Position Overrides</h3>
            <div class="form-group">
              <label>Use Hardcoded Position:</label>
              <input type="checkbox" id="useHardcodedPosition">
            </div>
            <div class="form-group">
              <label>Latitude:</label>
              <input type="number" step="any" id="latitude">
            </div>
            <div class="form-group">
              <label>Longitude:</label>
              <input type="number" step="any" id="longitude">
            </div>
          </div>
          
          <div class="section">
            <h3 style="font-size: 16px; margin-bottom: 15px;">Orientation Overrides</h3>
            <div class="form-group">
              <label>Use Hardcoded Orientation:</label>
              <input type="checkbox" id="useHardcodedOrientation">
            </div>
            <div class="form-group">
              <label>Use Hardcoded Yaw (Only):</label>
              <input type="checkbox" id="useHardcodeYaw">
            </div>
            <div class="form-group">
              <label>Pitch:</label>
              <input type="number" step="any" id="pitch">
            </div>
            <div class="form-group">
              <label>Roll:</label>
              <input type="number" step="any" id="roll">
            </div>
            <div class="form-group">
              <label>Yaw:</label>
              <input type="number" step="any" id="yaw">
            </div>
          </div>
          
          <button type="submit">Deploy Calibration to Device</button>
        </form>

        <script>
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
          if (getVal('pitch') !== undefined) payload.pitch = getVal('pitch');
          if (getVal('roll') !== undefined) payload.roll = getVal('roll');
          if (getVal('yaw') !== undefined) payload.yaw = getVal('yaw');
          
          payload.useHardcodedPosition = document.getElementById('useHardcodedPosition').checked;
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
        </script>
      </div>
    </body>
    </html>
    `);
});

server.listen(80, () => {
    console.log('Relay server running on port 80');
});
