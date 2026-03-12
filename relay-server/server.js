const express = require('express');
const WebSocket = require('ws');
const http = require('http');
const path = require('path');

const app = express();
app.use(express.json());
app.use(express.static(path.join(__dirname, 'public')));
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

const devices = {};
// Buffer to hold latest image from each device
const latestImages = {};

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
        } else {
            if (ws.deviceId) {
                latestImages[ws.deviceId] = message;
            }
        }
    });

    ws.on('close', () => {
        if (ws.deviceId) {
            console.log(`Device ${ws.deviceId} disconnected.`);
            delete devices[ws.deviceId];
            delete latestImages[ws.deviceId];
        }
    });
});

app.get('/:deviceId/capture', (req, res) => {
    const deviceId = req.params.deviceId;

    // Instantly check the RAM cache for the latest pushed image
    if (latestImages[deviceId]) {
        res.set('Content-Type', 'image/jpeg');
        return res.send(latestImages[deviceId]);
    }
    
    // If no image is in the cache yet, return an error instantly
    res.status(404).send(`No image available for ${deviceId} yet. It may still be booting.`);
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

// Endpoint to calibrate camera using OpenCV's PnP
app.get('/calibrateCamera', (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'calibrateCamera.html'));
});

// POST endpoint to clear calibration
app.post('/:deviceId/clear', (req, res) => {
    const deviceId = req.params.deviceId;
    const deviceSocket = devices[deviceId];

    // Check if the drone is connected
    if (!deviceSocket || deviceSocket.readyState !== WebSocket.OPEN) {
        return res.status(404).json({ error: `Device ${deviceId} is offline.` });
    }

    // Send the CLEAR command down the WebSocket
    deviceSocket.send("CLEAR");

    res.json({ status: "success", message: `Memory erased on ${deviceId}` });
});

// Serve the Calibration Sensors Web Dashboard
app.get('/calibrate', (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'calibrateSensors.html'));
});

server.listen(80, () => {
    console.log('Relay server running on port 80');
});
