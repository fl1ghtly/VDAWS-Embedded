const express = require('express');
const WebSocket = require('ws');
const http = require('http');

const app = express();
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

server.listen(80, () => {
    console.log('Relay server running on port 80');
});
