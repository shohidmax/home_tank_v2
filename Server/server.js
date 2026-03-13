const express = require('express');
const WebSocket = require('ws');
const path = require('path');
const http = require('http');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

const PORT = 8080;

// Serve the dashboard HTML file
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, '../Dashboard/index.html'));
});

// Broadcast function to send data to all connected clients
function broadcast(data) {
    wss.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(data);
        }
    });
}

wss.on('connection', (ws) => {
    console.log('New client connected');

    ws.on('message', (message) => {
        const msgString = message.toString();
        console.log('Received:', msgString);
        
        // Broadcast received message to everyone (ESP32 data -> Web, and Web Commands -> ESP32)
        // In a complex app, you'd filter this, but for this setup, broadcasting is simplest.
        broadcast(msgString);
    });

    ws.on('close', () => {
        console.log('Client disconnected');
    });
});

server.listen(PORT, () => {
    console.log(`Server started on http://localhost:${PORT}`);
    console.log(`Update your ESP32 code with IP: [YOUR_PC_IP]`);
});