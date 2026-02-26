console.log("WebRTC JS Loaded");

// ------------------------------------------
// CONNECT TO SIGNALING SERVER
// ------------------------------------------
window.ws = new WebSocket("ws://192.168.1.187:8765");

let pc = new RTCPeerConnection({
    iceServers: [{ urls: ["stun:stun.l.google.com:19302"] }]
});


// ------------------------------------------
// HANDLE VIDEO STREAM
// ------------------------------------------
pc.ontrack = (event) => {
    console.log("📡 Attaching Jetson video stream");
    document.getElementById("webrtcVideo").srcObject = event.streams[0];
};


// ------------------------------------------
// SEND OFFER
// ------------------------------------------
ws.onopen = async () => {
    console.log("WS CONNECTED!");

    pc.addTransceiver("video", { direction: "recvonly" });

    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);

    ws.send(JSON.stringify({
        offer: {
            sdp: offer.sdp,
            type: offer.type
        }
    }));
};


// ------------------------------------------
// HANDLE ANSWER + ICE
// ------------------------------------------
ws.onmessage = async (msg) => {
    const data = JSON.parse(msg.data);

    if (data.answer) {
        console.log("Received ANSWER");
        await pc.setRemoteDescription(new RTCSessionDescription(data.answer));
    }

    if (data.ice) {
        pc.addIceCandidate(data.ice).catch(console.error);
    }
};


// ------------------------------------------
// SEND COMMANDS (zoom / record)
// ------------------------------------------
function sendCommand(cmd) {
    console.log("Sent:", cmd);
    ws.send(JSON.stringify({ command: cmd }));
}

document.getElementById("zoomIn").onclick = () => sendCommand("zoom_in");
document.getElementById("zoomOut").onclick = () => sendCommand("zoom_out");

document.getElementById("startRecord").onclick = () => sendCommand("start_record");
document.getElementById("stopRecord").onclick = () => sendCommand("stop_record");


// ------------------------------------------
// VIDEO BANDWIDTH + LATENCY
// ------------------------------------------
let lastBytes = 0;
let lastTs = 0;

async function updateVideoStats() {
    const stats = await pc.getStats();
    stats.forEach(report => {

        if (report.type === "inbound-rtp" && report.kind === "video") {

            if (lastTs) {
                const bitrate = (report.bytesReceived - lastBytes) * 8 /
                    ((report.timestamp - lastTs) / 1000);

                document.getElementById("videoBandwidth").textContent =
                    (bitrate / 1e6).toFixed(2);
            }

            lastBytes = report.bytesReceived;
            lastTs = report.timestamp;

            if (report.jitter) {
                document.getElementById("videoLatency").textContent =
                    (report.jitter * 1000).toFixed(0);
            }
        }
    });
}
setInterval(updateVideoStats, 1000);
