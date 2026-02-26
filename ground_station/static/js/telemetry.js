/**
 * telemetry.js — Polls /api/data every 2 seconds and updates the dashboard
 *               with live sensor readings and swarm telemetry.
 *
 * Data source: serial_bridge.py reads the ESP32-S3 UART (IF-3) and exposes
 * the latest state via Flask's /api/data endpoint.
 *
 * DOM elements updated (defined in index.html):
 *   #sensorTemp       — water temperature in °C
 *   #sensorPH         — water pH
 *   #sensorSalinity   — water salinity in PSU
 *   #swarmMemberCount — number of nodes in the swarm
 *   #swarmLeaderID    — node_id of the current leader
 *   #swarmHeading     — leader heading in degrees
 *   #swarmFormation   — active formation pattern
 *   #swarmObstacle    — avoidance state (CLEAR / WARNING / DANGER)
 */

const TELEMETRY_POLL_MS = 2000;

/** Update a single DOM element, falling back to "--" if the value is null. */
function setText(id, value, unit) {
    const el = document.getElementById(id);
    if (!el) return;
    if (value === null || value === undefined) {
        el.textContent = "--";
    } else {
        el.textContent = unit ? `${value}${unit}` : String(value);
    }
}

/**
 * Apply a CSS colour class to the obstacle state span so the operator gets
 * an instant visual cue without reading the text.
 */
function setObstacleState(state) {
    const el = document.getElementById("swarmObstacle");
    if (!el) return;

    el.textContent = state || "--";
    el.style.color = {
        "CLEAR":    "limegreen",
        "WARNING":  "orange",
        "DANGER":   "red",
        "DISABLED": "gray",
    }[state] || "white";
}

async function fetchTelemetry() {
    try {
        const response = await fetch("/api/data");
        if (!response.ok) return;
        const data = await response.json();

        // Sensor data
        setText("sensorTemp",     data.temp_c    !== undefined ? Number(data.temp_c).toFixed(1) : null);
        setText("sensorPH",       data.ph        !== undefined ? Number(data.ph).toFixed(2)     : null);
        setText("sensorSalinity", data.salinity  !== undefined ? Number(data.salinity).toFixed(1) : null);

        // Swarm telemetry
        setText("swarmMemberCount", data.member_count);
        setText("swarmLeaderID",    data.leader_id);
        setText("swarmHeading",     data.heading_deg !== null && data.heading_deg !== undefined
                                        ? Number(data.heading_deg).toFixed(1)
                                        : null);
        setText("swarmFormation",   data.formation);

        // Obstacle / avoidance state
        setObstacleState(data.obstacle_state);

    } catch (err) {
        console.warn("Telemetry fetch failed:", err.message);
    }
}

// Start polling on page load
fetchTelemetry();
setInterval(fetchTelemetry, TELEMETRY_POLL_MS);
