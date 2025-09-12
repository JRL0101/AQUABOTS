# Aquabot_25030
Repository for all files used in the final product produced by Aquabot team 25030, academic year 24/25

## Backend and Frontend

This repository now includes a simple MQTT-enabled backend service and a web dashboard.

- **backend/** contains a FastAPI application that connects to an MQTT broker and exposes REST and WebSocket APIs.
- **frontend/** provides a small React-based interface for map display, sensor charting, and command controls.

The backend uses an in-memory user database with `operator` and `viewer` roles. Tokens are returned from `/token` and should be included as `Bearer` tokens when calling protected endpoints.

The frontend can be opened directly by loading `frontend/index.html` in a browser.
