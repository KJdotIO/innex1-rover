# INNEX Mission Control

Polished local dashboard spike for rover operations.

The app runs in mock mode by default, so it can be developed away from the
Jetson. When the Jetson bridge exists, set `VITE_ROVER_API_BASE` to the bridge
origin and the dashboard will call the same curated endpoints.

```bash
cd apps/mission-control
npm install
npm run dev
```

For a live Jetson bench check over Tailscale, use the Vite proxy. This avoids
browser CORS and self-signed certificate friction while still reading the real
Jetson bridge:

```bash
ROVER_API_TARGET=https://100.71.241.9:9443 \
VITE_ROVER_API_BASE=/rover \
npm run dev -- --host 127.0.0.1
```

The current dashboard can adapt the existing browser Gamepad bridge
`GET /api/state` response, so it will show drivetrain state, controller
online/offline status, wheel encoder ticks, command freshness, and whether the
velocity gate is holding zero. Topics not exposed by that bridge, such as power
and localisation, are shown as unavailable rather than invented.

Planned bridge shape:

- `GET /api/state` returns one reduced mission-control state document.
- `POST /api/actions/start-mission`
- `POST /api/actions/abort-mission`
- `POST /api/actions/start-rosbag`
- `POST /api/actions/stop-rosbag`
- `POST /api/actions/reset-motion-inhibit`
- `POST /api/actions/zero-command`

Keep the browser away from raw ROS graph access. The Jetson bridge should expose
only curated state and explicit actions.
