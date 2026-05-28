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
