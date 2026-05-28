import React, { useCallback, useEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";
import {
  Activity,
  AlertTriangle,
  BatteryCharging,
  Camera,
  Check,
  CircleGauge,
  Compass,
  Disc3,
  Flag,
  Gauge,
  HardDrive,
  Joystick,
  MapPinned,
  Pause,
  Play,
  Power,
  Radio,
  RotateCcw,
  ShieldAlert,
  Square,
  Timer,
  Wifi,
  X
} from "lucide-react";
import "./styles.css";

type Health = "nominal" | "warning" | "fault" | "offline";
type ActionName =
  | "start-mission"
  | "abort-mission"
  | "start-rosbag"
  | "stop-rosbag"
  | "reset-motion-inhibit"
  | "zero-command";

interface MissionControlState {
  connection: {
    route: "mock" | "tailscale" | "rover-wifi";
    jetsonOnline: boolean;
    rosOnline: boolean;
    latencyMs: number;
    lastUpdateIso: string;
  };
  safety: {
    estopActive: boolean;
    motionInhibited: boolean;
    gateOpen: boolean;
    canMove: boolean;
    faultText: string;
  };
  mission: {
    phase: string;
    autonomyMode: string;
    timeRemainingS: number;
    cycleCount: number;
    lastFailureReason: string;
    rosbagRecording: boolean;
  };
  drivetrain: {
    state: string;
    faultCode: number;
    commandAgeS: number;
    safeCommand: { linear: number; angular: number };
    gatedCommand: { linear: number; angular: number };
    encoderTicks: number[];
    wheelRps: number[];
  };
  localisation: {
    ready: boolean;
    pose: { x: number; y: number; yawDeg: number };
    aprilTags: number;
    tfHealthy: boolean;
  };
  power: {
    batteryV: number;
    currentA: number;
    estimatedRemainingMin: number;
  };
  cameras: {
    frontFps: number;
    rearFps: number;
    frontStale: boolean;
    rearStale: boolean;
  };
  preflight: Array<{ label: string; ok: boolean; detail: string }>;
  events: Array<{ time: string; level: Health; text: string }>;
}

const API_BASE = import.meta.env.VITE_ROVER_API_BASE ?? "";

const nowLabel = () =>
  new Intl.DateTimeFormat("en-GB", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit"
  }).format(new Date());

const clamp = (value: number, min: number, max: number) =>
  Math.max(min, Math.min(max, value));

function makeMockState(seed: number, recording = true): MissionControlState {
  const phase = seed % 70 < 36 ? "Manual bench checkout" : "Autonomy standby";
  const safeLinear = Math.sin(seed / 9) * 0.18;
  const safeAngular = Math.cos(seed / 13) * 0.42;
  const gateOpen = seed % 90 < 76;
  const inhibited = seed % 90 >= 76 && seed % 90 < 84;
  const fault = seed % 90 >= 84;
  const frontTicks = Math.round(14192 + seed * 12);

  return {
    connection: {
      route: "mock",
      jetsonOnline: true,
      rosOnline: true,
      latencyMs: 34 + Math.round(Math.sin(seed / 8) * 12),
      lastUpdateIso: new Date().toISOString()
    },
    safety: {
      estopActive: false,
      motionInhibited: inhibited,
      gateOpen,
      canMove: gateOpen && !inhibited && !fault,
      faultText: fault ? "Drivetrain fault simulation" : "No active faults"
    },
    mission: {
      phase,
      autonomyMode: "manual",
      timeRemainingS: Math.max(0, 900 - seed * 3),
      cycleCount: 2,
      lastFailureReason: fault ? "Encoder velocity below threshold" : "None",
      rosbagRecording: recording
    },
    drivetrain: {
      state: fault ? "FAULT" : gateOpen ? "READY" : "INHIBITED",
      faultCode: fault ? 3 : 0,
      commandAgeS: 0.06 + Math.abs(Math.sin(seed / 7)) * 0.12,
      safeCommand: { linear: safeLinear, angular: safeAngular },
      gatedCommand: {
        linear: gateOpen ? safeLinear : 0,
        angular: gateOpen ? safeAngular : 0
      },
      encoderTicks: [
        frontTicks,
        frontTicks + 44,
        frontTicks - 108,
        frontTicks - 82
      ],
      wheelRps: [0.42, 0.44, 0.41, 0.43].map(
        (value, index) => value + Math.sin(seed / (6 + index)) * 0.05
      )
    },
    localisation: {
      ready: seed % 50 < 42,
      pose: {
        x: 1.8 + Math.sin(seed / 18) * 0.4,
        y: -0.7 + Math.cos(seed / 20) * 0.35,
        yawDeg: (seed * 3) % 360
      },
      aprilTags: seed % 40 < 28 ? 3 : 1,
      tfHealthy: seed % 60 < 55
    },
    power: {
      batteryV: 24.7 - Math.sin(seed / 14) * 0.3,
      currentA: 6.8 + Math.abs(Math.sin(seed / 5)) * 8,
      estimatedRemainingMin: 48
    },
    cameras: {
      frontFps: 8.8 + Math.sin(seed / 8) * 0.6,
      rearFps: 3.2 + Math.cos(seed / 11) * 0.4,
      frontStale: false,
      rearStale: seed % 80 > 72
    },
    preflight: [
      { label: "Jetson reachable", ok: true, detail: "Tailscale route alive" },
      { label: "ROS graph alive", ok: true, detail: "Core topics fresh" },
      {
        label: "Motion gate",
        ok: gateOpen,
        detail: gateOpen ? "Gate open" : "Gate is holding zero"
      },
      {
        label: "Teensy telemetry",
        ok: !fault,
        detail: fault ? "Stall fault simulated" : "Four encoders fresh"
      },
      {
        label: "Camera streams",
        ok: seed % 80 <= 72,
        detail: seed % 80 <= 72 ? "Front/rear within budget" : "Rear stale"
      },
      {
        label: "Localisation",
        ok: seed % 50 < 42,
        detail: seed % 50 < 42 ? "Start-zone status ready" : "Waiting for tags"
      }
    ],
    events: [
      {
        time: nowLabel(),
        level: fault ? "fault" : "nominal",
        text: fault ? "Velocity gate commanded zero" : "Telemetry refresh accepted"
      },
      {
        time: "T-00:12",
        level: "nominal",
        text: "Browser dashboard running in mock mode"
      },
      {
        time: "T-02:04",
        level: "warning",
        text: "Remote controls remain read-only until lab arming"
      }
    ]
  };
}

async function fetchState(seed: number, recording: boolean) {
  if (!API_BASE) return makeMockState(seed, recording);

  const response = await fetch(`${API_BASE}/api/state`, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`State request failed with HTTP ${response.status}`);
  }
  return (await response.json()) as MissionControlState;
}

async function sendAction(action: ActionName) {
  if (!API_BASE) {
    await new Promise((resolve) => window.setTimeout(resolve, 350));
    return { ok: true, mock: true };
  }

  const response = await fetch(`${API_BASE}/api/actions/${action}`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ source: "mission-control" })
  });
  if (!response.ok) {
    throw new Error(`${action} failed with HTTP ${response.status}`);
  }
  return response.json();
}

function formatSeconds(seconds: number) {
  const mins = Math.floor(seconds / 60);
  const secs = seconds % 60;
  return `${mins}:${secs.toString().padStart(2, "0")}`;
}

function healthLabel(health: Health) {
  if (health === "nominal") return "nominal";
  if (health === "warning") return "watch";
  if (health === "fault") return "fault";
  return "offline";
}

function StatusPill({
  health,
  label
}: {
  health: Health;
  label: string;
}) {
  return (
    <span className={`status-pill ${health}`}>
      <span />
      {label}
    </span>
  );
}

function Metric({
  label,
  value,
  detail
}: {
  label: string;
  value: string;
  detail?: string;
}) {
  return (
    <div className="metric">
      <span>{label}</span>
      <strong>{value}</strong>
      {detail ? <small>{detail}</small> : null}
    </div>
  );
}

function CommandButton({
  icon,
  label,
  action,
  variant = "secondary",
  onAction,
  busy
}: {
  icon: React.ReactNode;
  label: string;
  action: ActionName;
  variant?: "primary" | "secondary" | "danger";
  onAction: (action: ActionName, label: string) => void;
  busy?: boolean;
}) {
  return (
    <button
      className={`command-button ${variant}`}
      type="button"
      disabled={busy}
      onClick={() => onAction(action, label)}
    >
      {icon}
      <span>{busy ? "Sending" : label}</span>
    </button>
  );
}

function CameraFeed({
  title,
  fps,
  stale,
  rear
}: {
  title: string;
  fps: number;
  stale: boolean;
  rear?: boolean;
}) {
  return (
    <div className={`camera-feed ${rear ? "rear" : ""}`}>
      <div className="camera-grid" />
      <div className="camera-horizon" />
      <div className="camera-crosshair" />
      <div className="camera-label">
        <span>{title}</span>
        <strong>{fps.toFixed(1)} FPS</strong>
      </div>
      <StatusPill health={stale ? "warning" : "nominal"} label={stale ? "stale" : "live"} />
    </div>
  );
}

function RoverPlane({ state }: { state: MissionControlState }) {
  const x = clamp(50 + state.drivetrain.gatedCommand.angular * 35, 14, 86);
  const y = clamp(50 - state.drivetrain.gatedCommand.linear * 130, 14, 86);
  return (
    <div className="rover-plane">
      <div className="plane-axis horizontal" />
      <div className="plane-axis vertical" />
      <div
        className="rover-marker"
        style={{
          left: `${x}%`,
          top: `${y}%`,
          transform: `translate(-50%, -50%) rotate(${state.localisation.pose.yawDeg}deg)`
        }}
      >
        <span />
      </div>
      <div className="plane-readout">
        <span>pose</span>
        <strong>
          {state.localisation.pose.x.toFixed(2)}, {state.localisation.pose.y.toFixed(2)}
        </strong>
      </div>
    </div>
  );
}

function App() {
  const [seed, setSeed] = useState(1);
  const [state, setState] = useState(() => makeMockState(1));
  const [error, setError] = useState<string | null>(null);
  const [busyAction, setBusyAction] = useState<ActionName | null>(null);
  const [operatorLog, setOperatorLog] = useState<string[]>([
    "Dashboard initialised in local mock mode."
  ]);

  useEffect(() => {
    const timer = window.setInterval(() => {
      setSeed((value) => value + 1);
    }, 1200);
    return () => window.clearInterval(timer);
  }, []);

  useEffect(() => {
    let cancelled = false;
    fetchState(seed, state.mission.rosbagRecording)
      .then((nextState) => {
        if (cancelled) return;
        setState(nextState);
        setError(null);
      })
      .catch((err: Error) => {
        if (cancelled) return;
        setError(err.message);
      });
    return () => {
      cancelled = true;
    };
  }, [seed, state.mission.rosbagRecording]);

  const routeLabel = API_BASE ? state.connection.route : "mock";
  const canMoveHealth: Health = state.safety.canMove
    ? "nominal"
    : state.safety.estopActive || state.drivetrain.faultCode
      ? "fault"
      : "warning";

  const runAction = useCallback(async (action: ActionName, label: string) => {
    setBusyAction(action);
    try {
      await sendAction(action);
      setOperatorLog((items) => [
        `${nowLabel()} ${label} accepted${API_BASE ? "" : " (mock)"}`,
        ...items
      ].slice(0, 5));
      if (action === "start-rosbag" || action === "stop-rosbag") {
        setState((current) => ({
          ...current,
          mission: {
            ...current.mission,
            rosbagRecording: action === "start-rosbag"
          }
        }));
      }
    } catch (err) {
      setOperatorLog((items) => [
        `${nowLabel()} ${label} failed: ${(err as Error).message}`,
        ...items
      ].slice(0, 5));
    } finally {
      setBusyAction(null);
    }
  }, []);

  const wheelSummary = useMemo(
    () =>
      state.drivetrain.wheelRps
        .map((value, index) => `${["FL", "FR", "RL", "RR"][index]} ${value.toFixed(2)}`)
        .join("  /  "),
    [state.drivetrain.wheelRps]
  );

  return (
    <main className="mission-shell">
      <div className="video-sheen" />
      <div className="grid-overlay" />
      <header className="topbar">
        <div className="brand-lockup">
          <span className="brand-mark">INX</span>
          <div>
            <p>INNEX-1</p>
            <h1>Mission Control</h1>
          </div>
        </div>
        <nav aria-label="Dashboard sections">
          <a href="#overview">Overview</a>
          <a href="#drivetrain">Drive</a>
          <a href="#preflight">Pre-flight</a>
          <a href="#actions">Actions</a>
        </nav>
        <div className="connection-strip">
          <StatusPill
            health={state.connection.jetsonOnline ? "nominal" : "offline"}
            label={routeLabel}
          />
          <span>{state.connection.latencyMs} ms</span>
        </div>
      </header>

      <section className="hero-console" id="overview">
        <div className="hero-copy">
          <span className="eyebrow">Rover operations // local console</span>
          <h2>One screen for the things that decide whether the rover moves.</h2>
          <p>
            A curated dashboard for safety, drivetrain, mission state, cameras,
            power and evidence capture. Foxglove stays the microscope; this is
            the operator console.
          </p>
          <div className="hero-actions">
            <CommandButton
              icon={<Play size={16} />}
              label="Start mission"
              action="start-mission"
              variant="primary"
              onAction={runAction}
              busy={busyAction === "start-mission"}
            />
            <CommandButton
              icon={<Disc3 size={16} />}
              label={state.mission.rosbagRecording ? "Stop rosbag" : "Start rosbag"}
              action={state.mission.rosbagRecording ? "stop-rosbag" : "start-rosbag"}
              onAction={runAction}
              busy={busyAction === "start-rosbag" || busyAction === "stop-rosbag"}
            />
            <CommandButton
              icon={<Square size={16} />}
              label="Abort mission"
              action="abort-mission"
              variant="danger"
              onAction={runAction}
              busy={busyAction === "abort-mission"}
            />
          </div>
        </div>

        <div className="mission-clock">
          <span>Mission timer</span>
          <strong>{formatSeconds(state.mission.timeRemainingS)}</strong>
          <p>{state.mission.phase}</p>
          <StatusPill
            health={state.mission.rosbagRecording ? "nominal" : "warning"}
            label={state.mission.rosbagRecording ? "rosbag recording" : "rosbag idle"}
          />
        </div>
      </section>

      {error ? (
        <section className="error-banner">
          <AlertTriangle size={18} />
          <span>{error}</span>
        </section>
      ) : null}

      <section className="status-grid">
        <Metric
          label="Can rover move?"
          value={state.safety.canMove ? "YES" : "NO"}
          detail={state.safety.faultText}
        />
        <Metric
          label="Gate"
          value={state.safety.gateOpen ? "OPEN" : "ZERO"}
          detail="/cmd_vel_safe -> /cmd_vel_gated"
        />
        <Metric
          label="Battery"
          value={`${state.power.batteryV.toFixed(1)} V`}
          detail={`${state.power.currentA.toFixed(1)} A draw`}
        />
        <Metric
          label="Localisation"
          value={state.localisation.ready ? "READY" : "WAIT"}
          detail={`${state.localisation.aprilTags} AprilTags`}
        />
      </section>

      <section className="console-grid">
        <article className="panel safety-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Safety</span>
              <h3>Motion authority</h3>
            </div>
            <StatusPill health={canMoveHealth} label={healthLabel(canMoveHealth)} />
          </div>
          <div className="safety-stack">
            <div>
              <ShieldAlert size={18} />
              <span>E-stop</span>
              <strong>{state.safety.estopActive ? "ACTIVE" : "CLEAR"}</strong>
            </div>
            <div>
              <Power size={18} />
              <span>Motion inhibit</span>
              <strong>{state.safety.motionInhibited ? "ACTIVE" : "CLEAR"}</strong>
            </div>
            <div>
              <Gauge size={18} />
              <span>Drivetrain</span>
              <strong>{state.drivetrain.state}</strong>
            </div>
          </div>
          <div className="button-row">
            <CommandButton
              icon={<RotateCcw size={16} />}
              label="Reset inhibit"
              action="reset-motion-inhibit"
              onAction={runAction}
              busy={busyAction === "reset-motion-inhibit"}
            />
            <CommandButton
              icon={<Pause size={16} />}
              label="Zero command"
              action="zero-command"
              onAction={runAction}
              busy={busyAction === "zero-command"}
            />
          </div>
        </article>

        <article className="panel drive-panel" id="drivetrain">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Drivetrain</span>
              <h3>Command path</h3>
            </div>
            <CircleGauge size={22} />
          </div>
          <div className="command-path">
            <div>
              <span>safe</span>
              <strong>
                {state.drivetrain.safeCommand.linear.toFixed(2)} /{" "}
                {state.drivetrain.safeCommand.angular.toFixed(2)}
              </strong>
            </div>
            <div>
              <span>gated</span>
              <strong>
                {state.drivetrain.gatedCommand.linear.toFixed(2)} /{" "}
                {state.drivetrain.gatedCommand.angular.toFixed(2)}
              </strong>
            </div>
            <div>
              <span>age</span>
              <strong>{state.drivetrain.commandAgeS.toFixed(2)} s</strong>
            </div>
          </div>
          <RoverPlane state={state} />
          <p className="fine-print">{wheelSummary}</p>
        </article>

        <article className="panel camera-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Vision</span>
              <h3>Camera streams</h3>
            </div>
            <Camera size={22} />
          </div>
          <div className="camera-stack">
            <CameraFeed
              title="Front OAK"
              fps={state.cameras.frontFps}
              stale={state.cameras.frontStale}
            />
            <CameraFeed
              title="Rear OAK"
              fps={state.cameras.rearFps}
              stale={state.cameras.rearStale}
              rear
            />
          </div>
        </article>

        <article className="panel mission-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Mission</span>
              <h3>Autonomy state</h3>
            </div>
            <Flag size={22} />
          </div>
          <div className="mission-list">
            <Metric label="Mode" value={state.mission.autonomyMode.toUpperCase()} />
            <Metric label="Cycles" value={state.mission.cycleCount.toString()} />
            <Metric label="Failure" value={state.mission.lastFailureReason} />
          </div>
        </article>

        <article className="panel nav-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Navigation</span>
              <h3>Pose and graph</h3>
            </div>
            <MapPinned size={22} />
          </div>
          <div className="nav-radar">
            <Compass className="compass" size={42} />
            <div className="radar-ring one" />
            <div className="radar-ring two" />
            <div className="pose-chip">
              yaw {Math.round(state.localisation.pose.yawDeg)} deg
            </div>
          </div>
          <div className="split-readout">
            <StatusPill health={state.localisation.tfHealthy ? "nominal" : "warning"} label="TF" />
            <StatusPill health={state.localisation.ready ? "nominal" : "warning"} label="start-zone" />
          </div>
        </article>

        <article className="panel preflight-panel" id="preflight">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Pre-flight</span>
              <h3>Readiness checklist</h3>
            </div>
            <Activity size={22} />
          </div>
          <div className="checklist">
            {state.preflight.map((item) => (
              <div className="check-row" key={item.label}>
                {item.ok ? <Check size={16} /> : <X size={16} />}
                <div>
                  <strong>{item.label}</strong>
                  <span>{item.detail}</span>
                </div>
              </div>
            ))}
          </div>
        </article>
      </section>

      <section className="lower-grid" id="actions">
        <article className="panel operator-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Operator log</span>
              <h3>Recent actions</h3>
            </div>
            <HardDrive size={22} />
          </div>
          <div className="event-log">
            {operatorLog.map((item) => (
              <p key={item}>{item}</p>
            ))}
          </div>
        </article>

        <article className="panel telemetry-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Telemetry</span>
              <h3>Live channels</h3>
            </div>
            <Radio size={22} />
          </div>
          <div className="channel-grid">
            <StatusPill health="nominal" label="/drivetrain/status" />
            <StatusPill health="nominal" label="/drivetrain/telemetry" />
            <StatusPill health="nominal" label="/mission/state" />
            <StatusPill health="nominal" label="/power/telemetry" />
            <StatusPill health={state.cameras.rearStale ? "warning" : "nominal"} label="/camera_rear" />
            <StatusPill health={state.localisation.ready ? "nominal" : "warning"} label="/localisation" />
          </div>
        </article>

        <article className="panel remote-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Remote mode</span>
              <h3>Tailscale safe view</h3>
            </div>
            <Wifi size={22} />
          </div>
          <p>
            Remote access should stay read-only unless somebody in the lab has
            armed the rover and can see the physical machine. Motion buttons are
            wired as explicit bridge actions, not raw ROS graph access.
          </p>
          <div className="split-readout">
            <span><Joystick size={14} /> Controller read-only</span>
            <span><Timer size={14} /> Last update {nowLabel()}</span>
          </div>
        </article>
      </section>
    </main>
  );
}

createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
