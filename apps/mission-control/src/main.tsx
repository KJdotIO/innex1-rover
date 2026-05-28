import React, { useCallback, useEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";
import {
  AlertTriangle,
  Camera,
  Check,
  CircleStop,
  Gauge,
  HardDrive,
  Pause,
  Play,
  RotateCcw,
  ShieldCheck,
  Square,
  Wifi,
  X
} from "lucide-react";
import "./styles.css";

type Health = "ok" | "warn" | "bad" | "off";
type ActionName =
  | "start-mission"
  | "abort-mission"
  | "start-rosbag"
  | "stop-rosbag"
  | "reset-motion-inhibit"
  | "zero-command";

interface DashboardState {
  connection: {
    route: "mock" | "tailscale" | "rover-wifi";
    jetsonOnline: boolean;
    rosOnline: boolean;
    latencyMs: number;
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
    mode: string;
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
    tfHealthy: boolean;
    aprilTags: number;
    pose: { x: number; y: number; yawDeg: number };
  };
  power: {
    batteryV: number;
    currentA: number;
  };
  cameras: {
    frontFps: number;
    rearFps: number;
    frontStale: boolean;
    rearStale: boolean;
  };
  preflight: Array<{ label: string; ok: boolean; detail: string }>;
  events: Array<{ level: Health; text: string }>;
}

const API_BASE = import.meta.env.VITE_ROVER_API_BASE ?? "";

function makeMockState(tick: number, recording = false): DashboardState {
  const gateOpen = tick % 70 < 58;
  const inhibited = tick % 70 >= 58 && tick % 70 < 64;
  const fault = tick % 70 >= 64;
  const safeLinear = Math.sin(tick / 8) * 0.12;
  const safeAngular = Math.cos(tick / 11) * 0.36;

  return {
    connection: {
      route: "mock",
      jetsonOnline: true,
      rosOnline: true,
      latencyMs: 28 + Math.round(Math.abs(Math.sin(tick / 6)) * 18)
    },
    safety: {
      estopActive: false,
      motionInhibited: inhibited,
      gateOpen,
      canMove: gateOpen && !inhibited && !fault,
      faultText: fault ? "Encoder stall simulated" : "No active faults"
    },
    mission: {
      phase: "Manual checkout",
      mode: "manual",
      timeRemainingS: Math.max(0, 900 - tick * 2),
      cycleCount: 2,
      lastFailureReason: fault ? "Wheel velocity below threshold" : "None",
      rosbagRecording: recording
    },
    drivetrain: {
      state: fault ? "FAULT" : gateOpen ? "READY" : "INHIBITED",
      faultCode: fault ? 3 : 0,
      commandAgeS: 0.05 + Math.abs(Math.sin(tick / 5)) * 0.1,
      safeCommand: { linear: safeLinear, angular: safeAngular },
      gatedCommand: {
        linear: gateOpen ? safeLinear : 0,
        angular: gateOpen ? safeAngular : 0
      },
      encoderTicks: [14020, 14082, 13984, 13991].map(
        (value, index) => value + tick * (index + 3)
      ),
      wheelRps: [0.32, 0.33, 0.31, 0.32].map(
        (value, index) => value + Math.sin(tick / (7 + index)) * 0.04
      )
    },
    localisation: {
      ready: tick % 40 < 34,
      tfHealthy: tick % 54 < 50,
      aprilTags: tick % 40 < 34 ? 3 : 1,
      pose: {
        x: 1.82 + Math.sin(tick / 18) * 0.2,
        y: -0.42 + Math.cos(tick / 18) * 0.2,
        yawDeg: (tick * 2) % 360
      }
    },
    power: {
      batteryV: 24.6 - Math.sin(tick / 14) * 0.2,
      currentA: 5.8 + Math.abs(Math.sin(tick / 6)) * 5.2
    },
    cameras: {
      frontFps: 8.6 + Math.sin(tick / 9) * 0.4,
      rearFps: 3.0 + Math.cos(tick / 10) * 0.3,
      frontStale: false,
      rearStale: tick % 80 > 73
    },
    preflight: [
      { label: "Jetson", ok: true, detail: "Reachable" },
      { label: "ROS graph", ok: true, detail: "Core topics fresh" },
      {
        label: "Safety gate",
        ok: gateOpen,
        detail: gateOpen ? "Open" : "Holding zero"
      },
      {
        label: "Teensy",
        ok: !fault,
        detail: fault ? "Fault active" : "Telemetry fresh"
      },
      {
        label: "Cameras",
        ok: tick % 80 <= 73,
        detail: tick % 80 <= 73 ? "Fresh" : "Rear stale"
      },
      {
        label: "Localisation",
        ok: tick % 40 < 34,
        detail: tick % 40 < 34 ? "Ready" : "Waiting"
      }
    ],
    events: [
      {
        level: fault ? "bad" : "ok",
        text: fault ? "Gate closed after drivetrain fault" : "Telemetry fresh"
      },
      { level: "warn", text: "Remote controls should stay read-only from home" },
      { level: "ok", text: "Dashboard running with mock data" }
    ]
  };
}

async function fetchState(tick: number, recording: boolean) {
  if (!API_BASE) return makeMockState(tick, recording);
  const response = await fetch(`${API_BASE}/api/state`, { cache: "no-store" });
  if (!response.ok) throw new Error(`State request failed: ${response.status}`);
  return (await response.json()) as DashboardState;
}

async function sendAction(action: ActionName) {
  if (!API_BASE) {
    await new Promise((resolve) => window.setTimeout(resolve, 200));
    return;
  }

  const response = await fetch(`${API_BASE}/api/actions/${action}`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ source: "mission-control" })
  });
  if (!response.ok) throw new Error(`${action} failed: ${response.status}`);
}

function formatSeconds(seconds: number) {
  const minutes = Math.floor(seconds / 60);
  const remainder = seconds % 60;
  return `${minutes}:${remainder.toString().padStart(2, "0")}`;
}

function statusFromBoolean(ok: boolean): Health {
  return ok ? "ok" : "warn";
}

function Status({ value, label }: { value: Health; label: string }) {
  return (
    <span className={`status ${value}`}>
      <span />
      {label}
    </span>
  );
}

function Stat({
  label,
  value,
  muted
}: {
  label: string;
  value: string;
  muted?: string;
}) {
  return (
    <div className="stat">
      <span>{label}</span>
      <strong>{value}</strong>
      {muted ? <small>{muted}</small> : null}
    </div>
  );
}

function ActionButton({
  action,
  label,
  icon,
  danger,
  busy,
  onAction
}: {
  action: ActionName;
  label: string;
  icon: React.ReactNode;
  danger?: boolean;
  busy?: boolean;
  onAction: (action: ActionName, label: string) => void;
}) {
  return (
    <button
      className={`action ${danger ? "danger" : ""}`}
      disabled={busy}
      type="button"
      onClick={() => onAction(action, label)}
    >
      {icon}
      {busy ? "Sending" : label}
    </button>
  );
}

function CameraView({
  label,
  fps,
  stale
}: {
  label: string;
  fps: number;
  stale: boolean;
}) {
  return (
    <div className="camera-view">
      <div className="camera-rule horizontal" />
      <div className="camera-rule vertical" />
      <div className="camera-meta">
        <strong>{label}</strong>
        <span>{stale ? "stale" : `${fps.toFixed(1)} fps`}</span>
      </div>
    </div>
  );
}

function Panel({
  title,
  icon,
  children
}: {
  title: string;
  icon?: React.ReactNode;
  children: React.ReactNode;
}) {
  return (
    <section className="panel">
      <header className="panel-header">
        <h2>{title}</h2>
        {icon}
      </header>
      {children}
    </section>
  );
}

function App() {
  const [tick, setTick] = useState(1);
  const [state, setState] = useState(() => makeMockState(1));
  const [busyAction, setBusyAction] = useState<ActionName | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [log, setLog] = useState<string[]>(["Dashboard started in mock mode."]);

  useEffect(() => {
    const timer = window.setInterval(() => setTick((value) => value + 1), 1200);
    return () => window.clearInterval(timer);
  }, []);

  useEffect(() => {
    let cancelled = false;
    fetchState(tick, state.mission.rosbagRecording)
      .then((nextState) => {
        if (cancelled) return;
        setState(nextState);
        setError(null);
      })
      .catch((err: Error) => {
        if (!cancelled) setError(err.message);
      });
    return () => {
      cancelled = true;
    };
  }, [tick, state.mission.rosbagRecording]);

  const runAction = useCallback(async (action: ActionName, label: string) => {
    setBusyAction(action);
    try {
      await sendAction(action);
      setLog((items) => [`${label} accepted${API_BASE ? "" : " (mock)"}`, ...items].slice(0, 5));
      if (action === "start-rosbag" || action === "stop-rosbag") {
        setState((current) => ({
          ...current,
          mission: { ...current.mission, rosbagRecording: action === "start-rosbag" }
        }));
      }
    } catch (err) {
      setLog((items) => [`${label} failed: ${(err as Error).message}`, ...items].slice(0, 5));
    } finally {
      setBusyAction(null);
    }
  }, []);

  const wheelRows = useMemo(
    () =>
      ["FL", "FR", "RL", "RR"].map((name, index) => ({
        name,
        ticks: state.drivetrain.encoderTicks[index],
        rps: state.drivetrain.wheelRps[index]
      })),
    [state.drivetrain.encoderTicks, state.drivetrain.wheelRps]
  );

  const route = API_BASE ? state.connection.route : "mock";
  const canMoveState: Health = state.safety.canMove
    ? "ok"
    : state.safety.estopActive || state.drivetrain.faultCode
      ? "bad"
      : "warn";

  return (
    <main className="dashboard">
      <header className="topbar">
        <div>
          <h1>INNEX Mission Control</h1>
          <p>Local dashboard spike</p>
        </div>
        <div className="top-status">
          <Status value={state.connection.jetsonOnline ? "ok" : "off"} label={`Jetson ${route}`} />
          <Status value={state.connection.rosOnline ? "ok" : "off"} label="ROS" />
          <span>{state.connection.latencyMs} ms</span>
        </div>
      </header>

      {error ? (
        <div className="alert">
          <AlertTriangle size={16} />
          {error}
        </div>
      ) : null}

      <section className="summary">
        <Stat label="Can move" value={state.safety.canMove ? "Yes" : "No"} muted={state.safety.faultText} />
        <Stat label="Mission" value={state.mission.phase} muted={`${state.mission.mode} · ${formatSeconds(state.mission.timeRemainingS)}`} />
        <Stat label="Drivetrain" value={state.drivetrain.state} muted={`fault ${state.drivetrain.faultCode}`} />
        <Stat label="Power" value={`${state.power.batteryV.toFixed(1)} V`} muted={`${state.power.currentA.toFixed(1)} A`} />
        <Stat label="Localisation" value={state.localisation.ready ? "Ready" : "Waiting"} muted={`${state.localisation.aprilTags} tags`} />
      </section>

      <section className="grid">
        <Panel title="Camera" icon={<Camera size={18} />}>
          <div className="camera-grid">
            <CameraView label="Front" fps={state.cameras.frontFps} stale={state.cameras.frontStale} />
            <CameraView label="Rear" fps={state.cameras.rearFps} stale={state.cameras.rearStale} />
          </div>
        </Panel>

        <Panel title="Safety" icon={<ShieldCheck size={18} />}>
          <div className="rows">
            <div><span>E-stop</span><Status value={state.safety.estopActive ? "bad" : "ok"} label={state.safety.estopActive ? "active" : "clear"} /></div>
            <div><span>Motion inhibit</span><Status value={state.safety.motionInhibited ? "warn" : "ok"} label={state.safety.motionInhibited ? "active" : "clear"} /></div>
            <div><span>Velocity gate</span><Status value={state.safety.gateOpen ? "ok" : "warn"} label={state.safety.gateOpen ? "open" : "zero"} /></div>
            <div><span>Movement</span><Status value={canMoveState} label={state.safety.canMove ? "allowed" : "blocked"} /></div>
          </div>
        </Panel>

        <Panel title="Actions" icon={<CircleStop size={18} />}>
          <div className="actions">
            <ActionButton action="start-mission" label="Start mission" icon={<Play size={15} />} onAction={runAction} busy={busyAction === "start-mission"} />
            <ActionButton action="abort-mission" label="Abort mission" icon={<Square size={15} />} danger onAction={runAction} busy={busyAction === "abort-mission"} />
            <ActionButton
              action={state.mission.rosbagRecording ? "stop-rosbag" : "start-rosbag"}
              label={state.mission.rosbagRecording ? "Stop rosbag" : "Start rosbag"}
              icon={<HardDrive size={15} />}
              onAction={runAction}
              busy={busyAction === "start-rosbag" || busyAction === "stop-rosbag"}
            />
            <ActionButton action="zero-command" label="Zero command" icon={<Pause size={15} />} onAction={runAction} busy={busyAction === "zero-command"} />
            <ActionButton action="reset-motion-inhibit" label="Reset inhibit" icon={<RotateCcw size={15} />} onAction={runAction} busy={busyAction === "reset-motion-inhibit"} />
          </div>
        </Panel>

        <Panel title="Drivetrain" icon={<Gauge size={18} />}>
          <div className="command-table">
            <div>
              <span>/cmd_vel_safe</span>
              <strong>{state.drivetrain.safeCommand.linear.toFixed(2)} m/s · {state.drivetrain.safeCommand.angular.toFixed(2)} rad/s</strong>
            </div>
            <div>
              <span>/cmd_vel_gated</span>
              <strong>{state.drivetrain.gatedCommand.linear.toFixed(2)} m/s · {state.drivetrain.gatedCommand.angular.toFixed(2)} rad/s</strong>
            </div>
            <div>
              <span>Command age</span>
              <strong>{state.drivetrain.commandAgeS.toFixed(2)} s</strong>
            </div>
          </div>
          <table>
            <thead>
              <tr><th>Wheel</th><th>Ticks</th><th>RPS</th></tr>
            </thead>
            <tbody>
              {wheelRows.map((wheel) => (
                <tr key={wheel.name}>
                  <td>{wheel.name}</td>
                  <td>{wheel.ticks}</td>
                  <td>{wheel.rps.toFixed(2)}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </Panel>

        <Panel title="Mission">
          <div className="rows">
            <div><span>Mode</span><strong>{state.mission.mode}</strong></div>
            <div><span>Timer</span><strong>{formatSeconds(state.mission.timeRemainingS)}</strong></div>
            <div><span>Cycles</span><strong>{state.mission.cycleCount}</strong></div>
            <div><span>Rosbag</span><Status value={state.mission.rosbagRecording ? "ok" : "warn"} label={state.mission.rosbagRecording ? "recording" : "idle"} /></div>
            <div><span>Last failure</span><strong>{state.mission.lastFailureReason}</strong></div>
          </div>
        </Panel>

        <Panel title="Localisation">
          <div className="rows">
            <div><span>Start zone</span><Status value={statusFromBoolean(state.localisation.ready)} label={state.localisation.ready ? "ready" : "waiting"} /></div>
            <div><span>TF</span><Status value={statusFromBoolean(state.localisation.tfHealthy)} label={state.localisation.tfHealthy ? "healthy" : "check"} /></div>
            <div><span>Pose</span><strong>{state.localisation.pose.x.toFixed(2)}, {state.localisation.pose.y.toFixed(2)}, {Math.round(state.localisation.pose.yawDeg)}°</strong></div>
          </div>
        </Panel>

        <Panel title="Pre-flight">
          <div className="checks">
            {state.preflight.map((item) => (
              <div className="check" key={item.label}>
                {item.ok ? <Check size={15} /> : <X size={15} />}
                <span>{item.label}</span>
                <strong>{item.detail}</strong>
              </div>
            ))}
          </div>
        </Panel>

        <Panel title="Event log">
          <div className="events">
            {state.events.map((item, index) => (
              <div className="event" key={`${item.text}-${index}`}>
                <Status value={item.level} label={item.level} />
                <span>{item.text}</span>
              </div>
            ))}
            {log.map((item) => (
              <div className="event" key={item}>
                <Status value="ok" label="local" />
                <span>{item}</span>
              </div>
            ))}
          </div>
        </Panel>

        <Panel title="Remote access" icon={<Wifi size={18} />}>
          <p className="note">
            Tailscale is fine for monitoring. Keep motion controls disabled from
            home unless somebody in the lab has armed the rover and can see it.
          </p>
          <p className="note">
            Future Jetson bridge should expose curated state and explicit actions,
            not raw ROS graph access.
          </p>
        </Panel>
      </section>
    </main>
  );
}

createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
