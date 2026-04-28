import { useEffect, useMemo, useRef, useState } from "react";
import { parameterGroups, quickCommands } from "./parameterCatalog.js";
import {
  closeSerialPort,
  createLineParser,
  openSerialPort,
  parseParameterLine,
  requestSerialPort,
  sendCommand,
  supportsWebSerial
} from "./serial.js";

function groupedCount(parameters, values) {
  return parameters.filter((parameter) => values[parameter.key] !== undefined).length;
}

function toneClass(tone) {
  return `tone-${tone ?? "neutral"}`;
}

export default function App() {
  const [status, setStatus] = useState("Disconnected");
  const [portReady, setPortReady] = useState(false);
  const [firmwareVersion, setFirmwareVersion] = useState("Unknown");
  const [parameterValues, setParameterValues] = useState({});
  const [draftValues, setDraftValues] = useState({});
  const [logLines, setLogLines] = useState([]);
  const [busyKey, setBusyKey] = useState("");
  const [selectedGroup, setSelectedGroup] = useState(parameterGroups[0].id);
  const [customCommand, setCustomCommand] = useState("");
  const [transportWarning, setTransportWarning] = useState("");

  const portRef = useRef(null);
  const readerRef = useRef(null);
  const readLoopRef = useRef(null);

  const webSerialSupported = supportsWebSerial();

  const activeGroup = useMemo(
    () => parameterGroups.find((group) => group.id === selectedGroup) ?? parameterGroups[0],
    [selectedGroup]
  );

  useEffect(() => {
    return () => {
      if (portRef.current) {
        closeSerialPort(portRef.current, readerRef.current, null);
      }
    };
  }, []);

  async function startReadLoop(port) {
    const parser = createLineParser((line) => {
      if (!line) {
        return;
      }

      setLogLines((current) => [line, ...current].slice(0, 200));

      if (/BruGi version/i.test(line)) {
        setFirmwareVersion(line);
      }

      const parsed = parseParameterLine(line);
      if (parsed) {
        setParameterValues((current) => ({ ...current, [parsed.key]: parsed.value }));
        setDraftValues((current) => ({ ...current, [parsed.key]: parsed.value }));
      }
    });

    while (port.readable) {
      const reader = port.readable.getReader();
      readerRef.current = reader;

      try {
        while (true) {
          const { value, done } = await reader.read();
          if (done) {
            break;
          }
          if (value) {
            parser(value);
          }
        }
      } catch (error) {
        setLogLines((current) => [`[read error] ${error.message}`, ...current].slice(0, 200));
      } finally {
        reader.releaseLock();
        readerRef.current = null;
      }

      break;
    }
  }

  async function connect() {
    if (!webSerialSupported) {
      setTransportWarning("This browser does not support Web Serial. Use a recent Chromium-based browser.");
      return;
    }

    setTransportWarning("");
    setStatus("Requesting serial device...");

    try {
      const port = await requestSerialPort();
      await openSerialPort(port, 115200);
      portRef.current = port;
      setPortReady(true);
      setStatus("Connected at 115200 baud");
      readLoopRef.current = startReadLoop(port);
      await sendCommand(port, "ver");
      await sendCommand(port, "par");
    } catch (error) {
      setStatus("Connection failed");
      setTransportWarning(error.message);
    }
  }

  async function disconnect() {
    if (!portRef.current) {
      return;
    }

    setStatus("Disconnecting...");
    await closeSerialPort(portRef.current, readerRef.current, null);
    portRef.current = null;
    setPortReady(false);
    setStatus("Disconnected");
  }

  async function runCommand(commandKey, command) {
    if (!portRef.current) {
      setTransportWarning("Connect to the controller first.");
      return;
    }

    setBusyKey(commandKey);
    try {
      await sendCommand(portRef.current, command);
      if (command === "par") {
        setStatus("Refreshing parameter snapshot...");
      } else {
        setStatus(`Sent: ${command}`);
      }
    } catch (error) {
      setTransportWarning(error.message);
    } finally {
      setBusyKey("");
    }
  }

  async function applyParameter(key) {
    if (!portRef.current) {
      setTransportWarning("Connect to the controller first.");
      return;
    }

    const value = draftValues[key];
    if (value === undefined || value === "") {
      return;
    }

    setBusyKey(key);
    try {
      await sendCommand(portRef.current, `par ${key} ${value}`);
      setStatus(`Updated ${key}`);
      setParameterValues((current) => ({ ...current, [key]: String(value) }));
    } catch (error) {
      setTransportWarning(error.message);
    } finally {
      setBusyKey("");
    }
  }

  async function applyGroup() {
    for (const parameter of activeGroup.parameters) {
      if (draftValues[parameter.key] !== undefined && draftValues[parameter.key] !== parameterValues[parameter.key]) {
        // eslint-disable-next-line no-await-in-loop
        await applyParameter(parameter.key);
      }
    }
  }

  return (
    <div className="app-shell">
      <div className="ambient ambient-left" />
      <div className="ambient ambient-right" />

      <header className="hero">
        <div>
          <p className="eyebrow">STM32 BRUGI CONSOLE</p>
          <h1>Camera gimbal tuning without the old desktop baggage.</h1>
          <p className="lede">
            Connect straight to the controller over Web Serial, inspect live firmware responses,
            and push BruGi-compatible commands from a focused browser panel.
          </p>
        </div>
        <div className="hero-card">
          <div className="status-strip">
            <span className={`status-dot ${portReady ? "live" : ""}`} />
            <span>{status}</span>
          </div>
          <div className="stat-grid">
            <div>
              <span className="stat-label">Firmware</span>
              <strong>{firmwareVersion}</strong>
            </div>
            <div>
              <span className="stat-label">Protocol</span>
              <strong>BruGi Serial</strong>
            </div>
            <div>
              <span className="stat-label">Transport</span>
              <strong>{webSerialSupported ? "Web Serial" : "Unsupported"}</strong>
            </div>
            <div>
              <span className="stat-label">Captured Params</span>
              <strong>{Object.keys(parameterValues).length}</strong>
            </div>
          </div>
          <div className="hero-actions">
            <button className="action primary" onClick={connect} disabled={portReady}>
              Connect Controller
            </button>
            <button className="action ghost" onClick={disconnect} disabled={!portReady}>
              Disconnect
            </button>
          </div>
        </div>
      </header>

      {transportWarning ? <div className="warning-banner">{transportWarning}</div> : null}

      <main className="layout">
        <section className="panel command-panel">
          <div className="panel-heading">
            <div>
              <p className="section-kicker">Control Deck</p>
              <h2>Quick actions</h2>
            </div>
            <span className="panel-note">Maps directly to BruGi serial commands</span>
          </div>
          <div className="command-grid">
            {quickCommands.map((command) => (
              <button
                key={command.key}
                className={`command-card ${toneClass(command.tone)}`}
                onClick={() => runCommand(command.key, command.command)}
                disabled={!portReady || busyKey === command.key}
              >
                <span>{command.label}</span>
                <code>{command.command}</code>
              </button>
            ))}
          </div>

          <div className="terminal-send">
            <label htmlFor="custom-command">Raw command</label>
            <div className="raw-row">
              <input
                id="custom-command"
                value={customCommand}
                onChange={(event) => setCustomCommand(event.target.value)}
                placeholder="par gyroPitchKp 20000"
              />
              <button
                className="action ghost"
                disabled={!portReady || !customCommand.trim()}
                onClick={async () => {
                  await runCommand("raw", customCommand.trim());
                  setCustomCommand("");
                }}
              >
                Send
              </button>
            </div>
          </div>
        </section>

        <section className="panel config-panel">
          <div className="panel-heading">
            <div>
              <p className="section-kicker">Config Surface</p>
              <h2>Parameter editor</h2>
            </div>
            <button className="action primary" disabled={!portReady} onClick={applyGroup}>
              Apply Visible Group
            </button>
          </div>

          <div className="group-tabs">
            {parameterGroups.map((group) => (
              <button
                key={group.id}
                className={`tab ${group.id === activeGroup.id ? "active" : ""}`}
                onClick={() => setSelectedGroup(group.id)}
              >
                <span>{group.title}</span>
                <small>{groupedCount(group.parameters, parameterValues)}/{group.parameters.length}</small>
              </button>
            ))}
          </div>

          <div className="group-intro">
            <h3>{activeGroup.title}</h3>
            <p>{activeGroup.description}</p>
          </div>

          <div className="parameter-list">
            {activeGroup.parameters.map((parameter) => (
              <article className="parameter-card" key={parameter.key}>
                <div className="parameter-meta">
                  <div>
                    <strong>{parameter.label}</strong>
                    <p>{parameter.key}</p>
                  </div>
                  {parameter.unit ? <span className="unit-pill">{parameter.unit}</span> : null}
                </div>
                <div className="parameter-controls">
                  <input
                    value={draftValues[parameter.key] ?? ""}
                    onChange={(event) =>
                      setDraftValues((current) => ({
                        ...current,
                        [parameter.key]: event.target.value
                      }))
                    }
                    placeholder={parameterValues[parameter.key] ?? "unknown"}
                  />
                  <button
                    className="action ghost"
                    disabled={!portReady || busyKey === parameter.key}
                    onClick={() => applyParameter(parameter.key)}
                  >
                    Apply
                  </button>
                </div>
                <div className="parameter-foot">
                  <span>Current: {parameterValues[parameter.key] ?? "not loaded"}</span>
                  <button
                    className="link-button"
                    disabled={!portReady}
                    onClick={() => runCommand(`refresh-${parameter.key}`, `par ${parameter.key}`)}
                  >
                    Refresh
                  </button>
                </div>
              </article>
            ))}
          </div>
        </section>

        <section className="panel log-panel">
          <div className="panel-heading">
            <div>
              <p className="section-kicker">Live Feed</p>
              <h2>Firmware terminal</h2>
            </div>
            <button className="action ghost" onClick={() => setLogLines([])}>
              Clear
            </button>
          </div>
          <div className="log-view">
            {logLines.length === 0 ? (
              <p className="empty-state">No serial data yet. Connect and request `ver` or `par`.</p>
            ) : (
              logLines.map((line, index) => (
                <div className="log-line" key={`${line}-${index}`}>
                  {line}
                </div>
              ))
            )}
          </div>
        </section>
      </main>
    </div>
  );
}
