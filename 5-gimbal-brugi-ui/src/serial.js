const textEncoder = new TextEncoder();

export function supportsWebSerial() {
  return typeof navigator !== "undefined" && "serial" in navigator;
}

export async function requestSerialPort() {
  return navigator.serial.requestPort();
}

export async function openSerialPort(port, baudRate = 115200) {
  await port.open({
    baudRate,
    dataBits: 8,
    stopBits: 1,
    parity: "none",
    flowControl: "none"
  });
}

export async function closeSerialPort(port, reader, writer) {
  try {
    await reader?.cancel?.();
  } catch (error) {
    console.warn("reader cancel failed", error);
  }

  try {
    writer?.releaseLock?.();
  } catch (error) {
    console.warn("writer release failed", error);
  }

  try {
    await port?.close?.();
  } catch (error) {
    console.warn("port close failed", error);
  }
}

export function createLineParser(onLine) {
  let buffer = "";
  const decoder = new TextDecoder();

  return (value) => {
    buffer += decoder.decode(value, { stream: true });
    const lines = buffer.split(/\r?\n/);
    buffer = lines.pop() ?? "";
    lines.forEach((line) => onLine(line.trim()));
  };
}

export async function sendCommand(port, command) {
  const writer = port.writable.getWriter();
  try {
    await writer.write(textEncoder.encode(`${command}\r\n`));
  } finally {
    writer.releaseLock();
  }
}

export function parseParameterLine(line) {
  const match = line.match(/^([A-Za-z0-9_]+)\s+(-?\d+)$/);
  if (!match) {
    return null;
  }

  return {
    key: match[1],
    value: match[2]
  };
}
