export const parameterGroups = [
  {
    id: "pid",
    title: "Stabilization",
    description: "Core pitch and roll control gains and attitude filter timing.",
    parameters: [
      { key: "gyroPitchKp", label: "Pitch Kp", type: "int", unit: "raw" },
      { key: "gyroPitchKi", label: "Pitch Ki", type: "int", unit: "raw" },
      { key: "gyroPitchKd", label: "Pitch Kd", type: "int", unit: "raw" },
      { key: "gyroRollKp", label: "Roll Kp", type: "int", unit: "raw" },
      { key: "gyroRollKi", label: "Roll Ki", type: "int", unit: "raw" },
      { key: "gyroRollKd", label: "Roll Kd", type: "int", unit: "raw" },
      { key: "accTimeConstant", label: "ACC Time Constant", type: "int", unit: "s" },
      { key: "accTimeConstant2", label: "Alt ACC Constant", type: "int", unit: "s" },
      { key: "enableGyro", label: "Enable Gyro", type: "bool" },
      { key: "enableACC", label: "Enable ACC", type: "bool" }
    ]
  },
  {
    id: "mounting",
    title: "Mounting",
    description: "Offsets, sensor orientation, and motor direction mapping.",
    parameters: [
      { key: "angleOffsetPitch", label: "Pitch Offset", type: "int", unit: "0.01 deg" },
      { key: "angleOffsetRoll", label: "Roll Offset", type: "int", unit: "0.01 deg" },
      { key: "dirMotorPitch", label: "Pitch Motor Dir", type: "int" },
      { key: "dirMotorRoll", label: "Roll Motor Dir", type: "int" },
      { key: "motorNumberPitch", label: "Pitch Motor Index", type: "int" },
      { key: "motorNumberRoll", label: "Roll Motor Index", type: "int" },
      { key: "axisReverseZ", label: "Reverse Z", type: "bool" },
      { key: "axisSwapXY", label: "Swap XY", type: "bool" }
    ]
  },
  {
    id: "power",
    title: "Motor Power",
    description: "Motor authority, battery scaling, and voltage protection.",
    parameters: [
      { key: "maxPWMmotorPitch", label: "Pitch Max PWM", type: "int" },
      { key: "maxPWMmotorRoll", label: "Roll Max PWM", type: "int" },
      { key: "maxPWMfpvPitch", label: "Pitch FPV PWM", type: "int" },
      { key: "maxPWMfpvRoll", label: "Roll FPV PWM", type: "int" },
      { key: "motorPowerScale", label: "Power Scale", type: "bool" },
      { key: "refVoltageBat", label: "Ref Voltage", type: "int", unit: "0.01 V" },
      { key: "cutoffVoltage", label: "Cutoff Voltage", type: "int", unit: "0.01 V" }
    ]
  },
  {
    id: "rc",
    title: "RC Input",
    description: "Absolute and relative RC behavior, limits, channels, and filtering.",
    parameters: [
      { key: "rcAbsolutePitch", label: "Pitch Absolute RC", type: "bool" },
      { key: "rcAbsoluteRoll", label: "Roll Absolute RC", type: "bool" },
      { key: "maxRCPitch", label: "Pitch Max RC", type: "int", unit: "deg" },
      { key: "minRCPitch", label: "Pitch Min RC", type: "int", unit: "deg" },
      { key: "maxRCRoll", label: "Roll Max RC", type: "int", unit: "deg" },
      { key: "minRCRoll", label: "Roll Min RC", type: "int", unit: "deg" },
      { key: "rcGainPitch", label: "Pitch RC Gain", type: "int" },
      { key: "rcGainRoll", label: "Roll RC Gain", type: "int" },
      { key: "rcLPFPitch", label: "Pitch RC LPF", type: "int", unit: "0.1 s" },
      { key: "rcLPFRoll", label: "Roll RC LPF", type: "int", unit: "0.1 s" },
      { key: "rcMid", label: "RC Mid", type: "int", unit: "us" },
      { key: "rcChannelPitch", label: "Pitch Channel", type: "int" },
      { key: "rcChannelRoll", label: "Roll Channel", type: "int" },
      { key: "rcChannelAux", label: "Aux Channel", type: "int" }
    ]
  },
  {
    id: "fpv",
    title: "FPV & Modes",
    description: "FPV handoff, freeze, alternate timing, and auxiliary mode switches.",
    parameters: [
      { key: "fpvGainPitch", label: "FPV Pitch Gain", type: "int" },
      { key: "fpvGainRoll", label: "FPV Roll Gain", type: "int" },
      { key: "rcLPFPitchFpv", label: "FPV Pitch LPF", type: "int", unit: "0.1 s" },
      { key: "rcLPFRollFpv", label: "FPV Roll LPF", type: "int", unit: "0.1 s" },
      { key: "fpvFreezePitch", label: "Freeze Pitch", type: "bool" },
      { key: "fpvFreezeRoll", label: "Freeze Roll", type: "bool" },
      { key: "fpvSwPitch", label: "FPV Pitch Switch", type: "int" },
      { key: "fpvSwRoll", label: "FPV Roll Switch", type: "int" },
      { key: "altSwAccTime", label: "Alt ACC Switch", type: "int" }
    ]
  },
  {
    id: "calibration",
    title: "Calibration",
    description: "Startup calibration policy and stored raw offsets.",
    parameters: [
      { key: "gyroCal", label: "Calibrate Gyro at Boot", type: "bool" },
      { key: "gyrOffsetX", label: "Gyro Offset X", type: "int" },
      { key: "gyrOffsetY", label: "Gyro Offset Y", type: "int" },
      { key: "gyrOffsetZ", label: "Gyro Offset Z", type: "int" },
      { key: "accOffsetX", label: "ACC Offset X", type: "int" },
      { key: "accOffsetY", label: "ACC Offset Y", type: "int" },
      { key: "accOffsetZ", label: "ACC Offset Z", type: "int" }
    ]
  },
  {
    id: "trace",
    title: "Trace",
    description: "Streaming output settings for live diagnostics.",
    parameters: [
      { key: "fTrace", label: "Fast Trace", type: "int" },
      { key: "sTrace", label: "Slow Trace", type: "int" }
    ]
  }
];

export const quickCommands = [
  { key: "ver", label: "Version", command: "ver", tone: "neutral" },
  { key: "he", label: "Help", command: "he", tone: "neutral" },
  { key: "par", label: "Refresh Params", command: "par", tone: "primary" },
  { key: "sd", label: "Defaults", command: "sd", tone: "warning" },
  { key: "re", label: "Restore", command: "re", tone: "neutral" },
  { key: "we", label: "Save", command: "we", tone: "success" },
  { key: "gc", label: "Gyro Cal", command: "gc", tone: "warning" },
  { key: "ac", label: "ACC Cal", command: "ac", tone: "warning" },
  { key: "sbv", label: "Save Battery Ref", command: "sbv", tone: "neutral" }
];
