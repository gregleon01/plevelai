#include <Arduino.h>
#include <ArduinoJson.h>

// DM556 drivers expect STEP/DIR pulses referenced to +5 V on the UNO R4.
// Keep direction setup + pulse widths generous so each move is clean.

// Pin mapping (adjust to your driver wiring)
constexpr uint8_t PAN_STEP_PIN    = 2;
constexpr uint8_t PAN_DIR_PIN     = 3;
constexpr uint8_t PAN_ENABLE_PIN  = 0xFF;  // driver enable not wired
constexpr uint8_t PAN_LIMIT_PIN   = 0xFF;  // no limit switch yet

constexpr uint8_t TILT_STEP_PIN   = 5;
constexpr uint8_t TILT_DIR_PIN    = 6;
constexpr uint8_t TILT_ENABLE_PIN = 0xFF;
constexpr uint8_t TILT_LIMIT_PIN  = 0xFF;

// Motion parameters (deg/sec, steps/deg, etc.)
constexpr float PAN_STEPS_PER_DEG  = 3200.0f / 360.0f;
constexpr float TILT_STEPS_PER_DEG = 3200.0f / 360.0f;

constexpr float PAN_MAX_DPS  = 90.0f;
constexpr float TILT_MAX_DPS = 90.0f;

constexpr float PAN_HOME_DPS  = 45.0f;
constexpr float TILT_HOME_DPS = 45.0f;

constexpr long PAN_HOME_BACKOFF_STEPS  = 160;
constexpr long TILT_HOME_BACKOFF_STEPS = 160;

constexpr uint32_t TELEMETRY_PERIOD_MS = 250;
constexpr size_t   COMMAND_QUEUE_CAP   = 8;

// Pulse timing tuned for DM556 (minimum 2.5 microseconds) with comfortable margin.
constexpr uint16_t STEP_PULSE_HIGH_US = 15;
constexpr uint16_t STEP_PULSE_LOW_US  = 15;
constexpr uint16_t DIR_SETUP_DELAY_US = 20;

/* Laser TTL configuration (defaults can be overridden at runtime via
   "config"). */
constexpr uint8_t  LASER_TTL_PIN              = 10;
constexpr bool     LASER_DEFAULT_ACTIVE_LOW   = false;
constexpr float    LASER_DEFAULT_MIN_CONF     = 0.0f;
constexpr uint16_t LASER_DEFAULT_PULSE_MS     = 500;
constexpr uint16_t LASER_DEFAULT_SETTLE_MS    = 0;
constexpr uint16_t LASER_MAX_PULSE_MS         = 1000;
constexpr uint16_t LASER_MAX_SETTLE_MS        = 500;

enum class MotionState : uint8_t {
  Idle,
  Moving,
  HomingSeek,
  HomingRelease
};

struct Axis {
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t enablePin;
  uint8_t limitPin;
  float stepsPerDeg;
  float moveStepsPerSec;
  float homeStepsPerSec;
  long targetSteps;
  long currentSteps;
  long homeBackoffSteps;
  MotionState state;
  bool homed;
  unsigned long lastStepMicros;
  int8_t lastDir;
};

struct TargetCommand {
  float panDeg;
  float tiltDeg;
  float confidence;
  float groundX;
  float groundY;
  float groundZ;
  uint32_t queuedMs;
  uint16_t pulseMs;
  uint16_t settleMs;
  bool fireLaser;
};

struct CommandQueue {
  TargetCommand buffer[COMMAND_QUEUE_CAP];
  size_t head = 0;
  size_t tail = 0;
  size_t count = 0;

  bool push(const TargetCommand &cmd, bool *dropped = nullptr) {
    bool droppedOldest = false;
    if (count >= COMMAND_QUEUE_CAP) {
      droppedOldest = true;
      // Drop the oldest command so new targets aren't rejected when detections arrive quickly.
      head = (head + 1) % COMMAND_QUEUE_CAP;
      --count;
    }
    buffer[tail] = cmd;
    tail = (tail + 1) % COMMAND_QUEUE_CAP;
    ++count;
    if (dropped) {
      *dropped = droppedOldest;
    }
    return true;
  }

  bool pop(TargetCommand &out) {
    if (count == 0) {
      return false;
    }
    out = buffer[head];
    head = (head + 1) % COMMAND_QUEUE_CAP;
    --count;
    return true;
  }

  void clear() {
    head = tail = count = 0;
  }

  size_t size() const {
    return count;
  }
};

Axis panAxis{
  PAN_STEP_PIN,
  PAN_DIR_PIN,
  PAN_ENABLE_PIN,
  PAN_LIMIT_PIN,
  PAN_STEPS_PER_DEG,
  0.0f,
  0.0f,
  0,
  0,
  PAN_HOME_BACKOFF_STEPS,
  MotionState::Idle,
  false,
  0UL,
  0
};

Axis tiltAxis{
  TILT_STEP_PIN,
  TILT_DIR_PIN,
  TILT_ENABLE_PIN,
  TILT_LIMIT_PIN,
  TILT_STEPS_PER_DEG,
  0.0f,
  0.0f,
  0,
  0,
  TILT_HOME_BACKOFF_STEPS,
  MotionState::Idle,
  false,
  0UL,
  0
};

CommandQueue g_queue;
float g_lastCommandConfidence = 0.0f;
bool  g_homeRequested         = true;
unsigned long g_lastTelemetryMs = 0UL;
String g_serialBuffer;

struct LaserConfig {
  bool activeLow           = LASER_DEFAULT_ACTIVE_LOW;
  uint16_t defaultPulseMs  = LASER_DEFAULT_PULSE_MS;
  uint16_t defaultSettleMs = LASER_DEFAULT_SETTLE_MS;
  float minConfidence      = LASER_DEFAULT_MIN_CONF;
};

LaserConfig g_laserConfig;

struct LaserState {
  bool armed = false;
  bool pending = false;
  bool active = false;
  uint16_t pulseMs = 0;
  uint16_t settleMs = 0;
  unsigned long settleDeadlineMs = 0;
  unsigned long offDeadlineMs    = 0;
  float commandConfidence        = 0.0f;
  unsigned long lastFireMs       = 0;
};

LaserState g_laserState;

void resetLaserConfig() {
  g_laserConfig.activeLow        = LASER_DEFAULT_ACTIVE_LOW;
  g_laserConfig.defaultPulseMs   = LASER_DEFAULT_PULSE_MS;
  g_laserConfig.defaultSettleMs  = LASER_DEFAULT_SETTLE_MS;
  g_laserConfig.minConfidence    = LASER_DEFAULT_MIN_CONF;
}

uint8_t laserActiveLevel()   { return g_laserConfig.activeLow ? LOW : HIGH; }
uint8_t laserInactiveLevel() { return g_laserConfig.activeLow ? HIGH : LOW; }

void laserSetFiring(bool firing) {
  digitalWrite(LASER_TTL_PIN, firing ? laserActiveLevel() : laserInactiveLevel());
}

void laserDisarm() {
  g_laserState.armed  = false;
  g_laserState.pending = false;
  g_laserState.active  = false;
  g_laserState.pulseMs  = 0;
  g_laserState.settleMs = 0;
  g_laserState.settleDeadlineMs = 0;
  g_laserState.offDeadlineMs    = 0;
  g_laserState.commandConfidence = 0.0f;
  laserSetFiring(false);
}

void laserArmForCommand(const TargetCommand &cmd) {
  if (!cmd.fireLaser || cmd.pulseMs == 0) {
    laserDisarm();
    return;
  }
  g_laserState.armed  = true;
  g_laserState.pending = true;
  g_laserState.active  = false;
  g_laserState.pulseMs  = cmd.pulseMs;
  g_laserState.settleMs = cmd.settleMs;
  g_laserState.settleDeadlineMs = 0;
  g_laserState.offDeadlineMs    = 0;
  g_laserState.commandConfidence = cmd.confidence;
  laserSetFiring(false);
}

bool laserBusy() {
  return g_laserState.pending || g_laserState.active;
}

void updateLaserState(unsigned long nowMs) {
  if (!g_laserState.armed) {
    return;
  }

  if (g_laserState.active) {
    if (nowMs >= g_laserState.offDeadlineMs) {
      laserSetFiring(false);
      g_laserState.active = false;
      g_laserState.armed  = false;
      g_laserState.offDeadlineMs = 0;
    }
    return;
  }

  if (!g_laserState.pending) {
    return;
  }

  if (panAxis.state != MotionState::Idle || tiltAxis.state != MotionState::Idle) {
    return;
  }

  if (g_laserState.settleDeadlineMs == 0) {
    g_laserState.settleDeadlineMs = nowMs + g_laserState.settleMs;
    return;
  }

  if (nowMs < g_laserState.settleDeadlineMs) {
    return;
  }

  laserSetFiring(true);
  g_laserState.active  = true;
  g_laserState.pending = false;
  g_laserState.offDeadlineMs = nowMs + g_laserState.pulseMs;
  g_laserState.lastFireMs    = nowMs;
}

float degToSteps(float deg, float stepsPerDeg) {
  return deg * stepsPerDeg;
}

long roundToLong(float value) {
  return (value >= 0.0f)
       ? static_cast<long>(value + 0.5f)
       : static_cast<long>(value - 0.5f);
}

unsigned long intervalFor(float stepsPerSec) {
  if (stepsPerSec <= 0.0f) {
    return 1000UL;
  }
  const float us = 1000000.0f / stepsPerSec;
  return static_cast<unsigned long>(us > 1.0f ? us : 1.0f);
}

bool hasLimit(const Axis &axis) {
  return axis.limitPin != 0xFF;
}

bool limitActive(const Axis &axis) {
  return hasLimit(axis) && (digitalRead(axis.limitPin) == LOW);
}

void setEnable(const Axis &axis, bool enable) {
  if (axis.enablePin == 0xFF) {
    return;
  }
  digitalWrite(axis.enablePin, enable ? LOW : HIGH);
}

void toggleStep(const Axis &axis) {
  digitalWrite(axis.stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_HIGH_US);
  digitalWrite(axis.stepPin, LOW);
  delayMicroseconds(STEP_PULSE_LOW_US);
}

bool stepIfDue(Axis &axis, int direction, unsigned long nowMicros,
               unsigned long interval) {
  if (nowMicros - axis.lastStepMicros < interval) {
    return false;
  }
  axis.lastStepMicros = nowMicros;

  const int dirLevel = (direction > 0) ? HIGH : LOW;
  if (axis.lastDir != direction) {
    axis.lastDir = direction;
    digitalWrite(axis.dirPin, dirLevel);
    delayMicroseconds(DIR_SETUP_DELAY_US);
  } else {
    digitalWrite(axis.dirPin, dirLevel);
  }

  toggleStep(axis);
  axis.currentSteps += direction;
  return true;
}

void refreshAxisSpeeds() {
  panAxis.moveStepsPerSec  = PAN_MAX_DPS  * panAxis.stepsPerDeg;
  panAxis.homeStepsPerSec  = PAN_HOME_DPS * panAxis.stepsPerDeg;
  tiltAxis.moveStepsPerSec = TILT_MAX_DPS  * tiltAxis.stepsPerDeg;
  tiltAxis.homeStepsPerSec = TILT_HOME_DPS * tiltAxis.stepsPerDeg;
}

void invalidateAxisHome(Axis &axis) {
  axis.homed          = false;
  axis.state          = MotionState::Idle;
  axis.targetSteps    = axis.currentSteps;
  axis.lastStepMicros = 0;
  axis.lastDir        = 0;
}

void updateAxis(Axis &axis, unsigned long nowMicros) {
  const unsigned long moveInterval = intervalFor(axis.moveStepsPerSec);
  const unsigned long homeInterval = intervalFor(axis.homeStepsPerSec);

  switch (axis.state) {
    case MotionState::Idle:
      break;

    case MotionState::Moving: {
      if (axis.currentSteps == axis.targetSteps) {
        axis.state = MotionState::Idle;
        break;
      }
      const int dir = (axis.currentSteps < axis.targetSteps) ? 1 : -1;
      stepIfDue(axis, dir, nowMicros, moveInterval);
      break;
    }

    case MotionState::HomingSeek: {
      if (!limitActive(axis)) {
        stepIfDue(axis, -1, nowMicros, homeInterval);
      } else {
        axis.currentSteps = -axis.homeBackoffSteps;
        axis.targetSteps  = 0;
        axis.state        = MotionState::HomingRelease;
        axis.homed        = true;
      }
      break;
    }

    case MotionState::HomingRelease: {
      if (axis.currentSteps >= axis.targetSteps) {
        axis.currentSteps = axis.targetSteps;
        axis.state        = MotionState::Idle;
        axis.lastDir      = 0;
        break;
      }
      stepIfDue(axis, 1, nowMicros, homeInterval);
      break;
    }
  }
}

void beginHome() {
  g_queue.clear();
  g_lastCommandConfidence = 0.0f;
  laserDisarm();

  auto prepare = [](Axis &axis) {
    axis.targetSteps    = 0;
    axis.lastStepMicros = 0;
    axis.lastDir        = 0;
    digitalWrite(axis.dirPin, LOW);
    if (hasLimit(axis)) {
      axis.state = MotionState::HomingSeek;
      axis.homed = false;
    } else {
      axis.state = MotionState::Idle;
      axis.homed = true;
    }
  };

  prepare(panAxis);
  prepare(tiltAxis);

  setEnable(panAxis, true);
  setEnable(tiltAxis, true);
}

void emitAck(const char *status, const char *detail = nullptr) {
  StaticJsonDocument<256> doc;
  doc["status"] = status;
  if (detail) {
    doc["detail"] = detail;
  }
  doc["queue"]             = g_queue.size();
  doc["pan_homed"]         = panAxis.homed;
  doc["tilt_homed"]        = tiltAxis.homed;
  doc["laser_pending"]     = g_laserState.pending;
  doc["laser_active"]      = g_laserState.active;
  doc["laser_active_low"]  = g_laserConfig.activeLow;
  doc["laser_default_pulse_ms"]  = g_laserConfig.defaultPulseMs;
  doc["laser_default_settle_ms"] = g_laserConfig.defaultSettleMs;
  doc["laser_min_conf"]          = g_laserConfig.minConfidence;
  doc["pan_steps_per_deg"]       = panAxis.stepsPerDeg;
  doc["tilt_steps_per_deg"]      = tiltAxis.stepsPerDeg;
  serializeJson(doc, Serial);
  Serial.println();
}

void emitTelemetry() {
  StaticJsonDocument<512> doc;
  doc["status"] = "telemetry";
  doc["time_ms"] = millis();
  doc["queue"]   = g_queue.size();

  auto pan = doc.createNestedObject("pan");
  pan["steps"]        = panAxis.currentSteps;
  pan["target"]       = panAxis.targetSteps;
  pan["homed"]        = panAxis.homed;
  pan["steps_per_deg"] = panAxis.stepsPerDeg;

  auto tilt = doc.createNestedObject("tilt");
  tilt["steps"]        = tiltAxis.currentSteps;
  tilt["target"]       = tiltAxis.targetSteps;
  tilt["homed"]        = tiltAxis.homed;
  tilt["steps_per_deg"] = tiltAxis.stepsPerDeg;

  auto laser = doc.createNestedObject("laser");
  laser["armed"]        = g_laserState.armed;
  laser["pending"]      = g_laserState.pending;
  laser["active"]       = g_laserState.active;
  laser["pulse_ms"]     = g_laserState.pulseMs;
  laser["settle_ms"]    = g_laserState.settleMs;
  laser["last_fire_ms"] = g_laserState.lastFireMs;
  laser["conf"]         = g_laserState.commandConfidence;
  laser["active_low"]   = g_laserConfig.activeLow;
  laser["default_pulse_ms"]  = g_laserConfig.defaultPulseMs;
  laser["default_settle_ms"] = g_laserConfig.defaultSettleMs;
  laser["min_conf"]          = g_laserConfig.minConfidence;

  doc["last_conf"] = g_lastCommandConfidence;
  serializeJson(doc, Serial);
  Serial.println();
}

TargetCommand buildCommand(float panDeg, float tiltDeg,
                           float confidence = 1.0f,
                           uint16_t pulseMs = 0,
                           uint16_t settleMs = 0,
                           bool fireLaser = false) {
  TargetCommand cmd{};
  cmd.panDeg      = panDeg;
  cmd.tiltDeg     = tiltDeg;
  cmd.confidence  = confidence;
  cmd.groundX     = NAN;
  cmd.groundY     = NAN;
  cmd.groundZ     = NAN;
  cmd.queuedMs    = millis();
  cmd.pulseMs     = pulseMs;
  cmd.settleMs    = settleMs;
  cmd.fireLaser   = fireLaser;
  return cmd;
}

bool queueMotorsCheck() {
  if (!panAxis.homed || !tiltAxis.homed) {
    emitAck("error", "motors_check_home_required");
    g_homeRequested = true;
    return false;
  }

  g_queue.clear();

  const TargetCommand sequence[] = {
    buildCommand( 0.0f,  30.0f),
    buildCommand( 0.0f, -30.0f),
    buildCommand(20.0f,   0.0f),
    buildCommand(-20.0f,  0.0f),
    buildCommand( 0.0f,  15.0f, 1.0f, 500, 0, true),
    buildCommand( 0.0f, -15.0f, 1.0f, 500, 0, true)
  };

  for (const auto &cmd : sequence) {
    g_queue.push(cmd);
  }

  emitAck("queued", "motors_check");
  return true;
}

bool handleMoveCommand(JsonVariant root) {
  if (!panAxis.homed || !tiltAxis.homed) {
    emitAck("error", "home_required");
    return false;
  }

  JsonVariant joints = root["joints"];
  if (joints.isNull()) {
    emitAck("error", "missing_joints");
    return false;
  }

  TargetCommand cmd{};
  cmd.panDeg = joints["pan"].as<float>();
  cmd.tiltDeg = joints["tilt"].as<float>();
  cmd.confidence = root["conf"].as<float>();
  cmd.groundX = root["target_ground"][0] | NAN;
  cmd.groundY = root["target_ground"][1] | NAN;
  cmd.groundZ = root["target_ground"][2] | NAN;
  cmd.queuedMs = millis();

  uint16_t pulseMs = g_laserConfig.defaultPulseMs;
  uint16_t settleMs = g_laserConfig.defaultSettleMs;
  bool fireLaser = false;
  bool fireSpecified = false;

  if (!root["pulse_ms"].isNull()) {
    pulseMs = root["pulse_ms"].as<uint16_t>();
  }
  if (!root["settle_ms"].isNull()) {
    settleMs = root["settle_ms"].as<uint16_t>();
  }

  JsonVariant laser = root["laser"];
  if (!laser.isNull()) {
    if (!laser["pulse_ms"].isNull()) {
      pulseMs = laser["pulse_ms"].as<uint16_t>();
    }
    if (!laser["settle_ms"].isNull()) {
      settleMs = laser["settle_ms"].as<uint16_t>();
    }
    if (!laser["enable"].isNull()) {
      fireLaser = laser["enable"].as<bool>();
      fireSpecified = true;
    } else if (!laser["fire"].isNull()) {
      fireLaser = laser["fire"].as<bool>();
      fireSpecified = true;
    }
  }

  if (!root["fire"].isNull()) {
    fireLaser = root["fire"].as<bool>();
    fireSpecified = true;
  }

  if (!fireSpecified && cmd.confidence >= g_laserConfig.minConfidence) {
    fireLaser = true;
  }

  if (pulseMs > LASER_MAX_PULSE_MS) {
    pulseMs = LASER_MAX_PULSE_MS;
  }
  if (settleMs > LASER_MAX_SETTLE_MS) {
    settleMs = LASER_MAX_SETTLE_MS;
  }

  cmd.fireLaser = fireLaser;
  cmd.pulseMs   = pulseMs;
  cmd.settleMs  = settleMs;

  bool dropped = false;
  g_queue.push(cmd, &dropped);

  if (dropped) {
    emitAck("queued", "dropped_oldest");
  } else {
    emitAck("queued");
  }
  return true;
}

void handleHomeCommand() {
  g_homeRequested = true;
  emitAck("homing");
}

void handleConfigCommand(JsonVariant root) {
  bool changed     = false;
  bool axisChanged = false;
  bool panChanged  = false;
  bool tiltChanged = false;

  if (!root["reset"].isNull() && root["reset"].as<bool>()) {
    resetLaserConfig();
    changed = true;
  }

  JsonVariant laser = root["laser"];
  if (!laser.isNull()) {
    if (!laser["active_low"].isNull()) {
      bool activeLow = laser["active_low"].as<bool>();
      if (activeLow != g_laserConfig.activeLow) {
        g_laserConfig.activeLow = activeLow;
        changed = true;
      }
    }
    if (!laser["default_pulse_ms"].isNull()) {
      uint16_t pulse = laser["default_pulse_ms"].as<uint16_t>();
      if (pulse > LASER_MAX_PULSE_MS) {
        pulse = LASER_MAX_PULSE_MS;
      }
      if (pulse != g_laserConfig.defaultPulseMs) {
        g_laserConfig.defaultPulseMs = pulse;
        changed = true;
      }
    }
    if (!laser["default_settle_ms"].isNull()) {
      uint16_t settle = laser["default_settle_ms"].as<uint16_t>();
      if (settle > LASER_MAX_SETTLE_MS) {
        settle = LASER_MAX_SETTLE_MS;
      }
      if (settle != g_laserConfig.defaultSettleMs) {
        g_laserConfig.defaultSettleMs = settle;
        changed = true;
      }
    }
    if (!laser["min_conf"].isNull()) {
      float minConf = laser["min_conf"].as<float>();
      if (minConf < 0.0f) {
        minConf = 0.0f;
      } else if (minConf > 1.0f) {
        minConf = 1.0f;
      }
      if (minConf != g_laserConfig.minConfidence) {
        g_laserConfig.minConfidence = minConf;
        changed = true
      }
    }
  }

  JsonVariant axes = root["axis"];
  if (!axes.isNull()) {
    if (!axes["reset"].isNull() && axes["reset"].as<bool>()) {
      panAxis.stepsPerDeg  = PAN_STEPS_PER_DEG;
      tiltAxis.stepsPerDeg = TILT_STEPS_PER_DEG;
      panChanged  = true;
      tiltChanged = true;
    }

    JsonVariant panCfg = axes["pan"];
    if (!panCfg.isNull()) {
      if (!panCfg["steps_per_deg"].isNull()) {
        float steps = panCfg["steps_per_deg"].as<float>();
        if (steps > 0.0f && steps < 10000.0f && steps != panAxis.stepsPerDeg) {
          panAxis.stepsPerDeg = steps;
          panChanged = true;
        }
      }
    }

    JsonVariant tiltCfg = axes["tilt"];
    if (!tiltCfg.isNull()) {
      if (!tiltCfg["steps_per_deg"].isNull()) {
        float steps = tiltCfg["steps_per_deg"].as<float>();
        if (steps > 0.0f && steps < 10000.0f && steps != tiltAxis.stepsPerDeg) {
          tiltAxis.stepsPerDeg = steps;
          tiltChanged = true;
        }
      }
    }
  }

  if (panChanged) {
    invalidateAxisHome(panAxis);
    axisChanged = true;
  }
  if (tiltChanged) {
    invalidateAxisHome(tiltAxis);
    axisChanged = true;
  }

  if (axisChanged) {
    refreshAxisSpeeds();
    g_queue.clear();
    g_lastCommandConfidence = 0.0f;
    g_homeRequested = true;
    changed = true;
  }

  if (changed) {
    laserDisarm();
    emitAck("config", "applied");
  } else {
    emitAck("config", "no_change");
  }
}

void processLine(const String &line) {
  StaticJsonDocument<256> doc;
  auto err = deserializeJson(doc, line);
  if (err) {
    emitAck("error", "json_parse");
    return;
  }

  const char *cmd = doc["cmd"];
  if (!cmd) {
    emitAck("error", "missing_cmd");
    return;
  }

  if      (strcmp(cmd, "move")         == 0) handleMoveCommand(doc);
  else if (strcmp(cmd, "home")         == 0) handleHomeCommand();
  else if (strcmp(cmd, "config")       == 0) handleConfigCommand(doc);
  else if (strcmp(cmd, "motors_check") == 0) queueMotorsCheck();
  else if (strcmp(cmd, "ping")         == 0) emitAck("pong");
  else                                        emitAck("error", "unknown_cmd");
}

void serviceSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      if (!g_serialBuffer.isEmpty()) {
        processLine(g_serialBuffer);
        g_serialBuffer = "";
      }
    } else if (c == '\r') {
      // ignore carriage return
    } else {
      if (g_serialBuffer.length() < 500) {
        g_serialBuffer += c;
      }
    }
  }
}

void tryDispatchCommand() {
  if (!panAxis.homed || !tiltAxis.homed) {
    return;
  }
  if (panAxis.state != MotionState::Idle || tiltAxis.state != MotionState::Idle) {
    return;
  }
  if (laserBusy()) {
    return;
  }

  TargetCommand cmd;
  if (!g_queue.pop(cmd)) {
    return;
  }

  panAxis.targetSteps  = roundToLong(degToSteps(cmd.panDeg,  panAxis.stepsPerDeg));
  tiltAxis.targetSteps = roundToLong(degToSteps(cmd.tiltDeg, tiltAxis.stepsPerDeg));
  panAxis.state  = MotionState::Moving;
  tiltAxis.state = MotionState::Moving;
  g_lastCommandConfidence = cmd.confidence;
  laserArmForCommand(cmd);
  emitAck("dispatch");
}

void setup() {
  resetLaserConfig();
  pinMode(LASER_TTL_PIN, OUTPUT);
  laserDisarm();

  Serial.begin(115200);

  pinMode(panAxis.stepPin, OUTPUT);
  pinMode(panAxis.dirPin, OUTPUT);
  pinMode(tiltAxis.stepPin, OUTPUT);
  pinMode(tiltAxis.dirPin, OUTPUT);

  digitalWrite(panAxis.stepPin, LOW);
  digitalWrite(panAxis.dirPin, LOW);
  digitalWrite(tiltAxis.stepPin, LOW);
  digitalWrite(tiltAxis.dirPin, LOW);
  panAxis.lastDir  = 0;
  tiltAxis.lastDir = 0;

  if (hasLimit(panAxis)) {
    pinMode(panAxis.limitPin, INPUT_PULLUP);
  }
  if (hasLimit(tiltAxis)) {
    pinMode(tiltAxis.limitPin, INPUT_PULLUP);
  }

  if (panAxis.enablePin != 0xFF) {
    pinMode(panAxis.enablePin, OUTPUT);
  }
  if (tiltAxis.enablePin != 0xFF) {
    pinMode(tiltAxis.enablePin, OUTPUT);
  }

  refreshAxisSpeeds();
  setEnable(panAxis, true);
  setEnable(tiltAxis, true);
}

void loop() {
  const unsigned long nowMicros = micros();
  serviceSerial();

  if (g_homeRequested) {
    g_homeRequested = false;
    beginHome();
  }

  updateAxis(panAxis, nowMicros);
  updateAxis(tiltAxis, nowMicros);

  const unsigned long nowMs = millis();
  updateLaserState(nowMs);
  tryDispatchCommand();

  if (nowMs - g_lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    g_lastTelemetryMs = nowMs;
    emitTelemetry();
  }
}
