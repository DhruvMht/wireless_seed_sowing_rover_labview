// ESP32 Arduino core 3.x. Existing L298N wiring; ENA/ENB jumpers ON.
// Coordinates stay in GRAPH UNITS: 1 unit = 10 cm.
// Protocol v2: incoming-segment seed flags and pause-to-drop planting. No encoders.
#include <Arduino.h>
#include <WiFi.h>
#include <Network.h>
#include <lwip/sockets.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>

const int SERVO_PIN = 18;
bool servoActive = false, servoNetwork = false, servoCenterOnly = false, seedDropActive = false;
bool servoAttached = false;
uint32_t servoStarted = 0;
uint16_t servoStage = 0;

const int IN1 = 27, IN2 = 26, IN3 = 25, IN4 = 33;
const char *AP_NAME = "Rover_AP";
const char *AP_PASSWORD = "RoverPath32";
const uint16_t TCP_PORT = 5000;
const float CM_PER_UNIT = 10.0f;
const size_t MAX_POINTS = 256;
const uint32_t HEARTBEAT_MS = 1500, SETTLE_MS = 200;
// The VI sends PING every 200 ms, including while idle. A lost Wi-Fi
// client can leave TCP connected without a FIN; release that stale owner.
const uint32_t SESSION_TIMEOUT_MS = 3000;
const uint32_t MAX_SEGMENT_MS = 60000, MAX_JOG_MS = 3000;

struct Point { float x, y; bool seed; };
enum Motion { IDLE, GATE_SETTLE, TURNING, TURN_SETTLE, DRIVING, SEED_SETTLE, SEEDING, DRIVE_SETTLE, JOGGING };
struct LineBuffer { char text[128]; size_t used = 0; bool discard = false; };
Point path[MAX_POINTS];
Motion motion = IDLE;
LineBuffer serialLine, tcpLine;
NetworkServer server(TCP_PORT, 1);
NetworkClient client;
bool session = false, uploading = false, ready = false, running = false;
bool networkMotion = false, estopLatched = false;
size_t expected = 0, received = 0, target = 1;
float msPerCm = 0, left90Ms = 0, right90Ms = 0;
float headingDeg = 0, segmentHeading = 0;
uint32_t phaseStart = 0, phaseDuration = 0, driveDuration = 0, lastPing = 0;
bool plantingEnabled = false, plantingConfigured = false, segmentSeeds = false;
uint32_t seedSpacingCm = 10, driveProgress = 0, nextSeedAt = 0, seedInterval = 0;
char tx[2048];
size_t txUsed = 0;

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void setMotors(char direction) {
  // Forward was physically confirmed. Swap pivot commands to match the chassis:
  // the original 'l' drove the physical left wheels forward and right backward.
  digitalWrite(IN1, direction == 'f' || direction == 'l');
  digitalWrite(IN2, direction == 'b' || direction == 'r');
  digitalWrite(IN3, direction == 'f' || direction == 'r');
  digitalWrite(IN4, direction == 'b' || direction == 'l');
}

void setArcMotors(char direction) {
  // Physical check confirmed IN1/IN2 drive the right side, IN3/IN4 the left.
  // Forward arc: outside wheels forward, inside wheels electrically braked.
  stopMotors();
  if (direction == 'l') digitalWrite(IN1, HIGH);
  else if (direction == 'r') digitalWrite(IN3, HIGH);
}

void stopServo() {
  if (servoAttached) ledcDetach(SERVO_PIN);
  servoAttached = false;
  servoActive = false; seedDropActive = false;
  pinMode(SERVO_PIN, OUTPUT); digitalWrite(SERVO_PIN, LOW);
}

void halt() {
  stopMotors();
  // Stop the cycle and command the tested closed position, retaining holding torque.
  // Do not attach a previously unused servo: boot/connection alone causes no motion.
  servoActive = false; seedDropActive = false;
  if (servoAttached && !ledcWrite(SERVO_PIN, (1500UL * 65535UL + 10000UL) / 20000UL)) stopServo();
  motion = IDLE; running = false; networkMotion = false;
  ready = false; uploading = false; expected = received = 0;
  plantingEnabled = false; plantingConfigured = false; segmentSeeds = false;
}

void disconnectClient() {
  // Loss of a network session stops all movement and invalidates its path.
  halt();
  client.stop(); session = false; txUsed = 0; tcpLine = LineBuffer{};
}

void reply(const char *format, ...) {
  char line[160];
  va_list args; va_start(args, format);
  int n = vsnprintf(line, sizeof(line) - 2, format, args);
  va_end(args);
  if (n < 0 || n >= (int)sizeof(line) - 2) return;
  // Drop serial diagnostics rather than blocking motor servicing.
  if (Serial.availableForWrite() >= n + 2) Serial.println(line);
  if (!session) return;
  line[n++] = '\r'; line[n++] = '\n';
  if (txUsed + n > sizeof(tx)) { disconnectClient(); return; }
  memcpy(tx + txUsed, line, n); txUsed += n;
}

void flushReplies() {
  if (!session || !txUsed) return;
  // Do not let a slow reader block Stop or timed motor servicing.
  int n = send(client.fd(), tx, txUsed, MSG_DONTWAIT);
  if (n > 0) {
    txUsed -= n; memmove(tx, tx + n, txUsed);
  } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
    disconnectClient();
  }
}

float wrapDegrees(float angle) {
  while (angle > 180) angle -= 360;
  while (angle <= -180) angle += 360;
  return angle;
}

uint32_t durationMs(float value) {
  return (uint32_t)ceilf(value); // Caller validates bounds first.
}

void phase(Motion next, uint32_t duration) {
  motion = next; phaseDuration = duration; phaseStart = millis();
}

bool preflight(float initialHeading) {
  float heading = initialHeading;
  for (size_t i = 1; i < received; ++i) {
    float dx = path[i].x - path[i-1].x, dy = path[i].y - path[i-1].y;
    float distance = hypotf(dx, dy);
    if (distance < 0.0001f) continue;
    float desired = atan2f(dy, dx) * 180.0f / PI;
    float turn = wrapDegrees(desired - heading);
    float turnMs = fabsf(turn) / 90.0f * (turn >= 0 ? left90Ms : right90Ms);
    float moveMs = distance * CM_PER_UNIT * msPerCm;
    if (!isfinite(moveMs) || moveMs > MAX_SEGMENT_MS || turnMs > MAX_SEGMENT_MS) return false;
    heading = desired;
  }
  return true;
}

bool closeGate() {
  if (!servoAttached) {
    if (!ledcAttach(SERVO_PIN, 50, 16)) return false;
    servoAttached = true;
  }
  return ledcWrite(SERVO_PIN, (1500UL * 65535UL + 10000UL) / 20000UL);
}

void beginDriveChunk() {
  uint32_t endAt = driveDuration;
  if (segmentSeeds && nextSeedAt <= driveDuration) endAt = nextSeedAt;
  setMotors('f'); phase(DRIVING, endAt - driveProgress);
}

void finishSegment() {
  stopMotors(); headingDeg = segmentHeading; phase(DRIVE_SETTLE, SETTLE_MS);
  reply("EST_POINT,%u,%.4f,%.4f", (unsigned)target, path[target].x, path[target].y);
}

void beginSegment() {
  while (target < received) {
    float dx = path[target].x - path[target-1].x;
    float dy = path[target].y - path[target-1].y;
    float distance = hypotf(dx, dy);
    if (distance < 0.0001f) { ++target; continue; }
    segmentHeading = atan2f(dy, dx) * 180.0f / PI;
    float turn = wrapDegrees(segmentHeading - headingDeg);
    driveDuration = durationMs(distance * CM_PER_UNIT * msPerCm);
    driveProgress = 0;
    segmentSeeds = plantingEnabled && path[target].seed;
    seedInterval = durationMs(seedSpacingCm * msPerCm);
    nextSeedAt = seedInterval;
    if (fabsf(turn) > 0.01f) {
      uint32_t turnTime = durationMs(fabsf(turn) / 90.0f * (turn > 0 ? left90Ms : right90Ms));
      setMotors(turn > 0 ? 'l' : 'r'); phase(TURNING, turnTime);
    } else {
      beginDriveChunk();
    }
    return;
  }
  halt(); reply("DONE");
}

void serviceMotion() {
  if (networkMotion && (!session || (uint32_t)(millis() - lastPing) > HEARTBEAT_MS)) {
    halt(); reply("STOPPED,HEARTBEAT_TIMEOUT"); return;
  }
  if (motion == IDLE || (uint32_t)(millis() - phaseStart) < phaseDuration) return;
  switch (motion) {
    case GATE_SETTLE:
      beginSegment(); break;
    case TURNING:
      stopMotors(); headingDeg = segmentHeading; phase(TURN_SETTLE, SETTLE_MS); break;
    case TURN_SETTLE:
      beginDriveChunk(); break;
    case DRIVING:
      stopMotors(); driveProgress += phaseDuration;
      if (segmentSeeds && driveProgress == nextSeedAt && nextSeedAt <= driveDuration) {
        phase(SEED_SETTLE, SETTLE_MS);
      } else finishSegment();
      break;
    case SEED_SETTLE:
      if (!closeGate()) { halt(); reply("ERR,SERVO_PWM"); break; }
      servoActive = true; servoNetwork = true; servoCenterOnly = false; seedDropActive = true;
      servoStarted = millis(); servoStage = 0;
      phase(SEEDING, 750); break;
    case SEEDING:
      if (servoActive) break;
      nextSeedAt += seedInterval;
      if (driveProgress < driveDuration) beginDriveChunk();
      else finishSegment();
      break;
    case DRIVE_SETTLE:
      ++target; beginSegment(); break;
    case JOGGING:
      stopMotors(); motion = IDLE; networkMotion = false; reply("JOG_DONE"); break;
    default: break;
  }
}

bool number(const char *s, float &v) {
  if (!s || !*s) return false;
  char *end; errno = 0; v = strtof(s, &end);
  return end != s && !*end && !errno && isfinite(v);
}

bool whole(const char *s, long &v) {
  if (!s || !*s) return false;
  char *end; errno = 0; v = strtol(s, &end, 10);
  return end != s && !*end && !errno;
}

void handleLine(char *line, bool fromNetwork) {
  char *part[6]; size_t count = 1; part[0] = line;
  for (char *p = line; *p; ++p) {
    if (*p == ',') {
      if (count == 6) { reply("ERR,FORMAT"); return; }
      *p = 0; part[count++] = p + 1;
    }
  }
  const char *cmd = part[0];
  if ((!strcmp(cmd, "STOP") || !strcmp(cmd, "s")) && count == 1) {
    halt(); reply("STOPPED"); return;
  }
  if (!strcmp(cmd, "ESTOP") && count == 1) {
    halt(); estopLatched = true; reply("ESTOPPED"); return;
  }
  if (!strcmp(cmd, "RESET") && count == 1) {
    halt(); estopLatched = false; reply("OK,RESET"); return;
  }
  if (!strcmp(cmd, "PING") && count == 1) {
    if (fromNetwork) lastPing = millis();
    reply("PONG"); return;
  }
  if (!strcmp(cmd, "STATUS") && count == 1) {
    reply("STATUS,%s,CAL=%u,READY=%u", estopLatched ? "ESTOP" : ((motion != IDLE || servoActive) ? "MOVING" : "IDLE"),
          (unsigned)(msPerCm > 0 && left90Ms > 0 && right90Ms > 0), (unsigned)ready); return;
  }
  if (!strcmp(cmd, "VERSION") && count == 1) { reply("VERSION,ROVER,2"); return; }
  if (estopLatched) { reply("ERR,ESTOP_LATCHED"); return; }
  if (motion != IDLE || running || servoActive) { reply("ERR,BUSY"); return; }
  if ((!strcmp(cmd, "SERVO_TEST") || !strcmp(cmd, "SERVO_CENTER") || !strcmp(cmd, "SEED_DROP")) && count == 1) {
    halt();
    stopServo();
    if (!ledcAttach(SERVO_PIN, 50, 16)) { reply("ERR,SERVO_PWM"); return; }
    servoAttached = true;
    servoActive = true; servoNetwork = fromNetwork; servoCenterOnly = !strcmp(cmd, "SERVO_CENTER"); seedDropActive = !strcmp(cmd, "SEED_DROP");
    servoStarted = millis(); lastPing = millis(); servoStage = 0;
    if (!ledcWrite(SERVO_PIN, (1500UL * 65535UL + 10000UL) / 20000UL)) {
      stopServo(); reply("ERR,SERVO_PWM"); return;
    }
    reply(seedDropActive ? "OK,SEED_DROP" : (servoCenterOnly ? "OK,SERVO_CENTER" : "OK,SERVO_TEST")); return;
  }
  float a, b, c; long index;
  if (!strcmp(cmd, "CAL") && count == 4) {
    if (!number(part[1], a) || !number(part[2], b) || !number(part[3], c) ||
        a < 1 || a > 1000 || b < 1 || b > 10000 || c < 1 || c > 10000) {
      reply("ERR,CAL_RANGE"); return;
    }
    msPerCm = a; left90Ms = b; right90Ms = c; reply("OK,CAL"); return;
  }
  // Keep original f/b/l/r commands for one-second calibration jogs.
  if (!strcmp(cmd, "ARC") && count == 3) {
    long time;
    if (strlen(part[1]) != 1 || (part[1][0] != 'l' && part[1][0] != 'r') ||
        !whole(part[2], time) || time < 1 || time > MAX_JOG_MS) {
      reply("ERR,ARC_RANGE"); return;
    }
    halt(); networkMotion = fromNetwork; lastPing = millis();
    setArcMotors(part[1][0]); phase(JOGGING, (uint32_t)time);
    reply("OK,ARC"); return;
  }
  bool shortJog = count == 1 && strlen(cmd) == 1 && strchr("fblr", cmd[0]);
  if ((!strcmp(cmd, "JOG") && count == 3) || shortJog) {
    char direction = shortJog ? cmd[0] : part[1][0];
    long time = 1000;
    if ((!shortJog && (strlen(part[1]) != 1 || !whole(part[2], time))) ||
        !strchr("fblr", direction) || time < 1 || time > MAX_JOG_MS) {
      reply("ERR,JOG_RANGE"); return;
    }
    halt(); networkMotion = fromNetwork; lastPing = millis();
    setMotors(direction); phase(JOGGING, (uint32_t)time); reply("OK,JOG"); return;
  }
  // Path ownership belongs to the TCP connection; serial is for jog/calibration/stop.
  if (!fromNetwork) { reply("ERR,USE_TCP_FOR_PATH"); return; }
  if (!strcmp(cmd, "PATH_BEGIN") && count == 2) {
    halt();
    if (!whole(part[1], index) || index < 2 || index > (long)MAX_POINTS) {
      reply("ERR,POINT_COUNT"); return;
    }
    expected = (size_t)index; uploading = true; reply("OK,PATH_BEGIN"); return;
  }
  if (!strcmp(cmd, "POINT") && count == 5) {
    long seed;
    if (!uploading || !whole(part[1], index) || index != (long)received || received >= expected ||
        !number(part[2], a) || !number(part[3], b) || fabsf(a) > 1000 || fabsf(b) > 1000 ||
        !whole(part[4], seed) || (seed != 0 && seed != 1) || (received == 0 && seed != 0)) {
      halt(); reply("ERR,POINT_UPLOAD_ABORTED"); return;
    }
    path[received++] = {a, b, seed == 1}; reply("OK,POINT,%ld", index); return;
  }
  if (!strcmp(cmd, "PATH_END") && count == 1) {
    if (!uploading || received != expected) { halt(); reply("ERR,INCOMPLETE_PATH"); return; }
    uploading = false; ready = true; reply("READY,%u", (unsigned)received); return;
  }
  if (!strcmp(cmd, "PLANT") && count == 3) {
    long enabled, spacing;
    if (!ready || !whole(part[1], enabled) || (enabled != 0 && enabled != 1) ||
        !whole(part[2], spacing) || (spacing != 10 && spacing != 20)) {
      reply("ERR,PLANT_SETTINGS"); return;
    }
    plantingEnabled = enabled == 1; seedSpacingCm = (uint32_t)spacing;
    plantingConfigured = true; reply("OK,PLANT"); return;
  }
  if (!strcmp(cmd, "START") && count == 2) {
    if (!ready) { reply("ERR,NO_PATH"); return; }
    if (!plantingConfigured) { reply("ERR,PLANT_SETTINGS"); return; }
    if (!(msPerCm > 0 && left90Ms > 0 && right90Ms > 0)) { reply("ERR,CALIBRATE_FIRST"); return; }
    if (!number(part[1], a) || a < -180 || a > 180) { reply("ERR,HEADING"); return; }
    if (!preflight(a)) { reply("ERR,SEGMENT_TOO_LONG"); return; }
    if (plantingEnabled && !closeGate()) { halt(); reply("ERR,SERVO_PWM"); return; }
    headingDeg = a; target = 1; running = true; ready = false;
    networkMotion = true; lastPing = millis();
    reply("OK,START");
    if (session && running) {
      if (plantingEnabled) phase(GATE_SETTLE, 450);
      else beginSegment();
    }
    return;
  }
  reply("ERR,COMMAND");
}

void feedByte(LineBuffer &buffer, int value, bool fromNetwork) {
  if (value < 0) return;
  if (value == '\n') {
    if (!buffer.discard && buffer.used) {
      buffer.text[buffer.used] = 0; handleLine(buffer.text, fromNetwork);
    }
    buffer = LineBuffer{}; return;
  }
  if (value == '\r' || buffer.discard) return;
  if (value < 32 || value > 126 || buffer.used >= sizeof(buffer.text) - 1) {
    buffer.discard = true; halt(); reply("ERR,LINE_REJECTED"); return;
  }
  buffer.text[buffer.used++] = (char)value;
}

void serviceServo() {
  if (!servoActive) return;
  uint32_t elapsed = millis() - servoStarted;
  if (servoNetwork && millis() - lastPing > HEARTBEAT_MS) {
    halt(); reply("ERR,HEARTBEAT_TIMEOUT"); return;
  }
  const uint32_t duration = seedDropActive ? 750 : (servoCenterOnly ? 4000 : 6000);
  if (elapsed >= duration) {
    if (seedDropActive) {
      // Keep the closed reference energized; DONE is timing, not position feedback.
      if (!ledcWrite(SERVO_PIN, (1500UL * 65535UL + 10000UL) / 20000UL)) {
        halt(); reply("ERR,SERVO_PWM"); return;
      }
      servoActive = false; seedDropActive = false;
      reply("SEED_DONE");
    } else { stopServo(); reply("SERVO_DONE"); }
    return;
  }
  uint16_t stage = elapsed / 20;
  if (stage != servoStage) {
    servoStage = stage;
    uint32_t pulse = 1500;
    if (seedDropActive) {
      // Approx. 20 degrees for the observed 500..2500 us / 180-degree range.
      // 0..100 ms: open; 100..200: dwell; 200..300: close; 300..750: settle; then hold.
      if (elapsed < 100) pulse = 1500 + elapsed * 222 / 100;
      else if (elapsed < 200) pulse = 1722;
      else if (elapsed < 300) pulse = 1722 - (elapsed - 200) * 222 / 100;
    } else if (!servoCenterOnly) {
      // Free-horn range test only. Slow ramps; STOP remains responsive.
      if (elapsed >= 1000 && elapsed < 2000) pulse = 1500 - (elapsed - 1000);
      else if (elapsed >= 2000 && elapsed < 4000) pulse = 500 + (elapsed - 2000);
      else if (elapsed >= 4000 && elapsed < 5000) pulse = 2500 - (elapsed - 4000);
    }
    if (!ledcWrite(SERVO_PIN, (pulse * 65535UL + 10000UL) / 20000UL)) {
      halt(); reply("ERR,SERVO_PWM");
    }
  }
}

void setup() {
  pinMode(SERVO_PIN, OUTPUT); digitalWrite(SERVO_PIN, LOW);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); stopMotors();
  Serial.begin(115200);
  Network.begin();
  WiFi.mode(WIFI_AP);
  IPAddress ip(192, 168, 4, 1), mask(255, 255, 255, 0);
  if (!WiFi.softAPConfig(ip, ip, mask) || !WiFi.softAP(AP_NAME, AP_PASSWORD)) {
    estopLatched = true; Serial.println("ERROR: Wi-Fi AP startup failed"); return;
  }
  server.begin(); server.setNoDelay(true);
  Serial.println("Rover_AP / RoverPath32 | TCP 192.168.4.1:5000");
  Serial.println("Calibration required: CAL,ms_per_cm,left_90_ms,right_90_ms");
  Serial.println("Newline commands: f b l r s; JOG,f,1000; STOP; ESTOP; RESET");
}

void loop() {
  serviceServo();
  serviceMotion();
  // Bound work per pass so a flooded stream cannot starve motion updates.
  for (int i = 0; i < 64 && Serial.available(); ++i) feedByte(serialLine, Serial.read(), false);
  if (session && (!client.connected() ||
                  (uint32_t)(millis() - lastPing) > SESSION_TIMEOUT_MS)) {
    Serial.println("SESSION_CLOSED,DISCONNECTED_OR_HEARTBEAT_EXPIRED");
    disconnectClient();
  }
  if (!session) {
    NetworkClient incoming = server.accept();
    if (incoming) {
      halt(); client = incoming; session = true; lastPing = millis();
      client.setNoDelay(true); tcpLine = LineBuffer{};
      reply("HELLO,ROVER,2,CM_PER_UNIT=10");
    }
  }
  for (int i = 0; i < 256 && session && client.available(); ++i) {
    feedByte(tcpLine, client.read(), true);
    serviceMotion();
  }
  flushReplies(); serviceMotion();
  delay(1); // Scheduler yield only; no long movement delay.
}
