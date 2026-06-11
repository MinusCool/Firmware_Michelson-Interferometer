#include "AccelStepper.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <esp_timer.h>

static volatile bool gPin26Locked = false;

class Heartbeat {
public:
  static constexpr uint8_t pin = 26;
  static constexpr uint32_t intervalMs = 500;

  static void begin() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    xTaskCreatePinnedToCore(task, "heartbeat", 2048, nullptr, 1, nullptr, 0);
  }

private:
  static void task(void*) {
    bool state = false;
    TickType_t last = xTaskGetTickCount();
    while (true) {
      if (!gPin26Locked) {
        state = !state;
        digitalWrite(pin, state ? HIGH : LOW);
      }
      vTaskDelayUntil(&last, pdMS_TO_TICKS(intervalMs));
    }
  }
};

class InterferometerController {
private:
  // ===== WiFi / MQTT =====
  const char* ssid = "MinusCool";
  const char* password = "Sibambang123";
  const char* mqttServer = "358587e8751a4fbf8655e5a2fb5be16c.s1.eu.hivemq.cloud";
  const int mqttPort = 8883;
  const char* mqttUser = "MinusCool";
  const char* mqttPassword = "Interfero123";

  static constexpr const char* TOPIC_CMD    = "motor/commands";
  static constexpr const char* TOPIC_DATA   = "motor/data";
  static constexpr const char* TOPIC_STATUS = "motor/status";

  WiFiClientSecure espClient;
  PubSubClient mqttClient;
  WiFiServer server;

  // ===== Packet spec =====
  static constexpr uint16_t MAGIC = 0xB547;
  static constexpr uint8_t VERSION = 1;
  static constexpr uint8_t MSG_TYPE_DATA = 0;

  // Header biner tetap 38 byte sesuai Tabel 1 proposal.
  // Untuk varian STRING, header tidak berbentuk fixed-width 38 byte,
  // tetapi semua field informasinya tetap dibawa sebagai teks.
  static constexpr uint16_t HEADER_LEN = 38;

  static constexpr uint8_t FLAGS_DEFAULT = 0x00;
  static constexpr uint8_t FLAG_ZEROCOPY_HINT = 0x01;

  static constexpr uint8_t CHANNEL_COUNT_1 = 1;

  // sample_fmt:
  // 1 = payload sampel int16 little-endian
  // 2 = payload sampel string ASCII, dipisahkan spasi
  static constexpr uint8_t SAMPLE_FMT_INT16_LE = 1;
  static constexpr uint8_t SAMPLE_FMT_STRING_ASCII = 2;

  static constexpr int VARIANT_STRING = 1;
  static constexpr int VARIANT_BIN = 2;
  static constexpr int VARIANT_BIN_ZC = 3;

  static constexpr size_t PACKET_SAMPLE_CAP = 64;
  int16_t packetSamples[PACKET_SAMPLE_CAP];
  size_t packetSampleCount = 0;

  uint64_t currentRunId = 0;
  uint32_t seqCounter = 0;
  int currentVariant = VARIANT_STRING;

  // ===== Motion / ADS =====
  static constexpr uint8_t STEP_PIN_ROT = 14;
  static constexpr uint8_t DIR_PIN_ROT  = 12;
  static constexpr uint8_t STEP_PIN_LIN = 32;
  static constexpr uint8_t DIR_PIN_LIN  = 33;

  static constexpr uint8_t ADS_CH_HOME  = 0;
  static constexpr uint8_t ADS_CH_MAIN  = 1;
  static constexpr uint8_t ADS_CH_LIMIT = 2;
  static constexpr int ADS12_LIMIT_THRESH = 2048;
  static constexpr uint8_t laserHomeRot = 26;

  static constexpr int STEPS_PER_REV = 200;
  static constexpr int MICROSTEP = 16;
  static constexpr int RATIO = 3;
  static constexpr float DEGREE_PER_STEP = 360.0f / (STEPS_PER_REV * MICROSTEP * RATIO);
  static constexpr float LEADSCREW_LEAD = 8.0f;
  static constexpr float MM_PER_STEP = LEADSCREW_LEAD / (STEPS_PER_REV * MICROSTEP);

  Adafruit_ADS1115 ads;
  AccelStepper stepperRot;
  AccelStepper stepperLin;

  int distance = 0;
  int speed = 0;
  int repetitions = 0;
  int angle = 0;
  int targetBounces = 0;
  int bounceCount = 0;
  int switchHitCount = 0;
  int blinkCount = 0;

  bool isHomedLin = false;
  bool isHomedRot = false;
  bool mqttConnected = false;
  bool modeLin = false;
  bool isCounting = false;

  unsigned long currentMillis = 0;
  unsigned long previousMillis = 0;
  unsigned long lastRiseTime = 0;
  unsigned long pulseStartTime = 0;
  unsigned long pulseEndTime = 0;
  unsigned long pulseWidth = 0;
  unsigned long period = 0;
  unsigned long lastPeriodTime = 0;

  static constexpr unsigned long sensorIntervalMs = 1;
  static constexpr unsigned long checkIntervalMs = 5;

  bool lastState = LOW;
  float measuredFrequency = 0.0f;
  float measuredDutyCycle = 0.0f;

  TaskHandle_t sensorTaskHandle = nullptr;
  volatile bool mainAdcEnabled = false;
  volatile int lastMain12 = 0;

  volatile bool sequenceActive = false;

public:
  InterferometerController()
    : mqttClient(espClient),
      server(1234),
      stepperRot(AccelStepper::DRIVER, STEP_PIN_ROT, DIR_PIN_ROT),
      stepperLin(AccelStepper::DRIVER, STEP_PIN_LIN, DIR_PIN_LIN) {}

  void begin() {
    Serial.begin(115200);

    configureMotors();
    configurePins();

    Wire.begin();
    Wire.setClock(800000);

    if (!ads.begin(0x48)) {
      Serial.println("ADS1115 not found! Check wiring/address.");
    }
    ads.setGain(GAIN_ONE);
    ads.setDataRate(RATE_ADS1115_860SPS);

    xTaskCreatePinnedToCore(sensorTaskTrampoline, "adc_main", 4096, this, 2, &sensorTaskHandle, 0);

    connectWiFi();
    setupMQTT();
    server.begin();
    delay(100);

    int initMain = readADS12(ADS_CH_MAIN);
    lastMain12 = initMain;
    lastState = (initMain > 2000) ? HIGH : LOW;
  }

  void run() {
    if (mqttConnected) {
      if (!mqttClient.connected()) {
        connectMQTT();
      }
      mqttClient.loop();
      delay(0);
    } else {
      handleLocalCommunication();
    }
  }

private:
  // ===== Utilities =====

  static uint64_t monotonicUs() {
    return (uint64_t)esp_timer_get_time();
  }

  void pumpMqtt() {
    if (!mqttConnected) {
      delay(0);
      return;
    }
    if (!mqttClient.connected()) {
      connectMQTT();
    }
    mqttClient.loop();
    delay(0);
  }

  static uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len) {
    crc = ~crc;
    while (len--) {
      crc ^= *data++;
      for (uint8_t k = 0; k < 8; ++k) {
        crc = (crc & 1U) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
      }
    }
    return ~crc;
  }

  static void writeLe16(uint8_t* dst, uint16_t v) {
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
  }

  static void writeLe32(uint8_t* dst, uint32_t v) {
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
    dst[2] = (uint8_t)((v >> 16) & 0xFF);
    dst[3] = (uint8_t)((v >> 24) & 0xFF);
  }

  static void writeLe64(uint8_t* dst, uint64_t v) {
    for (int i = 0; i < 8; ++i) {
      dst[i] = (uint8_t)((v >> (8 * i)) & 0xFF);
    }
  }

  int extractValue(const String& command, const String& key) {
    int start = command.indexOf(key);
    if (start == -1) return 0;
    start += key.length();
    int end = command.indexOf(';', start);
    if (end == -1) end = command.length();
    return command.substring(start, end).toInt();
  }

  uint64_t extractUInt64Value(const String& command, const String& key) {
    int start = command.indexOf(key);
    if (start == -1) return 0ULL;
    start += key.length();
    int end = command.indexOf(';', start);
    if (end == -1) end = command.length();
    String s = command.substring(start, end);
    s.trim();
    return (uint64_t)strtoull(s.c_str(), nullptr, 10);
  }

  void mqttPublishText(const char* topic, const String& message) {
    if (!mqttClient.connected()) {
      Serial.println("MQTT not connected");
      return;
    }
    mqttClient.publish(topic, message.c_str());
  }

  void mqttPublishBinary(const uint8_t* data, size_t len) {
    if (!mqttClient.connected()) {
      Serial.println("MQTT not connected");
      return;
    }
    mqttClient.publish(TOPIC_DATA, data, (unsigned int)len);
  }

  // ===== Setup =====

  void configureMotors() {
    int maxSpeed = abs((1000 / 60) * (STEPS_PER_REV * MICROSTEP));
    stepperRot.setMaxSpeed(maxSpeed);
    stepperRot.setAcceleration(maxSpeed);
    stepperLin.setMaxSpeed(maxSpeed);
    stepperLin.setAcceleration(maxSpeed);
  }

  void configurePins() {
    pinMode(laserHomeRot, OUTPUT);
  }

  void connectWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
      delay(0);
    }
    Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
  }

  void setupMQTT() {
    espClient.setInsecure();
    mqttClient.setServer(mqttServer, mqttPort);

    mqttClient.setCallback([this](char* topic, byte* payload, unsigned int length) {
      String inTopic = String(topic);
      if (inTopic != TOPIC_CMD) return;

      String message;
      message.reserve(length);
      for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
      }
      message.trim();

      if (message.startsWith("SYNC_REQ:")) {
        handleSyncRequest(message);
        return;
      }

      bool isValidCmd =
        message.startsWith("Mode:Linear") || message.startsWith("Mode:Rotasi");

      if (!isValidCmd) {
        Serial.println("Ignoring non-command payload on command topic");
        return;
      }

      if (sequenceActive) {
        Serial.println("Ignoring command because sequence is active");
        return;
      }

      handleCommand(message);
    });

    connectMQTT();
  }

  void connectMQTT() {
    Serial.println("Connecting to MQTT...");
    String clientId = "ESP32Client-" + String((uint32_t)(millis() & 0xFFFFFF));
    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      Serial.println("MQTT connected");
      mqttConnected = true;
      mqttClient.subscribe(TOPIC_CMD);
    } else {
      Serial.println("MQTT failed, retrying in 5s");
      delay(5000);
      delay(0);
    }
  }

  void handleLocalCommunication() {
    WiFiClient client = server.available();
    if (client) {
      while (client.connected()) {
        if (client.available()) {
          String message = client.readStringUntil('\n');
          message.trim();
          handleCommand(message);
          client.println("Executed: " + message);
        }
        delay(0);
      }
      client.stop();
    }
  }

  // ===== Clock sync =====

  void handleSyncRequest(const String& message) {
    uint64_t syncId = extractUInt64Value(message, "id=");
    uint64_t t1PcUs = extractUInt64Value(message, "t1_pc_us=");

    uint64_t t2EspUs = monotonicUs();
    uint64_t t3EspUs = monotonicUs();

    String ack =
      "SYNC_ACK:id=" + String((unsigned long long)syncId) +
      ";t1_pc_us=" + String((unsigned long long)t1PcUs) +
      ";t2_esp_us=" + String((unsigned long long)t2EspUs) +
      ";t3_esp_us=" + String((unsigned long long)t3EspUs);

    mqttPublishText(TOPIC_STATUS, ack);
    Serial.println("SYNC_ACK sent: " + ack);
  }

  // ===== Command handling =====

  void handleCommand(const String& command) {
    if (!(command.startsWith("Mode:Linear") || command.startsWith("Mode:Rotasi"))) {
      Serial.println("Unknown command: " + command);
      return;
    }

    if (sequenceActive) {
      Serial.println("Command ignored: sequence already running");
      return;
    }

    sequenceActive = true;

    distance = extractValue(command, "Distance:");
    angle = extractValue(command, "Angle:");
    speed = extractValue(command, "Speed:");
    repetitions = extractValue(command, "Repetitions:");
    currentRunId = extractUInt64Value(command, "RunId:");
    currentVariant = extractValue(command, "Variant:");

    if (currentRunId == 0ULL) {
      currentRunId = monotonicUs();
    }
    if (currentVariant < VARIANT_STRING || currentVariant > VARIANT_BIN_ZC) {
      currentVariant = VARIANT_STRING;
    }

    if (speed <= 0 || repetitions <= 0) {
      mqttPublishText(TOPIC_STATUS, "ERR:invalid parameters");
      Serial.println("Invalid parameters");
      sequenceActive = false;
      return;
    }

    modeLin = command.startsWith("Mode:Linear");
    isHomedLin = false;
    isHomedRot = false;
    targetBounces = repetitions;
    bounceCount = 0;
    seqCounter = 0;
    packetSampleCount = 0;

    while (targetBounces > bounceCount) {
      pumpMqtt();
      moveStepper();
    }

    mqttPublishText(TOPIC_STATUS, "DONE:run_id=" + String((unsigned long long)currentRunId));
    sequenceActive = false;
  }

  // ===== ADS =====

  int readADS12(uint8_t ch) {
    int16_t raw = ads.readADC_SingleEnded(ch);
    if (raw < 0) raw = 0;
    return (int)((raw * 4095L) / 32767L);
  }

  static void sensorTaskTrampoline(void* arg) {
    static_cast<InterferometerController*>(arg)->sensorTask();
  }

  void sensorTask() {
    TickType_t last = xTaskGetTickCount();
    while (true) {
      if (mainAdcEnabled) {
        int v = readADS12(ADS_CH_MAIN);
        lastMain12 = v;
        vTaskDelayUntil(&last, pdMS_TO_TICKS(sensorIntervalMs));
      } else {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
      }
    }
  }

  // ===== Homing =====

  void homingLinear() {
    switchHitCount = 0;
    mainAdcEnabled = false;

    bool limConvInFlight = false;
    int lastLimit12 = 4095;
    uint32_t lastKickMs = 0;
    const uint32_t adcKickPeriodMs = checkIntervalMs;

    while (true) {
      pumpMqtt();

      unsigned long currentTime = millis();
      float steps_per_mm = 1.0f / MM_PER_STEP;

      if (currentTime - lastKickMs >= adcKickPeriodMs) {
        lastKickMs = currentTime;

        if (!limConvInFlight) {
          ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_2, false);
          limConvInFlight = true;
        } else if (ads.conversionComplete()) {
          int16_t raw = ads.getLastConversionResults();
          if (raw < 0) raw = 0;
          lastLimit12 = (int)((raw * 4095L) / 32767L);
          limConvInFlight = false;
        }
      }

      int switchState = (lastLimit12 < ADS12_LIMIT_THRESH) ? HIGH : LOW;

      static uint32_t tDbg = 0;
      if (currentTime - tDbg > 100) {
        tDbg = currentTime;
        Serial.printf("A2(limit)=%d -> state=%d\n", lastLimit12, switchState);
      }

      if (!isHomedLin && switchHitCount == 0) {
        stepperLin.setMaxSpeed(10000);
        stepperLin.move(100 * steps_per_mm);
      }

      if (switchState == HIGH && switchHitCount == 0) {
        Serial.println("Limit switch pertama aktif! Motor mundur cepat 1mm.");
        stepperLin.setMaxSpeed(2000);
        stepperLin.move(-1 * steps_per_mm);
        switchHitCount = 1;
      }

      if (switchHitCount == 1 && stepperLin.distanceToGo() == 0) {
        Serial.println("Motor maju perlahan setelah mundur.");
        stepperLin.setMaxSpeed(500);
        stepperLin.move(100 * steps_per_mm);
        switchHitCount = 2;
      }

      if (switchHitCount == 2 && switchState == HIGH) {
        float extraMm = distance * 0.5f;
        float backoffMm = 30.0f + extraMm;
        Serial.printf("Limit switch kedua aktif! Motor mundur %.2fmm.\n", backoffMm);

        stepperLin.setMaxSpeed(10000);
        stepperLin.move(-(long)(backoffMm * steps_per_mm));
        switchHitCount = 3;
      }

      if (switchHitCount == 3 && stepperLin.distanceToGo() == 0) {
        Serial.println("Linear homing selesai.");
        stepperLin.setCurrentPosition(0);
        isHomedLin = true;
        break;
      }

      if (!isHomedLin) {
        stepperLin.run();
      }
    }
  }

  void homingRotation() {
    int localSwitchHitCount = 0;
    mainAdcEnabled = false;

    gPin26Locked = true;
    digitalWrite(laserHomeRot, HIGH);
    Serial.println("Homing motor rotasi...");

    bool homeConvInFlight = false;
    int lastHome12 = 4095;
    uint32_t lastKickMs = 0;
    const uint32_t adcKickPeriodMs = checkIntervalMs;

    while (true) {
      pumpMqtt();

      unsigned long currentTime = millis();
      float steps_per_degree = 1.0f / DEGREE_PER_STEP;

      if (currentTime - lastKickMs >= adcKickPeriodMs) {
        lastKickMs = currentTime;

        if (!homeConvInFlight) {
          ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
          homeConvInFlight = true;
        } else if (ads.conversionComplete()) {
          int16_t raw = ads.getLastConversionResults();
          if (raw < 0) raw = 0;
          lastHome12 = (int)((raw * 4095L) / 32767L);
          homeConvInFlight = false;
        }
      }

      int nilaiSensor = lastHome12;

      static uint32_t tDbg = 0;
      if (currentTime - tDbg > 100) {
        tDbg = currentTime;
        Serial.printf("Home(A0)=%d\n", nilaiSensor);
      }

      if (!isHomedRot && localSwitchHitCount == 0) {
        stepperRot.setMaxSpeed(1500);
        stepperRot.move(-100 * steps_per_degree);
      }

      if (localSwitchHitCount == 0 && nilaiSensor < 3000) {
        Serial.println("Sensor aktif! Mundur 100 langkah.");
        stepperRot.setMaxSpeed(400);
        stepperRot.move(100);
        localSwitchHitCount = 1;
      }

      if (localSwitchHitCount == 1 && stepperRot.distanceToGo() == 0) {
        Serial.println("Bergerak lambat ke arah sensor kembali.");
        stepperRot.setMaxSpeed(80);
        stepperRot.move(-1000);
        localSwitchHitCount = 2;
      }

      if (localSwitchHitCount == 2 && nilaiSensor < 2800) {
        Serial.println("Sensor aktif kedua kali. Homing rotasi selesai.");
        stepperRot.setCurrentPosition(0);
        isHomedRot = true;
        digitalWrite(laserHomeRot, LOW);
        gPin26Locked = false;
        break;
      }

      if (!isHomedRot) {
        stepperRot.run();
      }
    }
  }

  // ===== Data plane =====

  void resetPacketBuffer() {
    packetSampleCount = 0;
  }

  void appendSampleToPacket(int value, uint8_t repId) {
    if (packetSampleCount < PACKET_SAMPLE_CAP) {
      int clamped = value;
      if (clamped < -32768) clamped = -32768;
      if (clamped > 32767)  clamped = 32767;
      packetSamples[packetSampleCount++] = (int16_t)clamped;
    }

    if (packetSampleCount >= PACKET_SAMPLE_CAP) {
      publishCurrentPacket(repId);
    }
  }

  void flushPacket(uint8_t repId) {
    if (packetSampleCount > 0) {
      publishCurrentPacket(repId);
    }
  }

  void publishCurrentPacket(uint8_t repId) {
    if (packetSampleCount == 0) return;

    uint64_t tSendUs = monotonicUs();

    if (currentVariant == VARIANT_STRING) {
      // STRING tetap memakai format teks.
      // Namun field metadata dibuat ekuivalen dengan header biner Tabel 1:
      // magic, version, msg_type, header_len, flags, rep_id, seq,
      // t_send_us, sample_count, channel_count, sample_fmt,
      // payload_bytes, payload_crc32, run_id, dan payload.
      //
      // Perbedaan dengan BIN/BIN+ZC hanya format representasi:
      // - STRING: metadata teks + payload sampel ASCII
      // - BIN: header biner + payload int16 LE
      // - BIN+ZC: header biner + payload int16 LE + flag zero-copy

      String sampleText;
      sampleText.reserve(packetSampleCount * 8);

      for (size_t i = 0; i < packetSampleCount; ++i) {
        if (i > 0) sampleText += ' ';
        sampleText += String((int)packetSamples[i]);
      }

      uint16_t payloadBytes = (uint16_t)sampleText.length();

      uint32_t payloadCrc32 = crc32Update(
        0,
        (const uint8_t*)sampleText.c_str(),
        payloadBytes
      );

      auto buildStringHeader = [&](uint16_t headerLen) -> String {
        String header;
        header.reserve(220);

        header += "magic=0xB547";
        header += "|version=";
        header += String((int)VERSION);

        header += "|msg_type=";
        header += String((int)MSG_TYPE_DATA);

        // Untuk STRING, header_len adalah panjang header teks sampai tepat
        // sebelum byte pertama payload sampel. Karena field header_len sendiri
        // ikut memengaruhi panjang teks, nilainya dihitung iteratif di bawah.
        header += "|header_len=";
        header += String((int)headerLen);

        header += "|flags=";
        header += String((int)FLAGS_DEFAULT);

        header += "|rep_id=";
        header += String((int)repId);

        header += "|seq=";
        header += String((unsigned long)seqCounter);

        header += "|t_send_us=";
        header += String((unsigned long long)tSendUs);

        header += "|sample_count=";
        header += String((int)packetSampleCount);

        header += "|channel_count=";
        header += String((int)CHANNEL_COUNT_1);

        header += "|sample_fmt=";
        header += String((int)SAMPLE_FMT_STRING_ASCII);

        header += "|payload_bytes=";
        header += String((int)payloadBytes);

        header += "|payload_crc32=";
        header += String((unsigned long)payloadCrc32);

        header += "|run_id=";
        header += String((unsigned long long)currentRunId);

        // Bagian setelah tanda '=' ini adalah payload sampel string.
        header += "|payload=";

        return header;
      };

      uint16_t stringHeaderLen = 0;
      String stringHeader;

      // Hitung header_len teks secara stabil.
      // Biasanya stabil dalam 2 iterasi, tetapi diberi 5 iterasi agar aman.
      for (int i = 0; i < 5; ++i) {
        stringHeader = buildStringHeader(stringHeaderLen);
        uint16_t newHeaderLen = (uint16_t)stringHeader.length();

        if (newHeaderLen == stringHeaderLen) {
          break;
        }

        stringHeaderLen = newHeaderLen;
      }

      String payload;
      payload.reserve(stringHeader.length() + sampleText.length());
      payload += stringHeader;
      payload += sampleText;

      mqttPublishText(TOPIC_DATA, payload);

    } else {
      const uint16_t payloadBytes = (uint16_t)(packetSampleCount * 2U);
      uint8_t payload[PACKET_SAMPLE_CAP * 2];

      for (size_t i = 0; i < packetSampleCount; ++i) {
        int16_t v = packetSamples[i];
        payload[2 * i]     = (uint8_t)(v & 0xFF);
        payload[2 * i + 1] = (uint8_t)((v >> 8) & 0xFF);
      }

      uint32_t crc = crc32Update(0, payload, payloadBytes);

      uint8_t header[HEADER_LEN];
      memset(header, 0, sizeof(header));

      uint8_t flags = (currentVariant == VARIANT_BIN_ZC)
        ? FLAG_ZEROCOPY_HINT
        : FLAGS_DEFAULT;

      // Layout header biner 38 byte sesuai proposal:
      //  0..1   magic              uint16 LE
      //  2      version            uint8
      //  3      msg_type           uint8
      //  4..5   header_len         uint16 LE
      //  6      flags              uint8
      //  7      rep_id             uint8
      //  8..11  seq                uint32 LE
      // 12..19  t_send_us          uint64 LE
      // 20..21  sample_count       uint16 LE
      // 22      channel_count      uint8
      // 23      sample_fmt         uint8
      // 24..25  payload_bytes      uint16 LE
      // 26..29  payload_crc32      uint32 LE
      // 30..37  run_id             uint64 LE
      // 38..    payload            N byte

      writeLe16(header + 0, MAGIC);
      header[2] = VERSION;
      header[3] = MSG_TYPE_DATA;
      writeLe16(header + 4, HEADER_LEN);
      header[6] = flags;
      header[7] = repId;
      writeLe32(header + 8, seqCounter);
      writeLe64(header + 12, tSendUs);
      writeLe16(header + 20, (uint16_t)packetSampleCount);
      header[22] = CHANNEL_COUNT_1;
      header[23] = SAMPLE_FMT_INT16_LE;
      writeLe16(header + 24, payloadBytes);
      writeLe32(header + 26, crc);
      writeLe64(header + 30, currentRunId);

      uint8_t frame[HEADER_LEN + PACKET_SAMPLE_CAP * 2];
      memcpy(frame, header, HEADER_LEN);
      memcpy(frame + HEADER_LEN, payload, payloadBytes);

      mqttPublishBinary(frame, HEADER_LEN + payloadBytes);
    }

    seqCounter++;
    packetSampleCount = 0;
    pumpMqtt();
  }

  // ===== Motion + sampling =====

  void moveStepper() {
    pulseWidth = 0;
    period = 0;
    measuredFrequency = 0.0f;
    measuredDutyCycle = 0.0f;
    blinkCount = 0;

    if (!isHomedRot || !isHomedLin) {
      homingLinear();
      homingRotation();
    }

    const uint8_t repId = (uint8_t)(bounceCount + 1);
    resetPacketBuffer();

    if (!modeLin) {
      Serial.print("Mode Rotasi, repetition ");
      Serial.println(repId);

      float steps_per_degree = 1.0f / DEGREE_PER_STEP;
      float targetSpeed = (angle * steps_per_degree) / speed;

      stepperRot.setMaxSpeed(targetSpeed);

      mainAdcEnabled = true;
      isCounting = true;
      stepperRot.moveTo(angle * steps_per_degree);

      while (stepperRot.distanceToGo() != 0) {
        stepperRot.run();
        readSensor(repId);
        pumpMqtt();
      }

      isCounting = false;
      mainAdcEnabled = false;

      flushPacket(repId);

      Serial.print("Measured Frequency: ");
      Serial.print(measuredFrequency, 2);
      Serial.print(" Hz, Duty Cycle: ");
      Serial.print(measuredDutyCycle, 2);
      Serial.print(" %, Blink Count: ");
      Serial.println(blinkCount);

      targetSpeed = (angle * steps_per_degree) / 0.05f;
      stepperRot.setMaxSpeed(targetSpeed);
      stepperRot.moveTo(0);

      while (stepperRot.distanceToGo() != 0) {
        stepperRot.run();
        pumpMqtt();
      }

      bounceCount++;
    } else {
      Serial.print("Mode Linear, repetition ");
      Serial.println(repId);

      float steps_per_mm = 1.0f / MM_PER_STEP;
      float targetSpeed = (distance * steps_per_mm) / speed;

      stepperLin.setMaxSpeed(targetSpeed);

      mainAdcEnabled = true;
      isCounting = true;
      stepperLin.moveTo(distance * steps_per_mm);

      while (stepperLin.distanceToGo() != 0) {
        stepperLin.run();
        readSensor(repId);
        pumpMqtt();
      }

      isCounting = false;
      mainAdcEnabled = false;

      flushPacket(repId);

      Serial.print("Measured Frequency: ");
      Serial.print(measuredFrequency, 2);
      Serial.print(" Hz, Duty Cycle: ");
      Serial.print(measuredDutyCycle, 2);
      Serial.print(" %, Blink Count: ");
      Serial.println(blinkCount);

      targetSpeed = (distance * steps_per_mm) / 0.05f;
      stepperLin.setMaxSpeed(targetSpeed);
      stepperLin.moveTo(0);

      while (stepperLin.distanceToGo() != 0) {
        stepperLin.run();
        pumpMqtt();
      }

      bounceCount++;
    }
  }

  void readSensor(uint8_t repId) {
    currentMillis = millis();
    if (currentMillis - previousMillis < sensorIntervalMs) {
      return;
    }
    previousMillis = currentMillis;

    int sensorValue = (int)lastMain12;
    appendSampleToPacket(sensorValue, repId);

    bool currentState = (sensorValue > 2000);

    if (currentState == HIGH && lastState == LOW) {
      lastRiseTime = currentMillis;
      period = currentMillis - lastPeriodTime;
      lastPeriodTime = currentMillis;

      if (period > 0) {
        measuredFrequency = 1000.0f / period;
      }

      pulseStartTime = currentMillis;
      if (isCounting) {
        blinkCount++;
      }
    }

    if (currentState == LOW && lastState == HIGH) {
      pulseEndTime = currentMillis;
      pulseWidth = pulseEndTime - pulseStartTime;

      if (period > 0 && pulseWidth <= period) {
        measuredDutyCycle = (pulseWidth * 100.0f) / period;
      }
    }

    lastState = currentState;
  }
};

InterferometerController interferometer;

void setup() {
  Heartbeat::begin();
  interferometer.begin();
}

void loop() {
  interferometer.run();
}