/*
 * ============================================================================
 * ESP32 AWS IoT Core -> UPUS-SKB Vending Machine Controller
 * ============================================================================
 *
 * This replaces the previous RS-485 layer (which implemented an unrelated
 * "M109E" 20-byte / CRC16-Modbus protocol) with the actual protocol from
 * "UPUS_Communications_Protocols - Technical Document V3":
 *
 *   Frame = Header(0x55) + Type(0xA1) + Address(1) + FuncCode(1) +
 *           DataLen(1) + Data(DataLen) + Parity(1)
 *   Parity = XOR of every byte before it (Header..last Data byte)
 *   Multi-byte data fields are big-endian.
 *   UART: 9600 baud, 8N1 (changeable via E0, not used here).
 *
 * Verified the parity formula against every worked example in the doc
 * (DF, DD, D9, D0, E0, D1, E1, D2, E2, E4, E5, E6, A0) - all match.
 *
 * Device ID: SV-KMXN5F-001 (change per device)
 *
 * Topics:
 * - Subscribe: machines/SV-KMXN5F/commands   (dispense commands come in here)
 * - Publish:   machines/SV-KMXN5F/telemetry  (status/heartbeat/results go out)
 *
 * To trigger a slot, publish this JSON to the subscribe topic:
 *     {"slot": 27, "duration": 6}
 *   slot     -> global slot number (1-based). Mapped to a board address +
 *               channel number using CHANNELS_PER_BOARD / BOARD_ADDRESSES
 *               below - edit those to match your actual hardware.
 *   duration -> seconds to power the channel (optional, defaults to 6)
 *
 * Result comes back as JSON on the telemetry topic:
 *     {"machine_id":"SV-KMXN5F","slot":27,"success":true}
 *
 * Dispense method: E1 "Write Channel Status" (CH silk-screen switch
 * electrical control) - power channel ON, wait `duration` seconds, power
 * it OFF. This is the method your header comment noted as "confirmed
 * reliable in prior CoolTerm testing" for this hardware.
 *
 * If your channels are wired with a drop/position sensor instead, use
 * motorControlUntilSignal() (function E4) as shown further down - it lets
 * the board itself run the motor until the sensor signal changes state
 * (with a timeout fallback), rather than a fixed ON/OFF duration.
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// =============================================================
// MACHINE ID - Change this for each machine
// =============================================================
#define MACHINE_ID "SV-KMXN5F"

// =============================================================
// WiFi Credentials
// =============================================================
const char* WIFI_SSID = "Sohub";
const char* WIFI_PASSWORD = "sohub@2022";

// =============================================================
// AWS IoT Core Endpoint
// =============================================================
#define AWS_IOT_ENDPOINT "a33heglrlmj5d9-ats.iot.ap-southeast-1.amazonaws.com"
#define AWS_IOT_PORT 443

// =============================================================
// AWS Root CA Certificate
// =============================================================
static const char AWS_CERT_CA[] = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

// =============================================================
// Device Certificate
// =============================================================
static const char AWS_CERT_CRT[] = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUUQKBSouzUde3XiuKZ6kQVeZ1DMEwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI2MDkxNzA0Mzgy
MloXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBANjbF/gZceIli4xS1U0Y
ZieecaAY2dkPkzcodNWqFIsrH//DggQj3v+qf3/8WwMiFKp8PAtLIosHgR28ZTbG
KBstw7Zz6WAemkRlXFwvbkZ932l50xm3wcAMyPE85vIrLNLDMq9/lTB6BMBrP7SC
4S6EwXOB2Cxp5nJ6MIYzG/ocjHpqxk/m9YIrMapk4LYzqsV3qP1RH11LNWX8f5Fe
cxwm4RXTxpm52SPwXD0/66ccA8D/sNh9fvxsgBsqg4EKv+rEkV7TLzsGIPM+xCZ8
cW0AzK9nEC/xFjIQorMdfvjJ4VSd5hIyU3lnoo9ye1OuFRUTSxIEbn1Mgz4ibc0q
QncCAwEAAaNgMF4wHwYDVR0jBBgwFoAUUjeFbnqNnHeFrjq0qKo32ssOoNYwHQYD
VR0OBBYEFHSg+YPb+/135O22Qqpcs14sllLfMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQB/U4M/h8/BX8Qh9yZvheSSem3s
HkAYQXLswFy7spmslKA6xg4Gzue1u83R+U1k0M+zpFfVVppsowh4Dgrhi/g9VRba
nOgzI34jG+sS5Cp1Fw44/fEof6cTWU4Z2hAkZhTk1y5x/80/zowQUkY4L8SyZ89h
ZxNGT1NsvO/RkHlaipDEF/JZ2TSDoDBrzeSpNscoYvts8bXBJ75UYiCVc+c+NCR7
2tI/BR1UaLGq6oReHG6IeziV0Ph/IQa1wM90dFxGDndBcSFgDLRl8KhBo6GdFeXv
3sNwTZQpQLgt3N/svJcT/ohHnIuzTJ6L9+k0d+QOFg+w61ko0+9zX1ev8b2E
-----END CERTIFICATE-----
)EOF";

// =============================================================
// Private Key
// =============================================================
static const char AWS_CERT_PRIVATE[] = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEogIBAAKCAQEA2NsX+Blx4iWLjFLVTRhmJ55xoBjZ2Q+TNyh01aoUiysf/8OC
BCPe/6p/f/xbAyIUqnw8C0siiweBHbxlNsYoGy3DtnPpYB6aRGVcXC9uRn3faXnT
GbfBwAzI8Tzm8iss0sMyr3+VMHoEwGs/tILhLoTBc4HYLGnmcnowhjMb+hyMemrG
T+b1gisxqmTgtjOqxXeo/VEfXUs1Zfx/kV5zHCbhFdPGmbnZI/BcPT/rpxwDwP+w
2H1+/GyAGyqDgQq/6sSRXtMvOwYg8z7EJnxxbQDMr2cQL/EWMhCisx1++MnhVJ3m
EjJTeWeij3J7U64VFRNLEgRufUyDPiJtzSpCdwIDAQABAoIBAHHlWMh47MR/PbT8
yIUOfJDwefNBOiH0OTjuKbcS9mlFxC+8CqyL/pK/hOm+rDqQCtSkHZE87C6jSWBJ
08sZ9nSX68QHSxqt2A52QjU0wDGL6OxoDMTkK4Hn9R4nA5FLteYYmaVtV/k3DQnX
CoWPfB/HoCc/mj9xe5TCsnu5mkcZAflI7m1XongeYzQ8hbbYfg+mWH4QXpAzSc7d
MuXKEcBSjhUGNuW+ppH/df9rFeeaNudWNITOT04pzL3hlJR6E7nJ0HWGJSpREVrl
aqnWbJ5oYeVL/0oQfo77QBTkBqD4bv6iS/N/2yP8wWG78TPan63JAfbD3hVzl87p
X0Emo2kCgYEA7TMxFP80nJeDiNuhMxiRuFEkkdI3vL6bNJYbzdqQJyahKFDf/A9L
Gj8f5TfLqYmkmNy5v0+Qif69aq1UHkAMZWfcj1dk57qZLP6fSmfisZsjIcHWtasr
F9Rm6JlUOmLdu1TRDxHyi9nrDs0IhAPhTcgk4yg8JBFo3XpemUPKIjUCgYEA6gse
HgCPICY+aVcjGXB07+b/YeU/tmeMXvPWTHCv4al3TiLcdBULmK73x1SURB45ZNsd
+A6sFn4YjZTSTxro27LX3fpSXKZkMbkHhuY2Cos2GkRdmrl9oZyMBZami/B2w/XA
CxYG7JSIQEOxyAMKZnLdKwaLM6GRGUjr4QM253sCgYBT80lM0CIzxeA04N7Fsc1p
ObbCsW8uqxtNWk9O5Ssd4CsrlKwhjtwa1vqX6s53G5MY0w4eqq9SV8l8Zwodhrec
cEz8oobBCSbLaPbBo9D04R5nBS7Tr1gaH/+oFRRdcb5eXd98APMiEieCXojECIeG
Ou7rQsM+VLcTpAq1Hev9fQKBgCehFEtuOs3ty9mYaJgRI6vPiGSyJbSkGe+mCB5P
YmiqgkoXuw5o1drXnZsB5k0WdQ9vzsZG7bqtRUkL7wR4AP/aa5TpovKPNh35P54G
8xWb9uBBvTvKtswbrtXvaWn+oI2T2h3GcAu1JtEC5WKBqUwpnOuBIybt75zBbTdf
jPdrAoGATNIKr/2lcwqZoMtLVUjZcWVnQuyHpDqIZi6yJruOaJrSfRsgN9nxFpI9
oeJhZ3j1PDhWIgwfwusq5EV+TKA8DiMvEJ/mT4VNBou87aAvcMOe4tVA+Tm1bDf5
RuC9Ry61l0aGlGiluozVSf6ucHD1h+Is1g4kIWwGFA2yZwhUso4=
-----END RSA PRIVATE KEY-----
)EOF";

// =============================================================
// MQTT Topics (auto-generated from MACHINE_ID)
// =============================================================
String TOPIC_SUBSCRIBE = "machines/" + String(MACHINE_ID) + "/commands";
String TOPIC_PUBLISH   = "machines/" + String(MACHINE_ID) + "/telemetry";

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// =============================================================
// RS-485 Wiring (UART2)
// =============================================================
#define RS485_BAUD    9600   // per doc: 9600, N, 8, 1 (default; changeable via E0)
#define RS485_TX_PIN  17     // GPIO17 (UART2 TX)
#define RS485_RX_PIN  16     // GPIO16 (UART2 RX)
#define RS485_DIR_PIN -1     // set to a GPIO if your transceiver needs manual DE/RE;
                              // -1 = auto-direction module (no manual toggling needed)

// =============================================================
// UPUS-SKB Protocol Constants (from the technical document)
// =============================================================
#define CMD_HEADER      0x55
#define TYPE_CODE       0xA1
#define ADDR_BROADCAST  0xFF

// Function codes (doc section "Function Code Introduction")
#define FUNC_BROADCAST_ADDRESSING  0xDF  // address must be broadcast (0xFF)
#define FUNC_READ_MCU_ID           0xDD
#define FUNC_READ_TASK_STATUS      0xD9
#define FUNC_READ_SYS_CONFIG       0xD0
#define FUNC_WRITE_SYS_CONFIG      0xE0
#define FUNC_READ_CHANNEL_STATUS   0xD1
#define FUNC_WRITE_CHANNEL_STATUS  0xE1  // CH silk-screen switch electrical control
#define FUNC_READ_CHANNEL_SIGNAL   0xD2
#define FUNC_UNLOCK                0xE2
#define FUNC_MOTOR_UNTIL_STATE     0xE4  // run until signal reaches specified state, then stop
#define FUNC_MOTOR_UNTIL_COUNT     0xE5  // run until signal changes N times, then stop
#define FUNC_MOTOR_UNTIL_REACH     0xE6  // run until signal reaches specified state, then stop
#define FUNC_AUTO_UPLOAD           0xA0

// =============================================================
// Hardware map: global slot number -> (board address, channel)
// =============================================================
// Address Code = dial-switch value + 1 (all-off = 1, all-on = 32). If you
// run several UPUS-SKB boards on the same RS-485 bus, list every board's
// dial-switch address here in the order their slots should be numbered.
const uint8_t BOARD_ADDRESSES[] = { 1 };   // e.g. {1, 2, 3} for three boards
const uint8_t NUM_BOARDS = sizeof(BOARD_ADDRESSES) / sizeof(BOARD_ADDRESSES[0]);

// Channels per board: confirmed UPUS-SKB25-V3.0 = 25 channels.
const uint8_t CHANNELS_PER_BOARD = 25;

const unsigned long DEFAULT_DURATION_SEC = 6;

// =============================================================
// Low-level RS-485 I/O
// =============================================================
void rs485Send(uint8_t* frame, int len) {
  if (RS485_DIR_PIN >= 0) {
    digitalWrite(RS485_DIR_PIN, HIGH);
    delayMicroseconds(10);
  }
  Serial2.write(frame, len);
  Serial2.flush();
  if (RS485_DIR_PIN >= 0) {
    digitalWrite(RS485_DIR_PIN, LOW);
  }
}

int rs485Read(uint8_t* buf, int maxLen, unsigned long timeoutMs) {
  unsigned long start = millis();
  int count = 0;
  while (millis() - start < timeoutMs && count < maxLen) {
    if (Serial2.available()) {
      buf[count++] = Serial2.read();
    }
  }
  return count;
}

// =============================================================
// UPUS-SKB frame build / parse
// Frame = Header + Type + Address + FuncCode + DataLen + Data[DataLen] + Parity
// Parity = XOR of every preceding byte (verified against every worked
// example in the doc, e.g. E1: 55 A1 01 E1 02 01 01 -> parity 16).
// =============================================================
int buildFrame(uint8_t address, uint8_t funcCode, const uint8_t* data, uint8_t dataLen, uint8_t* outFrame) {
  outFrame[0] = CMD_HEADER;
  outFrame[1] = TYPE_CODE;
  outFrame[2] = address;
  outFrame[3] = funcCode;
  outFrame[4] = dataLen;
  for (int i = 0; i < dataLen; i++) outFrame[5 + i] = data[i];

  uint8_t parity = 0;
  for (int i = 0; i < 5 + dataLen; i++) parity ^= outFrame[i];
  outFrame[5 + dataLen] = parity;

  return 6 + dataLen; // total frame length
}

// Sends a command and waits for a validated response.
// On success returns the response data length (>=0) and fills respData.
// On failure (timeout / bad header / address or func mismatch / bad parity)
// returns -1.
int sendUpusCommand(uint8_t address, uint8_t funcCode, const uint8_t* data, uint8_t dataLen,
                     uint8_t* respData, unsigned long timeoutMs = 300) {
  uint8_t frame[32];
  int frameLen = buildFrame(address, funcCode, data, dataLen, frame);

  Serial.print("   TX: ");
  for (int i = 0; i < frameLen; i++) Serial.printf("%02X ", frame[i]);
  Serial.println();

  while (Serial2.available()) Serial2.read(); // clear stale bytes
  rs485Send(frame, frameLen);

  uint8_t resp[64];
  int n = rs485Read(resp, sizeof(resp), timeoutMs);

  Serial.printf("   RX (%d bytes): ", n);
  if (n == 0) {
    Serial.println("(none)");
  } else {
    for (int i = 0; i < n; i++) Serial.printf("%02X ", resp[i]);
    Serial.println();
  }

  if (n < 6) { Serial.println("   \u274C No/short response"); return -1; }
  if (resp[0] != CMD_HEADER || resp[1] != TYPE_CODE) { Serial.println("   \u274C Bad header/type"); return -1; }
  if (resp[2] != address)  { Serial.println("   \u274C Address mismatch"); return -1; }
  if (resp[3] != funcCode) { Serial.println("   \u274C Function code mismatch"); return -1; }

  uint8_t respLen = resp[4];
  int expectedTotal = 5 + respLen + 1;
  if (n < expectedTotal) { Serial.println("   \u274C Incomplete frame"); return -1; }

  uint8_t parity = 0;
  for (int i = 0; i < 5 + respLen; i++) parity ^= resp[i];
  if (parity != resp[5 + respLen]) { Serial.println("   \u274C Parity check failed"); return -1; }

  for (int i = 0; i < respLen; i++) respData[i] = resp[5 + i];
  return respLen;
}

// =============================================================
// Function-code helpers
// =============================================================

// E1 - Write Channel Status: turn a channel's power on/off.
// Data: [channel][state] (state 0x00=off, 0x01=on)
// Resp: [channel][state]  (state 0xE5 = incorrect channel ID)
bool writeChannelStatus(uint8_t boardAddr, uint8_t channel, bool on) {
  uint8_t data[2] = { channel, (uint8_t)(on ? 0x01 : 0x00) };
  uint8_t resp[8];
  int n = sendUpusCommand(boardAddr, FUNC_WRITE_CHANNEL_STATUS, data, 2, resp);
  if (n < 2) return false;
  if (resp[1] == 0xE5) { Serial.println("   \u274C Incorrect channel ID"); return false; }
  return resp[1] == (on ? 0x01 : 0x00);
}

// D2 - Read Channel Signal Status (0 = on/high, 1 = off/low) for a single channel.
// Useful for confirming a drop sensor tripped. Returns -1 on failure.
int readChannelSignalStatus(uint8_t boardAddr, uint8_t channel) {
  uint8_t data[1] = { channel };
  uint8_t resp[8];
  int n = sendUpusCommand(boardAddr, FUNC_READ_CHANNEL_SIGNAL, data, 1, resp);
  if (n < 2) return -1;
  return resp[1];
}

// E4 - Motor Control: run the motor on `channel` until its signal reaches
// `stopOnHigh` state, or until `timeoutMs` elapses (doc's overtime unit is
// 100ms, max 255 units = 25500ms). Use this instead of writeChannelStatus()
// if your channels are wired with a drop/position sensor.
// Resp report: 0x01 completed, 0x03 overtime, 0xE2 task already running.
bool motorControlUntilSignal(uint8_t boardAddr, uint8_t channel, unsigned long timeoutMs, bool stopOnHigh) {
  uint8_t overtimeUnits = (uint8_t)min(timeoutMs / 100, (unsigned long)255);
  uint8_t data[3] = { channel, overtimeUnits, (uint8_t)(stopOnHigh ? 0x00 : 0x01) };
  uint8_t resp[8];
  // The board only replies once the motor task finishes or overtimes, so the
  // response wait must cover the full requested run time, not a fixed value.
  unsigned long waitMs = (unsigned long)overtimeUnits * 100 + 500;
  int n = sendUpusCommand(boardAddr, FUNC_MOTOR_UNTIL_STATE, data, 3, resp, waitMs);
  if (n < 2) return false;
  switch (resp[1]) {
    case 0x01: Serial.println("   \u2705 Motor: execution completed"); return true;
    case 0x03: Serial.println("   \u26A0\uFE0F  Motor: overtime"); return false;
    case 0xE2: Serial.println("   \u274C Motor: another task already running"); return false;
    default:   Serial.printf("   \u26A0\uFE0F  Motor: unexpected report 0x%02X\n", resp[1]); return false;
  }
}

// DD - Read MCU ID (diagnostic helper, not required for dispensing).
bool readMcuId(uint8_t boardAddr) {
  uint8_t resp[16];
  int n = sendUpusCommand(boardAddr, FUNC_READ_MCU_ID, nullptr, 0, resp);
  if (n <= 0) return false;
  Serial.print("   MCU ID: ");
  for (int i = 0; i < n; i++) Serial.printf("%02X", resp[i]);
  Serial.println();
  return true;
}

// =============================================================
// Slot mapping: global slot number -> (board address, channel)
// Channel numbers on the board are 1-based (CH silk-screen number).
// =============================================================
bool resolveSlot(int slot, uint8_t &boardAddr, uint8_t &channel) {
  if (slot < 1) return false;
  int boardIdx = (slot - 1) / CHANNELS_PER_BOARD;
  if (boardIdx >= NUM_BOARDS) return false;
  boardAddr = BOARD_ADDRESSES[boardIdx];
  channel = (uint8_t)(((slot - 1) % CHANNELS_PER_BOARD) + 1);
  return true;
}

// =============================================================
// Trigger a slot using E4 (Motor Control, run until signal reaches the
// specified state and stop) - confirmed working on this hardware:
//   55 A1 01 E4 03 02 3C 00 2C
//   = address 01, channel 02, overtime 0x3C (60 x 100ms = 6s), stop-on-high (00)
// STOP_ON_HIGH_LEVEL below controls that third data byte for every slot.
// =============================================================
#define STOP_ON_HIGH_LEVEL true   // true -> data byte 0x00 (stop when signal is high/connected)
                                   // false -> data byte 0x01 (stop when signal is low/off)

bool triggerSlot(int slot, unsigned long durationSec) {
  uint8_t boardAddr, channel;
  if (!resolveSlot(slot, boardAddr, channel)) {
    Serial.printf("\u274C Slot %d not in configured range (boards=%d x %d channels)\n",
                  slot, NUM_BOARDS, CHANNELS_PER_BOARD);
    return false;
  }

  Serial.printf("\U0001F527 Slot %d -> board 0x%02X, channel %d, duration %lus\n",
                slot, boardAddr, channel, durationSec);

  unsigned long timeoutMs = durationSec * 1000;
  return motorControlUntilSignal(boardAddr, channel, timeoutMs, STOP_ON_HIGH_LEVEL);
}

// =============================================================
// MQTT Callback
// =============================================================
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  Serial.print("\U0001F4E9 Message received on: ");
  Serial.println(topic);

  Serial.print("   Payload: ");
  for (unsigned int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println();

  // Expected payload: {"slot": 27, "duration": 6}
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.println("   \u274C JSON parse failed!");
    return;
  }

  if (!doc.containsKey("slot")) {
    Serial.println("   \u274C Missing 'slot' field in payload");
    return;
  }

  int slot = doc["slot"];
  unsigned long duration = doc["duration"] | DEFAULT_DURATION_SEC;

  bool ok = triggerSlot(slot, duration);

  String msg = "{\"machine_id\":\"" + String(MACHINE_ID) +
               "\",\"slot\":" + String(slot) +
               ",\"success\":" + String(ok ? "true" : "false") + "}";
  mqttClient.publish(TOPIC_PUBLISH.c_str(), msg.c_str());
}

// =============================================================
// Connect to WiFi
// =============================================================
void connectWiFi() {
  Serial.println("\n\U0001F310 Connecting to WiFi...");
  Serial.print("   SSID: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n\u2705 WiFi Connected!");
    Serial.print("   IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("   Signal: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("\n\u274C WiFi Failed!");
  }
}

// =============================================================
// Sync Time
// =============================================================
void syncTime() {
  Serial.println("\n\u23F0 Syncing time...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  struct tm timeinfo;
  int retry = 0;
  while (!getLocalTime(&timeinfo) && retry < 15) {
    Serial.print(".");
    delay(500);
    retry++;
  }

  if (getLocalTime(&timeinfo)) {
    Serial.println("\n\u2705 Time synced!");
    Serial.print("   ");
    Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
  }
}

// =============================================================
// Connect to AWS IoT
// =============================================================
void connectAWS() {
  Serial.println("\n\u2601\uFE0F  Connecting to AWS IoT...");
  Serial.print("   Client ID: ");
  Serial.println(MACHINE_ID);

  if (mqttClient.connect(MACHINE_ID)) {
    Serial.println("\u2705 MQTT Connected!");

    if (mqttClient.subscribe(TOPIC_SUBSCRIBE.c_str())) {
      Serial.print("\u2705 Subscribed: ");
      Serial.println(TOPIC_SUBSCRIBE);
    }

    String msg = "{\"machine_id\":\"" + String(MACHINE_ID) +
                 "\",\"status\":\"online\"}";
    mqttClient.publish(TOPIC_PUBLISH.c_str(), msg.c_str());
    Serial.print("\u2705 Published: ");
    Serial.println(TOPIC_PUBLISH);

  } else {
    Serial.print("\u274C MQTT Failed! Code: ");
    Serial.println(mqttClient.state());
  }
}

// =============================================================
// Setup
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  ESP32 AWS IoT - UPUS-SKB Vending Ctrl");
  Serial.println("========================================");
  Serial.print("  Machine: ");
  Serial.println(MACHINE_ID);
  Serial.printf("  Boards: %d x %d channels each\n", NUM_BOARDS, CHANNELS_PER_BOARD);
  Serial.println("========================================\n");

  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  while (Serial2.available()) Serial2.read();
  if (RS485_DIR_PIN >= 0) {
    pinMode(RS485_DIR_PIN, OUTPUT);
    digitalWrite(RS485_DIR_PIN, LOW);
  }
  Serial.printf("\u2705 UART2 configured: %d baud (TX:GPIO%d, RX:GPIO%d)\n\n",
                RS485_BAUD, RS485_TX_PIN, RS485_RX_PIN);

  connectWiFi();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\u26A0\uFE0F  No WiFi. Restarting in 10s...");
    delay(10000);
    ESP.restart();
  }

  syncTime();

  wifiClient.setCACert(AWS_CERT_CA);
  wifiClient.setCertificate(AWS_CERT_CRT);
  wifiClient.setPrivateKey(AWS_CERT_PRIVATE);

  static const char *alpnProtocols[] = {"x-amzn-mqtt-ca", NULL};
  wifiClient.setAlpnProtocols(alpnProtocols);

  mqttClient.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(2048);
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(30);

  Serial.println("\u2705 MQTT configured\n");

  connectAWS();
}

// =============================================================
// Loop
// =============================================================
void loop() {
  static unsigned long lastReconnect = 0;
  static unsigned long lastHeartbeat = 0;

  mqttClient.loop();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\u26A0\uFE0F  WiFi lost! Reconnecting...");
    connectWiFi();
    delay(5000);
    return;
  }

  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastReconnect > 5000) {
      lastReconnect = now;
      Serial.println("\u26A0\uFE0F  MQTT lost! Reconnecting...");
      connectAWS();
    }
  }

  if (mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastHeartbeat > 30000) {
      lastHeartbeat = now;

      String msg = "{\"machine_id\":\"" + String(MACHINE_ID) +
                   "\",\"status\":\"alive\",\"uptime\":" +
                   String(millis() / 1000) + "}";
      mqttClient.publish(TOPIC_PUBLISH.c_str(), msg.c_str());

      Serial.print("\U0001F49A Heartbeat (uptime: ");
      Serial.print(millis() / 1000);
      Serial.println("s)");
    }
  }

  delay(100);
}
