#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <mcp_can.h>
#include <Arduino_LSM6DSOX.h>

// ---------------------------
// USER CONFIGURABLE SETTINGS
// ---------------------------
#define SD_CS 4              // SD card CS pin
#define CAN_CS 10            // MCP2515 CS pin
#define CAN_INT 9            // MCP2515 INT pin
#define POT_PIN A0           // Rotary potentiometer pin
const int potMidRaw = 2048;  // Midpoint for 12-bit range (4095 / 2)

int left_angle;
int right_angle;
int angle;

// Logging frequency (Line ~25)
// Lower = faster logging. Example: 100 = log every 100 ms (~10 Hz).
const unsigned long LOG_INTERVAL_MS = 100;

// ---------------------------
// Global Objects
// ---------------------------
RTC_DS3231 rtc;
MCP_CAN CAN0(CAN_CS);
File kellyFile, bmsFile, imuFile;
char kellyFilename[20], bmsFilename[20], imuFilename[20];

// ---------------------------
// Kelly Data Variables
// ---------------------------
float speedRPM, motorCurrent, batteryVoltage;
float throttleVal, ctrlTemp, motorTemp;
uint16_t errorCode;
String statusStr;

// ---------------------------
// BMS Data Variables (example signals)
// ---------------------------
float packVoltage, packCurrent, soc;
float cellTemp, maxCellTemp, minCellTemp;

// ---------------------------
// IMU + Rotary Pot Variables
// ---------------------------
float r[3] = {0.5, 0.0, 0.0}; // Vector for external point

// ---------------------------
// Helpers
// ---------------------------
String getTimestamp() {
  DateTime now = rtc.now();
  unsigned long ms = millis() % 1000;
  char ts[30];
  sprintf(ts, "%04d-%02d-%02d %02d:%02d:%02d.%03lu",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second(), ms);
  return String(ts);
}

void createNewLogFiles() {
  int idx = 1;
  // Kelly
  do { sprintf(kellyFilename, "kelly_%d.csv", idx); idx++; } while (SD.exists(kellyFilename));
  kellyFile = SD.open(kellyFilename, FILE_WRITE);
  if (kellyFile) {
    kellyFile.println("Timestamp,Speed,RPM,Current(A),Voltage(V),Throttle(V),CtrlTemp(C),MotorTemp(C),Error,Status");
    kellyFile.close();
  }

  // BMS
  idx = 1;
  do { sprintf(bmsFilename, "bms_%d.csv", idx); idx++; } while (SD.exists(bmsFilename));
  bmsFile = SD.open(bmsFilename, FILE_WRITE);
  if (bmsFile) {
    bmsFile.println("Timestamp,PackVoltage(V),PackCurrent(A),SOC(%),CellTemp(C),MaxCellTemp(C),MinCellTemp(C)");
    bmsFile.close();
  }

  // IMU
  idx = 1;
  do { sprintf(imuFilename, "imu_%d.csv", idx); idx++; } while (SD.exists(imuFilename));
  imuFile = SD.open(imuFilename, FILE_WRITE);
  if (imuFile) {
    imuFile.println("Timestamp,Ax(m/s^2),Ay(m/s^2),Az(m/s^2),Gx(rad/s),Gy(rad/s),Gz(rad/s),PotAngle(deg)");
    imuFile.close();
  }

  Serial.println("Log files created.");
}

// ---------------------------
// Setup
// ---------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // SD Init
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed!");
    while (1);
  }
  Serial.println("SD init OK.");

  // RTC Init
  if (!rtc.begin()) {
    Serial.println("RTC failed!");
    while (1);
  }

  // CAN Init
  if (CAN0.begin(MCP_ANY, 250000, MCP_16MHZ) == CAN_OK)
    Serial.println("CAN Init OK!");
  else {
    Serial.println("CAN Init Failed!");
    while (1);
  }
  CAN0.setMode(MCP_NORMAL);

  // IMU Init
  if (!IMU.begin()) {
    Serial.println("IMU failed!");
    while (1);
  }
  Serial.println("IMU OK.");

  // Create files
  createNewLogFiles();
}

// ---------------------------
// Loop
// ---------------------------
void loop() {
  static unsigned long lastLog = 0;
  if (millis() - lastLog < LOG_INTERVAL_MS) return;
  lastLog = millis();

  String timestamp = getTimestamp();

  // ---------------- KELLY CAN ----------------
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    if (rxId == 0x0CF11E05) {
      speedRPM = (rxBuf[1] << 8 | rxBuf[0]);
      motorCurrent = (rxBuf[3] << 8 | rxBuf[2]) / 10.0;
      batteryVoltage = (rxBuf[5] << 8 | rxBuf[4]) / 10.0;
      errorCode = (rxBuf[7] << 8 | rxBuf[6]);
    }

    if (rxId == 0x0CF11F05) {
      throttleVal = (rxBuf[0] / 255.0) * 5.0;
      ctrlTemp = rxBuf[1] - 40;
      motorTemp = rxBuf[2] - 30;

      byte status = rxBuf[4];
      if (status == 0) statusStr = "Neutral";
      else if (status == 1) statusStr = "Forward";
      else if (status == 2) statusStr = "Backward";
    }

    kellyFile = SD.open(kellyFilename, FILE_WRITE);
    if (kellyFile) {
      kellyFile.print(timestamp); kellyFile.print(",");
      kellyFile.print(speedRPM); kellyFile.print(",");
      kellyFile.print(motorCurrent); kellyFile.print(",");
      kellyFile.print(batteryVoltage); kellyFile.print(",");
      kellyFile.print(throttleVal); kellyFile.print(",");
      kellyFile.print(ctrlTemp); kellyFile.print(",");
      kellyFile.print(motorTemp); kellyFile.print(",");
      kellyFile.print(errorCode); kellyFile.print(",");
      kellyFile.println(statusStr);
      kellyFile.close();
    }
  }

  // ---------------- BMS CAN ----------------
  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    if (rxId == 0x100) { // Example ID for pack voltage/current/SOC
      packVoltage = (rxBuf[1] << 8 | rxBuf[0]) * 0.1;
      packCurrent = (rxBuf[3] << 8 | rxBuf[2]) * 0.1;
      soc = rxBuf[4] * 0.5;
    }
    if (rxId == 0x101) { // Example ID for temps
      cellTemp = rxBuf[0] - 40;
      maxCellTemp = rxBuf[1] - 40;
      minCellTemp = rxBuf[2] - 40;
    }

    bmsFile = SD.open(bmsFilename, FILE_WRITE);
    if (bmsFile) {
      bmsFile.print(timestamp); bmsFile.print(",");
      bmsFile.print(packVoltage); bmsFile.print(",");
      bmsFile.print(packCurrent); bmsFile.print(",");
      bmsFile.print(soc); bmsFile.print(",");
      bmsFile.print(cellTemp); bmsFile.print(",");
      bmsFile.print(maxCellTemp); bmsFile.print(",");
      bmsFile.println(minCellTemp);
      bmsFile.close();
    }
  }

  // ---------------- IMU + Pot ----------------
  float ax, ay, az, gx, gy, gz;
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // Convert accel to m/s^2
    ax *= 9.80665; ay *= 9.80665; az *= 9.80665;

    // Convert gyro to rad/s
    gx *= PI / 180.0; gy *= PI / 180.0; gz *= PI / 180.0;

    // Rotary Pot
    int sensorValue10 = analogRead(POT_PIN);  // device effectively outputs 0–1023

   // Convert 10-bit reading to 12-bit scale (multiply range by 4)
    int sensorValue = sensorValue10 << 2;  // same as sensorValue10 * 4

  // Mapping values (now using 12-bit range)
  if (sensorValue < potMidRaw) {
    // Left rotation
    left_angle = map(sensorValue, 0, potMidRaw - 1, -135, 0);
    angle = left_angle;
  }
  else if (sensorValue > potMidRaw) {
    // Right rotation
    right_angle = map(sensorValue, potMidRaw + 1, 4095, 0, 135);
    angle = right_angle;
  }
  else {
    // Exact midpoint
    angle = 0;
  }

    imuFile = SD.open(imuFilename, FILE_WRITE);
    if (imuFile) {
      imuFile.print(timestamp); imuFile.print(",");
      imuFile.print(ax); imuFile.print(",");
      imuFile.print(ay); imuFile.print(",");
      imuFile.print(az); imuFile.print(",");
      imuFile.print(gx); imuFile.print(",");
      imuFile.print(gy); imuFile.print(",");
      imuFile.print(gz); imuFile.print(",");
      imuFile.println(angle);
      imuFile.close();
    }
  }
}

