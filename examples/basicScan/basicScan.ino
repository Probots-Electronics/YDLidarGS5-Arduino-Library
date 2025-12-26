#include "YDLidarGS5.h"

#define DEBUG_BAUD 115200
#define LIDAR_BAUD 921600

YDLidarGS5 lidar(Serial2);
bool done = false;

void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(2000);

  Serial.println("\n=== YDLidar GS5 Robot Scan ===");

  Serial2.begin(
    LIDAR_BAUD,
    SERIAL_8N1,
    RX_PIN,
    TX_PIN
  );

  lidar.begin();
  lidar.stopScan();
  delay(200);

  Serial.println("Fetching device info...");
  if (!lidar.getDeviceInfo()) {
    Serial.println("ERROR: Failed to get device info. Halting.");
    while (1) {}
  }
  delay(200);

  Serial.println("Fetching version info...");
  lidar.getVersionInfo();
  delay(200);

  Serial.println("Setting edge mode...");
  lidar.setEdge(1);
  delay(500);
}

void loop() {
  if (done)
    return;

  for (int scan = 1; scan <= 10; scan++) {

    Serial.printf("\n--- Scan %d ---\n", scan);

    if (!lidar.startScan()) {
      Serial.println("ERROR: Failed to start scan");
      break;
    }

    delay(120);  // allow frame to arrive

    LidarPoint points[MAX_SCAN_POINTS];
    uint16_t count = 0;

    if (!lidar.getScanData(points, count)) {
      Serial.println("ERROR: No valid scan data");
      lidar.stopScan();
      continue;
    }

    Serial.printf("Valid points: %d\n", count);
    Serial.println("Angle(deg)\tDistance(mm)\tIntensity");

    uint16_t minDist = 65535;
    uint16_t maxDist = 0;

    for (uint16_t i = 0; i < count; i++) {

      Serial.printf(
        "%7.2f\t\t%5d\t\t%3d\n",
        points[i].angle,
        points[i].distance,
        points[i].intensity
      );

      if (points[i].distance < minDist)
        minDist = points[i].distance;
      if (points[i].distance > maxDist)
        maxDist = points[i].distance;
    }

    Serial.printf(
      "Distance range: %d mm → %d mm\n",
      minDist,
      maxDist
    );

    lidar.stopScan();
    delay(300);
  }

  Serial.println("\n=== Scanning complete ===");
  done = true;
}
