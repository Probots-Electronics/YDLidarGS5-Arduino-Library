#ifndef YDLIDAR_GS5_H
#define YDLIDAR_GS5_H

#include <Arduino.h>
#include <cstdint>

#define TX_PIN 17
#define RX_PIN 16

#define FETCH_DEVICE_INFO   0x61
#define FETCH_VERSION_INFO  0x62
#define START_SCAN          0x63
#define STOP_SCAN           0x64
#define SET_EDGE_MODE       0x69

#define MAX_SCAN_POINTS 160

struct LidarPoint {
  float angle;
  uint16_t distance;
  uint8_t intensity;
};

class YDLidarGS5 {
public:
  explicit YDLidarGS5(HardwareSerial& serial);

  void begin();

  bool getDeviceInfo();
  void getVersionInfo();

  bool startScan();
  void stopScan();

  bool getScanData(LidarPoint* points, uint16_t& count);
  void setEdge(uint8_t mode);

private:
  HardwareSerial& _serial;

  float d_compensateK0 = 0;
  float d_compensateB0 = 0;
  float d_compensateK1 = 0;
  float d_compensateB1 = 0;
  float bias = 0;

  void sendCommand(uint8_t cmd, const uint8_t* data, uint16_t len);
  int  readResponse(uint8_t* buffer, int maxLen);
};

#endif
