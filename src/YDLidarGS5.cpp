#include "YDLidarGS5.h"
#include <math.h>
#include <string.h>


YDLidarGS5::YDLidarGS5(HardwareSerial& serial)
  : _serial(serial),
    d_compensateK0(0),
    d_compensateB0(0),
    d_compensateK1(0),
    d_compensateB1(0),
    bias(0) {
}

void YDLidarGS5::begin() {
}

/* ================= CHECKSUM ================= */
static uint8_t gs5Checksum(const uint8_t* data, uint16_t len)
{
  uint8_t sum = 0;
  for (uint16_t i = 0; i < len; i++)
    sum += data[i];
  return sum;
}

/* ================= SEND COMMAND ================= */

void YDLidarGS5::sendCommand(uint8_t cmd,
                            const uint8_t* data,
                            uint16_t len)
{
  while (_serial.available())
    _serial.read();

  const uint8_t header[4] = {0xA5, 0xA5, 0xA5, 0xA5};
  _serial.write(header, 4);

  uint8_t frame[4 + len];
  frame[0] = 0x00;              
  frame[1] = cmd;
  frame[2] = len & 0xFF;
  frame[3] = (len >> 8) & 0xFF;

  if (data && len)
    memcpy(&frame[4], data, len);

  _serial.write(frame, 4 + len);
  _serial.write(gs5Checksum(frame, 4 + len));
}


/* ================= READ RESPONSE ================= */

int YDLidarGS5::readResponse(uint8_t* buffer, int maxLen)
{
  int count = 0;
  unsigned long start = millis();

  while (millis() - start < 120) {
    while (_serial.available() && count < maxLen) {
      buffer[count++] = _serial.read();
    }
  }
  return count;
}

/* ================= DEVICE INFO ================= */

bool YDLidarGS5::getDeviceInfo()
{
  sendCommand(FETCH_DEVICE_INFO, nullptr, 0);

  uint8_t buf[32];
  int n = readResponse(buf, sizeof(buf));
  if (n < 18)
    return false;

  for (int i = 0; i + 17 < n; i++) {
    if (buf[i] == 0xA5 && buf[i+1] == 0xA5 &&
        buf[i+2] == 0xA5 && buf[i+3] == 0xA5) {

      const uint8_t* p = &buf[i + 8];

      int16_t K0   = (p[1] << 8) | p[0];
      int16_t B0   = (p[3] << 8) | p[2];
      int16_t K1   = (p[5] << 8) | p[4];
      int16_t B1   = (p[7] << 8) | p[6];
      int8_t  Bias = p[8];

      d_compensateK0 = K0 / 10000.0f;
      d_compensateB0 = B0 / 10000.0f;
      d_compensateK1 = K1 / 10000.0f;
      d_compensateB1 = B1 / 10000.0f;
      bias           = Bias / 10.0f;
      return true;
    }
  }
  return false;
}

void YDLidarGS5::getVersionInfo()
{
  sendCommand(FETCH_VERSION_INFO, nullptr, 0);
  uint8_t buf[32];
  readResponse(buf, sizeof(buf));
}

/* ================= EDGE MODE ================= */

void YDLidarGS5::setEdge(uint8_t mode)
{
  uint8_t payload[1] = { mode };
  sendCommand(SET_EDGE_MODE, payload, 1);
  delay(800);
}

/* ================= SCAN CONTROL ================= */

bool YDLidarGS5::startScan()
{
  sendCommand(START_SCAN, nullptr, 0);

  uint8_t buf[16];
  int n = readResponse(buf, sizeof(buf));

  for (int i = 0; i + 3 < n; i++) {
    if (buf[i] == 0xA5 && buf[i+1] == 0xA5 &&
        buf[i+2] == 0xA5 && buf[i+3] == 0xA5)
      return true;
  }
  return false;
}

void YDLidarGS5::stopScan()
{
  sendCommand(STOP_SCAN, nullptr, 0);
  delay(100);
  while (_serial.available())
    _serial.read();
}

/* ================= ROBOT-READY SCAN ================= */

bool YDLidarGS5::getScanData(LidarPoint* points, uint16_t& count)
{
  static uint8_t buf[400];
  int n = readResponse(buf, sizeof(buf));

  if (n < 330)
    return false;

  int h = -1;
  for (int i = 0; i + 3 < n; i++) {
    if (buf[i] == 0xA5 && buf[i+1] == 0xA5 &&
        buf[i+2] == 0xA5 && buf[i+3] == 0xA5) {
      h = i;
      break;
    }
  }

  if (h < 0)
    return false;

  int data = h + 8 + 2; // header + frame + ENV

  constexpr int RAW_POINTS = 160;
  count = 0;

  for (int i = 0; i < RAW_POINTS && count < MAX_SCAN_POINTS; i++) {

    int idx = data + i * 2;
    if (idx + 1 >= n)
      break;

    uint16_t raw = (buf[idx + 1] << 8) | buf[idx];
    uint16_t dist_raw = raw & 0x07FF; // Use 11 bits for distance
    uint8_t  inten = raw >> 11;      // Use 5 bits for intensity

    if (dist_raw == 0 || inten < 8)
      continue;

    // Geometric compensation from the YDLIDAR GS5 Development Manual
    float i_biased = i - bias;
    float angle_p = atan((1 - d_compensateK0 - d_compensateB0 * i_biased) / (d_compensateK0 + d_compensateB0 * i_biased));
    float angle_q = atan((1 - d_compensateK1 - d_compensateB1 * i_biased) / (d_compensateK1 + d_compensateB1 * i_biased));
    
    float angle_i = (angle_p + angle_q) / 2.0f;
    float corrected_dist = dist_raw * cos(angle_q - angle_i) * (d_compensateK1 + d_compensateB1 * i_biased) / cos(angle_q);

    points[count].angle = angle_i * 180.0f / M_PI;
    points[count].distance  = static_cast<uint16_t>(corrected_dist);
    points[count].intensity = inten;

    count++;
  }

  return count > 0;
}
