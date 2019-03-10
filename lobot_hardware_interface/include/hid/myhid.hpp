#ifndef MYHID_H
#define MYHID_H

#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "hid/hidapi.h"

#define FRAME_HEADER 0x55

class MyHid {
 public:
  MyHid() = default;
  MyHid(const unsigned short vi, const unsigned short pi)
      : vendorId(vi), productId(pi) {}

  void Close() { hid_close(myDevice); }
  unsigned short GetProductId() const { return productId; }
  unsigned short GetVendorId() const { return vendorId; }
  bool IsConnected() const { return connected; }
  int MakeAndSendCmd(const unsigned cmd);
  int MakeAndSendCmd(const unsigned cmd,
                     const std::initializer_list<unsigned> &argv);
  int MakeAndSendCmd(const unsigned cmd, const std::vector<unsigned> &argv);
  std::vector<unsigned> Read(const size_t length);
  int Open();
  void SetProductId(const unsigned short pi) { productId = pi; }
  void SetVendorId(const unsigned short vi) { vendorId = vi; }

 private:
  unsigned short vendorId;
  unsigned short productId;
  bool connected = false;
  hid_device *myDevice = nullptr;
};

inline int MyHid::Open() {
  try {
    if (hid_init()) {
      throw std::runtime_error("HID initialization failed!");
    }

    myDevice = hid_open(vendorId, productId, nullptr);
    if (!myDevice) {
      throw std::runtime_error("Cannot open HID device!");
    }
  } catch (std::runtime_error err) {
    std::cerr << err.what() << std::endl;
    return -1;
  }
  connected = true;
  return 0;
}

inline int MyHid::MakeAndSendCmd(const unsigned cmd) {
  if (!connected) {
    return -1;
  }

  unsigned char sendBuff[256] = {0};
  sendBuff[0] = 0x00;  // Report ID
  sendBuff[1] = FRAME_HEADER;
  sendBuff[2] = FRAME_HEADER;
  sendBuff[3] = 2;
  sendBuff[4] = static_cast<unsigned char>(cmd);
  if (hid_write(myDevice, sendBuff, sendBuff[3] + 3) != -1) {
    return 0;
  }
  return -1;
}

inline int MyHid::MakeAndSendCmd(const unsigned cmd,
                                 const std::initializer_list<unsigned> &argv) {
  if (!connected) {
    return -1;
  }

  unsigned cnt = 0;
  unsigned char sendBuff[256] = {0};
  sendBuff[0] = 0x00;  // Report ID
  sendBuff[1] = FRAME_HEADER;
  sendBuff[2] = FRAME_HEADER;
  sendBuff[4] = static_cast<unsigned char>(cmd);
  for (const auto arg : argv) {
    sendBuff[5 + cnt] = static_cast<unsigned char>(arg);
    ++cnt;
  }
  sendBuff[3] = static_cast<unsigned char>(cnt + 2);
  if (hid_write(myDevice, sendBuff, sendBuff[3] + 3) != -1) {
    return 0;
  }
  return -1;
}

inline int MyHid::MakeAndSendCmd(const unsigned cmd,
                                 const std::vector<unsigned> &argv) {
  if (!connected) {
    return -1;
  }

  unsigned cnt = 0;
  unsigned char sendBuff[256] = {0};
  sendBuff[0] = 0x00;  // Report ID
  sendBuff[1] = FRAME_HEADER;
  sendBuff[2] = FRAME_HEADER;
  sendBuff[4] = static_cast<unsigned char>(cmd);
  for (const auto arg : argv) {
    sendBuff[5 + cnt] = static_cast<unsigned char>(arg);
    ++cnt;
  }
  sendBuff[3] = static_cast<unsigned char>(cnt + 2);
  if (hid_write(myDevice, sendBuff, sendBuff[3] + 3) != -1) {
    return 0;
  }
  return -1;
}

inline std::vector<unsigned> MyHid::Read(const size_t length) {
  std::vector<unsigned> data;
  data.reserve(length);
  if (length > 256) {
    return data;
  }
  unsigned char recvBuff[256] = {0};

  hid_read(myDevice, recvBuff, length + 2);
  if (recvBuff[0] == FRAME_HEADER && recvBuff[1] == FRAME_HEADER &&
      recvBuff[2] == length) {
    size_t i = 3;
    while (i < length + 2) {
      data.push_back(recvBuff[i]);
      ++i;
    }
  }
  return data;
}

#endif  // MYHID_H
