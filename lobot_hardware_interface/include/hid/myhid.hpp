#ifndef MYHID_H
#define MYHID_H

#include <initializer_list>
#include <stdexcept>
#include <vector>

#include "hid/hidapi.h"

#define FRAME_HEADER 0x55

class MyHid
{
public:
  MyHid() = default;

  MyHid(const unsigned short vendor_id, const unsigned short product_id)
    : vendor_id_(vendor_id), product_id_(product_id)
  {
  }

  void close();

  unsigned short getProductId() const
  {
    return product_id_;
  }

  unsigned short getVendorId() const
  {
    return vendor_id_;
  }

  bool isConnected() const
  {
    return connected_;
  }

  int makeAndSendCmd(const unsigned cmd);

  int makeAndSendCmd(const unsigned cmd, const std::initializer_list<unsigned>& argv);

  int makeAndSendCmd(const unsigned cmd, const std::vector<unsigned>& argv);

  void open();

  void read(std::vector<unsigned>& data, const size_t length);

  void setProductId(const unsigned short product_id)
  {
    product_id_ = product_id;
  }

  void setVendorId(const unsigned short vendor_id)
  {
    vendor_id_ = vendor_id;
  }

private:
  unsigned short vendor_id_;
  unsigned short product_id_;
  bool connected_ = false;
  hid_device* device_ = nullptr;
};

inline void MyHid::close()
{
  if (connected_)
  {
    hid_close(device_);
    hid_exit();
  }
}

inline int MyHid::makeAndSendCmd(const unsigned cmd)
{
  if (!connected_)
  {
    return -1;
  }

  unsigned char send_buffer[256] = { 0 };
  send_buffer[0] = 0x00;  // Report ID
  send_buffer[1] = FRAME_HEADER;
  send_buffer[2] = FRAME_HEADER;
  send_buffer[3] = 2;
  send_buffer[4] = static_cast<unsigned char>(cmd);
  if (hid_write(device_, send_buffer, send_buffer[3] + 3) != -1)
  {
    return 0;
  }
  return -1;
}

inline int MyHid::makeAndSendCmd(const unsigned cmd, const std::initializer_list<unsigned>& argv)
{
  if (!connected_)
  {
    return -1;
  }

  unsigned cnt = 0;
  unsigned char send_buffer[256] = { 0 };
  send_buffer[0] = 0x00;  // Report ID
  send_buffer[1] = FRAME_HEADER;
  send_buffer[2] = FRAME_HEADER;
  send_buffer[4] = static_cast<unsigned char>(cmd);
  for (const auto& arg : argv)
  {
    send_buffer[5 + cnt] = static_cast<unsigned char>(arg);
    ++cnt;
  }
  send_buffer[3] = static_cast<unsigned char>(cnt + 2);
  if (hid_write(device_, send_buffer, send_buffer[3] + 3) != -1)
  {
    return 0;
  }
  return -1;
}

inline int MyHid::makeAndSendCmd(const unsigned cmd, const std::vector<unsigned>& argv)
{
  if (!connected_)
  {
    return -1;
  }

  unsigned cnt = 0;
  unsigned char send_buffer[256] = { 0 };
  send_buffer[0] = 0x00;  // Report ID
  send_buffer[1] = FRAME_HEADER;
  send_buffer[2] = FRAME_HEADER;
  send_buffer[4] = static_cast<unsigned char>(cmd);
  for (const auto& arg : argv)
  {
    send_buffer[5 + cnt] = static_cast<unsigned char>(arg);
    ++cnt;
  }
  send_buffer[3] = static_cast<unsigned char>(cnt + 2);
  if (hid_write(device_, send_buffer, send_buffer[3] + 3) != -1)
  {
    return 0;
  }
  return -1;
}

inline void MyHid::open()
{
  hid_init();

  device_ = hid_open(vendor_id_, product_id_, nullptr);
  if (!device_)
  {
    throw std::runtime_error("Cannot open HID device!");
  }

  connected_ = true;
}

inline void MyHid::read(std::vector<unsigned>& data, const size_t length)
{
  if (length > 256)
  {
    return;
  }

  data.reserve(length);
  unsigned char receive_buffer[256] = { 0 };

  hid_read(device_, receive_buffer, length + 2);
  if (receive_buffer[0] == FRAME_HEADER && receive_buffer[1] == FRAME_HEADER && receive_buffer[2] == length)
  {
    size_t i = 3;
    while (i < length + 2)
    {
      data.push_back(receive_buffer[i]);
      ++i;
    }
  }
  return;
}

#endif  // MYHID_H
