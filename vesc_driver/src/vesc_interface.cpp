// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_interface.h"

#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <io_context/io_context.hpp>
#include <serial_driver/serial_driver.hpp>

#include "vesc_driver/vesc_packet_factory.h"

namespace vesc_driver
{

class VescInterface::Impl
{
public:
  Impl()
  : io_context_(1),
    serial_driver_(io_context_),
    rx_thread_run_(false),
    rx_buffer_()
  {}

  void rxThread();
  void processBuffer();

  drivers::common::IoContext io_context_;
  drivers::serial_driver::SerialDriver serial_driver_;
  std::shared_ptr<drivers::serial_driver::SerialPort> serial_port_;
  std::thread rx_thread_;
  bool rx_thread_run_;
  PacketHandlerFunction packet_handler_;
  ErrorHandlerFunction error_handler_;
  Buffer rx_buffer_;
};

void VescInterface::Impl::rxThread()
{
  std::vector<uint8_t> read_buffer(4096);

  while (rx_thread_run_) {
    size_t bytes_read = 0;
    try {
      bytes_read = serial_port_->receive(read_buffer);
    } catch (const std::exception & e) {
      if (rx_thread_run_ && error_handler_) {
        error_handler_(e.what());
      }
      continue;
    }

    if (bytes_read == 0) {
      continue;
    }

    rx_buffer_.insert(rx_buffer_.end(), read_buffer.begin(), read_buffer.begin() + bytes_read);
    processBuffer();
  }
}

void VescInterface::Impl::processBuffer()
{
  int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
  if (!rx_buffer_.empty())
  {
    Buffer::iterator iter(rx_buffer_.begin());
    Buffer::iterator iter_begin(rx_buffer_.begin());
    while (iter != rx_buffer_.end())
    {
      if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *iter ||
        VescFrame::VESC_SOF_VAL_LARGE_FRAME == *iter)
      {
        std::string error;
        VescPacketConstPtr packet =
          VescPacketFactory::createPacket(iter, rx_buffer_.end(), &bytes_needed, &error);
        if (packet)
        {
          if (std::distance(iter_begin, iter) > 0)
          {
            std::ostringstream ss;
            ss << "Out-of-sync with VESC, unknown data leading valid frame. Discarding "
               << std::distance(iter_begin, iter) << " bytes.";
            if (error_handler_) {
              error_handler_(ss.str());
            }
          }
          if (packet_handler_) {
            packet_handler_(packet);
          }
          iter = iter + packet->frame().size();
          iter_begin = iter;
          continue;
        } else if (bytes_needed > 0) {
          break;
        } else {
          if (error_handler_) {
            error_handler_(error);
          }
        }
      }

      ++iter;
    }

    if (iter == rx_buffer_.end()) {
      bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
    }

    if (std::distance(iter_begin, iter) > 0)
    {
      std::ostringstream ss;
      ss << "Out-of-sync with VESC, discarding " << std::distance(iter_begin, iter) << " bytes.";
      if (error_handler_) {
        error_handler_(ss.str());
      }
    }
    rx_buffer_.erase(rx_buffer_.begin(), iter);
  }
}

VescInterface::VescInterface(const std::string& port,
                             const PacketHandlerFunction& packet_handler,
                             const ErrorHandlerFunction& error_handler) :
  impl_(new Impl())
{
  setPacketHandler(packet_handler);
  setErrorHandler(error_handler);
  if (!port.empty()) {
    connect(port);
  }
}

VescInterface::~VescInterface()
{
  disconnect();
}

void VescInterface::setPacketHandler(const PacketHandlerFunction& handler)
{
  impl_->packet_handler_ = handler;
}

void VescInterface::setErrorHandler(const ErrorHandlerFunction& handler)
{
  impl_->error_handler_ = handler;
}

void VescInterface::connect(const std::string& port)
{
  if (isConnected())
  {
    throw SerialException("Already connected to serial port.");
  }

  try
  {
    const auto config = drivers::serial_driver::SerialPortConfig(
      115200, drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE, drivers::serial_driver::StopBits::ONE);
    impl_->serial_driver_.init_port(port, config);
    impl_->serial_port_ = impl_->serial_driver_.port();
    impl_->serial_port_->open();
    impl_->rx_thread_run_ = true;
    impl_->rx_thread_ = std::thread(&VescInterface::Impl::rxThread, impl_.get());
  }
  catch (const std::exception& e)
  {
    std::stringstream ss;
    ss << "Failed to open the serial port to the VESC. " << e.what();
    throw SerialException(ss.str().c_str());
  }
}

void VescInterface::disconnect()
{
  if (isConnected())
  {
    impl_->rx_thread_run_ = false;
    impl_->serial_port_->close();
    if (impl_->rx_thread_.joinable()) {
      impl_->rx_thread_.join();
    }
    impl_->serial_port_.reset();
    impl_->rx_buffer_.clear();
  }
}

bool VescInterface::isConnected() const
{
  return impl_->serial_port_ && impl_->serial_port_->is_open();
}

void VescInterface::send(const VescPacket& packet)
{
  const size_t written = impl_->serial_port_->send(packet.frame());
  if (written != packet.frame().size())
  {
    std::stringstream ss;
    ss << "Wrote " << written << " bytes, expected " << packet.frame().size() << ".";
    throw SerialException(ss.str().c_str());
  }
}

void VescInterface::requestFWVersion()
{
  send(VescPacketRequestFWVersion());
}

void VescInterface::requestState()
{
  send(VescPacketRequestValues());
}

void VescInterface::setDutyCycle(double duty_cycle)
{
  send(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setCurrent(double current)
{
  send(VescPacketSetCurrent(current));
}

void VescInterface::setBrake(double brake)
{
  send(VescPacketSetCurrentBrake(brake));
}

void VescInterface::setSpeed(double speed)
{
  send(VescPacketSetRPM(speed));
}

void VescInterface::setPosition(double position)
{
  send(VescPacketSetPos(position));
}

void VescInterface::setServo(double servo)
{
  send(VescPacketSetServoPos(servo));
}

}  // namespace vesc_driver
