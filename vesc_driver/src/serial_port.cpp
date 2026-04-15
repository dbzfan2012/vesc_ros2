// POSIX serial port implementation — drop-in for the serial::Serial API

#include "vesc_driver/serial_port.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace serial
{

Serial::Serial(const std::string& port,
               uint32_t baudrate,
               Timeout timeout,
               bytesize_t /*bytesize*/,
               parity_t /*parity*/,
               stopbits_t /*stopbits*/,
               flowcontrol_t /*flowcontrol*/)
  : port_(port), baudrate_(baudrate), timeout_(timeout), fd_(-1)
{
}

Serial::~Serial()
{
  if (isOpen()) {
    close();
  }
}

void Serial::setPort(const std::string& port) { port_ = port; }
void Serial::setBaudrate(uint32_t baudrate) { baudrate_ = baudrate; }
void Serial::setTimeout(Timeout timeout) { timeout_ = timeout; }

static speed_t baudrate_to_posix(uint32_t baudrate)
{
  switch (baudrate) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default:
      throw std::runtime_error("Unsupported baud rate: " + std::to_string(baudrate));
  }
}

void Serial::open()
{
  if (isOpen()) {
    throw std::runtime_error("Port already open");
  }

  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open serial port " + port_ + ": " + std::strerror(errno));
  }

  // Clear the O_NONBLOCK flag after opening so reads can block with select()
  int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
  }

  // Use cfmakeraw to ensure fully raw mode — no input processing, no output
  // processing, no signal generation. This is critical for binary serial
  // protocols like VESC where any byte transformation corrupts frames.
  cfmakeraw(&tty);

  speed_t baud = baudrate_to_posix(baudrate_);
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  // 8N1, no flow control
  tty.c_cflag &= ~CRTSCTS;       // no hardware flow control
  tty.c_cflag |= CLOCAL | CREAD; // enable receiver, ignore modem control

  // VMIN=0, VTIME=0 — we handle timeouts ourselves via select()
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  tcflush(fd_, TCIOFLUSH);
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
  }
}

void Serial::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Serial::isOpen() const
{
  return fd_ >= 0;
}

size_t Serial::available()
{
  if (!isOpen()) return 0;
  int count = 0;
  if (ioctl(fd_, FIONREAD, &count) < 0) {
    return 0;
  }
  return static_cast<size_t>(count);
}

size_t Serial::read(std::vector<uint8_t>& buffer, size_t size)
{
  if (!isOpen() || size == 0) return 0;

  // Use select() for timeout
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(fd_, &readfds);

  struct timeval tv;
  tv.tv_sec  = timeout_.read_timeout_ms / 1000;
  tv.tv_usec = (timeout_.read_timeout_ms % 1000) * 1000;

  int sel = ::select(fd_ + 1, &readfds, NULL, NULL, &tv);
  if (sel <= 0) {
    return 0; // timeout or error
  }

  std::vector<uint8_t> tmp(size);
  ssize_t n = ::read(fd_, tmp.data(), size);
  if (n <= 0) {
    return 0;
  }

  buffer.insert(buffer.end(), tmp.begin(), tmp.begin() + n);
  return static_cast<size_t>(n);
}

size_t Serial::write(const std::vector<uint8_t>& data)
{
  if (!isOpen() || data.empty()) return 0;

  ssize_t n = ::write(fd_, data.data(), data.size());
  if (n < 0) {
    throw std::runtime_error("Serial write failed: " + std::string(std::strerror(errno)));
  }
  return static_cast<size_t>(n);
}

} // namespace serial
