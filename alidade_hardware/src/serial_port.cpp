#include "alidade_hardware/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace alidade_hardware
{

namespace
{
std::string describe(const std::string & what, const std::string & path)
{
  return what + " " + path + " : " + std::strerror(errno);
}
}  // namespace

SerialPort::~SerialPort() { close(); }

void SerialPort::open(const std::string & path)
{
  close();
  // O_NONBLOCK a l'ouverture seulement : sans lui, open() peut bloquer sur
  // un port qui attend un signal de modem.
  int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error(describe("ouverture de", path));
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    ::close(fd);
    throw std::runtime_error(describe("tcgetattr sur", path));
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    ::close(fd);
    throw std::runtime_error(describe("tcsetattr sur", path));
  }
  tcflush(fd, TCIOFLUSH);

  fd_ = fd;
  path_ = path;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

size_t SerialPort::read_some(char * buffer, size_t max_bytes, std::chrono::milliseconds timeout)
{
  if (fd_ < 0) {
    throw std::runtime_error("port serie ferme");
  }
  pollfd pfd{fd_, POLLIN, 0};
  int ready = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (ready < 0) {
    if (errno == EINTR) {
      return 0;
    }
    throw std::runtime_error(describe("poll sur", path_));
  }
  if (ready == 0) {
    return 0;
  }
  // POLLHUP / POLLERR : le peripherique USB a ete debranche.
  if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
    throw std::runtime_error("peripherique deconnecte : " + path_);
  }
  ssize_t n = ::read(fd_, buffer, max_bytes);
  if (n < 0) {
    if (errno == EAGAIN || errno == EINTR) {
      return 0;
    }
    throw std::runtime_error(describe("lecture de", path_));
  }
  if (n == 0) {
    throw std::runtime_error("fin de fichier sur " + path_ + " : peripherique deconnecte");
  }
  return static_cast<size_t>(n);
}

void SerialPort::write_all(std::string_view data)
{
  if (fd_ < 0) {
    throw std::runtime_error("port serie ferme");
  }
  size_t done = 0;
  int attempts = 0;
  while (done < data.size()) {
    ssize_t n = ::write(fd_, data.data() + done, data.size() - done);
    if (n < 0) {
      if ((errno == EAGAIN || errno == EINTR) && ++attempts < 50) {
        pollfd pfd{fd_, POLLOUT, 0};
        ::poll(&pfd, 1, 20);
        continue;
      }
      throw std::runtime_error(describe("ecriture sur", path_));
    }
    done += static_cast<size_t>(n);
  }
}

}  // namespace alidade_hardware
