// Port serie POSIX minimal : ouverture en mode brut, lecture avec delai,
// ecriture complete. Le Pico est un CDC USB, la vitesse est sans effet mais
// le mode brut est indispensable : sans lui la couche tty traduit '\n' en
// "\r\n" et corrompt le tramage.
#ifndef ALIDADE_HARDWARE__SERIAL_PORT_HPP_
#define ALIDADE_HARDWARE__SERIAL_PORT_HPP_

#include <chrono>
#include <string>
#include <string_view>

namespace alidade_hardware
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Leve std::runtime_error si le port ne s'ouvre pas.
  void open(const std::string & path);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // Lit ce qui est disponible, au plus max_bytes, en attendant jusqu'a
  // timeout. Retourne le nombre d'octets, 0 si rien n'est arrive, et leve
  // std::runtime_error si le peripherique a disparu.
  size_t read_some(char * buffer, size_t max_bytes, std::chrono::milliseconds timeout);

  // Ecrit tout ou leve std::runtime_error.
  void write_all(std::string_view data);

  const std::string & path() const { return path_; }

private:
  int fd_ = -1;
  std::string path_;
};

}  // namespace alidade_hardware

#endif  // ALIDADE_HARDWARE__SERIAL_PORT_HPP_
