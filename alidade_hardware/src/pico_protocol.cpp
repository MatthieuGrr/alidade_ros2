#include "alidade_hardware/pico_protocol.hpp"

#include <charconv>
#include <limits>
#include <cstdio>
#include <vector>

namespace alidade_hardware
{

uint8_t checksum(std::string_view payload)
{
  uint8_t acc = 0;
  for (unsigned char c : payload) {
    acc ^= c;
  }
  return acc;
}

namespace
{

template <typename T>
bool parse_int(std::string_view field, T & out)
{
  // from_chars refuse les espaces, les signes '+' et les flottants : c'est
  // exactement la strictete du parseur du firmware, reproduite en face.
  long long value = 0;
  auto result = std::from_chars(field.data(), field.data() + field.size(), value);
  if (result.ec != std::errc() || result.ptr != field.data() + field.size()) {
    return false;
  }
  if (value < std::numeric_limits<T>::min() || value > std::numeric_limits<T>::max()) {
    return false;
  }
  out = static_cast<T>(value);
  return true;
}

std::vector<std::string_view> split(std::string_view text)
{
  std::vector<std::string_view> fields;
  size_t start = 0;
  while (start <= text.size()) {
    size_t end = text.find(' ', start);
    if (end == std::string_view::npos) {
      end = text.size();
    }
    fields.push_back(text.substr(start, end - start));
    start = end + 1;
  }
  return fields;
}

}  // namespace

std::optional<StateFrame> parse_state_frame(std::string_view line, ParseError * error)
{
  auto fail = [&](ParseError why) -> std::optional<StateFrame> {
    if (error) {
      *error = why;
    }
    return std::nullopt;
  };

  while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
    line.remove_suffix(1);
  }
  if (line.size() < 2 || line[0] != 'S' || line[1] != ' ') {
    return fail(ParseError::NotStateFrame);
  }

  const size_t last_space = line.rfind(' ');
  if (last_space == std::string_view::npos) {
    return fail(ParseError::FieldCount);
  }
  const std::string_view payload = line.substr(0, last_space);
  const std::string_view chk_text = line.substr(last_space + 1);

  const auto fields = split(payload);
  if (fields.size() != 12) {
    return fail(ParseError::FieldCount);
  }

  // Deux chiffres hexadecimaux majuscules, verifies avant les champs : une
  // trame corrompue est rejetee sans qu'on ait a interpreter son contenu.
  if (chk_text.size() != 2) {
    return fail(ParseError::Checksum);
  }
  unsigned expected = 0;
  auto hex = std::from_chars(chk_text.data(), chk_text.data() + 2, expected, 16);
  if (hex.ec != std::errc() || hex.ptr != chk_text.data() + 2) {
    return fail(ParseError::Checksum);
  }
  if (checksum(payload) != expected) {
    return fail(ParseError::Checksum);
  }

  StateFrame frame;
  bool ok = parse_int(fields[1], frame.t_ms);
  ok = ok && parse_int(fields[2], frame.enc[0]) && parse_int(fields[3], frame.enc[1]);
  ok = ok && parse_int(fields[4], frame.vel[0]) && parse_int(fields[5], frame.vel[1]);
  for (int i = 0; i < 3 && ok; ++i) {
    ok = parse_int(fields[6 + i], frame.gyro[i]);
  }
  for (int i = 0; i < 3 && ok; ++i) {
    ok = parse_int(fields[9 + i], frame.accel[i]);
  }
  if (!ok) {
    return fail(ParseError::NotNumeric);
  }
  return frame;
}

const char * to_string(ParseError error)
{
  switch (error) {
    case ParseError::NotStateFrame: return "pas une trame S";
    case ParseError::FieldCount: return "nombre de champs incorrect";
    case ParseError::NotNumeric: return "champ non numerique";
    case ParseError::Checksum: return "checksum invalide";
  }
  return "?";
}

std::string encode_velocity(int32_t left, int32_t right)
{
  char buf[48];
  std::snprintf(buf, sizeof(buf), "v %d %d\n", static_cast<int>(left), static_cast<int>(right));
  return buf;
}

std::string encode_stop() { return "s\n"; }

std::string encode_reset_counters() { return "r\n"; }

}  // namespace alidade_hardware
