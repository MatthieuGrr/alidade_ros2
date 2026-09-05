// Protocole ASCII de diffdrive_pico_firmware, cote Pi.
//
// Pur : aucune entree/sortie, pour etre teste sans materiel. Le firmware ne
// connait que des ticks et des ticks/s ; la conversion en unites physiques
// se fait dans PicoSystem, jamais ici.
#ifndef ALIDADE_HARDWARE__PICO_PROTOCOL_HPP_
#define ALIDADE_HARDWARE__PICO_PROTOCOL_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace alidade_hardware
{

// Trame S : etat spontane du Pico, emise a 50 Hz.
struct StateFrame
{
  uint32_t t_ms = 0;        // millisecondes depuis le demarrage du Pico
  int32_t enc[2] = {0, 0};  // compteurs cumules gauche, droite (jamais des deltas)
  int32_t vel[2] = {0, 0};  // vitesses mesurees en ticks/s
  int16_t gyro[3] = {0, 0, 0};   // BNO055 brut, 16 LSB par deg/s
  int16_t accel[3] = {0, 0, 0};  // BNO055 brut, 100 LSB par m/s^2
};

enum class ParseError
{
  NotStateFrame,   // la ligne ne commence pas par "S "
  FieldCount,      // pas exactement 12 champs + checksum
  NotNumeric,      // un champ n'est pas un entier
  Checksum,        // le XOR ne correspond pas
};

// OU exclusif de tous les octets de la charge utile (du 'S' inclus au dernier
// caractere precedant l'espace qui separe <az> de <chk>).
uint8_t checksum(std::string_view payload);

// Analyse une ligne complete, avec ou sans '\n' final.
std::optional<StateFrame> parse_state_frame(std::string_view line, ParseError * error = nullptr);

const char * to_string(ParseError error);

// Commandes Pi -> Pico, avec le '\n' terminal.
std::string encode_velocity(int32_t left_ticks_per_s, int32_t right_ticks_per_s);
std::string encode_stop();
std::string encode_reset_counters();

}  // namespace alidade_hardware

#endif  // ALIDADE_HARDWARE__PICO_PROTOCOL_HPP_
