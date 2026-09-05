// Tests du protocole, ecrits depuis la documentation du firmware et non
// depuis son code : le vecteur de checksum est celui du README.
#include <gtest/gtest.h>

#include "alidade_hardware/pico_protocol.hpp"

using namespace alidade_hardware;

TEST(Protocol, DocumentedFrameParses)
{
  ParseError why;
  auto frame = parse_state_frame("S 123456 1496 -1496 4362 -4362 12 -34 56 -78 910 -1011 68\n", &why);
  ASSERT_TRUE(frame.has_value()) << to_string(why);
  EXPECT_EQ(frame->t_ms, 123456u);
  EXPECT_EQ(frame->enc[0], 1496);
  EXPECT_EQ(frame->enc[1], -1496);
  EXPECT_EQ(frame->vel[0], 4362);
  EXPECT_EQ(frame->vel[1], -4362);
  EXPECT_EQ(frame->gyro[0], 12);
  EXPECT_EQ(frame->gyro[1], -34);
  EXPECT_EQ(frame->gyro[2], 56);
  EXPECT_EQ(frame->accel[0], -78);
  EXPECT_EQ(frame->accel[1], 910);
  EXPECT_EQ(frame->accel[2], -1011);
}

TEST(Protocol, ChecksumCoversPayloadOnly)
{
  EXPECT_EQ(checksum("S 123456 1496 -1496 4362 -4362 12 -34 56 -78 910 -1011"), 0x68);
}

TEST(Protocol, WithoutNewline)
{
  EXPECT_TRUE(parse_state_frame("S 123456 1496 -1496 4362 -4362 12 -34 56 -78 910 -1011 68").has_value());
  EXPECT_TRUE(parse_state_frame("S 123456 1496 -1496 4362 -4362 12 -34 56 -78 910 -1011 68\r\n").has_value());
}

TEST(Protocol, ImuAbsentIsZeros)
{
  std::string payload = "S 10 0 0 0 0 0 0 0 0 0 0";
  char chk[4];
  snprintf(chk, sizeof(chk), "%02X", checksum(payload));
  auto frame = parse_state_frame(payload + " " + chk);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->accel[2], 0);
}

TEST(Protocol, RejectsCorruption)
{
  ParseError why;
  EXPECT_FALSE(parse_state_frame("S 123456 1496 -1496 4362 -4362 12 -34 56 -78 910 -1011 69", &why));
  EXPECT_EQ(why, ParseError::Checksum);
  EXPECT_FALSE(parse_state_frame("S 123456 1497 -1496 4362 -4362 12 -34 56 -78 910 -1011 68", &why));
  EXPECT_EQ(why, ParseError::Checksum);
  EXPECT_FALSE(parse_state_frame("S 123456 1496 -1496 4362 -4362 12 -34 56 -78 910 -1011 6", &why));
  EXPECT_EQ(why, ParseError::Checksum);
  EXPECT_FALSE(parse_state_frame("S 123456 1496 -1496 4362 -4362 12 -34 56 -78 910 -1011 6g", &why));
  EXPECT_EQ(why, ParseError::Checksum);
}

TEST(Protocol, RejectsWrongShape)
{
  ParseError why;
  EXPECT_FALSE(parse_state_frame("", &why));
  EXPECT_EQ(why, ParseError::NotStateFrame);
  EXPECT_FALSE(parse_state_frame("ok\n", &why));
  EXPECT_EQ(why, ParseError::NotStateFrame);
  EXPECT_FALSE(parse_state_frame("S\n", &why));
  EXPECT_EQ(why, ParseError::NotStateFrame);
  // Un champ de moins : le checksum de la charge tronquee est recalcule pour
  // isoler l'erreur de forme de l'erreur de checksum.
  std::string payload = "S 1 2 3 4 5 6 7 8 9 10";
  char chk[4];
  snprintf(chk, sizeof(chk), "%02X", checksum(payload));
  EXPECT_FALSE(parse_state_frame(payload + " " + chk, &why));
  EXPECT_EQ(why, ParseError::FieldCount);
  // Deux espaces consecutifs : champ vide, donc compte faux.
  payload = "S 1  2 3 4 5 6 7 8 9 10 11";
  snprintf(chk, sizeof(chk), "%02X", checksum(payload));
  EXPECT_FALSE(parse_state_frame(payload + " " + chk, &why));
  EXPECT_EQ(why, ParseError::FieldCount);
}

TEST(Protocol, RejectsNonNumeric)
{
  ParseError why;
  std::string payload = "S 1 2 x 4 5 6 7 8 9 10 11";
  char chk[4];
  snprintf(chk, sizeof(chk), "%02X", checksum(payload));
  EXPECT_FALSE(parse_state_frame(payload + " " + chk, &why));
  EXPECT_EQ(why, ParseError::NotNumeric);
  payload = "S 1 2 3.5 4 5 6 7 8 9 10 11";
  snprintf(chk, sizeof(chk), "%02X", checksum(payload));
  EXPECT_FALSE(parse_state_frame(payload + " " + chk, &why));
  EXPECT_EQ(why, ParseError::NotNumeric);
  // int16 depasse pour un champ IMU
  payload = "S 1 2 3 4 5 6 7 8 9 10 40000";
  snprintf(chk, sizeof(chk), "%02X", checksum(payload));
  EXPECT_FALSE(parse_state_frame(payload + " " + chk, &why));
  EXPECT_EQ(why, ParseError::NotNumeric);
}

TEST(Protocol, Encoding)
{
  EXPECT_EQ(encode_velocity(0, 0), "v 0 0\n");
  EXPECT_EQ(encode_velocity(-1500, 2000), "v -1500 2000\n");
  EXPECT_EQ(encode_stop(), "s\n");
  EXPECT_EQ(encode_reset_counters(), "r\n");
}
