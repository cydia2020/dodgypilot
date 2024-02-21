#include "common_dbc.h"

namespace {

const Signal sigs_368[] = {
    {
      .name = "FrictionBrakePressure",
      .start_bit = 23,
      .msb  = 23,
      .lsb = 24,
      .size = 16,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_560[] = {
    {
      .name = "Regen",
      .start_bit = 1,
      .msb  = 1,
      .lsb = 8,
      .size = 10,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_789[] = {
    {
      .name = "FrictionBrakeCmd",
      .start_bit = 3,
      .msb  = 3,
      .lsb = 8,
      .size = 12,
      .is_signed = true,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "FrictionBrakeMode",
      .start_bit = 7,
      .msb  = 7,
      .lsb = 4,
      .size = 4,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "FrictionBrakeChecksum",
      .start_bit = 23,
      .msb  = 23,
      .lsb = 24,
      .size = 16,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "RollingCounter",
      .start_bit = 33,
      .msb  = 33,
      .lsb = 32,
      .size = 2,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_823[] = {
    {
      .name = "SteeringWheelCmd",
      .start_bit = 23,
      .msb  = 23,
      .lsb = 24,
      .size = 16,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "RollingCounter",
      .start_bit = 35,
      .msb  = 35,
      .lsb = 34,
      .size = 2,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SteeringWheelChecksum",
      .start_bit = 47,
      .msb  = 47,
      .lsb = 48,
      .size = 16,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};

const Msg msgs[] = {
  {
    .name = "EBCMFrictionBrakeStatus",
    .address = 0x170,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_368),
    .sigs = sigs_368,
  },
  {
    .name = "EBCMRegen",
    .address = 0x230,
    .size = 6,
    .num_sigs = ARRAYSIZE(sigs_560),
    .sigs = sigs_560,
  },
  {
    .name = "EBCMFrictionBrakeCmd",
    .address = 0x315,
    .size = 5,
    .num_sigs = ARRAYSIZE(sigs_789),
    .sigs = sigs_789,
  },
  {
    .name = "PACMParkAssitCmd",
    .address = 0x337,
    .size = 7,
    .num_sigs = ARRAYSIZE(sigs_823),
    .sigs = sigs_823,
  },
};

const Val vals[] = {
};

}

const DBC gm_global_a_chassis = {
  .name = "gm_global_a_chassis",
  .num_msgs = ARRAYSIZE(msgs),
  .msgs = msgs,
  .vals = vals,
  .num_vals = ARRAYSIZE(vals),
};

dbc_init(gm_global_a_chassis)