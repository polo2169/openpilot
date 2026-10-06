from opendbc.car.structs import CarParams
from opendbc.car.psa.values import CAR

Ecu = CarParams.Ecu

# Peugeot ECU list https://github.com/ludwig-v/arduino-psa-diag/blob/master/ECU_LIST.md

# Observed on logical CAN0 in the P87 captures received on 2026-10-06.
# This identifies this specimen; it is not an actuation validation.
FINGERPRINTS = {
  CAR.PSA_PEUGEOT_5008_P87: [{
    0x072: 5,
    0x0A8: 5,
    0x0A9: 5,
    0x0B2: 5,
    0x109: 8,
    0x10D: 8,
    0x112: 8,
    0x115: 8,
    0x117: 8,
    0x12A: 8,
    0x12B: 8,
    0x1E8: 8,
    0x208: 8,
    0x228: 8,
    0x2E7: 1,
    0x2E8: 8,
    0x2EB: 4,
    0x2ED: 7,
    0x2F5: 7,
    0x305: 7,
    0x30D: 8,
    0x329: 5,
    0x348: 8,
    0x349: 8,
    0x34D: 8,
    0x389: 5,
    0x38D: 8,
    0x3AD: 8,
    0x3B8: 8,
    0x3C8: 2,
    0x3C9: 8,
    0x3CD: 8,
    0x3E9: 2,
    0x3F2: 8,
    0x40D: 8,
    0x412: 8,
    0x42D: 4,
    0x432: 8,
    0x438: 8,
    0x44D: 8,
    0x452: 6,
    0x468: 8,
    0x488: 8,
    0x489: 8,
    0x48E: 4,
    0x492: 6,
    0x495: 4,
    0x4B2: 8,
    0x4CE: 8,
    0x4D2: 3,
    0x4E9: 3,
    0x4F2: 8,
    0x4FE: 3,
    0x50D: 8,
    0x50E: 8,
    0x517: 7,
    0x52E: 8,
    0x532: 5,
    0x54E: 8,
    0x552: 8,
    0x56E: 5,
    0x572: 8,
    0x578: 4,
    0x588: 8,
    0x58E: 8,
    0x592: 8,
    0x598: 4,
    0x5A8: 8,
    0x5AE: 5,
    0x5B2: 8,
    0x5B8: 8,
    0x5CE: 3,
    0x5D2: 6,
    0x5D8: 6,
    0x5EB: 8,
    0x5ED: 3,
    0x5EE: 4,
    0x5F8: 5,
    0x608: 8,
    0x612: 8,
    0x72B: 6,
    0x768: 8,
    0x775: 8,
    0x788: 8,
    0x789: 8,
    0x78D: 8,
    0x792: 8,
    0x795: 6,
    0x797: 6,
    0x7A8: 4,
  }],
}

FW_VERSIONS = {
  CAR.PSA_PEUGEOT_208: {
    (Ecu.abs, 0x6ad, None): [
      b'085095700857210527',
      b'085095706198220818',
      b'085095700473220908',
      b'085135705863191103',
      b'285160381930060025B',
      b'085095705963200902',
      b'085095709268220917',
      b'085135709971211107',
      b'085135702270200218',
    ],
  },
  CAR.PSA_PEUGEOT_508: {
    (Ecu.abs, 0x6ad, None): [
      b'085065315928191130',
      b'085095308910190312',
      b'085095701207240115',
      b'085135702688200218',
      b'085044414569231210',
      b'369042321125160806',
    ],
  },

  CAR.PSA_PEUGEOT_3008: {
    # BOITEVIT - Automatic transmission  (EAT6/8)
    (Ecu.transmission, 0x6A9, None): [
        ## Peugeot 3008 II (Phase I, 2016) 1.6 PureTech (180 Hp) Automatic S&S /2018, 2019, 2020
        b'1614191B101502000000',
        b'\xff\xff\x00\x000`\x08\x01\x13\x01%\x06\x08\xff\xff\xff\x00\x02\x00\x00\x01\x934t',
        #
    ],

    # INJ - Engine (VCU)
    (Ecu.engine, 0x6A8, None): [
        ## Peugeot 3008 II (Phase I, 2016) 1.6 PureTech (180 Hp) Automatic S&S /2018, 2019, 2020
        b'000D170047100',
        b'\xff\xff\x00\x00\x03&\t\x02\x13\x01C1\x04\xff\xff\xff\x00\x02\x00\x00\x01\x93YW',
        #
    ],

    # ABRASR - ABS/ESP
    (Ecu.abs, 0x6AD, None): [
        ## Peugeot 3008 II (Phase I, 2016) 1.6 PureTech (180 Hp) Automatic S&S /2018, 2019, 2020
        b'085065201906190129',
        b'\x00\x00\x00\x00\x03\x92\x01\x06\x11\x01\x06\x18\x02\xff\xff\xff\x00\x02\x00\x00\x01\x92\x83\x12',
        #
    ],
  },
  CAR.PSA_CITROEN_C4_SPACETOURER: {
    # INJ - Engine
    (Ecu.engine, 0x6A8, None): [
        b'3306SA00280507 ',
        b'\xff\xff\x00\x00\x130\x11\x01 \x01(\x13\x04\x1b\x0c\x16\xfdH\xc0\x8e\x02\x969\x00',
    ],

    # BOITEVIT - Automatic transmission
    (Ecu.transmission, 0x6A9, None): [
        b'161719CA103155000000',
        b'\xff\xff\x00\x000`\x05\x0c\x13\x01;\x07\x00\xff\xff\xff\x00\x02\x00\x00\x01\x93V\x97',
    ],

    # ABRASR - ABS/ESP
    (Ecu.abs, 0x6AD, None): [
        b'085095200769200218',
        b"\x00\x00\x00\x00\x03\x92\x03\x07\x13\x01\xa4'\x00\xff\xff\xff\x00\x02\x00\x00\x01\x93\x94\x16",
    ],
},
}
