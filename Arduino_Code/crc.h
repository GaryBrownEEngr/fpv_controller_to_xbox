#pragma once

// crc algorithm extracted from the ExpressLRS project. CRC is annoying because it can be very hard to get a match unless you use the same code. We never agree on the shift direction and such.
uint8_t crc8_table[256];

void gen_elrs_crc8(){
  uint8_t poly = 0xD5;
  uint8_t crc;

  for (uint16_t i = 0; i < 256; i++){
      crc = i;
      for (uint8_t j = 0; j < 8; j++){
          crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
      }
      crc8_table[i] = crc & 0xFF;
  }
}

uint8_t calc_elrs_crc8(const uint8_t *data, uint16_t len){
  uint8_t crc = 0x00;
  while (len--){
      crc = crc8_table[crc ^ *data++];
  }
  return crc;
}