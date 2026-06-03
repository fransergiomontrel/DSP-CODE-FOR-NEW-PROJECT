#include "CRC16.h"

uint16_t w_Crc;

void crc16_init(){
    w_Crc = 0xffff;
}

uint16_t crc16_data(uint16_t value){
    uint16_t i;
    w_Crc ^= value << 8;
    for (i=0; i < 8; i++)
        w_Crc = w_Crc & 0x8000 ? (w_Crc << 1) ^ 0x1021 : w_Crc << 1;
    return w_Crc & 0xffff;
}
