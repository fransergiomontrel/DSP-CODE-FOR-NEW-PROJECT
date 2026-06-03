#include "LCD_I2C.h"
#include "string.h"

struct I2CMSG *libPtr;

void init_LCD(struct I2CMSG *i2cmsg) {
    libPtr = i2cmsg;


    DELAY_US(30000);
    LCD_command(0x02);
    DELAY_US(40);
    LCD_command(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    DELAY_US(40);
    LCD_command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    DELAY_US(40);
    LCD_command(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
    DELAY_US(1500);
    LCD_command(LCD_CLEARDISPLAY | LCD_RETURNHOME);
    DELAY_US(1500);

}

Uint16 I2CB_WriteData(struct I2CMSG *msg)
{
    Uint16 i;

    //
    // Wait until the STP bit is cleared from any previous master communication.
    // Clearing of this bit by the module is delayed until after the SCD bit is
    // set. If this bit is not checked prior to initiating a new message, the
    // I2C could get confused.
    //
    if(I2cbRegs.I2CMDR.bit.STP == 1)
    {
        return I2C_STP_NOT_READY_ERROR;
    }

    //
    // Setup slave address
    //
    I2cbRegs.I2CSAR.all = msg->SlaveAddress;

    //
    // Check if bus busy
    //
    if(I2cbRegs.I2CSTR.bit.BB == 1)
    {
        return I2C_BUS_BUSY_ERROR;
    }

    //
    // Setup number of bytes to send
    // MsgBuffer + Address
    //
    I2cbRegs.I2CCNT = msg->NumOfBytes+2;

    //
    // Setup data to send
    //
    I2cbRegs.I2CDXR.all = msg->MemoryHighAddr;
    I2cbRegs.I2CDXR.all = msg->MemoryLowAddr;

    for (i=0; i < msg->NumOfBytes; i++)
    {
        I2cbRegs.I2CDXR.all = *(msg->MsgBuffer+i);
    }

    //
    // Send start as master transmitter
    //
    I2cbRegs.I2CMDR.all = 0x6E20;

    return I2C_SUCCESS;
}

void I2C_out(Uint16 data){

    *libPtr->MsgBuffer = data;  // 36  ASCII
    I2CB_WriteData(libPtr);
}

void LCD_command(unsigned int c){

    unsigned int commandEo;
    unsigned int commandEi;
    commandEo = 0x00;       //LED = 1, E = 0, RW = 0, RS = 0
    commandEi = 0x04;       //LED = 1, E = 1, RW = 0, RS = 0

    unsigned int result = 0;

    result = (c&0xf0) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);
    result = (c&0xf0) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);

    result = (c&0xf0) | commandEo;
    I2C_out(result | LCD_BACKLIGHT);
    result = (c&0xf0) | commandEo;
    I2C_out(result | LCD_BACKLIGHT);

    result = (c&0xf0) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);
    result = (c&0xf0) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);


    DELAY_US(4000);

    result = (c<<4) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);
    result = (c<<4) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);

    result = (c<<4) | commandEo;
    I2C_out(result | LCD_BACKLIGHT);
    result = (c<<4) | commandEo;
    I2C_out(result | LCD_BACKLIGHT);

    result = (c<<4) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);
    result = (c<<4) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);

    DELAY_US(4000);
}

void LCD_write(unsigned int c){

    unsigned int commandEo;
    unsigned int commandEi;

    commandEo = 0x01;       //LED = 1, E = 0, RW = 0, RS = 1
    commandEi = 0x05;       //LED = 1, E = 1, RW = 0, RS = 1

    unsigned int result = 0;

    result = (c&0xf0) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);
    result = (c&0xf0) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);

    result = (c&0xf0) | commandEo;
    I2C_out(result | LCD_BACKLIGHT);
    result = (c&0xf0) | commandEo;
    I2C_out(result | LCD_BACKLIGHT);

    result = (c&0xf0) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);
    result = (c&0xf0) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);

    DELAY_US(4000);

    result = (c<<4) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);
    result = (c<<4) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);

    result = (c<<4) | commandEo;
    I2C_out(result | LCD_BACKLIGHT);
    result = (c<<4) | commandEo;
    I2C_out(result | LCD_BACKLIGHT);

    result = (c<<4) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);
    result = (c<<4) | commandEi;
    I2C_out(result | LCD_BACKLIGHT);

    DELAY_US(4000);
}

void intro(void){

    LCD_write(32);
    LCD_write(32);
    LCD_write(68);
    LCD_write(65);
    LCD_write(84);
    LCD_write(65);
    LCD_write(32);
    LCD_write(65);
    LCD_write(67);
    LCD_write(81);
    LCD_write(85);
    LCD_write(73);
    LCD_write(83);
    LCD_write(73);
    LCD_write(84);
    LCD_write(73);
    LCD_write(79);
    LCD_write(78);

    LCD_command(0xC0);    // LINE 2
    LCD_write(32);
    LCD_write(32);
    LCD_write(32);
    LCD_write(83);
    LCD_write(89);
    LCD_write(83);
    LCD_write(84);
    LCD_write(69);
    LCD_write(77);
    LCD_write(32);
    LCD_write(45);
    LCD_write(32);
    LCD_write(67);
    LCD_write(73);
    LCD_write(83);
    LCD_write(69);
    LCD_write(73);
    LCD_write(32);
    LCD_write(32);
    LCD_write(32);

    LCD_command(0x94);    // LINE 3
    LCD_write(32);
    LCD_write(32);
    LCD_write(32);
    LCD_write(32);
    LCD_write(32);
    LCD_write(32);
    LCD_write(32);
    LCD_write(32);
    LCD_write(50);
    LCD_write(48);
    LCD_write(49);
    LCD_write(56);

    LCD_command(0xD4);    // LINE 4
    LCD_write(32);
    LCD_write(32);
    LCD_write(73);
    LCD_write(110);
    LCD_write(105);
    LCD_write(99);
    LCD_write(105);
    LCD_write(97);
    LCD_write(108);
    LCD_write(105);
    LCD_write(122);
    LCD_write(97);
    LCD_write(110);
    LCD_write(100);
    LCD_write(111);
    LCD_write(46);
    LCD_write(46);
    LCD_write(46);

    LCD_command(0x80);    // LINE 1
}
void printText(char* text, Uint16 line, Uint16 col){
    int16 row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    LCD_command(LCD_SETDDRAMADDR | (row_offsets[line - 1] + (col - 1)));
    int16 i;
    for(i = 0; i < strlen(text); i++)
        LCD_write((Uint16)text[i]);
}
void clear(void){
    LCD_command(LCD_CLEARDISPLAY);
    DELAY_US(2000);
}
