#ifndef LCD_I2C_h
#define LCD_I2C_h

#include "F28x_Project.h"

#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0x04//00000100  // Enable bit
#define Rw 0x02//00000010  // Read/Write bit
#define Rs 0x01//00000001  // Register select bit

//I2C Confs
#define I2C_SLAVE_ADDR        0x3F
//#define I2C_SLAVE_ADDR        0x27
#define I2C_NUMBYTES          2
#define I2C_EEPROM_HIGH_ADDR  0x00
#define I2C_EEPROM_LOW_ADDR   0x30

//Variables
extern struct I2CMSG *libPtr;

//Function prototypes
Uint16  I2CB_WriteData(struct I2CMSG *);
void    init_LCD(struct I2CMSG *);
void    I2C_out(Uint16 data);

void    LCD_command(Uint16 c);
void    LCD_write(Uint16 val);
void    intro(void);
void    printText(char*, Uint16, Uint16);
void    clear(void);

#endif

