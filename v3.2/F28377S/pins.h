/*! \file pins.h
 *  \brief Arquivo de declara��o dos pinos utilizados.
 *  \warning Talvez alguns pinos estejam desatualizados, a ser verificado em uma revis�o futura.
 */

#ifndef PINS_h
#define PINS_h

//On-board leds
#define LED1   44 // Led D5
#define LED2   65 // Led D6
#define LED3   64 // Led D7

//I2C Header
#define SDAB  34 // I2C
#define SCLB  35 // I2C

//SCI A
#define TXA   29 // TX.A FIBRA
#define RXA   28 // RX.A FIBRA

//SCI B
#define TXB   22 // TX.B USB
#define RXB   23 // RX.B USB

//SCI C
#define TXC   89 // TX.C FIBRA
#define RXC   90 // RX.C FIBRA

// SCI D
#define TXD   47 // TX.D FIBRA
#define RXD   46 // RX.D FIBRA

//I2C
#define SDA   66
#define SCL   69

//JP6
#define DEBUG1 32
#define DEBUG2 33
#define DEBUG3 34
#define DEBUG4 35
#define DEBUG5 36
#define DEBUG6 37
#define DEBUG7 38
#define DEBUG8 39

//MICREL - PORTA
#define KSZ_D0      GpioDataRegs.GPADAT.bit.GPIO0
#define KSZ_D1      GpioDataRegs.GPADAT.bit.GPIO1
#define KSZ_D2      GpioDataRegs.GPADAT.bit.GPIO2
#define KSZ_D3      GpioDataRegs.GPADAT.bit.GPIO3
#define KSZ_D4      GpioDataRegs.GPADAT.bit.GPIO4
#define KSZ_D5      GpioDataRegs.GPADAT.bit.GPIO5
#define KSZ_D6      GpioDataRegs.GPADAT.bit.GPIO6
#define KSZ_D7      GpioDataRegs.GPADAT.bit.GPIO7
#define KSZ_D8      GpioDataRegs.GPADAT.bit.GPIO8
#define KSZ_D9      GpioDataRegs.GPADAT.bit.GPIO9
#define KSZ_D10     GpioDataRegs.GPADAT.bit.GPIO10
#define KSZ_D11     GpioDataRegs.GPADAT.bit.GPIO11
#define KSZ_D12     GpioDataRegs.GPADAT.bit.GPIO12
#define KSZ_D13     GpioDataRegs.GPADAT.bit.GPIO13
#define KSZ_D14     GpioDataRegs.GPADAT.bit.GPIO14
#define KSZ_D15     GpioDataRegs.GPADAT.bit.GPIO15

#define KSZ_CMD     GpioDataRegs.GPADAT.bit.GPIO16
#define KSZ_INTRN   GpioDataRegs.GPADAT.bit.GPIO17
#define KSZ_RDN     GpioDataRegs.GPADAT.bit.GPIO18
#define KSZ_CSN     GpioDataRegs.GPADAT.bit.GPIO19
#define KSZ_PME     GpioDataRegs.GPADAT.bit.GPIO20
#define KSZ_WRN     GpioDataRegs.GPADAT.bit.GPIO21
#define KSZ_FXSD2   GpioDataRegs.GPADAT.bit.GPIO24

//MICREL - PORTC
#define KSZ_RSTN    GpioDataRegs.GPCDAT.bit.GPIO80
#define KSZ_EGPIO0  GpioDataRegs.GPCDAT.bit.GPIO87
#define KSZ_EGPIO1  GpioDataRegs.GPCDAT.bit.GPIO86
#define KSZ_EGPIO2  GpioDataRegs.GPCDAT.bit.GPIO85
#define KSZ_EGPIO6  GpioDataRegs.GPCDAT.bit.GPIO83
#define KSZ_P1LED0  GpioDataRegs.GPCDAT.bit.GPIO82
#define KSZ_P2LED0  GpioDataRegs.GPCDAT.bit.GPIO81
#define KSZ_FXSD1   GpioDataRegs.GPCDAT.bit.GPIO94

//SFP1 - OTICO
#define SFP1_MOD2    42
#define SFP1_MOD1    43
#define SFP1_MOD0    77
#define SFP1_LOS     76
#define SFP1_TX_DIS  78
#define SFP1_RX_FLT  79

//SFP2 - OTICO
#define SFP2_MOD2    66
#define SFP2_MOD1    69
#define SFP2_MOD0    73
#define SFP2_LOS     71
#define SFP2_TX_DIS  74
#define SFP2_RX_FLT  75

#endif
