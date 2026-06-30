/*! \file datatypes.h
 *  \brief Arquivo com a declara��o de tipos.
 *
 */

#ifndef _DATATYPES_H_
#define _DATATYPES_H_

#include <stdint.h>
#include "defines.h"

#ifndef uint8_t
typedef unsigned char uint8_t;
#endif

#ifndef int8_t
typedef signed char int8_t;
#endif

#ifndef boolean
typedef unsigned char boolean;
#endif 

#define TMS320_BOARD_COUNT            5U
#define TMS320_CHANNEL_COUNT          6U
#define TMS320_CHANNEL_FLOAT_COUNT    2U
#define TMS320_ANALOG_FLOAT_COUNT     5U
#define TMS320_BOARD_STATUS_SIZE      1U
#define TMS320_BOARD_ALARM_SIZE       1U
#define TMS320_CRC_SIZE               2U
#define TMS320_SAMPLE_REQUEST_SIZE    3U
#define TMS320_SAMPLE_FASOR_COMMAND   0x0CU
//#define TMS320_SAMPLE_DISSIPATION     0x0DU


#define TMS320_BOARD_PAYLOAD_SIZE     \
    (((TMS320_CHANNEL_COUNT * TMS320_CHANNEL_FLOAT_COUNT) + \
      TMS320_ANALOG_FLOAT_COUNT) * \
     (uint32_t)sizeof(float) + TMS320_BOARD_STATUS_SIZE + \
     TMS320_BOARD_ALARM_SIZE)
#define TMS320_PAYLOAD_SIZE           (TMS320_BOARD_COUNT * TMS320_BOARD_PAYLOAD_SIZE)
#define TMS320_UART_FRAME_SIZE        (TMS320_PAYLOAD_SIZE + TMS320_CRC_SIZE)


typedef union
{
    float retangular_float;
    uint32_t retangular_int;
}uint32_to_float_t;

typedef union
{
    float ads1118_float;
    uint32_t ads1118_int;
}uint16_to_float_t;

typedef struct 
{
    uint8_t frequency;
    uint8_t number_ieds;
}tms320_sync_frame_t;

//#pragma PACKED
typedef struct 
{
    float channel1[TMS320_CHANNEL_FLOAT_COUNT];
    float channel2[TMS320_CHANNEL_FLOAT_COUNT];
    float channel3[TMS320_CHANNEL_FLOAT_COUNT];
    float channel4[TMS320_CHANNEL_FLOAT_COUNT];
    float channel5[TMS320_CHANNEL_FLOAT_COUNT];
    float channel6[TMS320_CHANNEL_FLOAT_COUNT];
    float analog[TMS320_ANALOG_FLOAT_COUNT];
    int8_t alarm;
    int8_t status;
} tms320_board_data_t;
//#pragma UNPACKED

//#pragma PACKED
typedef struct 
{
    tms320_board_data_t boards[TMS320_BOARD_COUNT];
} tms320_data_t;
//#pragma UNPACKED


//#pragma PACKED
typedef struct
{
    tms320_data_t payload;
    uint16_t crc;
} tms320_uart_frame_t;
//#pragma UNPACKED

//! Pacote das estatisticas do sincronismo entre interfaces.
typedef struct {
    uint32_t T1,    ///< Valor m�dio utilizado para compensar a propaga��o na fibra 1.
    T1_Max,         ///< Valor m�ximo calculado na propaga��o da fibra 1.
    T1_Min;         ///< Valor minimo calculado na propaga��o da fibra 1.
    uint32_t T2,    ///< Valor m�dio utilizado para compensar a propaga��o na fibra 2.
    T2_Max,         ///< Valor m�ximo calculado na propaga��o da fibra 2.
    T2_Min;         ///< Valor minimo calculado na propaga��o da fibra 2.
    uint32_t T3,    ///< Valor m�dio utilizado para compensar a propaga��o na fibra 3.
    T3_Max,         ///< Valor m�ximo calculado na propaga��o da fibra 3.
    T3_Min;         ///< Valor minimo calculado na propaga��o da fibra 3.
    uint32_t T4,    ///< Valor m�dio utilizado para compensar a propaga��o na fibra 4.
    T4_Max,         ///< Valor m�ximo calculado na propaga��o da fibra 4.
    T4_Min;         ///< Valor minimo calculado na propaga��o da fibra 4.
    float DP1;      ///< Desvio padr�o calculado da propaga��o da fibra 1, necessita revis�o.
    float DP2;      ///< Desvio padr�o calculado da propaga��o da fibra 2, necessita revis�o.
    float DP3;      ///< Desvio padr�o calculado da propaga��o da fibra 3, necessita revis�o.
    float DP4;      ///< Desvio padr�o calculado da propaga��o da fibra 4, necessita revis�o.
} t_sync_data;

typedef struct {
    float PTC_1;
    float S420_1;
    float PTC_2;
    float S420_2;
} t_temp_val;


//! Pacote de transmiss�o dos fasores de tens�o.
/*! Nesse pacote s�o enviados os fasores calculados, al�m de informa��es adicionais do horario de recebimento do pedido pelo PC,
 *  estatisticas do sincronismo com as interfaces de corrente.
 *  \sa t_sync_data, tms320_sync_frame_t
 */
typedef struct {

    //tms320_sync_frame_t time;      ///< Vari�vel para armazenar o pacote de dados do sincronismo.
    t_sync_data sync_data;  ///< Vari�vel para armazenar o pacote de estatisticas do sincronismo.

} t_voltage_phasor_frame;

//! Pacote de transmiss�o dos fasores de corrente.
/*!
 *  \sa tms320_sync_frame_t
 */


#endif
