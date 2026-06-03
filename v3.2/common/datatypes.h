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

//! Pacote de dados do sincronismo.
typedef struct {
    uint32_t timestampi; ///< Timestamp enviado pelo PC para as interfaces.
    uint16_t phasor; ///< Flag de transmiss�o de fasores (1) ou dados brutos (0, n�o utilizada).
    uint16_t transformers; ///< True (1) - Dois transformadores False (0) - Um transformador.
} t_sync_frame;

//! Pacote das estatisticas do sincronismo entre interfaces.
typedef struct {
    uint32_t T1,    ///< Valor m�dio utilizado para compensar a propaga��o na fibra 1.
    T1_Max,         ///< Valor m�ximo calculado na propaga��o da fibra 1.
    T1_Min;         ///< Valor minimo calculado na propaga��o da fibra 1.
    uint32_t T2,    ///< Valor m�dio utilizado para compensar a propaga��o na fibra 2.
    T2_Max,         ///< Valor m�ximo calculado na propaga��o da fibra 2.
    T2_Min;         ///< Valor minimo calculado na propaga��o da fibra 2.
    float DP1;      ///< Desvio padr�o calculado da propaga��o da fibra 1, necessita revis�o.
    float DP2;      ///< Desvio padr�o calculado da propaga��o da fibra 2, necessita revis�o.
} t_sync_data;

typedef struct {
    float PTC_1;
    float S420_1;
    float PTC_2;
    float S420_2;
} t_temp_val;


//! Pacote de dados brutos.
typedef struct {
//    uint16_t A138[RESULTS_BUFFER_SIZE]; ///< Valores lidos na Fase A do Secund�rio.
//    uint16_t B138[RESULTS_BUFFER_SIZE]; ///< Valores lidos na Fase B do Secund�rio.
//    uint16_t C138[RESULTS_BUFFER_SIZE]; ///< Valores lidos na Fase C do Secund�rio.
//    uint16_t A230[RESULTS_BUFFER_SIZE]; ///< Valores lidos na Fase A do Prim�rio.
//    uint16_t B230[RESULTS_BUFFER_SIZE]; ///< Valores lidos na Fase B do Prim�rio.
//    uint16_t C230[RESULTS_BUFFER_SIZE]; ///< Valores lidos na Fase C do Prim�rio.

    uint16_t A138[1]; ///< Valores lidos na Fase A do Secund�rio.
    uint16_t B138[1]; ///< Valores lidos na Fase B do Secund�rio.
    uint16_t C138[1]; ///< Valores lidos na Fase C do Secund�rio.
    uint16_t A230[1]; ///< Valores lidos na Fase A do Prim�rio.
    uint16_t B230[1]; ///< Valores lidos na Fase B do Prim�rio.
    uint16_t C230[1]; ///< Valores lidos na Fase C do Prim�rio.
}t_adc_results;

//! Pacote de transmiss�o dos dados brutos de tens�o.
/*! Nesse pacote s�o enviados todos os pontos coletados al�m de informa��es adicionais do horario de recebimento do pedido pelo PC,
 *  estatisticas do sincronismo com as interfaces de corrente.
 *  \sa t_adc_results, t_sync_data, t_sync_frame
 *  \warning Pacote n�o utilizado na vers�o atual, ser� corrigido/removido em uma revis�o posterior.
 */
typedef struct {
    int16_t day;                ///< Vari�vel para armazenar o dia da requisi��o (1 - 31).
    int16_t month;              ///< Vari�vel para armazenar o m�s da requisi��o (1 - 12).
    int16_t year;               ///< Vari�vel para armazenar o ano da requisi��o.
    int16_t hour;               ///< Vari�vel para armazenar a hora da requisi��o (0 - 23).
    int16_t minute;             ///< Vari�vel para armazenar o minuto da requisi��o (0 - 59).
    int16_t second;             ///< Vari�vel para armazenar o segundo da requisi��o (0 - 59).
    int16_t msecond;            ///< Vari�vel para armazenar os milisegundos da requisi��o (0 - 999).
    int16_t padding;            ///< Vari�vel para alinhamento de dados, valor fixo 0x1234.
    uint32_t timer1;
    uint32_t timer2;
    t_sync_frame time;          ///< Vari�vel para armazenar o pacote de dados do sincronismo.
    t_sync_data sync_data;      ///< Vari�vel para armazenar o pacote de estatisticas do sincronismo.
    volatile t_adc_results* AD; ///< Vari�vel para armazenar os dados lidos pelos ADs.
} t_voltage_data_frame;

//! Pacote de transmiss�o dos fasores de tens�o.
/*! Nesse pacote s�o enviados os fasores calculados, al�m de informa��es adicionais do horario de recebimento do pedido pelo PC,
 *  estatisticas do sincronismo com as interfaces de corrente.
 *  \sa t_sync_data, t_sync_frame
 */
typedef struct {
    int16_t day;            ///< Vari�vel para armazenar o dia da requisi��o (1 - 31).
    int16_t month;          ///< Vari�vel para armazenar o m�s da requisi��o (1 - 12).
    int16_t year;           ///< Vari�vel para armazenar o ano da requisi��o.
    int16_t hour;           ///< Vari�vel para armazenar a hora da requisi��o (0 - 23).
    int16_t minute;         ///< Vari�vel para armazenar o minuto da requisi��o (0 - 59).
    int16_t second;         ///< Vari�vel para armazenar o segundo da requisi��o (0 - 59).
    int16_t msecond;        ///< Vari�vel para armazenar os milisegundos da requisi��o (0 - 999).
    int16_t padding;        ///< Vari�vel para alinhamento de dados, valor fixo 0x1234.
    uint32_t timer1;
    uint32_t timer2;
    t_sync_frame time;      ///< Vari�vel para armazenar o pacote de dados do sincronismo.
    t_sync_data sync_data;  ///< Vari�vel para armazenar o pacote de estatisticas do sincronismo.
    float A138_A; ///< Amplitude calculada da Fase A do Secund�rio.
    float A138_P; ///< Fase calculada da Fase A do Secund�rio.
    float B138_A; ///< Amplitude calculada da Fase B do Secund�rio.
    float B138_P; ///< Fase calculada da Fase B do Secund�rio.
    float C138_A; ///< Amplitude calculada da Fase C do Secund�rio.
    float C138_P; ///< Fase calculada da Fase C do Secund�rio.
    float A230_A; ///< Amplitude calculada da Fase A do Prim�rio.
    float A230_P; ///< Fase calculada da Fase A do Prim�rio.
    float B230_A; ///< Amplitude calculada da Fase B do Prim�rio.
    float B230_P; ///< Fase calculada da Fase B do Prim�rio.
    float C230_A; ///< Amplitude calculada da Fase C do Prim�rio.
    float C230_P; ///< Fase calculada da Fase C do Prim�rio.
    uint16_t acquisition_counter; ///< Contador de aquisi��es para garantir a sincronia entre interfaces.
} t_voltage_phasor_frame;

//! Pacote de transmiss�o dos dados brutos de corrente.
/*!
*  \sa t_adc_results, t_sync_frame
*  \warning Pacote n�o utilizado na vers�o atual, ser� corrigido/removido em uma revis�o posterior.
*/
typedef struct {
    t_sync_frame timeC1;        ///< Vari�vel para armazenar o pacote de dados do sincronismo.
    uint64_t d_timer;
    t_temp_val temperatura;
    volatile t_adc_results* AD; ///< Vari�vel para armazenar os dados lidos pelos ADs.
} t_current_data_frame;

//! Pacote de transmiss�o dos fasores de corrente.
/*!
 *  \sa t_sync_frame
 */
typedef struct {
    t_sync_frame timeC1;    ///< Vari�vel para armazenar o pacote de dados do sincronismo.
    uint64_t d_timer;
    t_temp_val temperatura; ///< Vari�vel para armazenar os valores lidos dos sensores de temperatura.
    float A138_A;           ///< Amplitude calculada da Fase A do Secund�rio.
    float A138_P;           ///< Fase calculada da Fase A do Secund�rio.
    float B138_A;           ///< Amplitude calculada da Fase B do Secund�rio.
    float B138_P;           ///< Fase calculada da Fase B do Secund�rio.
    float C138_A;           ///< Amplitude calculada da Fase C do Secund�rio.
    float C138_P;           ///< Fase calculada da Fase C do Secund�rio.
    float A230_A;           ///< Amplitude calculada da Fase A do Prim�rio.
    float A230_P;           ///< Fase calculada da Fase A do Prim�rio.
    float B230_A;           ///< Amplitude calculada da Fase B do Prim�rio.
    float B230_P;           ///< Fase calculada da Fase B do Prim�rio.
    float C230_A;           ///< Amplitude calculada da Fase C do Prim�rio.
    float C230_P;           ///< Fase calculada da Fase C do Prim�rio.
    uint16_t acquisition_counter; ///< Contador de aquisi��es para garantir a sincronia entre interfaces.
} t_current_phasor_frame;

typedef struct {
    uint16_t PTP_1[100];
    uint16_t PTN1_1[100];
    uint16_t PTN2_1[100];
    uint16_t S420_1[100];
    uint16_t PTP_2[100];
    uint16_t PTN1_2[100];
    uint16_t PTN2_2[100];
    uint16_t S420_2[100];
} t_temp_data;

#endif
