#ifndef _SM_SERIAL_API_
#define _SM_SERIAL_API_

#define SOH	0x01 ///< Identificador do in�cio de frame de dados
#define EOT	0x04 ///< Identificador de fim de frame de dados
#define	ESC	0x1B ///< Sequencia de ESCAPE para Byte Stuffing

#define TX_STM_SOFT 82
#define RX_STM_SOFT 83

#define TX_FPGA_SOFT 28
#define RX_FPGA_SOFT 30

#ifdef __TMS320C28X__
    /** @brief Opera��o de escrita de um byte na mem�ria.
     * @details Esta fun��o � necess�ria pelo fato do TMS320C ter uma posi��o de mem�ria com 2 bytes.
     * @param pointer Ponteiro do in�cio da estrutura de dados a se acessada.
     * @param index Deslocamento (orientado a bytes) da posi��o desejada.
     * @param data Valor a ser escrito na mem�ria.
    */
    #define writeByte(pointer, index, data) {__byte((int*)pointer,index) = data;}

    /** @brief Opera��o de leitura de um byte da mem�ria.
     * @details Esta fun��o � necess�ria pelo fato do TMS320C ter uma posi��o de mem�ria com 2 bytes.
     * @param pointer Ponteiro do in�cio da estrutura de dados a se acessada.
     * @param index Deslocamento (orientado a bytes) da posi��o desejada.
     * @return Byte armazenado na posi��o desejada.
    */
    #define readByte(pointer, index) __byte((int*)pointer,index);
#endif

/**
 * @brief Identifica��o do tipo de frame
 * @details Cada frame transmitido possui uma identifica��o espec�fica que garante um tratamento espec�fico.
 */
typedef enum
{
    _NONE_ = 10,           ///< Frame sem identifica��o (geralmente usado em testes)
    DATA_FRAME_TYPE = 11,  ///< Frame de dados
    SYNC_FRAME = 12,
    SYNC_FRAME_60HZ = 13,  ///< Frame solicitando a sincronização em 60 Hz
    SYNC_FRAME_50HZ = 14,  ///< Frame solicitando a sincronização em 50 Hz
    SYNC_DELAY = 19,       ///< Frame para calculo de propaga��o
    SYNC_RESP = 20,        ///< Frame de resposta do frame de propaga��o
    VOLTAGE_PHASOR = 21,   ///< Frame com fasores de tens�o
    TMS320_DATA_CRC = 22, ///< Frame com fasores de corrente do primeiro m�dulo de corrente
    CURRENT_PHASOR_2 = 23, ///< Frame com fasores de corrente do segundo m�dulo de corrente
    CURRENT_PHASOR_X = 24, ///< Frame com fasores de corrente do m�dulo de corrente
    DATA_REQUEST = 30,     ///< Frame de Solicita��o de envio dos dados aquisitado
    TEMP_REQUEST = 34,     ///< Frame de Solicita��o de aquisi��o dos dados de temperatura
    END_CONVERSION = 41    ///< Frame para identifica��o do fim de aquisi��o de dados

} teSerialFrameType;

#endif
