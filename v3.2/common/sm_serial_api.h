#ifndef _SM_SERIAL_API_
#define _SM_SERIAL_API_

#define SOH	0x01 ///< Identificador do início de frame de dados
#define EOT	0x04 ///< Identificador de fim de frame de dados
#define	ESC	0x1B ///< Sequencia de ESCAPE para Byte Stuffing

#ifdef __TMS320C28X__
    /** @brief Operação de escrita de um byte na memória.
     * @details Esta função é necessária pelo fato do TMS320C ter uma posição de memória com 2 bytes.
     * @param pointer Ponteiro do início da estrutura de dados a se acessada.
     * @param index Deslocamento (orientado a bytes) da posição desejada.
     * @param data Valor a ser escrito na memória.
    */
    #define writeByte(pointer, index, data) {__byte((int*)pointer,index) = data;}

    /** @brief Operação de leitura de um byte da memória.
     * @details Esta função é necessária pelo fato do TMS320C ter uma posição de memória com 2 bytes.
     * @param pointer Ponteiro do início da estrutura de dados a se acessada.
     * @param index Deslocamento (orientado a bytes) da posição desejada.
     * @return Byte armazenado na posição desejada.
    */
    #define readByte(pointer, index) __byte((int*)pointer,index);
#endif

/**
 * @brief Identificação do tipo de frame
 * @details Cada frame transmitido possui uma identificação específica que garante um tratamento específico.
 */
typedef enum
{
    _NONE_ = 10,           ///< Frame sem identificação (geralmente usado em testes)
    DATA_FRAME_TYPE = 11,  ///< Frame de dados
    SYNC_FRAME = 12,       ///< Frame solicitando a sincronização
    CLOCK_FRAME = 13,      ///< Frame para ajuste de relógio
    VOLTAGE_DATA = 14,     ///< Frame com dados de tensão
    CURRENT_DATA_1 = 15,   ///< Frame com dados de corrente do primeiro módulo de corrente
    CURRENT_DATA_2 = 16,   ///< Frame com dados de corrente do segundo módulo de corrente
    CURRENT_DATA_X = 18,   ///< Frame com dados de corrente do módulo de corrente
    SYNC_DELAY = 19,       ///< Frame para calculo de propagação
    SYNC_RESP = 20,        ///< Frame de resposta do frame de propagação
    VOLTAGE_PHASOR = 21,   ///< Frame com fasores de tensão
    CURRENT_PHASOR_1 = 22, ///< Frame com fasores de corrente do primeiro módulo de corrente
    CURRENT_PHASOR_2 = 23, ///< Frame com fasores de corrente do segundo módulo de corrente
    CURRENT_PHASOR_X = 24, ///< Frame com fasores de corrente do módulo de corrente
    DATA_REQUEST = 30,     ///< Frame de Solicitação de envio dos dados aquisitados
    DATA_REQUEST_V = 31,   ///< Frame de Solicitação de envio dos dados de tensão
    DATA_REQUEST_I1 = 32,  ///< Frame de Solicitação de envio dos dados do primeiro módulo de corrente
    DATA_REQUEST_I2 = 33,  ///< Frame de Solicitação de envio dos dados do segundo módulo de corrente
    TEMP_REQUEST = 34,     ///< Frame de Solicitação de aquisição dos dados de temperatura
    END_CONVERSION = 41    ///< Frame para identificação do fim de aquisição de dados

} teSerialFrameType;

#endif
