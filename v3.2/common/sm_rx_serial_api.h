#ifndef _SM_RX_SERIAL_API_
#define _SM_RX_SERIAL_API_

/**
 * @file
 * @brief Máquina de estados que implementa o processo de recepção do protocolo CISEI para comunicação entre módulos.
 * @details Esta máquina de estados é baseada em StateMachine e é orientada ao evento de interrupção serial. A cada byte recebido, a função rx_interrupt() é evocada.
 * O estado inicial é SM_RX_WAITING
 * @author Afonso Ferreira Miguel.
 * \image html SM_RX.png "Diagramas de estado do processo de recepção"
*/

#include "sm.h"
#include "datatypes.h"
#include "sm_serial_api.h"

/**
 * @brief Informações sobre o frame sendo recebido
 * @details Esta estrutura armazena os dados de um frame sendo recebido pela porta de comunicação serial.
 */
typedef struct {
    uint8_t byte_received;              ///< Byte que acaba de ser recebido pela porta serial.
    teSerialFrameType rx_frame_type;    ///< Tipo do frame recebido pela comunicação serial. \see teSerialFrameType.
    uint8_t* pBuffer;                   ///< Ponteiro do buffer de recepção dos dados.
    uint32_t nBytes;                    ///< Número total de bytes do buffer de recepção. \see pBuffer
    uint32_t bytesReceived;             ///< Número de bytes válidos recebidos do frame corrente.
    uint8_t  frameReceived;             ///< Sinalizador que indica que um frame válido foi recebido.
    uint16_t chksum;                    ///< Variável temporária que é utilizada para o cálculo do CHKSUM. A cada byte recebido esta variável é atualizada.
    uint16_t chksumReceived;            ///< CHKSUM do frame recebido.
    StateMachine sm_rx_serial_api;      ///< Máquina de estados utilizada para a recepção dos dados. \see StateMachine
    boolean lock;                       ///< Sinalizador que bloqueia a alteração desta estrutura. Se estiver travado, ignora o dado recém recebido.
}CiseiRxChannel;

/**
* Função evocada a cada interrupção de recepção de dados pela porta serial.
* @details Cada byte recebido pela porta de comunicação serial evoca esta função que atualiza toda a estrutura de comunicação.
* @param rx Ponteiro para uma estrutura CiseiRxChannel com o estado da recepção serial.
* @param b Byte que acaba de ser recebido pela porta serial.
*/
void rx_interrupt(CiseiRxChannel* rx, uint8_t b);

/**
* Função que inicializa a estrutura para a comunicação serial.
* @param rx Ponteiro para uma estrutura CiseiRxChannel com o estado da recepção serial.
* @param pBuffer Ponteiro do buffer de recepção dos dados.
* @param nBytes Número total de bytes do buffer de recepção. \see pBuffer
*/
void init_rx_serial(CiseiRxChannel* rx, uint8_t* pBuffer, uint32_t nBytes);

/**
* Função que libera a recepção de dados. \see frameReceived
* @param rx Ponteiro para uma estrutura CiseiRxChannel com o estado da recepção serial.
*/
void rx_free_frame(CiseiRxChannel* rx);

/**
 * Função que esvazia e inicializa a estrutura de recepção. \see CiseiRxChannel
 * @param rx Ponteiro para uma estrutura CiseiRxChannel com o estado da recepção serial.
 */
void rx_reset_buffer(CiseiRxChannel* rx);

/**
* Função que sinaliza que um frame válido foi recebido.
* @param rx Ponteiro para uma estrutura CiseiRxChannel com o estado da recepção serial.
* @param bytesReceived Ponteiro para uma variável uint32_t que será atualizada com o número de bytes do frame válido recebido. Não é alterada se o frame não foi recebido.
* @return TRUE(1) se o um frame válido foi recebido e está disponível.
*/
uint8_t rx_frameReceived(CiseiRxChannel* rx, uint32_t* bytesReceived);

/**
* Função que retorna o tipo de frame recebido. \see teSerialFrameType
* @param rx Ponteiro para uma estrutura CiseiRxChannel com o estado da recepção serial.
* @return Valor do tipo de frame recebido. \see teSerialFrameType
*/
teSerialFrameType rx_getFrameType(CiseiRxChannel* rx);

STATE(SM_RX_WAITING);
STATE(SM_RX_RECEIVE_TYPE);
STATE(SM_RX_DATA);
STATE(SM_RX_BYTE_STUFFING);
STATE(SM_RX_CHECKSUM_LSB);
STATE(SM_RX_CHECKSUM_MSB);
STATE(SM_RX_CHKSUM_ERROR);
STATE(SM_RX_FULL_BUFFER);

#endif
