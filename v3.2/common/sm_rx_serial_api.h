#ifndef _SM_RX_SERIAL_API_
#define _SM_RX_SERIAL_API_

/**
 * @file
 * @brief M�quina de estados que implementa o processo de recep��o do protocolo CISEI para comunica��o entre m�dulos.
 * @details Esta m�quina de estados � baseada em StateMachine e � orientada ao evento de interrup��o serial. A cada byte recebido, a fun��o rx_interrupt() � evocada.
 * O estado inicial � SM_RX_WAITING
 * @author Afonso Ferreira Miguel.
 * \image html SM_RX.png "Diagramas de estado do processo de recep��o"
*/

#include "sm.h"
#include "datatypes.h"
#include "sm_serial_api.h"

/**
 * @brief Informa��es sobre o frame sendo recebido
 * @details Esta estrutura armazena os dados de um frame sendo recebido pela porta de comunica��o serial.
 */
typedef struct {
    uint8_t byte_received;              ///< Byte que acaba de ser recebido pela porta serial.
    teSerialFrameType rx_frame_type;    ///< Tipo do frame recebido pela comunica��o serial. \see teSerialFrameType.
    uint8_t* pBuffer;                   ///< Ponteiro do buffer de recep��o dos dados.
    uint32_t nBytes;                    ///< N�mero total de bytes do buffer de recep��o. \see pBuffer
    uint32_t bytesReceived;             ///< N�mero de bytes v�lidos recebidos do frame corrente.
    uint8_t  frameReceived;             ///< Sinalizador que indica que um frame v�lido foi recebido.
    uint16_t chksum;                    ///< Vari�vel tempor�ria que � utilizada para o c�lculo do CHKSUM. A cada byte recebido esta vari�vel � atualizada.
    uint16_t chksumReceived;            ///< CHKSUM do frame recebido.
    StateMachine sm_rx_serial_api;      ///< M�quina de estados utilizada para a recep��o dos dados. \see StateMachine
    boolean lock;                       ///< Sinalizador que bloqueia a altera��o desta estrutura. Se estiver travado, ignora o dado rec�m recebido.
}CiseiRxChannel;

/**
* Fun��o evocada a cada interrup��o de recep��o de dados pela porta serial.
* @details Cada byte recebido pela porta de comunica��o serial evoca esta fun��o que atualiza toda a estrutura de comunica��o.
* @param rx Ponteiro para uma estrutura CiseiRxChannel com o estado da recep��o serial.
* @param b Byte que acaba de ser recebido pela porta serial.
*/
void rx_interrupt(CiseiRxChannel* rx, uint8_t b);

/**
* Fun��o que inicializa a estrutura para a comunica��o serial.
* @param rx Ponteiro para uma estrutura CiseiRxChannel com o estado da recep��o serial.
* @param pBuffer Ponteiro do buffer de recep��o dos dados.
* @param nBytes N�mero total de bytes do buffer de recep��o. \see pBuffer
*/
void init_rx_serial(CiseiRxChannel* rx, uint8_t* pBuffer, uint32_t nBytes);

/**
* Fun��o que libera a recep��o de dados. \see frameReceived
* @param rx Ponteiro para uma estrutura CiseiRxChannel com o estado da recep��o serial.
*/
void rx_free_frame(CiseiRxChannel* rx);

/**
 * Fun��o que esvazia e inicializa a estrutura de recep��o. \see CiseiRxChannel
 * @param rx Ponteiro para uma estrutura CiseiRxChannel com o estado da recep��o serial.
 */
void rx_reset_buffer(CiseiRxChannel* rx);

/**
* Fun��o que sinaliza que um frame v�lido foi recebido.
* @param rx Ponteiro para uma estrutura CiseiRxChannel com o estado da recep��o serial.
* @param bytesReceived Ponteiro para uma vari�vel uint32_t que ser� atualizada com o n�mero de bytes do frame v�lido recebido. N�o � alterada se o frame n�o foi recebido.
* @return TRUE(1) se o um frame v�lido foi recebido e est� dispon�vel.
*/
uint8_t rx_frameReceived(CiseiRxChannel* rx, uint32_t* bytesReceived);

uint8_t rx_checkframeReceived(CiseiRxChannel* rx);

/**
* Fun��o que retorna o tipo de frame recebido. \see teSerialFrameType
* @param rx Ponteiro para uma estrutura CiseiRxChannel com o estado da recep��o serial.
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
