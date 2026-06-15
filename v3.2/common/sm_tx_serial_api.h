#ifndef _SM_TX_SERIAL_API_
#define _SM_TX_SERIAL_API_

/**
 * @file
 * @brief M�quina de estados que implementa o processo de transmiss�o do protocolo CISEI para comunica��o entre m�dulos.
 * @details Esta m�quina de estados � baseada em StateMachine e � orientada ao evento de interrup��o serial. A cada byte transmitido, a fun��o tx_interrupt() � evocada solicitando novo byte a ser transmitido.
 * O estado inicial � SM_TX_WAITING
 * @author Afonso Ferreira Miguel.
 * \image html SM_TX.png "Diagramas de estado do processo de recep��o"
*/

#include "sm.h"
#include "datatypes.h"
#include "sm_serial_api.h"

/**
 * @brief Ponteiro de fun��o (CALLBACK). A fun��o apontada � respons�vel por inserir na porta serial.
 * \see init_tx_serial
 */
typedef void (*tpTxByte)(uint8_t);

typedef struct {
    teSerialFrameType type:8;      ///< Tipo do frame a ser transmitido pela comunica��o serial. \see teSerialFrameType.
    uint8_t* ptr_payload;          ///< Ponteiro com os dados a serem transmitidos.
    uint32_t bytes_to_tx;          ///< Informa quantos dados ser�o transmitidos.
    uint32_t index_to_tx;          ///< �ndice que � incrementado a cada byte sendo transmitido.
    uint16_t chksum;               ///< Vari�vel tempor�ria que � utilizada para o c�lculo do CHKSUM. A cada byte transmitido esta vari�vel � atualizada.
    StateMachine sm_tx_serial_api; ///< M�quina de estados utilizada para a transmiss�o dos dados. \see StateMachine
    uint16_t isFinished;           ///< Sinalizador que informa que a transmiss�o foi conclu�da.
    tpTxByte pTxByte;              ///< Ponteiro do byte a ser transmitido (LEGADO - N�o � mais utilizada).
    boolean tx_FF;                 ///< Sinalizador que indica se um byte 0xFF deve ser constantemente transmitido quando nenhum frame est� dispon�vel para a comunica��o. Esta op��o � usada para sincronica��o entre o m�dulo de tens�o e a porta serial/usb do computador.
    boolean tx_start;              ///< Sinalizador que indica a necessidade de come�ar a transmiss�o de um frame.
    uint8_t estado;                ///< Vari�vel utilizada apenas para depura��o identificando o estado atual da m�quina de estados da transmiss�o.
} CiseiTxChannel;

/**
* Fun��o que inicializa a estrutura para a comunica��o serial de transmiss�o.
* @param tx Ponteiro para uma estrutura CiseiTxChannel com o estado da transmiss�o serial.
* @param pTxByteFunc Ponteiro da fun��o para transmiss�o de dados.
* @param dummy Sinalizador que indica a necessidade de ficar transmitindo o caracter 0xFF quando n�o houver frame a ser transmitindo.
*/
void init_tx_serial(CiseiTxChannel* tx, tpTxByte pTxByteFunc, boolean dummy);
/**
* Fun��o que dispara a comunica��o serial de transmiss�o.
* @param tx Ponteiro para uma estrutura CiseiTxChannel com o estado da transmiss�o serial.
* @param type Tipo do frame a ser transmitido pela comunica��o serial.
* @param pPayload Ponteiro de um vetor com os dados a serem transmitidos.
* @param nBytes N�mero de bytes do vetor com os dados a serem transmitidos.
\see teSerialFrameType.
*/
uint8_t start_tx_frame(CiseiTxChannel* tx, teSerialFrameType type, uint8_t* pPayload, uint32_t nBytes);

/**
* Fun��o evocada ao concluir a transmiss�o de dados pela porta serial.
* @param tx Ponteiro para uma estrutura CiseiTxChannel com o estado da transmiss�o serial.
*/
void tx_interrupt(CiseiTxChannel* tx);

/**
* Fun��o utilizada para encerrar a transmiss�o serial.
* @param tx Ponteiro para uma estrutura CiseiTxChannel com o estado da transmiss�o serial.
*/
uint16_t tx_end(CiseiTxChannel* tx);

uint8_t start_tx_frame_software(CiseiTxChannel* tx, teSerialFrameType type, uint8_t* pPayload, uint32_t nBytes);

STATE(SM_START);                ///< Dispara a comunica��o serial (inicia a transmiss�o do frame).
STATE(SM_TX_WAITING);           ///< Estado aguardando o disparo da transmiss�o serial
STATE(SM_TX_DATA);              ///< Transmitindo os dados do payload na porta serial
STATE(SM_TX_BYTE_STUFFING);     ///< Um byte ESC foi transmitido pela porta serial sinalizando bytestuffing.
STATE(SM_TX_PAYLOAD_SENDED);    ///< Estado sinalizando que o payload foi transmitido (LEGADO: n�o sendo usado).
STATE(SM_TX_CHKSUM_LSB);        ///< Estado sinalizando que o byte LSB de CHKSUM est� sendo transmitido.
STATE(SM_TX_CHKSUM_MSB);        ///< Estado sinalizando que o byte MSB de CHKSUM est� sendo transmitido.
STATE(SM_TX_FINALIZE);          ///< Estado sinalizando o fim da transmiss�o serial.

STATE(SM_START_SOFT);                ///< Dispara a comunica��o serial (inicia a transmiss�o do frame).
STATE(SM_TX_WAITING_SOFT);           ///< Estado aguardando o disparo da transmiss�o serial
STATE(SM_TX_DATA_SOFT);              ///< Transmitindo os dados do payload na porta serial
#endif
