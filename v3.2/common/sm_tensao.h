#ifndef _SM_TENSAO_
#define _SM_TENSAO_

#include <string.h>
#include "sm.h"
#include "datatypes.h"
#include "sm_rx_serial_api.h"
#include "sm_tx_serial_api.h"
#include "CRC16.h"
#include "CPLD_Api.h"

/**
 * @file
 * @brief Declara��o dos estados da maquina de estados de tens�o.
 * @details Estes estados s�o utilizados pela maquina de estados que
 * � executada nas interfaces de tens�o.
 * @author Lucas Murbach Pierin.
 */

extern StateMachine sm_tensao;

/**
 * @brief Estado inicial da m�quina de estados.
 * @details Inicializa a camada de abstra��o de hardware e configura a CPLD.
 */
STATE(SM_TENSAO_INIT);

/**
 * @brief Estado de testes.
 * @details Realiza os testes de leitura/escrita da mem�ria RAM e
 * comunica��o com os CADs.
 * @see CPLD_TestRAM(), CPLD_Test_SPI_AD()
 */
STATE(SM_TENSAO_TEST);

/**
 * @brief Estado configura��o.
 * @details Inicializa e configura o protocolo de comunica��o e
 * configura os CADs.
 * @see init_tx_serial(), init_rx_serial()
 */
STATE(SM_TENSAO_CFG);

/**
 * @brief Estado menu.
 * @details Nesse estado a maquina de estados (SM) fica aguardando o
 * recebimento de um pacote do PC, dependendo do tipo de pacote recebido
 * direciona a SM para o estado apropriado.
 * @see NEXT_STATE, teSerialFrameType
 */
STATE(SM_TENSAO_WAIT);


STATE(SM_CORRENTE_TEMPERATURA);

/**
 * @brief Estado de convers�o.
 * @details Neste estado a SM espera o final das convers�es.
 */
STATE(SM_TENSAO_CONV);

/**
 * @brief Estado de transmiss�o.
 * @details Neste estado s�o enviados pela interface de comunica��o USB
 * o pacote de dados de tens�o para o PC.
 * @see t_voltage_data_frame, t_voltage_phasor_frame
 */
STATE(SM_TENSAO_TX);

/**
 * @brief Estado de transmiss�o do pocte de sincronismo.
 * @details Neste estado � enviado pelos canais de comunica��o opticas
 * o pacote informando as interfaces de corrente para se prepararem para
 * receber o sincronismo para uma nova aquisi��o de dados.
 */
STATE(SM_TENSAO_TX_SYNC);

/**
 * @brief Estado de sincronismo.
 * @details Neste estado � enviado pelos canais de comunica��o opticas
 * o pulso de sincronismo j� com as compensa��es de propaga��o e tamanho
 * de fibra.
 */
STATE(SM_TENSAO_SYNC);

/**
 * @brief Estado de calculo da propaga��o.
 * @details Neste estado s�o enviado pelos canais de comunica��o opticas
 * varios pulsos similares ao de sincronismo com a finalidade de medir o
 * tempo de propaga��o do sinal das fibras, assim � realizada uma compensa��o
 * no momento de enviar o sincronismo para garantir que mesmo as interfaces
 * mais distantes recebam o pulso ao mesmo tempo.
 */
STATE(SM_TENSAO_DELAY);

/**
 * @brief Estado de requisi��o de dados.
 * @details Neste estado � enviado pelo canal de comunica��o optica 1
 * o pacote requisitando os dados de corrente e aguarda o recebimento.
 */
STATE(SM_TENSAO_REQ_I0);

/**
 * @brief Estado de requisi��o de dados.
 * @details Neste estado � enviado pelo canal de comunica��o optica 1
 * o pacote requisitando os dados de corrente e aguarda o recebimento.
 */
STATE(SM_TENSAO_REQ_I1);

/**
 * @brief Estado de requisi��o de dados.
 * @details Neste estado � enviado pelo canal de comunica��o optica 2
 * o pacote requisitando os dados de corrente e aguarda o recebimento.
 */
STATE(SM_TENSAO_REQ_I2);

/**
 * @brief Estado de requisi��o de dados.
 * @details Neste estado � enviado pelo canal de comunica��o optica 3
 * o pacote requisitando os dados de corrente e aguarda o recebimento.
 */
STATE(SM_TENSAO_REQ_I3);

/**
 * @brief Estado de requisi��o de dados.
 * @details Neste estado � enviado pelo canal de comunica��o optica 4
 * o pacote requisitando os dados de corrente e aguarda o recebimento.
 */
STATE(SM_TENSAO_REQ_I4);

/**
 * @brief Estado de requisi��o de dados.
 * @details Neste estado � enviado pelo canal de comunica��o optica 4
 * o pacote requisitando os dados de corrente e aguarda o recebimento.
 */
STATE(RET_TO_POL_CONVERT);

/**
 * @brief Estado de transmiss�o.
 * @details Neste estado s�o enviados pela interface de comunica��o USB
 * o pacote de dados de corrente da interface 2 para o PC.
 * @see t_current_data_frame, t_current_phasor_frame
 */
STATE(SM_SEND_DATA);

#endif
