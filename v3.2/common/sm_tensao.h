#ifndef _SM_TENSAO_
#define _SM_TENSAO_

#include "sm.h"
#include "datatypes.h"
#include "sm_rx_serial_api.h"
#include "sm_tx_serial_api.h"
#include "CRC16.h"
#include "CPLD_Api.h"

/**
 * @file
 * @brief Declaração dos estados da maquina de estados de tensão.
 * @details Estes estados são utilizados pela maquina de estados que
 * é executada nas interfaces de tensão.
 * @author Lucas Murbach Pierin.
 */

extern StateMachine sm_tensao;

/**
 * @brief Estado inicial da máquina de estados.
 * @details Inicializa a camada de abstração de hardware e configura a CPLD.
 */
STATE(SM_TENSAO_INIT);

/**
 * @brief Estado de testes.
 * @details Realiza os testes de leitura/escrita da memória RAM e
 * comunicação com os CADs.
 * @see CPLD_TestRAM(), CPLD_Test_SPI_AD()
 */
STATE(SM_TENSAO_TEST);

/**
 * @brief Estado configuração.
 * @details Inicializa e configura o protocolo de comunicação e
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

/**
 * @brief Estado de conversão.
 * @details Neste estado a SM espera o final das conversões.
 */
STATE(SM_TENSAO_CONV);

/**
 * @brief Estado para o calculo dos fasores.
 * @details Nesse estado são recuperados os pontos da memória RAM e são
 * calculados os fasores para as seis entradas.
 * @see t_voltage_phasor_frame
 */
STATE(SM_TENSAO_CALC_FAS);

/**
 * @brief Estado de transmissão.
 * @details Neste estado são enviados pela interface de comunicação USB
 * o pacote de dados de tensão para o PC.
 * @see t_voltage_data_frame, t_voltage_phasor_frame
 */
STATE(SM_TENSAO_TX);

/**
 * @brief Estado para finalização da transmissão.
 * @details Quando nesse estado a SM fica esperando a finalização da transmissão
 * dos dados para o PC.
 */
STATE(SM_TENSAO_WAIT_TX);

/**
 * @brief Estado de transmissão do sinalizador de final de conversão.
 * @details Neste estado é enviado um pacote informando o PC que as conversões
 * e calculo dos fasores foram finalizados, ficando disponiveis para requisição.
 */
STATE(SEND_CONV_END);

/**
 * @brief Estado de transmissão do pocte de sincronismo.
 * @details Neste estado é enviado pelos canais de comunicação opticas
 * o pacote informando as interfaces de corrente para se prepararem para
 * receber o sincronismo para uma nova aquisição de dados.
 */
STATE(SM_TENSAO_TX_SYNC);

/**
 * @brief Estado de sincronismo.
 * @details Neste estado é enviado pelos canais de comunicação opticas
 * o pulso de sincronismo já com as compensações de propagação e tamanho
 * de fibra.
 */
STATE(SM_TENSAO_SYNC);

/**
 * @brief Estado de calculo da propagação.
 * @details Neste estado são enviado pelos canais de comunicação opticas
 * varios pulsos similares ao de sincronismo com a finalidade de medir o
 * tempo de propagação do sinal das fibras, assim é realizada uma compensação
 * no momento de enviar o sincronismo para garantir que mesmo as interfaces
 * mais distantes recebam o pulso ao mesmo tempo.
 */
STATE(SM_TENSAO_DELAY);

/**
 * @brief Estado de requisição de dados.
 * @details Neste estado é enviado pelo canal de comunicação optica 1
 * o pacote requisitando os dados de corrente e aguarda o recebimento.
 */
STATE(SM_TENSAO_REQ_I1);

/**
 * @brief Estado de transmissão.
 * @details Neste estado são enviados pela interface de comunicação USB
 * o pacote de dados de corrente da interface 1 para o PC.
 * @see t_current_data_frame, t_current_phasor_frame
 */
STATE(SM_TENSAO_SEND_I1);

/**
 * @brief Estado de requisição de dados.
 * @details Neste estado é enviado pelo canal de comunicação optica 2
 * o pacote requisitando os dados de corrente e aguarda o recebimento.
 */
STATE(SM_TENSAO_REQ_I2);

/**
 * @brief Estado de transmissão.
 * @details Neste estado são enviados pela interface de comunicação USB
 * o pacote de dados de corrente da interface 2 para o PC.
 * @see t_current_data_frame, t_current_phasor_frame
 */
STATE(SM_TENSAO_SEND_I2);

#endif
