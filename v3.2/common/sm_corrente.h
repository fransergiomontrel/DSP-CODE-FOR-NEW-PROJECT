#ifndef _SM_CORRENTE_
#define _SM_CORRENTE_

#include "sm.h"
#include "datatypes.h"
#include "hal.h"
#include "sm_rx_serial_api.h"
#include "sm_tx_serial_api.h"
#include "chrono.h"
#include "stdlib.h"
#include "stdio.h"
#include "CRC16.h"
#include "CPLD_Api.h"


/**
 * @file
 * @brief Declara��o dos estados da maquina de estados de corrente.
 * @details Estes estados s�o utilizados pela maquina de estados que � executada nas interfaces de corrente.
 * @author Lucas Murbach Pierin.
 */

extern StateMachine sm_corrente;

/**
 * @brief Estado inicial da m�quina de estados.
 * @details Inicializa a camada de abstra��o de hardware e configura a CPLD.
 */
STATE(SM_CORRENTE_INIT);

/**
 * @brief Estado de testes.
 * @details Realiza os testes de leitura/escrita da mem�ria RAM e comunica��o com os CADs.
 * @see CPLD_TestRAM(), CPLD_Test_SPI_AD()
 */
STATE(SM_CORRENTE_TEST);

/**
 * @brief Estado configura��o.
 * @details Inicializa e configura o protocolo de comunica��o e configura os CADs.
 * @see init_tx_serial(), init_rx_serial()
 */
STATE(SM_CORRENTE_CFG);

/**
 * @brief Estado menu.
 * @details Nesse estado a maquina de estados (SM) fica aguardando o recebimento de um pacote da interface de tens�o,
 * dependendo do tipo de pacote recebido direciona a SM para o estado apropriado.
 * @see NEXT_STATE, teSerialFrameType
 */
STATE(SM_CORRENTE_WAIT);

/**
 * @brief Estado de sincronia.
 * @details Quando neste estado, a interface de corrente aguarda receber o pulso de sincronismo enviado pela interface de
 * tens�o para inciar uma nova aquisi��o de dados.
 */
STATE(SM_CORRENTE_SYNC);

/**
 * @brief Estado de convers�o.
 * @details Neste estado a SM espera o final das convers�es.
 */
STATE(SM_CORRENTE_CONV);

/**
 * @brief Estado de transmiss�o.
 * @details Neste estado s�o enviados pela interface de comunica��o �ptica o pacote de dados de corrente para a interface
 * de tens�o.
 * @see t_current_data_frame, t_current_phasor_frame
 */
STATE(SM_CORRENTE_TX);

/**
 * @brief Estado para medida de tempo de propaga��o.
 * @details Nesse estado a interface de corrente � colocado em um modo de "loopback" para a medida do tempo de propaga��o
 * na fibra �ptica utilizada.
 */
STATE(SM_CORRENTE_DELAY);

/**
 * @brief Estado para o calculo dos fasores.
 * @details Nesse estado s�o recuperados os pontos da mem�ria RAM e s�o calculados os fasores para as seis entradas.
 * @see t_current_phasor_frame
 */
STATE(SM_CORRENTE_FASOR);

/**
 * @brief Estado para a leitura dos sensores de temperatura.
 * @details Nesse estado s�o recuperados os valores lidos pelos conversores AD e � realizado o calculo para obter os valores de temperatura.
 * @see t_temp_data, t_temp_val
 */
STATE(SM_CORRENTE_TEMPERATURA);
 
#endif
