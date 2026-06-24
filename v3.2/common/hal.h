#ifndef INIT_HAL_h
#define INIT_HAL_h

/**
 * @file
 * @brief Camada de abstra��o de hardware.
 * @details Neste arquivo est�o as declara��es de tipos e fun��es que ser�o utilizados no projeto.
 * As fun��es aqui declaradas devem ser implementadas de acordo com o hardware a ser utilizado.
 * @author Lucas Murbach Pierin.
 */

#include <stdint.h>
#include "datatypes.h"
#include <math.h>
#include "defines.h"



#ifdef __TMS320C28X__

#include "F28x_Project.h"
#include "F28377S/pins.h"



#if defined (BOARD_NEW)
#if defined (TENSAO)
    #define set_TXA()  {GpioDataRegs.GPASET.all   = (1L<<TXA);}
    #define clr_TXA()  {GpioDataRegs.GPACLEAR.all = (1L<<TXA);}
#else
    #define set_TXA()  {GpioDataRegs.GPBSET.all   = (1L<<(TXA-32));}
    #define clr_TXA()  {GpioDataRegs.GPBCLEAR.all = (1L<<(TXA-32));}
#endif

    #define set_TXB()  {GpioDataRegs.GPASET.all   = (1L<<TXB);}
    #define clr_TXB()  {GpioDataRegs.GPACLEAR.all = (1L<<TXB);}

    #define set_TXC()  {GpioDataRegs.GPBSET.all   = (1L<<(TXC-64));}
    #define clr_TXC()  {GpioDataRegs.GPBCLEAR.all = (1L<<(TXC-64));}
#elif defined (BOARD_PREVIOUS)
    #define set_TXA()  {GpioDataRegs.GPBSET.all   = (1L<<(TXA-32));}
    #define clr_TXA()  {GpioDataRegs.GPBCLEAR.all = (1L<<(TXA-32));}

    #define set_TXB()  {GpioDataRegs.GPASET.all   = (1L<<TXB);}
    #define clr_TXB()  {GpioDataRegs.GPACLEAR.all = (1L<<TXB);}

    #define set_TXC()  {GpioDataRegs.GPBSET.all   = (1L<<(TXC-32));}
    #define clr_TXC()  {GpioDataRegs.GPBCLEAR.all = (1L<<(TXC-32));}
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

// Adicionado (por Almeida)
#define set_TXD()  {GpioDataRegs.GPBSET.all   |= (1L<<(TXD-32));}
#define clr_TXD()  {GpioDataRegs.GPBCLEAR.all |= (1L<<(TXD-32));}

#define clr_LED1()  {GpioDataRegs.GPBSET.all   = (1L<<(LED1-32));}
#define set_LED1()  {GpioDataRegs.GPBCLEAR.all = (1L<<(LED1-32));}

#define clr_LED2()  {GpioDataRegs.GPCSET.all   = (1L<<(LED2-64));}
#define set_LED2()  {GpioDataRegs.GPCCLEAR.all = (1L<<(LED2-64));}

#define clr_LED3()  {GpioDataRegs.GPCSET.all   = (1L<<(LED3-64));}
#define set_LED3()  {GpioDataRegs.GPCCLEAR.all = (1L<<(LED3-64));}

#define set_DEBUG1()  {GpioDataRegs.GPBSET.all   = (1L<<(DEBUG1-32));}
#define clr_DEBUG1()  {GpioDataRegs.GPBCLEAR.all = (1L<<(DEBUG1-32));}

#define set_DEBUG2()  {GpioDataRegs.GPBSET.all   = (1L<<(DEBUG2-32));}
#define clr_DEBUG2()  {GpioDataRegs.GPBCLEAR.all = (1L<<(DEBUG2-32));}

#define set_DEBUG3()  {GpioDataRegs.GPBSET.all   = (1L<<(DEBUG3-32));}
#define clr_DEBUG3()  {GpioDataRegs.GPBCLEAR.all = (1L<<(DEBUG3-32));}

#define set_DEBUG4()  {GpioDataRegs.GPBSET.all   = (1L<<(DEBUG4-32));}
#define clr_DEBUG4()  {GpioDataRegs.GPBCLEAR.all = (1L<<(DEBUG4-32));}

#define set_DEBUG5()  {GpioDataRegs.GPBSET.all   = (1L<<(DEBUG5-32));}
#define clr_DEBUG5()  {GpioDataRegs.GPBCLEAR.all = (1L<<(DEBUG5-32));}

#define set_DEBUG6()  {GpioDataRegs.GPBSET.all   = (1L<<(DEBUG6-32));}
#define clr_DEBUG6()  {GpioDataRegs.GPBCLEAR.all = (1L<<(DEBUG6-32));}

#define set_DEBUG7()  {GpioDataRegs.GPBSET.all   = (1L<<(DEBUG7-32));}
#define clr_DEBUG7()  {GpioDataRegs.GPBCLEAR.all = (1L<<(DEBUG7-32));}

#define set_DEBUG8()  {GpioDataRegs.GPBSET.all   = (1L<<(DEBUG8-32));}
#define clr_DEBUG8()  {GpioDataRegs.GPBCLEAR.all = (1L<<(DEBUG8-32));}

/**
* @brief Interrup��o de transmiss�o da SCI-A.
* @details Interrup��o para execu��o da m�quina de estadaos de transmiss�o do primeiro canal
* de comunica��o �ptica, dependendo da vers�o de interface � utilizado o SCI-D ao inv�s do SCI-A.
* @see smTX_D()
*/
interrupt void smTX_A(void);

/**
* @brief Interrup��o de recep��o da SCI-A.
* @details Interrup��o para execu��o da m�quina de estadaos de recep��o do primeiro canal
* de comunica��o �ptica, dependendo da vers�o de interface � utilizado o SCI-D ao inv�s do SCI-A.
* @see smRX_D()
*/
interrupt void smRX_A(void);

/**
* @brief Interrup��o de transmiss�o da SCI-B.
* @details Interrup��o para execu��o da m�quina de estadaos de transmiss�o da comunica��o
* via USB.
*/
interrupt void smTX_B(void);

/**
* @brief Interrup��o de recep��o da SCI-B.
* @details Interrup��o para execu��o da m�quina de estadaos de recep��o da comunica��o
* via USB.
*/
interrupt void smRX_B(void);

/**
* @brief Interrup��o de transmiss�o da SCI-C.
* @details Interrup��o para execu��o da m�quina de estadaos de transmiss�o do segundo canal
* de comunica��o �ptica.
*/
interrupt void smTX_C(void);

/**
* @brief Interrup��o de recep��o da SCI-C.
* @details Interrup��o para execu��o da m�quina de estadaos de recep��o  do segundo canal
* de comunica��o �ptica.
*/
interrupt void smRX_C(void);

// Adicionado (por Almeida)
/**
* @brief Interrup��o de transmiss�o da SCI-D.
* @details Interrup��o para execu��o da m�quina de estadaos de transmiss�o primeiro canal
* de comunica��o �ptica, dependendo da vers�o de interface � utilizado o SCI-A ao inv�s do SCI-D.
* @see smTX_A()
*/
interrupt void smTX_D(void);

/**
* @brief Interrup��o de recep��o da SCI-D.
* @details Interrup��o para execu��o da m�quina de estadaos de recep��o primeiro canal
* de comunica��o �ptica, dependendo da vers�o de interface � utilizado o SCI-A ao inv�s do SCI-D.
* @see smRX_A()
*/
interrupt void smRX_D(void);

/**
* @brief Interrup��o do conversor AD.
* @details Tratamento da interrup��o de fim de convers�o dos conversores AD.
*/
interrupt void readADC(void);

/**
* @brief Interrup��o de sincronia.
* @details Tratamento da interrup��o de sincronia entre as interfaces nos m�dulos de
* corrente e no m�dulo de tens�o � utilizada para obter o valor utilizado no c�lculo
* das compesa��es de sincronismo.
*/
interrupt void syncPulse(void);

/**
* @brief Interrup��o de timer.
* @details Tratamento da interrup��o do timer, utilizada no calculo das compesa��es
* de sincronismo.
*/
interrupt void timer1_isr(void);

/**
* @brief Interrup��o externa.
* @details Tratamento da interrup��o do externa, utilizada no m�dulo de tens�o para obter
*  o valor utilizado no c�lculo das compesa��es de sincronismo
*/
interrupt void xint2_isr(void);

/**
* @brief Interrup��o dummy.
* @details Tratamento da interrup��o que n�o faz nada.
*/
interrupt void doNothing(void);

#endif
#ifdef _MSC_VER
    void set_TXB();
    void clr_TXB();
#endif

/**
* @brief Fun��o de envio de dados pela softwart UART.
* @details Esta fun��o envia o dado passado como parametro utilizando a interface de
* comunica��o soft-serial.
* @param b Dado a ser enviado
*/
void tx_byte_soft(uint8_t * b);
/**
* @brief Fun��o de envio de dados pela SCI-A.
* @details Esta fun��o envia o dado passado como parametro utilizando a interface de
* comunica��o serial A.
* @param b Dado a ser enviado
*/
void tx_A_byte(uint8_t b);
/**
* @brief Fun��o de leitura de dados pela SCI-A.
* @details Esta fun��o retorna o dado recebido utilizando a interface de
* comunica��o serial A.
* @return Dado recebido pela interface de comunica��o
*/
void rx_byte_stm_soft(void);

uint8_t rx_A_byte();

/**
* @brief Fun��o de envio de dados pela SCI-B.
* @details Esta fun��o envia o dado passado como parametro utilizando a interface de
* comunica��o serial B.
* @param b Dado a ser enviado
*/
void tx_B_byte(uint8_t b);

/**
* @brief Fun��o de leitura de dados pela SCI-B.
* @details Esta fun��o retorna o dado recebido utilizando a interface de
* comunica��o serial B.
* @return Dado recebido pela interface de comunica��o.
*/
uint8_t rx_B_byte();

/**
* @brief Fun��o de envio de dados pela SCI-C.
* @details Esta fun��o envia o dado passado como parametro utilizando a interface de
* comunica��o serial C.
* @param b Dado a ser enviado.
*/
void tx_C_byte(uint8_t b);

/**
* @brief Fun��o de leitura de dados pela SCI-C.
* @details Esta fun��o retorna o dado recebido utilizando a interface de
* comunica��o serial C.
* @return Dado recebido pela interface de comunica��o.
*/
uint8_t rx_C_byte();

// Adicionado (por Almeida)
/**
* @brief Fun��o de envio de dados pela SCI-C.
* @details Esta fun��o envia o dado passado como parametro utilizando a interface de
* comunica��o serial C.
* @param b Dado a ser enviado.
*/
void tx_D_byte(uint8_t b);

/**
* @brief Fun��o de leitura de dados pela SCI-D.
* @details Esta fun��o retorna o dado recebido utilizando a interface de
* comunica��o serial D.
* @return Dado recebido pela interface de comunica��o.
*/
uint8_t rx_D_byte();

/**
* @brief Fun��o de inicializa��o da HAL para interfaces de tens�o.
* @details Esta fun��o inicializa a camada de abstra��o de hardware
* dos m�dulos de tens�o.
*/
void init_hal(void);


/**
* @brief Fun��o de inicializa��o da HAL para interfaces de corrente.
* @details Esta fun��o inicializa a camada de abstra��o de hardware
* dos m�dulos de corrente.
*/
void init_hal_C(void);

/**
* @brief Fun��o de configura��o dos CADs internos.
* @details Esta fun��o configura os Conversores Analogico-Digital internos
* do DSP.
* @warning Os conversores internos n�o s�o mais utilizados na vers�o atual.
*/
void config_ADC(void);

/**
* @brief Fun��o para troca de estado de um GPIO.
* @details Esta fun��o inverte o estado de sa�da de algum dos pinos de GPIO
* que � passado por parametro
* @param pin N�mero do GPIO escolhido.
*/
void togglePin(uint8_t);

/**
* @brief Fun��o selecionar o estado de um GPIO.
* @details Esta fun��o seleciona o estado de sa�da de algum dos pinos de GPIO
* que � passado por parametro.
* @param pin N�mero do GPIO escolhido.
* @param value Valor selecionado para o pino, 0 - LOW 1 - HIGH.
*/
void setPin(uint8_t, uint8_t);

/**
* @brief Fun��o para ler de estado de um pino.
* @details Esta fun��o retorna o estado de sa�da de algum dos pinos de GPIO
* que � passado por parametro
* @param pin N�mero do GPIO escolhido.
* @return Valor lido do pino, 0 - LOW 1 - HIGH.
*/
uint8_t readPin(uint8_t);

/**
* @brief Fun��o incializa��o do sistema.
* @details Nesta fun��o s�o inicializados os registradores de sistema e
* tamb�m os registradores de clock dos perif�ricos necess�rios.
*/
void sysInit(void);

/**
* @brief Fun��o incializa��o do GPIO.
* @details Nesta fun��o s�o inicializados e configurados os pinos de GPIO
* utilizados.
*/
void GPIOInit(void);

/**
* @brief Fun��o de configura��o do ePWM.
* @details Esta fun��o configura o m�dulo ePWM para gerar os pulsos de Start of
* convertion dos CADs internos.
* @warning N�o � utilizado na vers�o atual.
*/
void ConfigureEPWM(void);

/**
* @brief Fun��o de configura��o do estimulo do ePWM para os CADs.
* @details Esta fun��o configura os m�dulos do CAD para reagir aos estimulos
* gerados pelo ePWM para o inicio de convers�o.
* @warning N�o � utilizado na vers�o atual.
*/
void SetupADCsEpwm(void);
//void delay(uint8_t); //Fun��o n�o implementada, verificar.

/**
* @brief Fun��o de convers�o do valor lido.
* @details Esta fun��o calcula um equivalente em tens�o do valor digital
* gerado pelo CAD.
* @param adcres Valor retornado pelo CAD.
* @param bits Valor de bits do CAD.
* @return Valor equivalente em volts.
*/
float64 analogEq(Uint16 adcres, Uint16 bits);

/**
* @brief Fun��o incializa��o e configura��o dos timers.
* @details Nesta fun��o s�o inicializados e configurados os tr�s timers do
* DSP para as suas fun��es e temporiza��es.
*/
void configTimer(void);

/**
* @brief Fun��o reset do timer 0.
* @details Nesta fun��o o timer 0 do DSP � carregado com o valor programado
* em configTimer(), assim reiniciando sua contagem.
*/
void resetTimer0(void);

/**
* @brief Fun��o para configurar a SCI para sincronismo.
* @details Nesta fun��o os pinos utilizados para a interface de comunica��o
* serial s�o configurados como GPIO para serem utilizados no processo de
* sincronismo de interfaces.
*/
void configureSCI_sync(void);

/**
* @brief Fun��o para configurar a SCI para UART.
* @details Nesta fun��o os pinos utilizados para a interface de comunica��o
* serial s�o configurados novamente configurados como UART para relaizar
* a comunica��o e envio de dados entre as interfaces.
*/
void configureSCI_UART(void);

/**
* @brief Fun��o para iniciar a captura de dados.
* @details Nesta fun��o � feita a configura��o do Write Enable da CPLD e
* enviado o comando para iniciar a captura.
* @see CPLD_AD_CONV()
*/
void startCapture(void);

/**
* @brief Retorna o valor de relogio.
* @details Esta fun��o retorna o valor do timer 0 no momento em que � chamada,
* esse valor � utilizado para o cronometro.
* @return Valor de 32bits do timer, resolu��o de 1us.
* @see ChronoStart()
*/
uint32_t now(void);

void serial_rx_to_stm_Init(void);

void serial_tx_to_stm_Init(void);

void serial_tx_to_fpga_Init(void)

/**
* @brief Fun��o para inicializar a SCI-A.
*/
void serial_A_Init(void);

/**
* @brief Fun��o para inicializar a SCI-B.
*/
void serial_B_Init(void);

/**
* @brief Fun��o para inicializar a SCI-C.
*/
void serial_C_Init(void);

// Adicionado (por Almeida)
/**
* @brief Fun��o para inicializar a SCI-D.
*/
void serial_D_Init(void);

/**
* @brief Fun��o para reinicializa��o do Buffer de resultados.
* @details Percorre todo o buffer de resultados escrevendo zeros,
* facilita a vizualiza��o se ocorreu uma falha de aquisi��o.
* @warning N�o utilizado na vers�o atual.
*/
void resetResultBuffer(void);

/**
* @brief Fun��o para ativar as interrup��es de comunica��o serial.
* @details Essa fun��o ativa as flags de interrup��o de
* recep��o e transmiss�o de todas as interfaces de comunica��o
* utilizadas.
*/
void activateUART_Ints(void);

/**
* @brief Fun��o para inicializar as interrup��es.
* @details Esta fun��o configura os callbacks das interrup��es
* e inicializa os flags do PIE (Peripheral Interrupt Expansion)
* para as interrup��es utilizadas.
*/
void initInts();

/**
* @brief Fun��o para checar se uma aquisi��o terminou.
* @return TRUE(1) se ja terminou. FALSE(0) se ainda esta acontecendo.
*/
uint8_t endCapture();

/**
* @brief Fun��o para habilitar a interrup��o de sincronismo.
* @details Esta fun��o configura o callback da interrup��o
* e inicializa os flags de enable da interrup��o.
*/
void syncInt_ena();

/**
* @brief Fun��o para desabilitar a interrup��o de sincronismo.
*/
void syncInt_dis();

/**
* @brief Fun��o para habilitar/desabilitar as interrup��es de transmiss�o.
* @details Esta fun��o habilita ou desabilita as interrup��es de transmiss�o
* serial dependendo do valor passado como parametro.
* @param enable TRUE(1) para habilitar. FALSE(0) para desabilitar.
*/
void TXInts(uint8_t);

/**
* @brief Fun��o para habilitar/desabilitar as interrup��es de recep��o.
* @details Esta fun��o habilita ou desabilita as interrup��es de recep��o
* serial dependendo do valor passado como parametro.
* @param enable TRUE(1) para habilitar. FALSE(0) para desabilitar.
*/
void RXInts(uint8_t);

/**
* @brief Fun��o para popular o buffer de resultados.
* @details Escreve no buffer de resultados da fase A do secund�rio
* o valor passado por parametro na posi��o tamb�m passsada como parametro.
* @param index Posi��o do vetor a ser populada.
* @param value Valor que sera escrito.
* @warning N�o utilizado na vers�o atual.
*/
void setA138(uint16_t index, uint16_t value);

/**
* @brief Fun��o para popular o buffer de resultados.
* @details Escreve no buffer de resultados da fase B do secund�rio
* o valor passado por parametro na posi��o tamb�m passsada como parametro.
* @param index Posi��o do vetor a ser populada.
* @param value Valor que sera escrito.
* @warning N�o utilizado na vers�o atual.
*/
void setB138(uint16_t index, uint16_t value);

/**
* @brief Fun��o para popular o buffer de resultados.
* @details Escreve no buffer de resultados da fase C do secund�rio
* o valor passado por parametro na posi��o tamb�m passsada como parametro.
* @param index Posi��o do vetor a ser populada.
* @param value Valor que sera escrito.
* @warning N�o utilizado na vers�o atual.
*/
void setC138(uint16_t index, uint16_t value);

/**
* @brief Fun��o para popular o buffer de resultados.
* @details Escreve no buffer de resultados da fase A do prim�rio
* o valor passado por parametro na posi��o tamb�m passsada como parametro.
* @param index Posi��o do vetor a ser populada.
* @param value Valor que sera escrito.
* @warning N�o utilizado na vers�o atual.
*/
void setA230(uint16_t index, uint16_t value);

/**
* @brief Fun��o para popular o buffer de resultados.
* @details Escreve no buffer de resultados da fase B do prim�rio
* o valor passado por parametro na posi��o tamb�m passsada como parametro.
* @param index Posi��o do vetor a ser populada.
* @param value Valor que sera escrito.
* @warning N�o utilizado na vers�o atual.
*/
void setB230(uint16_t index, uint16_t value);

/**
* @brief Fun��o para popular o buffer de resultados.
* @details Escreve no buffer de resultados da fase C do prim�rio
* o valor passado por parametro na posi��o tamb�m passsada como parametro.
* @param index Posi��o do vetor a ser populada.
* @param value Valor que sera escrito.
* @warning N�o utilizado na vers�o atual.
*/
void setC230(uint16_t index, uint16_t value);

void phasors_int_to_float(CiseiRxChannel * rx_Fibra1, tms320_board_data_t * board_data);

void phasors_ret_to_polar(tms320_board_data_t * board_data);

void quickSort(uint16_t vet[], int16_t esq, int16_t dir);

float64 tempNTC(float res, uint16_t adc1, uint16_t adc2);

void ads1118_int_to_float(CiseiRxChannel * rx_Fibra, tms320_board_data_t * board_data);

void tms320_frame_crc(tms320_data_t * tms320_data, tms320_uart_frame_t * tms320_uart_frame);

extern uint16_t acquisition_counter;

#endif
