/*! \file CPLD_Api.h
 *  \brief Arquivo com a declara��o das fun��es da API de comunica��o com a CPLD.
 *
 */
/*
 * CPLD_Api.h
 *
 *  Created on: 22 de abr de 2021
 *  Author: CISEI
 */
#include "F28x_Project.h"
#include "stdlib.h"
#include "CRC16.h"

#ifndef CPLD_API_H_
#define CPLD_API_H_

#define LED1   44 // Led D5
#define LED2   65 // Led D6
#define LED3   64 // Led D7

#define RST 56
#define CONVST 57
#define M1 63
#define M0 62

//INPUT COMMAND WORD AND REGISTER WRITE OPERATION
#define CLEAR_HWORD 0X6000
#define READ_HWORD 0xC800
#define READ 0x4800
#define WRITE 0xD000
#define WRITE_MS 0xD200
#define WRITE_LS 0xD400
#define SET_HWORD 0xD800

//DEVICE CONFIGURATION AND REGISTER MAPS
#define DEVICE_ID_REG 0x02   //Device ID register
#define RST_PWRCTL_REG 0x04  //Reset and power control register
#define SDI_CTL_REG 0x08     //SDI data input control register
#define SDO_CTL_REG 0x0C     //SDO-x data input control register
#define DATAOUT_CTL_REG 0x10 //Ouput data control register
#define RANGE_SEL_REG 0x14   //Input range selection control register
#define ALARM_REG 0x20       //ALARM output register
#define ALARM_H_TH_REG 0x24  //ALARM high threshold and hysteresis register
#define ALARM_L_TH_REG 0x28  //ALARM low threshold register

//! Modos de acesso a CPLD
typedef enum{
   MODE_CFG_CPLD = 0, ///< Modo 0, configura��o da CPLD.
   MODE_CFG = 1,      ///< Modo 1, configura��o dos conversores AD.
   MODE_ACQ = 2,      ///< Modo 2, aquisi��o dos dados.
   MODE_RW = 3        ///< Modo 3, escrita e leitura da mem�ria.
}cpld_modes;

//! Frame para dos dados dos ADs
/*! Struct criado para receber e enviar comandos para os conversores AD. */
typedef struct{
    Uint32 AD0; ///< Vari�vel para armazenar o valor lido/escrito para o AD1.
    Uint32 AD1; ///< Vari�vel para armazenar o valor lido/escrito para o AD2.
    Uint32 AD2; ///< Vari�vel para armazenar o valor lido/escrito para o AD3.
    Uint32 AD3; ///< Vari�vel para armazenar o valor lido/escrito para o AD4.
    Uint32 AD4; ///< Vari�vel para armazenar o valor lido/escrito para o AD5.
    Uint32 AD5; ///< Vari�vel para armazenar o valor lido/escrito para o AD6.
} Data_Frame;

//! Fun��o de configura��o do GPIO
/*! Fun��o para a configura��o do GPIO para a comunica��o do DSP com a CPLD*/
void CPLD_GPIO_Config();
//! Fun��o para enviar o comando de reset a CPLD
void CPLD_RST();
//! Fun��o de leitura e escrita da CPLD
/*!
 * Fun��o utilizada para leitura e escrita de 8 bits da CPLD.
 * \param a Valor a ser escrito na CPLD.
 * \return Valor lido da CPLD.
 */
Uint16 CPLD_Read_Write_SPI(Uint16);
//! Fun��o de leitura e escrita da CPLD
/*!
 * Fun��o utilizada para leitura e escrita de 32 bits da CPLD.
 * \param a Valor a ser escrito na CPLD.
 * \return Valor lido da CPLD.
 * \sa CPLD_Read_Write_SPI()
 */
Uint32 CPLD_Read_Write_SPI_32(Uint32);
//! Fun��o utilizada para mudar o modo da CPLD.
/*!
 * \param a Modo desejado.
 * \sa cpld_modes
 */
void CPLD_Mode(Uint16);
//! Funcao de teste da m�moria ram
/*!
 * Nessa fun��o s�o lidos e escritos dados em todas as posi��es de mem�ria disponiveis para garantir o seu pleno funcionamento.
 */
void CPLD_TestRAM();
//! Fun��o utilizada indicar erro nos testes realizados.
/*!
 * \param a Indice do Led utilizado para identificar o erro (1 a 3).
 */
void CPLD_Error(Uint16);
//! Fun��o para reinicializa��o dos conversores AD.
void CPLD_RST_AD();
//! Fun��o de teste da comunica��os dos conversores.
/*!
 * Nessa fun��o � realizada a escirta e leitura de dados via SPI para garantir a comunica��o dos ADs com a CPLD.
 */
void CPLD_Test_SPI_AD();
//! Fun��o para realizar a leitura e escrita de dados nos ADs.
/*!
 * \param in Dados para serem escritos nos ADs.
 * \param out Dados lidos dos ADs.
 * \sa CPLD_Read_Write_SPI_32(), Data_Frame
 */
void CPLD_Read_Write_AD(Data_Frame*, Data_Frame*);
//! Fun��o de configura��o dos conversores.
/*!
 * Nessa fun��o � realizada a configura��o dos ADs para atender os requisitos do projeto.
 */
void CPLD_CFG_AD();
//! Fun��o de prepara��o para as convers�es.
/*!
 * Nessa fun��o � realizada a configura��o dos ADs e da CPLD deixando-os prontos para receber o comando de inicio de convers�o do DSP.
 */
void CPLD_AD_CONV();
//! Fun��o para habilitar/desabilitar o Write Enable da mem�ria RAM.
/*!
 * Nessa fun��o � realizada a configura��o da CPLD para habilitar ou desabilitar a prote��o de escrita na mem�ria RAM.
 * \param ena Modo desejado. 1 - Habilitar 0 - Desabilitar
 */
void CPLD_WE(Uint16);
//! Fun��o para obter as configura��es atuais da CPLD.
/*!
 * \return Valor lido da CPLD.
 */
Uint32 CPLD_Get_CFG();
//! Fun��o de teste para as convers�es.
/*!
 * \warning N�o utilizado no programa final, sera retirado em uma revis�o posterior.
 */
void Teste_Modo2();

#endif /* CPLD_API_H_ */
