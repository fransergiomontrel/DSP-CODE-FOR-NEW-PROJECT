#ifndef CRC16_H_
#define CRC16_H_

/**
 * @file
 * @brief Declaração das funções de calculo do CRC16.
 * @details Neste arquivo estão as declarações de funções que serão
 *  utilizadas para a inicialização e calculo do CRC16.
 * @author Afonso Ferreira Miguel.
 */

#include <stdio.h>
#include <stdint.h>
#include "stdlib.h"

/**
 * @brief Função para calcular o CRC.
 * @details Realiza o calculo do CRC16.
 * @param value Valor a ser adicionado no calculo do CRC
 * @return Valor calculado do CRC.
 */
uint16_t crc16_data(uint16_t);

/**
 * @brief Função que inicializa o CRC
 */
void crc16_init();



#endif /* CRC16_H_ */
