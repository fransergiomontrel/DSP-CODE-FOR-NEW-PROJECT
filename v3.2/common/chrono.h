/** @file
 * @brief Implementação de um cronômetro para as máquinas de estado.
 * @details Este cronômetro é baseado no modelo básico apresentado por Tannenbaum.
 * A estrutura apenas armazena o instante final, que é definido pela função ChronoStart() e usado pela função ChronoEnd().
*/
#ifndef _CHRONO_H_
#define _CHRONO_H_

#include "datatypes.h"
//#ifdef __TMS320C28X__
#include "hal.h"
//#else
//#include "..\..\Qt\USB_Commands_V2-1\hal.h"
//#endif
typedef uint32_t chrono; ///< Tipo de dado criado para armazenar o instante final de um cronômetro.

/**
 * Função evocada para disparar um cronômetro
 * @param c Ponteiro para uma variável do tipo chrono que irá armazenar o instante final do cronômetro.
 * @param deltaT Intervalo de tempo para que o cronômetro seja concluído.
 */
void ChronoStart(chrono *c, uint32_t deltaT);
/**
 * Função que avalia se um cronômetro já foi conluído.
 * @param c Ponteiro para uma variável do tipo chrono que armazena o instante final do cronômetro.
 * @return TRUE(1) se o instante final foi alcançado. FALSE(0) se não foi alcançado.
 */
boolean ChronoEnd(chrono *c);

#endif
