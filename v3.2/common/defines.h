/*! \file defines.h
 *  \brief Arquivo com as defini��es de valores fixos.
 */

/*! \def syncMax
 *  \brief N�mero de sincronias que ser�o utilizadas.
 *  Para se obter uma uma melhor sincronia entre as interfaces s�o realizadas varias tentativas para se obter o tempo de propaga��o dos sinais e
 *  compensar posteriormente com a m�dia dos valores obtidos.
 */

/*! \def RESULTS_BUFFER_SIZE
 *  \brief N�mero de amostras coletadas.
 *  O n�mero de amostras coletados � utilizado para a leitura dos dados da mem�ria externa e c�lculo da DFT (Discrete Fourier Transform).
 */

/*! \def K
 *  \brief N�mero da harmonica utilizada.
 *  Valor de K utilizado para o c�lculo da DFT.
 */

/*! \def TARGET_FREQ
 *  \brief Frequ�ncia do sinal a ser aquisitado (em Hz).
 *  \warning Valor n�o utilizado na vers�o atual, sera retirado em uma revis�o posterior.
 */

/*! \def BASE_FREQ
 *  \brief Frequ�ncia do processador (em MHz).
 *  \warning Valor n�o utilizado na vers�o atual, sera retirado em uma revis�o posterior.
 */

/*! \def NUM_CYCLES
 *  \brief N�mero de ciclos a serem aquisitados baseado no valor de TARGET_FREQ.
 *  \warning Valor n�o utilizado na vers�o atual, sera retirado em uma revis�o posterior.
 */

#ifndef DEFINES_h
#define DEFINES_h

#define IED_1 '1'
#define IED_2 '2'
#define IED_3 '3'
#define IED_4 '4'

#define NO_FRAME -1
#define INCOMPLETE_FRAME -2
#define WRONG_FRAME -3
#define ADS1118_CRC_NACK -4

#define NOM_FREQ_50HZ 0x32
#define NOM_FREQ_60HZ 0x3C

#define NO_FIBER     0x00
#define ONE_FIBER    0x01
#define TWO_FIBERS   0x02
#define THREE_FIBERS 0x03
#define FOUR_FIBERS  0x04

#define syncMax 256

#define RESULTS_BUFFER_SIZE 7215  // Number of samples per acquisition
#define LUT_FLOAT_FACTOR    32767  // Number of samples per acquisition
#define TARGET_FREQ         60        // Frequency of target signal (in Hz)
#define BASE_FREQ           200       // Frequency of main processor clock (in MHz)
#define NUM_CYCLES          (1*1)     // Number of cycles to acquire
#define K                   (1*1)     // K value for DFT

#endif
