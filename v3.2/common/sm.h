
#ifndef SM_H
#define SM_H

#include "chrono.h"
/**
 * @file
 * @brief Framework para implementa��o das m�quinas de estado.
 * @details Este framework utiliza callback como poteiro para cada estado.
 * Cada estado � implementado como uma fun��o, mascarada pelo define STATE().
 * @author Afonso Ferreira Miguel.
 */

/** Tipo de dado como ponteiro de fun��o (callback).
 */
typedef void (*pFuncao)(void *);

//! Estrutura de dados respons�vel por armazenar as informa��es de cada m�quina de estados.
typedef struct{
    pFuncao ptr;            ///< Ponteiro para o estado (fun��o) corrente desta m�quina de estados.
    unsigned char entry;    ///< Quando TRUE(1) indica que est� executando pela primeira vez um estado, geralmente logo ap�s ter vindo de outro.
    uint32_t c;               ///< Vari�vel de cron�metro para esta m�quina de estados.
    void* param;            ///< Ponteiro para qualquer par�metro enviado para esta m�quina de estados.
}StateMachine;

//Defini��es
/**
 * @brief Declara um estado.
 * @details De fato, � a implementa��o de uma fun��o que recebe o nome 'name'.
 * @param name � um nome para o estado.
 * @param \_sm\_ � o ponteiro para a estrutura de um estado declarado como StateMachine.
 */
#define STATE(name)         void name(StateMachine *_sm_)

/**
 * @brief Indica que na pr�xima rodada de execu��o da m�quina de estados, o estado ser� o indicado por 'name'.
 * @details Atribui ao ponteiro ptr da estrutura \_sm\_ (StateMachine) o endere�o do pr�ximo estado (fun��o) a ser executado.
 * @param name � o nome do estado.
 */
#define NEXT_STATE(name)    _sm_->ptr = (pFuncao)name

/**
 * @brief Inicializa uma m�quina de estados.
 * @details Inicializa os par�metros de uma m�quina de estados. O valor de entry � inicializado com 1, pois ser� a primeira vez que o estado ser� evocado
 * @see StateMachine.entry
 * @param sm � o nome da vari�vel do tipo StateMachine que armazena os dados da m�quina de estados.
 * @param name � o nome do estado inicial desta m�quina de estados.
 * @param par � o ponteiro para qualquer estrutura que deseje ser passada para a m�quina de estados. A m�quina de estados n�o depende deste valor. Se n�o for usado, colocar NULL.
 */
#define INIT(sm,name,par)       {sm.ptr = (pFuncao)name;sm.entry=1;sm.param = par;}

/**
 * @brief Executa o c�digo (fun��o) do estado corrente.
 * @details Chama a execu��o do estado corrente. � dividido em tr�s etapas. A primeira, salva em uma vari�vel tempor�ria o estado atual. A segunda, executa a func�o associada ao estado atual. A terceira, avalia se esta execu��o gerou a altera��o do estado corrente e, se sim, ativa a vari�vel StateMachine.entry deste estado.
 * @param sm � o nome da vari�vel do tipo StateMachine que armazena os dados da m�quina de estados.
 */
#define EXEC(sm)            {pFuncao temp=sm.ptr;sm.ptr(&sm);sm.entry=(temp != sm.ptr);}

/**
 * @brief Sinalizador que indica se acabou de ocorrer uma mudan�a de estado.
 * @details Apenas retorna o valor da vari�vel StateMachine.entry desta m�quina de estados. Usada geralmente como uma express�o de teste em um if dentro de um estado, esta serve para realizar uma a��o apenas na primeira vez que um estado � chamado.
 */
#define JUST_ARRIVED        (_sm_->entry)

/**
 * @brief Sinalizador que indica se uma m�quina de estados est� atualmente em um espec�fico estado.
 * @details Retorna TRUE(1) se o estado corrente da m�quina de estados for igual ao par�metro 'name'. Apenas realiza um teste ente a vari�vel StateMachine.ptr e o nome do estado em teste.
*/
#define COMPARE(sm,name)    (sm.ptr == (pFuncao)name)

/**
 * @brief Dispara o cron�metro associado a esta m�quina de estados.
 * @details Chama a fun��o ChronoStart() passando como par�metros o endere�o da vari�vel que cont�m o cron�metro espec�fico desta m�quina de estados e o tempo deste cron�metro.
 * @param deltaT Intervalo de tempo para que o cron�metro seja conclu�do.
 * @see ChronoStart()
 */
#define START(deltaT)       ChronoStart(&_sm_->c, deltaT)

/**
 * @brief Avalia se o cron�metro associado a esta m�quina de estados j� foi conlu�do.
 * @details Chama a fun��o ChronoEnd() passando como par�metros o endere�o da vari�vel que cont�m o cron�metro espec�fico desta m�quina de estados.
 * @see ChronoEnd()
 */
#define IS_FINISHED         ChronoEnd(&_sm_->c)
/**
 * @brief Recupera o endere�o do par�metro passado em INIT().
 */
#define SM_PARAM            (_sm_->param)

#endif // SM_H
