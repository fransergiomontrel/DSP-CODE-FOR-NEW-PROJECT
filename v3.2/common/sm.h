
#ifndef SM_H
#define SM_H

#include "chrono.h"
/**
 * @file
 * @brief Framework para implementação das máquinas de estado.
 * @details Este framework utiliza callback como poteiro para cada estado.
 * Cada estado é implementado como uma função, mascarada pelo define STATE().
 * @author Afonso Ferreira Miguel.
 */

/** Tipo de dado como ponteiro de função (callback).
 */
typedef void (*pFuncao)(void *);

//! Estrutura de dados responsável por armazenar as informações de cada máquina de estados.
typedef struct{
    pFuncao ptr;            ///< Ponteiro para o estado (função) corrente desta máquina de estados.
    unsigned char entry;    ///< Quando TRUE(1) indica que está executando pela primeira vez um estado, geralmente logo após ter vindo de outro.
    chrono c;               ///< Variável de cronômetro para esta máquina de estados.
    void* param;            ///< Ponteiro para qualquer parâmetro enviado para esta máquina de estados.
}StateMachine;

//Definições
/**
 * @brief Declara um estado.
 * @details De fato, é a implementação de uma função que recebe o nome 'name'.
 * @param name é um nome para o estado.
 * @param \_sm\_ é o ponteiro para a estrutura de um estado declarado como StateMachine.
 */
#define STATE(name)         void name(StateMachine *_sm_)

/**
 * @brief Indica que na próxima rodada de execução da máquina de estados, o estado será o indicado por 'name'.
 * @details Atribui ao ponteiro ptr da estrutura \_sm\_ (StateMachine) o endereço do próximo estado (função) a ser executado.
 * @param name é o nome do estado.
 */
#define NEXT_STATE(name)    _sm_->ptr = (pFuncao)name

/**
 * @brief Inicializa uma máquina de estados.
 * @details Inicializa os parâmetros de uma máquina de estados. O valor de entry é inicializado com 1, pois será a primeira vez que o estado será evocado
 * @see StateMachine.entry
 * @param sm é o nome da variável do tipo StateMachine que armazena os dados da máquina de estados.
 * @param name é o nome do estado inicial desta máquina de estados.
 * @param par é o ponteiro para qualquer estrutura que deseje ser passada para a máquina de estados. A máquina de estados não depende deste valor. Se não for usado, colocar NULL.
 */
#define INIT(sm,name,par)       {sm.ptr = (pFuncao)name;sm.entry=1;sm.param = par;}

/**
 * @brief Executa o código (função) do estado corrente.
 * @details Chama a execução do estado corrente. É dividido em três etapas. A primeira, salva em uma variável temporária o estado atual. A segunda, executa a funcão associada ao estado atual. A terceira, avalia se esta execução gerou a alteração do estado corrente e, se sim, ativa a variável StateMachine.entry deste estado.
 * @param sm é o nome da variável do tipo StateMachine que armazena os dados da máquina de estados.
 */
#define EXEC(sm)            {pFuncao temp=sm.ptr;sm.ptr(&sm);sm.entry=(temp != sm.ptr);}

/**
 * @brief Sinalizador que indica se acabou de ocorrer uma mudança de estado.
 * @details Apenas retorna o valor da variável StateMachine.entry desta máquina de estados. Usada geralmente como uma expressão de teste em um if dentro de um estado, esta serve para realizar uma ação apenas na primeira vez que um estado é chamado.
 */
#define JUST_ARRIVED        (_sm_->entry)

/**
 * @brief Sinalizador que indica se uma máquina de estados está atualmente em um específico estado.
 * @details Retorna TRUE(1) se o estado corrente da máquina de estados for igual ao parâmetro 'name'. Apenas realiza um teste ente a variável StateMachine.ptr e o nome do estado em teste.
*/
#define COMPARE(sm,name)    (sm.ptr == (pFuncao)name)

/**
 * @brief Dispara o cronômetro associado a esta máquina de estados.
 * @details Chama a função ChronoStart() passando como parâmetros o endereço da variável que contém o cronômetro específico desta máquina de estados e o tempo deste cronômetro.
 * @param deltaT Intervalo de tempo para que o cronômetro seja concluído.
 * @see ChronoStart()
 */
#define START(deltaT)       ChronoStart(&_sm_->c, deltaT)

/**
 * @brief Avalia se o cronômetro associado a esta máquina de estados já foi conluído.
 * @details Chama a função ChronoEnd() passando como parâmetros o endereço da variável que contém o cronômetro específico desta máquina de estados.
 * @see ChronoEnd()
 */
#define IS_FINISHED         ChronoEnd(&_sm_->c)
/**
 * @brief Recupera o endereço do parâmetro passado em INIT().
 */
#define SM_PARAM            (_sm_->param)

#endif // SM_H
