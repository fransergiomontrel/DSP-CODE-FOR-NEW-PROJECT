#include "chrono.h"

//Função para iniciar cronometro, deltaT em mS
void ChronoStart(chrono *c, uint32_t deltaT){

    resetTimer0();
    *c = now()+deltaT;
    
}

//Testa se ja se passaram deltaT uS
//Se sim, retorna true
boolean ChronoEnd(chrono *c){

    return(now() <= *c);
    
}
