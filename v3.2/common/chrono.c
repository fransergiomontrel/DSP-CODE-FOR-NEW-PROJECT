#include "chrono.h"

//Função para iniciar cronometro, deltaT em mS
void ChronoStart(chrono *c, uint32_t deltaT){
    #ifdef __TMS320C28X__
    *c = now() - deltaT;
    #else
    *c = now()+deltaT;
    #endif
}

//Testa se ja se passaram deltaT uS
//Se sim, retorna true
boolean ChronoEnd(chrono *c){
    return(now() <= *c);
}
