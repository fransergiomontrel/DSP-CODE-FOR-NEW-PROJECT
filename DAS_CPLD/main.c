//INCLUDES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <file.h>
#include <string.h>

#include "F28x_Project.h"
#include "SFO_V8.h"

#include "common/datatypes.h"
#include "common/hal.h"
#include "common/sm_serial_api.h"
#include "common/sm_rx_serial_api.h"
#include "common/sm_tx_serial_api.h"
#include "common/sm_tensao.h"
#include "common/sm_corrente.h"

#include "F28377S/pins.h"
#include "F28377S/sci_io.h"
#include "F28377S/LCD_I2C.h"

/**
 * main.c
 */
#define CORRENTE

int main(void)
{
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif


#ifdef CORRENTE
    INIT(sm_corrente, SM_CORRENTE_INIT, 0);

    while(1){
        EXEC(sm_corrente);
    }
#endif
#ifdef TENSAO
    INIT(sm_tensao, SM_TENSAO_INIT, 0);

    while(1){
        EXEC(sm_tensao);
    }
#endif

	return 0;
}
