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

#include "F28377S/pins.h"
#include "F28377S/sci_io.h"


/**
 * main.c
 */

int main(void)
{
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

    INIT(sm_tensao, SM_TENSAO_INIT, 0);

    while(1){
        EXEC(sm_tensao);
    }

	//return 0;
}
