#include "sm_corrente.h"


//#pragma DATA_SECTION (data_frame.current,".data_frame")

StateMachine sm_corrente;
uint32_t RXBytes_c;
uint8_t buffer_c[256];
uint16_t sendPhas = 0, CRC_retry_ = 0;
extern volatile uint8_t startCap;
extern volatile uint16_t delayF;
extern volatile uint8_t bufferFull;

extern volatile t_adc_results ADC_Results;
extern volatile t_temp_data Temp_Results;

extern CiseiRxChannel rx_Fibra1;
extern CiseiTxChannel tx_Fibra1;

extern float64 Xre1, Xim1;
extern float64 Xre2, Xim2;
extern float64 Xre3, Xim3;
extern float64 Xre4, Xim4;
extern float64 Xre5, Xim5;
extern float64 Xre6, Xim6;

extern struct{
    union{
        t_voltage_data_frame voltage;
        t_voltage_phasor_frame pVoltage;
    }voltage_u;
    union{
        t_current_data_frame current;
        t_current_phasor_frame pCurrent;
    }current_u;
} data_frame;

STATE(SM_CORRENTE_INIT){

    init_hal_C();
    CPLD_GPIO_Config();
    CPLD_RST_AD();

    NEXT_STATE(SM_CORRENTE_TEST);

}

STATE(SM_CORRENTE_TEST){

    CPLD_TestRAM();
    CPLD_Test_SPI_AD();

    NEXT_STATE(SM_CORRENTE_CFG);

}

STATE(SM_CORRENTE_CFG){

// Modificado (por Almeida)
#if defined (BOARD_NEW)
    init_tx_serial(&tx_Fibra1, tx_D_byte, 0);
#elif defined (BOARD_PREVIOUS)
    init_tx_serial(&tx_Fibra1, tx_A_byte, 0);
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    init_rx_serial(&rx_Fibra1, buffer_c, 256);

// Modificado (por Almeida)
#if defined (BOARD_NEW)
    tx_D_byte(0x01);
#elif defined (BOARD_PREVIOUS)
    tx_A_byte(0x01);
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    activateUART_Ints();

    CPLD_CFG_AD();

    data_frame.current_u.current.AD = &ADC_Results;

    NEXT_STATE(SM_CORRENTE_WAIT);
}

STATE(SM_CORRENTE_WAIT){
    GPIO_WritePin(54, 1);

    //DEBUG TEMP
//    rx_Fibra1.frameReceived = 1;
//    rx_Fibra1.rx_frame_type = TEMP_REQUEST;

    if(rx_frameReceived(&rx_Fibra1, &RXBytes_c)){
        if(rx_getFrameType(&rx_Fibra1) == SYNC_FRAME){
//            resetResultBuffer();
            //enable sync int
            NEXT_STATE(SM_CORRENTE_SYNC);
        }else if(rx_getFrameType(&rx_Fibra1) == DATA_REQUEST){
            if(rx_Fibra1.bytesReceived != 0)
                sendPhas = 1;
            else
                sendPhas = 0;
            NEXT_STATE(SM_CORRENTE_TX);
        }else if(rx_getFrameType(&rx_Fibra1) == SYNC_DELAY){
            NEXT_STATE(SM_CORRENTE_DELAY);
        }else if(rx_getFrameType(&rx_Fibra1) == TEMP_REQUEST){
            NEXT_STATE(SM_CORRENTE_TEMPERATURA);
        }
        rx_free_frame(&rx_Fibra1);
    }
}
uint16_t contFrame = 0, timeout = 0;

STATE(SM_CORRENTE_SYNC){
    if(JUST_ARRIVED){
        data_frame.current_u.pCurrent.A138_A =  -1;
        data_frame.current_u.pCurrent.B138_A =  -1;
        data_frame.current_u.pCurrent.C138_A =  -1;
        data_frame.current_u.pCurrent.A230_A =  -1;
        data_frame.current_u.pCurrent.B230_A =  -1;
        data_frame.current_u.pCurrent.C230_A =  -1;

        startCapture();
        GPIO_WritePin(54, 1);

// Modificado (por Almeida)
#if defined (BOARD_NEW)
    GpioDataRegs.GPBSET.bit.GPIO47 = 1;
#elif defined (BOARD_PREVIOUS)
    GpioDataRegs.GPBSET.bit.GPIO48 = 1;
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

        configureSCI_sync();
        GPIO_WritePin(CONVST, 0);
        START(100);
        contFrame++;
    }

// Modificado (por Almeida)
#if defined (BOARD_NEW)
    if(IS_FINISHED && (readPin(RXD) == 0) ){  // Novo - Placa Versao Nova
#elif defined (BOARD_PREVIOUS)
    if(IS_FINISHED && (readPin(RXA) == 0) ){  // Original - Placa Versao Anterior
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif
        startCap = 1;
        NEXT_STATE(SM_CORRENTE_CONV);
    }
}

STATE(SM_CORRENTE_CONV){
    if(JUST_ARRIVED){
        START(35000);
    }
    if(IS_FINISHED){
        GPIO_WritePin(54, 1);
        configureSCI_UART();
//        tx_A_byte(0xFF);
//        GPIO_WritePin(CONVST, 0);
        CPLD_WE(0);
        NEXT_STATE(SM_CORRENTE_FASOR);
    }
}

STATE(SM_CORRENTE_TX){
    if(JUST_ARRIVED){
        if(sendPhas)
            start_tx_frame(&tx_Fibra1, CURRENT_PHASOR_X, (uint8_t*)&data_frame.current_u, 2*sizeof(data_frame.current_u.pCurrent)); // Montar pacote de transferencia
        else
            start_tx_frame(&tx_Fibra1, CURRENT_DATA_X, (uint8_t*)&data_frame.current_u, 2*sizeof(data_frame.current_u.current)); // Montar pacote de transferencia
    }
    if(tx_end(&tx_Fibra1)){

        CPLD_WE(1);
        CPLD_Mode(MODE_RW);
        CPLD_Read_Write_SPI(0xFF);CPLD_Read_Write_SPI(0xFF);
        CPLD_Read_Write_SPI(0xFF);CPLD_Read_Write_SPI(0xFF);
        CPLD_Read_Write_SPI(0xFF);CPLD_Read_Write_SPI(0xFF);
        CPLD_WE(0);

        NEXT_STATE(SM_CORRENTE_WAIT);
        
    }
}

uint32_t t1, t2;

STATE(SM_CORRENTE_DELAY)
{
    if(JUST_ARRIVED){
        START(80);
//        clr_TXA();

// Modificado (por Almeida)
#if defined (BOARD_NEW)
    GpioDataRegs.GPBCLEAR.bit.GPIO47 = 1;
#elif defined (BOARD_PREVIOUS)
    GpioDataRegs.GPBCLEAR.bit.GPIO48 = 1;
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

        configureSCI_sync();
//        set_DEBUG1();
        delayF = 1;
    }

// Modificado (por Almeida)
#if defined (BOARD_NEW)
    if(IS_FINISHED || GpioDataRegs.GPBDAT.bit.GPIO47 == 1){     // Novo - Placa Versao Nova
#elif defined (BOARD_PREVIOUS)
    if(IS_FINISHED || GpioDataRegs.GPBDAT.bit.GPIO48 == 1){   // Original - Placa Versao Anterior
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif
//        clr_DEBUG1();
        configureSCI_UART();
        delayF = 0;
//        GPIO_WritePin(54, 0);
        NEXT_STATE(SM_CORRENTE_WAIT);
    }
}

//--------------------------------------------------------------------------------------
// Codigo de Teste: INICIO
//--------------------------------------------------------------------------------------
#define ADC_DATA_SAMPLES    (RESULTS_BUFFER_SIZE)
static uint16_t ad_data [ ADC_DATA_SAMPLES ];
//--------------------------------------------------------------------------------------
// Codigo de Teste: FIM
//--------------------------------------------------------------------------------------


STATE(SM_CORRENTE_FASOR){
    if(JUST_ARRIVED){
        GPIO_WritePin(54, 0);
        CRC_retry_ = 0;
    }

    crc16_init();

    Xre1 = Xim1 = 0.0;
    Xre2 = Xim2 = 0.0;
    Xre3 = Xim3 = 0.0;
    Xre4 = Xim4 = 0.0;
    Xre5 = Xim5 = 0.0;
    Xre6 = Xim6 = 0.0;

    // Modo de leitura/escrita da memoria RAM via CPLD
    CPLD_Mode(MODE_RW);

    Uint16 V_read[2],AD[6], canal = 0, byyte = 0;

    int ind = 0;

    //--------------------------------------------------------------------------------------
    // Codigo de Teste: INICIO
    //--------------------------------------------------------------------------------------
    for (ind = 0;  ind < ADC_DATA_SAMPLES; ind++)
        ad_data [ ind ] = 0;
    //--------------------------------------------------------------------------------------
    // Codigo de Teste: FIM
    //--------------------------------------------------------------------------------------


    for(ind = 0; ind < RESULTS_BUFFER_SIZE; ind++)
    {
        //--------------------------------------------------------------------------------------
        // Codigo de Teste: INICIO
        //--------------------------------------------------------------------------------------

        // 2 * pi * amostra
        float result = (ind * (2 * 3.1415f)) / RESULTS_BUFFER_SIZE;

        Uint16 sample = (sin (result) * 30000) + 32768;

        //--------------------------------------------------------------------------------------
        // Codigo de Teste: FIM
        //--------------------------------------------------------------------------------------

        for(canal = 0; canal < 6; canal++)
        {
            for(byyte = 0; byyte < 2; byyte++)
            {
                V_read[byyte] = CPLD_Read_Write_SPI(0x00);
                crc16_data(V_read[byyte]);
            }

            AD[canal] = V_read[0] + (V_read[1] << 8);

            //--------------------------------------------------------------------------------------
            // Codigo de Teste: INICIO
            //--------------------------------------------------------------------------------------
//            if (canal == 0) // Canal 0 da placa (primeiro AD) -> Esquerda para Direita
//            {
                //ad_data [ ind ] = AD [ canal ];

                //ad_data [ ind ] = sample;

//                AD[canal] = sample;
//            }
            //--------------------------------------------------------------------------------------
            // Codigo de Teste: FIM
            //--------------------------------------------------------------------------------------
        }


        float64 temp = cosl(2 * ind * K * M_PI / RESULTS_BUFFER_SIZE);
        Xre1 +=  AD[0] * temp;
        Xre2 +=  AD[1] * temp;
        Xre3 +=  AD[2] * temp;
        Xre4 +=  AD[3] * temp;
        Xre5 +=  AD[4] * temp;
        Xre6 +=  AD[5] * temp;

        temp = sinl(2 * ind * K * M_PI / RESULTS_BUFFER_SIZE);
        Xim1 += -(AD[0]) * temp;
        Xim2 += -(AD[1]) * temp;
        Xim3 += -(AD[2]) * temp;
        Xim4 += -(AD[3]) * temp;
        Xim5 += -(AD[4]) * temp;
        Xim6 += -(AD[5]) * temp;
    }

    Uint16 disp_timer[6], t_i, CRC_calc;
    for(t_i = 0; t_i < 6; t_i++){
        
        disp_timer[t_i] = CPLD_Read_Write_SPI(0x00);
        CRC_calc = crc16_data(disp_timer[t_i]);
        
    }

    data_frame.current_u.pCurrent.d_timer = disp_timer[0];
    data_frame.current_u.pCurrent.d_timer = (data_frame.current_u.pCurrent.d_timer << 8) | disp_timer[1];
    data_frame.current_u.pCurrent.d_timer = (data_frame.current_u.pCurrent.d_timer << 8) | disp_timer[2];
    data_frame.current_u.pCurrent.d_timer = (data_frame.current_u.pCurrent.d_timer << 8) | disp_timer[3];
    data_frame.current_u.pCurrent.d_timer = (data_frame.current_u.pCurrent.d_timer << 8) | disp_timer[4];
    data_frame.current_u.pCurrent.d_timer = (data_frame.current_u.pCurrent.d_timer << 8) | disp_timer[5];

    Uint16 CRC_CPLD_RAM;
    CRC_CPLD_RAM = CPLD_Read_Write_SPI(0x00);
    CRC_CPLD_RAM = CRC_CPLD_RAM + (CPLD_Read_Write_SPI(0x00) << 8);


    if(CRC_CPLD_RAM == CRC_calc){

        Xim1 = Xim1/RESULTS_BUFFER_SIZE;
        Xim2 = Xim2/RESULTS_BUFFER_SIZE;
        Xim3 = Xim3/RESULTS_BUFFER_SIZE;
        Xim4 = Xim4/RESULTS_BUFFER_SIZE;
        Xim5 = Xim5/RESULTS_BUFFER_SIZE;
        Xim6 = Xim6/RESULTS_BUFFER_SIZE;

        Xre1 = Xre1/RESULTS_BUFFER_SIZE;
        Xre2 = Xre2/RESULTS_BUFFER_SIZE;
        Xre3 = Xre3/RESULTS_BUFFER_SIZE;
        Xre4 = Xre4/RESULTS_BUFFER_SIZE;
        Xre5 = Xre5/RESULTS_BUFFER_SIZE;
        Xre6 = Xre6/RESULTS_BUFFER_SIZE;

        data_frame.current_u.pCurrent.A138_P = (float)(atan2l(Xim1, Xre1)*180)/M_PI;
        data_frame.current_u.pCurrent.B138_P = (float)(atan2l(Xim2, Xre2)*180)/M_PI;
        data_frame.current_u.pCurrent.C138_P = (float)(atan2l(Xim3, Xre3)*180)/M_PI;
        data_frame.current_u.pCurrent.A230_P = (float)(atan2l(Xim4, Xre4)*180)/M_PI;
        data_frame.current_u.pCurrent.B230_P = (float)(atan2l(Xim5, Xre5)*180)/M_PI;
        data_frame.current_u.pCurrent.C230_P = (float)(atan2l(Xim6, Xre6)*180)/M_PI;

        data_frame.current_u.pCurrent.A138_A =  (float)(sqrtl(powl(Xre1, 2.0) + powl(Xim1, 2.0)));
        data_frame.current_u.pCurrent.B138_A =  (float)(sqrtl(powl(Xre2, 2.0) + powl(Xim2, 2.0)));
        data_frame.current_u.pCurrent.C138_A =  (float)(sqrtl(powl(Xre3, 2.0) + powl(Xim3, 2.0)));
        data_frame.current_u.pCurrent.A230_A =  (float)(sqrtl(powl(Xre4, 2.0) + powl(Xim4, 2.0)));
        data_frame.current_u.pCurrent.B230_A =  (float)(sqrtl(powl(Xre5, 2.0) + powl(Xim5, 2.0)));
        data_frame.current_u.pCurrent.C230_A =  (float)(sqrtl(powl(Xre6, 2.0) + powl(Xim6, 2.0)));

        NEXT_STATE(SM_CORRENTE_WAIT);
    }else{
        CRC_retry_++;
        if(CRC_retry_ < 3)
            NEXT_STATE(SM_CORRENTE_FASOR);
        else{
            data_frame.current_u.pCurrent.A138_A =  -1;
            data_frame.current_u.pCurrent.B138_A =  -1;
            data_frame.current_u.pCurrent.C138_A =  -1;
            data_frame.current_u.pCurrent.A230_A =  -1;
            data_frame.current_u.pCurrent.B230_A =  -1;
            data_frame.current_u.pCurrent.C230_A =  -1;
            NEXT_STATE(SM_CORRENTE_WAIT);
        }

    }
}

STATE(SM_CORRENTE_TEMPERATURA){
    if(JUST_ARRIVED){
        bufferFull = 0;
        EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze counter
        EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
        EPwm1Regs.ETSEL.bit.SOCBEN = 1;  //enable SOCA
    }

    if(bufferFull == 1){
        const uint16_t meio = 49;

        quickSort((uint16_t*)Temp_Results.S420_1, 0, 99);
        quickSort((uint16_t*)Temp_Results.S420_2, 0, 99);

        quickSort((uint16_t*)Temp_Results.PTP_1,  0, 99);
        quickSort((uint16_t*)Temp_Results.PTN1_1, 0, 99);
        quickSort((uint16_t*)Temp_Results.PTN2_1, 0, 99);

        quickSort((uint16_t*)Temp_Results.PTP_2,  0, 99);
        quickSort((uint16_t*)Temp_Results.PTN1_2, 0, 99);
        quickSort((uint16_t*)Temp_Results.PTN2_2, 0, 99);


        data_frame.current_u.pCurrent.temperatura.PTC_1 = (float)tempNTC(100, Temp_Results.PTN2_1[meio], Temp_Results.PTP_1[meio]);
        data_frame.current_u.pCurrent.temperatura.PTC_2 = (float)tempNTC(100, Temp_Results.PTN2_2[meio], Temp_Results.PTP_2[meio]);

        data_frame.current_u.pCurrent.temperatura.S420_1 = (float)analogEq(Temp_Results.S420_1[meio], 12)/160.0;
        data_frame.current_u.pCurrent.temperatura.S420_2 = (float)analogEq(Temp_Results.S420_2[meio], 12)/160.0;

        NEXT_STATE(SM_CORRENTE_WAIT);
    }
}
