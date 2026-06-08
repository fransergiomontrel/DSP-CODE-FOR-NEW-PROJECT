#include "sm_tensao.h"

#define contador 0xFFFFFFFF

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

StateMachine sm_tensao;

struct {
    int16_t day;
    int16_t month;
    int16_t year;
    int16_t hour;
    int16_t minute;
    int16_t second;
    int16_t msecond;
} time_frame;

struct{
    union{
        t_voltage_data_frame voltage;
        t_voltage_phasor_frame pVoltage;
    }voltage_u;
    union{
        t_current_data_frame current;
        t_current_phasor_frame pCurrent;
    }current_u;
} data_frame;

chrono chrono1;
uint32_t RX_Bytes, RX_USB_Bytes;
uint8_t buffer_USB[256], buffer[256];
int64_t timeBuffer;
uint32_t T1, T2, T3, T4;
uint16_t syncTimes = 0, sendPhasor = 0, CRC_retry = 0, sec_transformer = 0;
int32_t delay_dT1 = 0, delay_dT2 = 0, delay_dT3 = 0;
uint32_t delay__T1, delay__T2, delay__T3;
uint32_t ordered_list_T[3];
char ordered_list_Tname [3] = {'1', '2', '3'};

long double Xre1 = 0.0, Xim1 = 0.0;
long double Xre2 = 0.0, Xim2 = 0.0;
long double Xre3 = 0.0, Xim3 = 0.0;
long double Xre4 = 0.0, Xim4 = 0.0;
long double Xre5 = 0.0, Xim5 = 0.0;
long double Xre6 = 0.0, Xim6 = 0.0;

t_sync_frame syncPayload;

extern volatile uint32_t delay_T1, delay_T2, delay_T3;
extern volatile uint16_t timer_end;
extern volatile uint8_t bufferFull;

extern uint32_t delay_v_T1[syncMax];
extern uint32_t delay_v_T2[syncMax];
extern uint32_t delay_v_T3[syncMax];

extern CiseiRxChannel rx_USB;
extern CiseiTxChannel tx_USB;
extern CiseiRxChannel rx_Fibra1;
extern CiseiTxChannel tx_Fibra1;
extern CiseiRxChannel rx_Fibra2;
extern CiseiTxChannel tx_Fibra2;
extern CiseiRxChannel rx_Fibra3;
extern CiseiTxChannel tx_Fibra3;

extern volatile t_adc_results ADC_Results;

STATE(SM_TENSAO_INIT){
    init_hal();
    CPLD_GPIO_Config();
    CPLD_RST_AD();

    NEXT_STATE(SM_TENSAO_TEST);
}

STATE(SM_TENSAO_TEST){
    CPLD_TestRAM();
    CPLD_Test_SPI_AD();

    NEXT_STATE(SM_TENSAO_CFG);
}

STATE(SM_TENSAO_CFG){

#if defined (BOARD_NEW)
    init_tx_serial(&tx_Fibra1, tx_D_byte, 0);
    #if defined (TENSAO)
        init_tx_serial(&tx_USB, tx_A_byte, 0);
    #else
        init_tx_serial(&tx_USB, tx_B_byte, 1);
    #endif
#elif defined (BOARD_PREVIOUS)
    init_tx_serial(&tx_Fibra1, tx_A_byte, 0);
    init_tx_serial(&tx_USB, tx_B_byte, 1);
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif
    init_tx_serial(&tx_Fibra2, tx_C_byte, 0);
    init_tx_serial(&tx_Fibra3, tx_B_byte, 0);

    init_rx_serial(&rx_Fibra1, (uint8_t*)&data_frame.current_u, 2*sizeof(data_frame.current_u));
    init_rx_serial(&rx_Fibra2, (uint8_t*)&data_frame.current_u, 2*sizeof(data_frame.current_u));
    init_rx_serial(&rx_Fibra3, (uint8_t*)&data_frame.current_u, 2*sizeof(data_frame.current_u));

    init_rx_serial(&rx_USB, buffer_USB, sizeof(buffer_USB));

#if defined (BOARD_NEW)
    tx_D_byte(0x01);
    #if defined (TENSAO)
        tx_A_byte(0x01);
    #else
        tx_B_byte(0x01);
    #endif
#elif defined (BOARD_PREVIOUS)
        tx_A_byte(0x01);
        tx_B_byte(0x01);
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    tx_C_byte(0x01);
    tx_B_byte(0x01);

    activateUART_Ints();
    CPLD_CFG_AD();

    data_frame.voltage_u.voltage.AD = &ADC_Results;
    NEXT_STATE(SM_TENSAO_WAIT);
}

STATE(SM_TENSAO_WAIT){
    if(rx_frameReceived(&rx_USB, &RX_USB_Bytes)){
        if(rx_getFrameType(&rx_USB) == CLOCK_FRAME){
            memcpy(&time_frame, rx_USB.pBuffer, sizeof(time_frame));
            data_frame.voltage_u.voltage.day     = time_frame.day;
            data_frame.voltage_u.voltage.month   = time_frame.month;
            data_frame.voltage_u.voltage.year    = time_frame.year;
            data_frame.voltage_u.voltage.hour    = time_frame.hour;
            data_frame.voltage_u.voltage.minute  = time_frame.minute;
            data_frame.voltage_u.voltage.second  = time_frame.second;
            data_frame.voltage_u.voltage.msecond = time_frame.msecond;
            data_frame.voltage_u.voltage.padding = 0x1234;
            data_frame.voltage_u.voltage.timer1 = data_frame.voltage_u.voltage.timer2 = 0;
//            rx_USB_free_frame(&rx_USB);
            resetTimer0();
            NEXT_STATE(SM_TENSAO_WAIT);
        }else if(rx_getFrameType(&rx_USB) == SYNC_FRAME){
            memcpy(&syncPayload, rx_USB.pBuffer, sizeof(t_sync_frame));
            sendPhasor = syncPayload.phasor;
            sec_transformer = syncPayload.transformers;
            resetResultBuffer();
#ifdef MULTI
            NEXT_STATE(SM_TENSAO_DELAY);
#endif
#ifdef SINGLE
            NEXT_STATE(SM_TENSAO_CONV);
#endif
        }else if(rx_getFrameType(&rx_USB) == DATA_REQUEST_V){
            memcpy(&syncPayload, rx_USB.pBuffer, sizeof(t_sync_frame));
            data_frame.voltage_u.voltage.time = syncPayload;
            NEXT_STATE(SM_TENSAO_TX);
        }
#ifdef MULTI
        else if(rx_getFrameType(&rx_USB) == DATA_REQUEST_I1){
            GPIO_WritePin(54, 0);
            memcpy(&syncPayload, rx_USB.pBuffer, sizeof(t_sync_frame));
            data_frame.current_u.current.timeC1 = syncPayload;
            NEXT_STATE(SM_TENSAO_REQ_I1);
        }else if(rx_getFrameType(&rx_USB) == DATA_REQUEST_I2){
            memcpy(&syncPayload, rx_USB.pBuffer, sizeof(t_sync_frame));
            NEXT_STATE(SM_TENSAO_REQ_I2);
        }
#endif
        rx_free_frame(&rx_USB);
    }
}

STATE(SM_TENSAO_DELAY){
    if(JUST_ARRIVED){
        delay_dT1 = delay_dT2 = syncTimes = timer_end = CpuTimer1.InterruptCount = 0;
        CpuTimer1Regs.PRD.all = 200000; //CPU Timer Period Register
        CpuTimer1Regs.TCR.bit.TRB = 1;
//        set_DEBUG1();
    }
    if(timer_end){
        timer_end = 0;
        if(sec_transformer == 0){
            if(delay_T1 < 200000){

                delay__T1 =  200000 - delay_T1;
                delay_dT1 += delay__T1;
                delay_v_T1[syncTimes] = delay__T1;
                syncTimes++;

            }
        }
            else if (sec_transformer == 1){
                if((delay_T1 < 200000) && (delay_T2 < 200000)){

                    delay__T1 =  200000 - delay_T1;
                    delay__T2 =  200000 - delay_T2;
                    delay_dT1 += delay__T1;
                    delay_dT2 += delay__T2;
                    delay_v_T1[syncTimes] = delay__T1;
                    delay_v_T2[syncTimes] = delay__T2;
                    syncTimes++;

                }
        }
            else if (sec_transformer == 2){
                if((delay_T1 < 200000) && (delay_T2 < 200000) && (delay_T3 < 200000)){

                    delay__T1 =  200000 - delay_T1;
                    delay__T2 =  200000 - delay_T2;
                    delay__T3 =  200000 - delay_T3;
                    delay_dT1 += delay__T1;
                    delay_dT2 += delay__T2;
                    delay_dT3 += delay__T3;
                    delay_v_T1[syncTimes] = delay__T1;
                    delay_v_T2[syncTimes] = delay__T2;
                    delay_v_T3[syncTimes] = delay__T3;
                    syncTimes++;

                }
        }


        if(syncTimes == syncMax){
            int o = 0;
            data_frame.voltage_u.voltage.sync_data.T1_Max = data_frame.voltage_u.voltage.sync_data.T2_Max = data_frame.voltage_u.voltage.sync_data.T3_Max = 0;
            data_frame.voltage_u.voltage.sync_data.T1_Min = data_frame.voltage_u.voltage.sync_data.T2_Min = data_frame.voltage_u.voltage.sync_data.T3_Min = 9999999;
            uint32_t dp1 = 0, dp2 = 0, dp3 = 0;
            data_frame.voltage_u.voltage.sync_data.T1 = (delay_dT1/(syncMax*2));//*(5E-9))/(1E-6);
            if(sec_transformer == 1)
                data_frame.voltage_u.voltage.sync_data.T2 = (delay_dT2/(syncMax*2));//*(5E-9))/(1E-6);
            else if(sec_transformer == 2)
                data_frame.voltage_u.voltage.sync_data.T2 = (delay_dT2/(syncMax*2));//*(5E-9))/(1E-6);
                data_frame.voltage_u.voltage.sync_data.T3 = (delay_dT3/(syncMax*2));//*(5E-9))/(1E-6);

                ordered_list_T[0] = data_frame.voltage_u.voltage.sync_data.T1;
                ordered_list_T[1] = data_frame.voltage_u.voltage.sync_data.T2;
                ordered_list_T[2] = data_frame.voltage_u.voltage.sync_data.T3;

            for(o = 0; o < syncMax; o++){
                uint32_t val1 = delay_v_T1[o];
                data_frame.voltage_u.voltage.sync_data.T1_Max = MAX(data_frame.voltage_u.voltage.sync_data.T1_Max,val1);
                data_frame.voltage_u.voltage.sync_data.T1_Min = MIN(data_frame.voltage_u.voltage.sync_data.T1_Min,val1);
                dp1 += pow(val1 - data_frame.voltage_u.voltage.sync_data.T1, 2);

                if(sec_transformer == 1){
                    uint32_t val2 = delay_v_T2[o];
                    data_frame.voltage_u.voltage.sync_data.T2_Max = MAX(data_frame.voltage_u.voltage.sync_data.T2_Max,val2);
                    data_frame.voltage_u.voltage.sync_data.T2_Min = MIN(data_frame.voltage_u.voltage.sync_data.T2_Min,val2);
                    dp2 += pow(val2 - data_frame.voltage_u.voltage.sync_data.T2, 2);
                }

                else if(sec_transformer == 2){
                    uint32_t val2 = delay_v_T2[o];
                    data_frame.voltage_u.voltage.sync_data.T2_Max = MAX(data_frame.voltage_u.voltage.sync_data.T2_Max,val2);
                    data_frame.voltage_u.voltage.sync_data.T2_Min = MIN(data_frame.voltage_u.voltage.sync_data.T2_Min,val2);
                    dp2 += pow(val2 - data_frame.voltage_u.voltage.sync_data.T2, 2);

                    uint32_t val3 = delay_v_T3[o];
                    data_frame.voltage_u.voltage.sync_data.T3_Max = MAX(data_frame.voltage_u.voltage.sync_data.T3_Max,val3);
                    data_frame.voltage_u.voltage.sync_data.T3_Min = MIN(data_frame.voltage_u.voltage.sync_data.T3_Min,val3);
                    dp3 += pow(val2 - data_frame.voltage_u.voltage.sync_data.T3, 2);

                }

            }

            data_frame.voltage_u.voltage.sync_data.DP1 = sqrtl(dp1/syncMax);
            if(sec_transformer == 1)
                data_frame.voltage_u.voltage.sync_data.DP2 = sqrtl(dp2/syncMax);
            else if(sec_transformer == 2)
                data_frame.voltage_u.voltage.sync_data.DP2 = sqrtl(dp2/syncMax);
                data_frame.voltage_u.voltage.sync_data.DP3 = sqrtl(dp3/syncMax);

            NEXT_STATE(SM_TENSAO_TX_SYNC);
        }
    }
    if(syncTimes < syncMax){
        CpuTimer1Regs.TCR.bit.TRB = 1;
        if(sec_transformer == 0){

            start_tx_frame(&tx_Fibra1, SYNC_DELAY, 0x0, 0x0);
            while(!tx_end(&tx_Fibra1));

        }else if(sec_transformer == 1) {
            
             start_tx_frame(&tx_Fibra1, SYNC_DELAY, 0x0, 0x0);
             start_tx_frame(&tx_Fibra2, SYNC_DELAY, 0x0, 0x0);
             while(!(tx_end(&tx_Fibra1) && tx_end(&tx_Fibra2)));

        }else if(sec_transformer == 2) {
            
             start_tx_frame(&tx_Fibra1, SYNC_DELAY, 0x0, 0x0);
             start_tx_frame(&tx_Fibra2, SYNC_DELAY, 0x0, 0x0);
             start_tx_frame(&tx_Fibra3, SYNC_DELAY, 0x0, 0x0);
             while(!(tx_end(&tx_Fibra1) && tx_end(&tx_Fibra2) && tx_end(&tx_Fibra3)));

        }

        START(100); //100 valor antigo
        while(!IS_FINISHED);

#if defined (BOARD_NEW)

            GpioDataRegs.GPBCLEAR.bit.GPIO47 = 1; // TX.D
            GpioDataRegs.GPCCLEAR.bit.GPIO89 = 1; // TX.C
            GpioDataRegs.GPACLEAR.bit.GPIO22 = 1; // TX.B

#elif defined (BOARD_PREVIOUS)
        GpioDataRegs.GPBCLEAR.all = 0x00410040;   //bit 16 e 6 (48 e 38)
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif
        configureSCI_sync();

        START(25);
        while(!IS_FINISHED);

        delay_T1 = delay_T2 = (uint32_t)-1;

#if defined (BOARD_NEW)

            GpioDataRegs.GPBSET.bit.GPIO47 = 1; // TX.D
            GpioDataRegs.GPCSET.bit.GPIO89 = 1; // TX.C
            GpioDataRegs.GPASET.bit.GPIO22 = 1; // TX.B

#elif defined (BOARD_PREVIOUS)
        GpioDataRegs.GPBSET.all = 0x00410040;   //bit 16 e 6
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif
        CpuTimer1Regs.TCR.bit.TSS = 0;

        START(1100);
        set_LED3();
        while(!IS_FINISHED);
        clr_LED3();

        configureSCI_UART();
    }
}

STATE(SM_TENSAO_TX_SYNC){
    if(JUST_ARRIVED){
        //Send sync to current module
        if(sec_transformer == 0){

            start_tx_frame(&tx_Fibra1, SYNC_FRAME, 0x0, 0x0);
            
        }else if(sec_transformer == 1){

             start_tx_frame(&tx_Fibra1, SYNC_FRAME, 0x0, 0x0);
             start_tx_frame(&tx_Fibra2, SYNC_FRAME, 0x0, 0x0);

        }else if(sec_transformer == 2)
             start_tx_frame(&tx_Fibra1, SYNC_FRAME, 0x0, 0x0);
             start_tx_frame(&tx_Fibra2, SYNC_FRAME, 0x0, 0x0);
             start_tx_frame(&tx_Fibra3, SYNC_FRAME, 0x0, 0x0);


        data_frame.voltage_u.pVoltage.A138_A =  -1;
        data_frame.voltage_u.pVoltage.B138_A =  -1;
        data_frame.voltage_u.pVoltage.C138_A =  -1;
        data_frame.voltage_u.pVoltage.A230_A =  -1;
        data_frame.voltage_u.pVoltage.B230_A =  -1;
        data_frame.voltage_u.pVoltage.C230_A =  -1;

        //wait
        START(1000);
    }
    if(IS_FINISHED){
        //Configure pins


        set_TXA();
        set_TXD();
        set_TXC();
        set_TXB();

        configureSCI_sync();
        NEXT_STATE(SM_TENSAO_SYNC);
    }
}

uint32_t i1, i2, i3;

STATE(SM_TENSAO_SYNC){
    startCapture();
    START(100);
    while(!IS_FINISHED);

#if defined (BOARD_NEW)

        GpioDataRegs.GPBCLEAR.bit.GPIO47 = 1; // TX.D
        GpioDataRegs.GPCCLEAR.bit.GPIO89 = 1; // TX.C
        GpioDataRegs.GPACLEAR.bit.GPIO22 = 1; // TX.B

#elif defined (BOARD_PREVIOUS)
    GpioDataRegs.GPBCLEAR.all = 0x00010040;  //bit 16 e 6
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    START(500);
    while(!IS_FINISHED);

    DINT;
    if(sec_transformer == 0)
    {
        
        i2 = contador - data_frame.voltage_u.voltage.sync_data.T1;
        CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
        CpuTimer2Regs.TCR.bit.TRB = 1;
        CpuTimer2Regs.TCR.bit.TSS = 0;
        GpioDataRegs.GPBSET.bit.GPIO47 = 1;
        while(CpuTimer2Regs.TIM.all >= i2);

    }
    else if(sec_transformer == 1)
    {
       
        if(data_frame.voltage_u.voltage.sync_data.T1 > data_frame.voltage_u.voltage.sync_data.T2){
            i1 = contador - (data_frame.voltage_u.voltage.sync_data.T1 - data_frame.voltage_u.voltage.sync_data.T2);
            i2 = contador - data_frame.voltage_u.voltage.sync_data.T1;
            CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
            CpuTimer2Regs.TCR.bit.TRB = 1;
            CpuTimer2Regs.TCR.bit.TSS = 0;
    #if defined (BOARD_NEW)
            GpioDataRegs.GPCSET.bit.GPIO89 = 1; // TX.C
            while(CpuTimer2Regs.TIM.all >= i1);
            GpioDataRegs.GPBSET.bit.GPIO47 = 1; // TX.D
    #elif defined (BOARD_PREVIOUS)
            GpioDataRegs.GPBSET.all = 0x00010000;  //bit 16 (48 -> Serial C)
            while(CpuTimer2Regs.TIM.all >= i1);
            GpioDataRegs.GPBSET.all = 0x00000040;  //bit 6 (38 -> Serial A)
            while(CpuTimer2Regs.TIM.all >= i2);
    #else
        #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
    #endif
            while(CpuTimer2Regs.TIM.all >= i2);
        }else{
            i1 = contador - (data_frame.voltage_u.voltage.sync_data.T2 - data_frame.voltage_u.voltage.sync_data.T1);
            i2 = contador - data_frame.voltage_u.voltage.sync_data.T1;
            CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
            CpuTimer2Regs.TCR.bit.TRB = 1;

            CpuTimer2Regs.TCR.bit.TSS = 0;
    #if defined (BOARD_NEW)
            GpioDataRegs.GPBSET.bit.GPIO47 = 1; // TX.D
            while(CpuTimer2Regs.TIM.all >= i1);
            GpioDataRegs.GPCSET.bit.GPIO89 = 1; // TX.C
    #elif defined (BOARD_PREVIOUS)
            GpioDataRegs.GPBSET.all = 0x00000040;  //bit 6 (38 -> Serial A.TX)
            while(CpuTimer2Regs.TIM.all >= i1);
            GpioDataRegs.GPBSET.all = 0x00010000;  //bit 16 (48 -> Serial C.TX)
    #else
        #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
    #endif
            while(CpuTimer2Regs.TIM.all >= i2);

    }
    }
    else if(sec_transformer == 2)
    {
       
        uint32_t temp;
        char temp_name;
        uint16_t i, j;

        for (i = 0; i < 2; i++) {
            for (j = i + 1; j < 3; j++) {
                if (ordered_list_T[i] > ordered_list_T[j])
                {

                    temp = ordered_list_T[i];
                    temp_name = ordered_list_Tname[i];

                    ordered_list_T[i] = ordered_list_T[j];
                    ordered_list_Tname[i] = ordered_list_Tname[j];

                    ordered_list_T[j] = temp;
                    ordered_list_Tname[j] = temp_name;

                }
            }
        }
        
        i1 = contador - (ordered_list_T[1] - ordered_list_T[0]);
        i2 = contador - (ordered_list_T[2] - ordered_list_T[1]);
        i3 = contador - ordered_list_T[2];

        //Check which among 3 slaver IED has first bigger time propagation
        switch (ordered_list_Tname[2])
        {
            case '1':

                CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
                CpuTimer2Regs.TCR.bit.TRB = 1;
                CpuTimer2Regs.TCR.bit.TSS = 0;
                GpioDataRegs.GPBSET.bit.GPIO47 = 1; // TX.D
                while(CpuTimer2Regs.TIM.all >= i2);
                //Check which among 2 slaver IED has first bigger time propagation
                switch (ordered_list_Tname[1])
                {
                    case '2':
                        GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                        while(CpuTimer2Regs.TIM.all >= i1);
                        GpioDataRegs.GPASET.bit.GPIO22 = 1; //TX.B
                    break;
                    case '3':
                        GpioDataRegs.GPASET.bit.GPIO22 = 1; //TX.B
                        while(CpuTimer2Regs.TIM.all >= i1);
                        GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                    break;
                }
                
                break;

            case '2':
                
                CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
                CpuTimer2Regs.TCR.bit.TRB = 1;
                CpuTimer2Regs.TCR.bit.TSS = 0;
                GpioDataRegs.GPCSET.bit.GPIO89 = 1; // TX.C
                while(CpuTimer2Regs.TIM.all >= i2);
                //Check which among 2 slaver IED has first bigger time propagation
                switch (ordered_list_Tname[1])
                {
                    case '1':
                        GpioDataRegs.GPBSET.bit.GPIO47 = 1; //TX.D
                        while(CpuTimer2Regs.TIM.all >= i1);
                        GpioDataRegs.GPASET.bit.GPIO22 = 1; //TX.B
                    break;
                    case '3':
                        GpioDataRegs.GPASET.bit.GPIO22 = 1; //TX.B
                        while(CpuTimer2Regs.TIM.all >= i1);
                        GpioDataRegs.GPBSET.bit.GPIO47 = 1; //TX.D
                    break;
                }

            break;

            case '3':
                
                CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
                CpuTimer2Regs.TCR.bit.TRB = 1;
                CpuTimer2Regs.TCR.bit.TSS = 0;
                GpioDataRegs.GPASET.bit.GPIO22 = 1; //TX.B
                while(CpuTimer2Regs.TIM.all >= i2);
                //Check which among 2 slaver IED has first bigger time propagation
                switch (ordered_list_Tname[1])
                {
                    case '1':
                        GpioDataRegs.GPBSET.bit.GPIO47 = 1; //TX.D
                        while(CpuTimer2Regs.TIM.all >= i1);
                        GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                    break;
                    case '2':
                        GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                        while(CpuTimer2Regs.TIM.all >= i1);
                        GpioDataRegs.GPBSET.bit.GPIO47 = 1; //TX.D
                    break;
                }


            break;
            
        } 

        while(CpuTimer2Regs.TIM.all >= i3);
    
    }

    //start capture
//    startCapture();
    __asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");
    __asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");
    __asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");
    __asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");
    __asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");

    GPIO_WritePin(CONVST, 1);
    acquisition_counter++;
    CpuTimer2Regs.TCR.bit.TSS = 1;


    EINT;

    configureSCI_UART();

    NEXT_STATE(SM_TENSAO_CONV);
}

STATE(SM_TENSAO_CONV){
    if(JUST_ARRIVED){
#ifdef SINGLE
        startCapture();
        GPIO_WritePin(CONVST, 1);
        acquisition_counter++;
#endif
        START(35000);
    }
    if(IS_FINISHED){
        GPIO_WritePin(CONVST, 0);
        CPLD_WE(0);
        NEXT_STATE(SM_TENSAO_CALC_FAS);
    }
}

STATE(SM_TENSAO_TX){
        if(sendPhasor)
        {
            data_frame.voltage_u.pVoltage.acquisition_counter = acquisition_counter;
            start_tx_frame(&tx_USB, VOLTAGE_PHASOR, (uint8_t*)&data_frame.voltage_u.pVoltage, 2*sizeof(data_frame.voltage_u.pVoltage)); // Montar pacote de transferencia
        }
        else
            start_tx_frame(&tx_USB, VOLTAGE_DATA, (uint8_t*)&data_frame.voltage_u.voltage, 2*sizeof(data_frame.voltage_u.voltage)); // Montar pacote de transferencia

        NEXT_STATE(SM_TENSAO_WAIT_TX);
//    }
}
STATE(SM_TENSAO_WAIT_TX){
    if(JUST_ARRIVED)
        START(9000000);
    if(tx_end(&tx_USB) || IS_FINISHED){
        CPLD_WE(1);
        CPLD_Mode(MODE_RW);
        CPLD_Read_Write_SPI(0xFF);CPLD_Read_Write_SPI(0xFF);
        CPLD_Read_Write_SPI(0xFF);CPLD_Read_Write_SPI(0xFF);
        CPLD_Read_Write_SPI(0xFF);CPLD_Read_Write_SPI(0xFF);
        CPLD_WE(0);

        NEXT_STATE(SM_TENSAO_WAIT);
    }

}

STATE(SM_TENSAO_REQ_I1){
    //Wait to recieve data via fiber
    if(JUST_ARRIVED){
        START(9000000);
        if(sendPhasor)
            start_tx_frame(&tx_Fibra1, DATA_REQUEST, (uint8_t*)&sendPhasor, 2*sizeof(sendPhasor));
        else
            start_tx_frame(&tx_Fibra1, DATA_REQUEST, 0x0, 0x0);
    }if(IS_FINISHED){
        rx_free_frame(&rx_Fibra1);
        NEXT_STATE(SM_TENSAO_WAIT);
    }
    if(rx_frameReceived(&rx_Fibra1, &RX_Bytes)){

        if(rx_getFrameType(&rx_Fibra1) == CURRENT_DATA_X || rx_getFrameType(&rx_Fibra1) == CURRENT_PHASOR_X){
            GPIO_WritePin(54, 1);
            NEXT_STATE(SM_TENSAO_SEND_I1);
        }else
            NEXT_STATE(SM_TENSAO_WAIT);
    }
}

STATE(SM_TENSAO_SEND_I1){
    if(JUST_ARRIVED){
        //Send data via USB
        data_frame.current_u.current.timeC1 = syncPayload;
        if(rx_getFrameType(&rx_Fibra1) == CURRENT_PHASOR_X)
        {
            data_frame.current_u.pCurrent.acquisition_counter = acquisition_counter;
            start_tx_frame(&tx_USB, CURRENT_PHASOR_1, (uint8_t*)&data_frame.current_u, 2*sizeof(data_frame.current_u.pCurrent)); // Montar pacote de transferencia
        }
        else
            start_tx_frame(&tx_USB, CURRENT_DATA_1, (uint8_t*)&data_frame.current_u, 2*sizeof(data_frame.current_u.current)); // Montar pacote de transferencia
    }
    if(tx_end(&tx_USB)){
        DELAY_US(10000);
        rx_free_frame(&rx_Fibra1);
        NEXT_STATE(SM_TENSAO_WAIT);
    }
}

STATE(SM_TENSAO_REQ_I2){
    //Wait to recieve data via fiber
    if(JUST_ARRIVED){
        START(9000000);
        if(sendPhasor)
            start_tx_frame(&tx_Fibra2, DATA_REQUEST, (uint8_t*)&sendPhasor, 2*sizeof(sendPhasor));
        else
            start_tx_frame(&tx_Fibra2, DATA_REQUEST, 0x0, 0x0);
    }if(IS_FINISHED){
        rx_free_frame(&rx_Fibra2);
        NEXT_STATE(SM_TENSAO_WAIT);
    }
    if(rx_frameReceived(&rx_Fibra2, &RX_Bytes)){
        if(rx_getFrameType(&rx_Fibra2) == CURRENT_DATA_X || rx_getFrameType(&rx_Fibra2) == CURRENT_PHASOR_X)
            NEXT_STATE(SM_TENSAO_SEND_I2);
        else
            NEXT_STATE(SM_TENSAO_WAIT);
    }
}

STATE(SM_TENSAO_SEND_I2){
    if(JUST_ARRIVED){
        //Send data via USB
        data_frame.current_u.current.timeC1 = syncPayload;
        if(rx_getFrameType(&rx_Fibra2) == CURRENT_PHASOR_X)
        {
            data_frame.current_u.pCurrent.acquisition_counter = acquisition_counter;
            start_tx_frame(&tx_USB, CURRENT_PHASOR_2, (uint8_t*)&data_frame.current_u, 2*sizeof(data_frame.current_u.pCurrent)); // Montar pacote de transferencia
        }
        else
            start_tx_frame(&tx_USB, CURRENT_DATA_2, (uint8_t*)&data_frame.current_u, 2*sizeof(data_frame.current_u.current)); // Montar pacote de transferencia
    }
    if(tx_end(&tx_USB)){
        DELAY_US(10000);
        rx_free_frame(&rx_Fibra2);
        NEXT_STATE(SM_TENSAO_WAIT);
    }
}

uint16_t testeCRC = 0;

//--------------------------------------------------------------------------------------
// Codigo de Teste: INICIO
//--------------------------------------------------------------------------------------
#define ADC_DATA_SAMPLES    (RESULTS_BUFFER_SIZE)
static uint16_t ad_data [ ADC_DATA_SAMPLES ];
//--------------------------------------------------------------------------------------
// Codigo de Teste: FIM
//--------------------------------------------------------------------------------------


STATE(SM_TENSAO_CALC_FAS){
    if(JUST_ARRIVED){
        CRC_retry = 0;
//        testeCRC++;
    }

    crc16_init();

    Xre1 = Xim1 = 0.0;
    Xre2 = Xim2 = 0.0;
    Xre3 = Xim3 = 0.0;
    Xre4 = Xim4 = 0.0;
    Xre5 = Xim5 = 0.0;
    Xre6 = Xim6 = 0.0;

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

    Uint16 trash[6], trash_i, CRC_calc;
    for(trash_i = 0; trash_i < 6; trash_i++){
        trash[trash_i] = CPLD_Read_Write_SPI(0x00);
        CRC_calc = crc16_data(trash[trash_i]);
    }

    Uint16 CRC_CPLD_RAM = 0;
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

        data_frame.voltage_u.pVoltage.A138_P = (float)(atan2l(Xim1, Xre1)*180)/M_PI;
        data_frame.voltage_u.pVoltage.B138_P = (float)(atan2l(Xim2, Xre2)*180)/M_PI;
        data_frame.voltage_u.pVoltage.C138_P = (float)(atan2l(Xim3, Xre3)*180)/M_PI;
        data_frame.voltage_u.pVoltage.A230_P = (float)(atan2l(Xim4, Xre4)*180)/M_PI;
        data_frame.voltage_u.pVoltage.B230_P = (float)(atan2l(Xim5, Xre5)*180)/M_PI;
        data_frame.voltage_u.pVoltage.C230_P = (float)(atan2l(Xim6, Xre6)*180)/M_PI;

        data_frame.voltage_u.pVoltage.A138_A =  (float)(sqrtl(powl(Xre1, 2.0) + powl(Xim1, 2.0)));
        data_frame.voltage_u.pVoltage.B138_A =  (float)(sqrtl(powl(Xre2, 2.0) + powl(Xim2, 2.0)));
        data_frame.voltage_u.pVoltage.C138_A =  (float)(sqrtl(powl(Xre3, 2.0) + powl(Xim3, 2.0)));
        data_frame.voltage_u.pVoltage.A230_A =  (float)(sqrtl(powl(Xre4, 2.0) + powl(Xim4, 2.0)));
        data_frame.voltage_u.pVoltage.B230_A =  (float)(sqrtl(powl(Xre5, 2.0) + powl(Xim5, 2.0)));
        data_frame.voltage_u.pVoltage.C230_A =  (float)(sqrtl(powl(Xre6, 2.0) + powl(Xim6, 2.0)));

        NEXT_STATE(SEND_CONV_END);
    }else{
        CRC_retry++;
        if(CRC_retry < 3)
            NEXT_STATE(SM_TENSAO_CALC_FAS);
        else{
            data_frame.voltage_u.pVoltage.A138_A =  -1;
            data_frame.voltage_u.pVoltage.B138_A =  -1;
            data_frame.voltage_u.pVoltage.C138_A =  -1;
            data_frame.voltage_u.pVoltage.A230_A =  -1;
            data_frame.voltage_u.pVoltage.B230_A =  -1;
            data_frame.voltage_u.pVoltage.C230_A =  -1;
            NEXT_STATE(SEND_CONV_END);
        }
    }
}

STATE(SEND_CONV_END){
    if(JUST_ARRIVED){
        start_tx_frame(&tx_USB, END_CONVERSION, (uint8_t*)&syncPayload, 2*sizeof(t_sync_frame)); // Montar pacote de transferencia
//        start_tx_frame(&tx_USB, END_CONVERSION,  0x0, 0x0); // Montar pacote de transferencia
        START(10000);
    }
    if(tx_end(&tx_USB) || IS_FINISHED){
        clr_DEBUG1();
        NEXT_STATE(SM_TENSAO_WAIT);
    }
}
