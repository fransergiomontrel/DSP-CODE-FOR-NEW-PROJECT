#include "sm_tensao.h"

#define contador 0xFFFFFFFF

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

StateMachine sm_tensao;

struct{

    t_voltage_phasor_frame pVoltage;   
    t_current_phasor_frame pCurrent;
    
} data_frame;

chrono chrono1;
uint32_t RX_Bytes, RX_USB_Bytes;
uint8_t buffer_USB[256], buffer[256];
int64_t timeBuffer;
uint32_t T1, T2, T3, T4;
uint16_t syncTimes = 0, nominal_frequency = 0, CRC_retry = 0, number_IEDs = 0;
int32_t delay_dT1 = 0, delay_dT2 = 0, delay_dT3 = 0, delay_dT4 = 0;
uint32_t delay__T1, delay__T2, delay__T3, delay__T4;
uint32_t ordered_list_T[4];

char ordered_list_Tname [4] = {IED_1, IED_2, IED_3, IED_4};

tms320_sync_frame_t syncPayload;

tms320_board_data_t board_data_0 = {0};
tms320_board_data_t board_data_1 = {0};
tms320_board_data_t board_data_2 = {0};
tms320_board_data_t board_data_3 = {0};
tms320_board_data_t board_data_4 = {0};

tms320_data_t tms320_data;
tms320_uart_frame_t tms320_uart_frame;

extern volatile uint32_t delay_T1, delay_T2, delay_T3, delay_T4;
extern volatile uint16_t timer_end;

extern uint32_t delay_v_T1[syncMax];
extern uint32_t delay_v_T2[syncMax];
extern uint32_t delay_v_T3[syncMax];
extern uint32_t delay_v_T4[syncMax];

extern CiseiRxChannel rx_USB;
extern CiseiTxChannel tx_USB;
extern CiseiRxChannel rx_Fibra0;
extern CiseiTxChannel tx_Fibra0;
extern CiseiRxChannel rx_Fibra1;
extern CiseiTxChannel tx_Fibra1;
extern CiseiRxChannel rx_Fibra2;
extern CiseiTxChannel tx_Fibra2;
extern CiseiRxChannel rx_Fibra3;
extern CiseiTxChannel tx_Fibra3;
extern CiseiRxChannel rx_Fibra4;
extern CiseiTxChannel tx_Fibra4;

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

    init_tx_serial_software(&tx_USB, tx_byte_soft, 0);
    init_tx_serial_software(&tx_Fibra0, tx_byte_soft, 0);
    init_tx_serial(&tx_Fibra1, tx_D_byte, 0);
    init_tx_serial(&tx_Fibra4, tx_A_byte, 0);
    init_tx_serial(&tx_Fibra2, tx_C_byte, 0);
    init_tx_serial(&tx_Fibra3, tx_B_byte, 0);

    init_rx_serial(&rx_USB, buffer_USB, sizeof(buffer_USB));
    init_rx_serial(&rx_Fibra0, (uint8_t*)&data_frame.pCurrent, 2*sizeof(data_frame.pCurrent));
    init_rx_serial(&rx_Fibra1, (uint8_t*)&data_frame.pCurrent, 2*sizeof(data_frame.pCurrent));
    init_rx_serial(&rx_Fibra2, (uint8_t*)&data_frame.pCurrent, 2*sizeof(data_frame.pCurrent));
    init_rx_serial(&rx_Fibra3, (uint8_t*)&data_frame.pCurrent, 2*sizeof(data_frame.pCurrent));
    init_rx_serial(&rx_Fibra4, (uint8_t*)&data_frame.pCurrent, 2*sizeof(data_frame.pCurrent));

    serial_tx_to_fpga_Init();
    
    tx_byte_soft(0x01);

    tx_D_byte(0x01);
    tx_A_byte(0x01);
    tx_C_byte(0x01);
    tx_B_byte(0x01);

    activateUART_Ints();
    //Did by FPGA itself
    //CPLD_CFG_AD();

    NEXT_STATE(SM_TENSAO_WAIT);
}

STATE(SM_TENSAO_WAIT){
    if(JUST_ARRIVED)
    {
        serial_rx_to_stm_Init();
        rx_byte_soft();
    }
    if(rx_frameReceived(&rx_USB, &RX_USB_Bytes)){

        if(rx_getFrameType(&rx_USB) == SYNC_FRAME){
            memcpy(&syncPayload, rx_USB.pBuffer, sizeof(tms320_sync_frame_t));
            nominal_frequency = syncPayload.frequency;
            number_IEDs = syncPayload.number_ieds;
            NEXT_STATE(SM_TENSAO_DELAY);

        }

        rx_free_frame(&rx_USB);
    }
}

STATE(SM_TENSAO_DELAY){
    if(JUST_ARRIVED){
        delay_dT1 = delay_dT2 = delay_dT3 = delay_dT4 = syncTimes = timer_end = CpuTimer1.InterruptCount = 0;
        CpuTimer1Regs.PRD.all = 200000; //CPU Timer Period Register
        CpuTimer1Regs.TCR.bit.TRB = 1;
//        set_DEBUG1();
    }
    if(timer_end){
        timer_end = 0;
        if(number_IEDs == ONE_FIBER){
            if(delay_T1 < 200000){

                delay__T1 =  200000 - delay_T1;
                delay_dT1 += delay__T1;
                delay_v_T1[syncTimes] = delay__T1;
                syncTimes++;

            }
        }
            else if (number_IEDs == TWO_FIBERS){
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
            else if (number_IEDs == THREE_FIBERS){
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
            else if (number_IEDs == FOUR_FIBERS){
                if((delay_T1 < 200000) && (delay_T2 < 200000) && (delay_T3 < 200000) && (delay_T4 < 200000)){

                    delay__T1 =  200000 - delay_T1;
                    delay__T2 =  200000 - delay_T2;
                    delay__T3 =  200000 - delay_T3;
                    delay__T4 =  200000 - delay_T4;
                    delay_dT1 += delay__T1;
                    delay_dT2 += delay__T2;
                    delay_dT3 += delay__T3;
                    delay_dT4 += delay__T4;
                    delay_v_T1[syncTimes] = delay__T1;
                    delay_v_T2[syncTimes] = delay__T2;
                    delay_v_T3[syncTimes] = delay__T3;
                    delay_v_T4[syncTimes] = delay__T4;
                    syncTimes++;

                }
        }

        if(syncTimes == syncMax){
            int o = 0;
            data_frame.pVoltage.sync_data.T1_Max = data_frame.pVoltage.sync_data.T2_Max =\
            data_frame.pVoltage.sync_data.T3_Max = data_frame.pVoltage.sync_data.T4_Max = 0;

            data_frame.pVoltage.sync_data.T1_Min = data_frame.pVoltage.sync_data.T2_Min =\
            data_frame.pVoltage.sync_data.T3_Min = data_frame.pVoltage.sync_data.T4_Min =  9999999;

            uint32_t dp1 = 0, dp2 = 0, dp3 = 0, dp4 = 0;

            data_frame.pVoltage.sync_data.T1 = (delay_dT1/(syncMax*2));//*(5E-9))/(1E-6);
            if(number_IEDs == TWO_FIBERS){

                data_frame.pVoltage.sync_data.T2 = (delay_dT2/(syncMax*2));//*(5E-9))/(1E-6);
            }
            else if(number_IEDs == THREE_FIBERS){

                data_frame.pVoltage.sync_data.T2 = (delay_dT2/(syncMax*2));//*(5E-9))/(1E-6);
                data_frame.pVoltage.sync_data.T3 = (delay_dT3/(syncMax*2));//*(5E-9))/(1E-6);

                ordered_list_T[0] = data_frame.pVoltage.sync_data.T1;
                ordered_list_T[1] = data_frame.pVoltage.sync_data.T2;
                ordered_list_T[2] = data_frame.pVoltage.sync_data.T3;

            }
            else if(number_IEDs == FOUR_FIBERS){

                data_frame.pVoltage.sync_data.T2 = (delay_dT2/(syncMax*2));//*(5E-9))/(1E-6);
                data_frame.pVoltage.sync_data.T3 = (delay_dT3/(syncMax*2));//*(5E-9))/(1E-6);
                data_frame.pVoltage.sync_data.T4 = (delay_dT4/(syncMax*2));//*(5E-9))/(1E-6);

                ordered_list_T[0] = data_frame.pVoltage.sync_data.T1;
                ordered_list_T[1] = data_frame.pVoltage.sync_data.T2;
                ordered_list_T[2] = data_frame.pVoltage.sync_data.T3;
                ordered_list_T[3] = data_frame.pVoltage.sync_data.T4;

            }            

            for(o = 0; o < syncMax; o++){
                uint32_t val1 = delay_v_T1[o];
                data_frame.pVoltage.sync_data.T1_Max = MAX(data_frame.pVoltage.sync_data.T1_Max,val1);
                data_frame.pVoltage.sync_data.T1_Min = MIN(data_frame.pVoltage.sync_data.T1_Min,val1);
                dp1 += pow(val1 - data_frame.pVoltage.sync_data.T1, 2);

                if(number_IEDs == TWO_FIBERS){
                    uint32_t val2 = delay_v_T2[o];
                    data_frame.pVoltage.sync_data.T2_Max = MAX(data_frame.pVoltage.sync_data.T2_Max,val2);
                    data_frame.pVoltage.sync_data.T2_Min = MIN(data_frame.pVoltage.sync_data.T2_Min,val2);
                    dp2 += pow(val2 - data_frame.pVoltage.sync_data.T2, 2);
                }

                else if(number_IEDs == THREE_FIBERS){

                    uint32_t val2 = delay_v_T2[o];
                    data_frame.pVoltage.sync_data.T2_Max = MAX(data_frame.pVoltage.sync_data.T2_Max,val2);
                    data_frame.pVoltage.sync_data.T2_Min = MIN(data_frame.pVoltage.sync_data.T2_Min,val2);
                    dp2 += pow(val2 - data_frame.pVoltage.sync_data.T2, 2);

                    uint32_t val3 = delay_v_T3[o];
                    data_frame.pVoltage.sync_data.T3_Max = MAX(data_frame.pVoltage.sync_data.T3_Max,val3);
                    data_frame.pVoltage.sync_data.T3_Min = MIN(data_frame.pVoltage.sync_data.T3_Min,val3);
                    dp3 += pow(val2 - data_frame.pVoltage.sync_data.T3, 2);

                }

                else if(number_IEDs == FOUR_FIBERS){

                    uint32_t val2 = delay_v_T2[o];
                    data_frame.pVoltage.sync_data.T2_Max = MAX(data_frame.pVoltage.sync_data.T2_Max,val2);
                    data_frame.pVoltage.sync_data.T2_Min = MIN(data_frame.pVoltage.sync_data.T2_Min,val2);
                    dp2 += pow(val2 - data_frame.pVoltage.sync_data.T2, 2);

                    uint32_t val3 = delay_v_T3[o];
                    data_frame.pVoltage.sync_data.T3_Max = MAX(data_frame.pVoltage.sync_data.T3_Max,val3);
                    data_frame.pVoltage.sync_data.T3_Min = MIN(data_frame.pVoltage.sync_data.T3_Min,val3);
                    dp3 += pow(val3 - data_frame.pVoltage.sync_data.T3, 2);

                    uint32_t val4 = delay_v_T4[o];
                    data_frame.pVoltage.sync_data.T4_Max = MAX(data_frame.pVoltage.sync_data.T4_Max,val4);
                    data_frame.pVoltage.sync_data.T4_Min = MIN(data_frame.pVoltage.sync_data.T4_Min,val4);
                    dp4 += pow(val4 - data_frame.pVoltage.sync_data.T4, 2);

                }

            }

            data_frame.pVoltage.sync_data.DP1 = sqrtl(dp1/syncMax);
            if(number_IEDs == TWO_FIBERS){

                data_frame.pVoltage.sync_data.DP2 = sqrtl(dp2/syncMax);
            }
            else if(number_IEDs == THREE_FIBERS){

                data_frame.pVoltage.sync_data.DP2 = sqrtl(dp2/syncMax);
                data_frame.pVoltage.sync_data.DP3 = sqrtl(dp3/syncMax);

            }
            else if(number_IEDs == FOUR_FIBERS){

                data_frame.pVoltage.sync_data.DP2 = sqrtl(dp2/syncMax);
                data_frame.pVoltage.sync_data.DP3 = sqrtl(dp3/syncMax);
                data_frame.pVoltage.sync_data.DP4 = sqrtl(dp4/syncMax);

            }
            NEXT_STATE(SM_TENSAO_TX_SYNC);
        }
    }
    if(syncTimes < syncMax){
        CpuTimer1Regs.TCR.bit.TRB = 1;
        if(number_IEDs == ONE_FIBER){

            start_tx_frame(&tx_Fibra1, SYNC_DELAY, 0x0, 0x0);
            while(!tx_end(&tx_Fibra1));

        }else if(number_IEDs == TWO_FIBERS) {
            
             start_tx_frame(&tx_Fibra1, SYNC_DELAY, 0x0, 0x0);
             start_tx_frame(&tx_Fibra2, SYNC_DELAY, 0x0, 0x0);
             while(!(tx_end(&tx_Fibra1) && tx_end(&tx_Fibra2)));

        }else if(number_IEDs == THREE_FIBERS) {
            
             start_tx_frame(&tx_Fibra1, SYNC_DELAY, 0x0, 0x0);
             start_tx_frame(&tx_Fibra2, SYNC_DELAY, 0x0, 0x0);
             start_tx_frame(&tx_Fibra3, SYNC_DELAY, 0x0, 0x0);
             while(!(tx_end(&tx_Fibra1) && tx_end(&tx_Fibra2) && tx_end(&tx_Fibra3)));

        }
        else if(number_IEDs == FOUR_FIBERS) {
            
             start_tx_frame(&tx_Fibra1, SYNC_DELAY, 0x0, 0x0);
             start_tx_frame(&tx_Fibra2, SYNC_DELAY, 0x0, 0x0);
             start_tx_frame(&tx_Fibra3, SYNC_DELAY, 0x0, 0x0);
             start_tx_frame(&tx_Fibra4, SYNC_DELAY, 0x0, 0x0);
             while(!(tx_end(&tx_Fibra1) && tx_end(&tx_Fibra2) && tx_end(&tx_Fibra3) && tx_end(&tx_Fibra4)));

        }

        START(100); //100 valor antigo
        while(!IS_FINISHED);

        GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1; // TX.A
        GpioDataRegs.GPCCLEAR.bit.GPIO86 = 1; // TX.B
        GpioDataRegs.GPCCLEAR.bit.GPIO89 = 1; // TX.C
        GpioDataRegs.GPCCLEAR.bit.GPIO93 = 1; // TX.D

        configureSCI_sync();

        START(25);
        while(!IS_FINISHED);

        delay_T1 = delay_T2 = delay_T3 = delay_T4 =  (uint32_t)-1;

        GpioDataRegs.GPCSET.bit.GPIO84 = 1; // TX.A
        GpioDataRegs.GPCSET.bit.GPIO86 = 1; // TX.B
        GpioDataRegs.GPCSET.bit.GPIO89 = 1; // TX.C
        GpioDataRegs.GPCSET.bit.GPIO93 = 1; // TX.D

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
        serial_tx_to_fpga_Init();
        if(nominal_frequency == NOM_FREQ_60HZ)
        {
            start_tx_frame_software(&tx_Fibra0, SYNC_FRAME_60HZ, 0x0, 0x0);
        }
        else if (nominal_frequency == NOM_FREQ_50HZ)
        {
            start_tx_frame_software(&tx_Fibra0, SYNC_FRAME_50HZ, 0x0, 0x0);
        }
        //Send sync to current module
        if(number_IEDs == ONE_FIBER){

            if(nominal_frequency == NOM_FREQ_60HZ)
            {
                start_tx_frame(&tx_Fibra1, SYNC_FRAME_60HZ, 0x0, 0x0);
            }
            else if (nominal_frequency == NOM_FREQ_50HZ)
            {
                start_tx_frame(&tx_Fibra1, SYNC_FRAME_50HZ, 0x0, 0x0);
            }
            
        }else if(number_IEDs == TWO_FIBERS){

            if(nominal_frequency == NOM_FREQ_60HZ)
            {
                start_tx_frame(&tx_Fibra1, SYNC_FRAME_60HZ, 0x0, 0x0);
                start_tx_frame(&tx_Fibra2, SYNC_FRAME_60HZ, 0x0, 0x0);
            }
            else if (nominal_frequency == NOM_FREQ_50HZ)
            {
                start_tx_frame(&tx_Fibra1, SYNC_FRAME_50HZ, 0x0, 0x0);
                start_tx_frame(&tx_Fibra2, SYNC_FRAME_50HZ, 0x0, 0x0);
            }           

        }else if(number_IEDs == THREE_FIBERS){

            if(nominal_frequency == NOM_FREQ_60HZ)
            {
                start_tx_frame(&tx_Fibra1, SYNC_FRAME_60HZ, 0x0, 0x0);
                start_tx_frame(&tx_Fibra2, SYNC_FRAME_60HZ, 0x0, 0x0);
                start_tx_frame(&tx_Fibra3, SYNC_FRAME_60HZ, 0x0, 0x0);
            }
            else if (nominal_frequency == NOM_FREQ_50HZ)
            {
                start_tx_frame(&tx_Fibra1, SYNC_FRAME_50HZ, 0x0, 0x0);
                start_tx_frame(&tx_Fibra2, SYNC_FRAME_50HZ, 0x0, 0x0);
                start_tx_frame(&tx_Fibra3, SYNC_FRAME_50HZ, 0x0, 0x0);
            }

        }else if(number_IEDs == FOUR_FIBERS){

            if(nominal_frequency == NOM_FREQ_60HZ)
            {
                start_tx_frame(&tx_Fibra1, SYNC_FRAME_60HZ, 0x0, 0x0);
                start_tx_frame(&tx_Fibra2, SYNC_FRAME_60HZ, 0x0, 0x0);
                start_tx_frame(&tx_Fibra3, SYNC_FRAME_60HZ, 0x0, 0x0);
                start_tx_frame(&tx_Fibra4, SYNC_FRAME_60HZ, 0x0, 0x0);
            }
            else if (nominal_frequency == NOM_FREQ_50HZ)
            {
                start_tx_frame(&tx_Fibra1, SYNC_FRAME_50HZ, 0x0, 0x0);
                start_tx_frame(&tx_Fibra2, SYNC_FRAME_50HZ, 0x0, 0x0);
                start_tx_frame(&tx_Fibra3, SYNC_FRAME_50HZ, 0x0, 0x0);
                start_tx_frame(&tx_Fibra4, SYNC_FRAME_50HZ, 0x0, 0x0);
            }
        }

        data_frame.pVoltage.A138_A =  -1;
        data_frame.pVoltage.B138_A =  -1;
        data_frame.pVoltage.C138_A =  -1;
        data_frame.pVoltage.A230_A =  -1;
        data_frame.pVoltage.B230_A =  -1;
        data_frame.pVoltage.C230_A =  -1;

        //wait
        START(1000);
    }
    if(IS_FINISHED){
        //Configure pins
        GpioDataRegs.GPASET.bit.GPIO28 = 1; // TX.0
        GpioDataRegs.GPCSET.bit.GPIO84 = 1; // TX.A
        GpioDataRegs.GPCSET.bit.GPIO86 = 1; // TX.B
        GpioDataRegs.GPCSET.bit.GPIO89 = 1; // TX.C
        GpioDataRegs.GPCSET.bit.GPIO93 = 1; // TX.D
        
        configureSCI_sync();
        NEXT_STATE(SM_TENSAO_SYNC);
    }
}

uint32_t i1, i2, i3, i4;

STATE(SM_TENSAO_SYNC){

    startCapture();

    START(100);
    while(!IS_FINISHED);

    GpioDataRegs.GPACLEAR.bit.GPIO28 = 1; // TX.0
    GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1; // TX.A
    GpioDataRegs.GPCCLEAR.bit.GPIO86 = 1; // TX.B
    GpioDataRegs.GPCCLEAR.bit.GPIO89 = 1; // TX.C
    GpioDataRegs.GPCCLEAR.bit.GPIO93 = 1; // TX.D

    START(500);
    while(!IS_FINISHED);

    DINT;
    if(number_IEDs == ONE_FIBER)
    {
        
        i2 = contador - data_frame.pVoltage.sync_data.T1;
        CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
        CpuTimer2Regs.TCR.bit.TRB = 1;
        CpuTimer2Regs.TCR.bit.TSS = 0;
        GpioDataRegs.GPCSET.bit.GPIO93 = 1;
        while(CpuTimer2Regs.TIM.all >= i2);

    }
    else if(number_IEDs == TWO_FIBERS)
    {
       
        if(data_frame.pVoltage.sync_data.T1 > data_frame.pVoltage.sync_data.T2){
            i1 = contador - (data_frame.pVoltage.sync_data.T1 - data_frame.pVoltage.sync_data.T2);
            i2 = contador - data_frame.pVoltage.sync_data.T1;
            CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
            CpuTimer2Regs.TCR.bit.TRB = 1;
            CpuTimer2Regs.TCR.bit.TSS = 0;
            GpioDataRegs.GPCSET.bit.GPIO93 = 1; // TX.D
            while(CpuTimer2Regs.TIM.all >= i2);
            GpioDataRegs.GPCSET.bit.GPIO89 = 1; // TX.C
        }else{
            i1 = contador - (data_frame.pVoltage.sync_data.T2 - data_frame.pVoltage.sync_data.T1);
            i2 = contador - data_frame.pVoltage.sync_data.T1;
            CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
            CpuTimer2Regs.TCR.bit.TRB = 1;
            CpuTimer2Regs.TCR.bit.TSS = 0;
            GpioDataRegs.GPCSET.bit.GPIO89 = 1; // TX.C
            while(CpuTimer2Regs.TIM.all >= i2);
            GpioDataRegs.GPCSET.bit.GPIO93 = 1; // TX.D
        }

    }
    else if(number_IEDs == THREE_FIBERS)
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
            case IED_1:

                CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
                CpuTimer2Regs.TCR.bit.TRB = 1;
                CpuTimer2Regs.TCR.bit.TSS = 0;
                GpioDataRegs.GPCSET.bit.GPIO93 = 1; // TX.D
                while(CpuTimer2Regs.TIM.all >= i2);
                //Check which among 2 slaver IED has first bigger time propagation
                switch (ordered_list_Tname[1])
                {
                    case IED_2:
                        GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                        while(CpuTimer2Regs.TIM.all >= i1);
                        GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                    break;
                    case IED_3:
                        GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                        while(CpuTimer2Regs.TIM.all >= i1);
                        GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                    break;
                }
                
                break;

            case IED_2:
                
                CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
                CpuTimer2Regs.TCR.bit.TRB = 1;
                CpuTimer2Regs.TCR.bit.TSS = 0;
                GpioDataRegs.GPCSET.bit.GPIO89 = 1; // TX.C
                while(CpuTimer2Regs.TIM.all >= i2);
                //Check which among 2 slaver IED has first bigger time propagation
                switch (ordered_list_Tname[1])
                {
                    case IED_1:
                        GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                        while(CpuTimer2Regs.TIM.all >= i1);
                        GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                    break;
                    case IED_3:
                        GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                        while(CpuTimer2Regs.TIM.all >= i1);
                        GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                    break;
                }

            break;

            case IED_3:
                
                CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
                CpuTimer2Regs.TCR.bit.TRB = 1;
                CpuTimer2Regs.TCR.bit.TSS = 0;
                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                while(CpuTimer2Regs.TIM.all >= i2);
                //Check which among 2 slaver IED has first bigger time propagation
                switch (ordered_list_Tname[1])
                {
                    case IED_1:
                        GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                        while(CpuTimer2Regs.TIM.all >= i1);
                        GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                    break;
                    case IED_2:
                        GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                        while(CpuTimer2Regs.TIM.all >= i1);
                        GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                    break;
                }

            break;
            
        } 

        while(CpuTimer2Regs.TIM.all >= i3);
    
    }
    else if(number_IEDs == FOUR_FIBERS)
    {
       
        uint32_t temp;
        char temp_name;
        uint16_t i, j;

        for (i = 0; i < 3; i++) {
            for (j = i + 1; j < 4; j++) {
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
        i3 = contador - (ordered_list_T[3] - ordered_list_T[2]);
        i4 = contador - ordered_list_T[3];

        //Check which among 4 slaver IED has first bigger time propagation
        switch (ordered_list_Tname[3])
        {
            case IED_1:

                CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
                CpuTimer2Regs.TCR.bit.TRB = 1;
                CpuTimer2Regs.TCR.bit.TSS = 0;
                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                while(CpuTimer2Regs.TIM.all >= i3);
                //Check which among 2 slaver IED has first bigger time propagation
                switch (ordered_list_Tname[2])
                {
                    case IED_2:
                        GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                        while(CpuTimer2Regs.TIM.all >= i2);
                        switch (ordered_list_Tname[1])
                        {
                            case IED_3:
                                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                            break;
                            case IED_4:
                                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                            break;
                        }
                    break;
                    case IED_3:
                        GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                        while(CpuTimer2Regs.TIM.all >= i2);
                        switch (ordered_list_Tname[1])
                        {
                            case IED_2:
                                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                            break;
                            case IED_4:
                                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                            break;
                        }
                    break;
                    case IED_4:
                        GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                        while(CpuTimer2Regs.TIM.all >= i2);
                        switch (ordered_list_Tname[1])
                        {
                            case IED_2:
                                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                            break;
                            case IED_3:
                                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                            break;
                        }
                    break;                   
                }                
            break;

            case IED_2:
                
                CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
                CpuTimer2Regs.TCR.bit.TRB = 1;
                CpuTimer2Regs.TCR.bit.TSS = 0;
                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                while(CpuTimer2Regs.TIM.all >= i3);
                //Check which among 2 slaver IED has first bigger time propagation
                switch (ordered_list_Tname[2])
                {
                    case IED_1:
                        GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                        while(CpuTimer2Regs.TIM.all >= i2);
                        switch (ordered_list_Tname[1])
                        {
                            case IED_3:
                                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                            break;
                            case IED_4:
                                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                            break;
                        }
                    break;
                    case IED_3:
                        GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                        while(CpuTimer2Regs.TIM.all >= i2);
                        switch (ordered_list_Tname[1])
                        {
                            case IED_1:
                                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                            break;
                            case IED_4:
                                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                            break;
                        }
                    break;
                    case IED_4:
                        GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                        while(CpuTimer2Regs.TIM.all >= i2);
                        switch (ordered_list_Tname[1])
                        {
                            case IED_1:
                                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                            break;
                            case IED_3:
                                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                            break;
                        }
                    break;
                    
                }

            break;

            case IED_3:
                
                CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
                CpuTimer2Regs.TCR.bit.TRB = 1;
                CpuTimer2Regs.TCR.bit.TSS = 0;
                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                while(CpuTimer2Regs.TIM.all >= i3);
                //Check which among 2 slaver IED has first bigger time propagation
                switch (ordered_list_Tname[2])
                {
                    case IED_1:
                        GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                        while(CpuTimer2Regs.TIM.all >= i2);
                        switch (ordered_list_Tname[1])
                        {
                            case IED_2:
                                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                            break;
                            case IED_4:
                                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                            break;
                        }
                    break;
                    case IED_2:
                        GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                        while(CpuTimer2Regs.TIM.all >= i2);
                        switch (ordered_list_Tname[1])
                        {
                            case IED_1:
                                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                            break;
                            case IED_4:
                                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                            break;
                        }
                    break;
                    case IED_4:
                        GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                        while(CpuTimer2Regs.TIM.all >= i2);
                        switch (ordered_list_Tname[1])
                        {
                            case IED_1:
                                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                            break;
                            case IED_2:
                                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                            break;
                        }
                    break;
                    
                }
                
            break;

            case IED_4:
                
                CpuTimer2Regs.PRD.all = contador; //CPU Timer Period Register
                CpuTimer2Regs.TCR.bit.TRB = 1;
                CpuTimer2Regs.TCR.bit.TSS = 0;
                GpioDataRegs.GPCSET.bit.GPIO84 = 1; //TX.A
                while(CpuTimer2Regs.TIM.all >= i3);
                //Check which among 2 slaver IED has first bigger time propagation
                switch (ordered_list_Tname[2])
                {
                    case IED_1:
                        GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                        while(CpuTimer2Regs.TIM.all >= i2);
                        switch (ordered_list_Tname[1])
                        {
                            case IED_2:
                                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                            break;
                            case IED_3:
                                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                            break;
                        }
                    break;
                    case IED_2:
                        GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                        while(CpuTimer2Regs.TIM.all >= i2);
                        switch (ordered_list_Tname[1])
                        {
                            case IED_1:
                                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                            break;
                            case IED_3:
                                GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                            break;
                        }
                    break;
                    case IED_3:
                        GpioDataRegs.GPCSET.bit.GPIO86 = 1; //TX.B
                        while(CpuTimer2Regs.TIM.all >= i2);
                        switch (ordered_list_Tname[1])
                        {
                            case IED_1:
                                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                            break;
                            case IED_2:
                                GpioDataRegs.GPCSET.bit.GPIO89 = 1; //TX.C
                                while(CpuTimer2Regs.TIM.all >= i1);
                                GpioDataRegs.GPCSET.bit.GPIO93 = 1; //TX.D
                            break;
                        }
                    break;                    
                }
            break;            
        } 
        while(CpuTimer2Regs.TIM.all >= i4);
    }

    //start capture
//    startCapture();
    __asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");
    __asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");
    __asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");
    __asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");
    __asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");__asm(" NOP");

    //GPIO_WritePin(CONVST, 1);
    GpioDataRegs.GPASET.bit.GPIO28 = 1; //TX.0
    acquisition_counter++;
    CpuTimer2Regs.TCR.bit.TSS = 1;

    EINT;

    configureSCI_UART();

    NEXT_STATE(SM_TENSAO_CONV);
}

STATE(SM_TENSAO_CONV){
    if(JUST_ARRIVED){
        START(35000);
    }
    if(IS_FINISHED){
        //GPIO_WritePin(CONVST, 0);
        GpioDataRegs.GPACLEAR.bit.GPIO28 = 1; //TX.0
        CPLD_WE(0);
        data_frame.pVoltage.acquisition_counter = acquisition_counter;
        NEXT_STATE(SM_TENSAO_REQ_I0);
    }
}

STATE(SM_TENSAO_REQ_I0){  
    //Wait to recieve data via fiber
    if(JUST_ARRIVED){
        serial_tx_to_fpga_Init();
        start_tx_frame(&tx_Fibra0, DATA_REQUEST, 0x0, 0x0);
        while(!tx_end(&tx_Fibra0));
        serial_rx_to_fpga_Init();
        rx_byte_soft();
        START(9000000);
    }if(IS_FINISHED){

        if(number_IEDs != NO_FIBER){
            NEXT_STATE(SM_TENSAO_REQ_I1);
        }
        else{
            NEXT_STATE(RET_TO_POL_CONVERT);
        }

    }
    if(rx_frameReceived(&rx_Fibra0, &RX_Bytes)){

        if(rx_getFrameType(&rx_Fibra0) == CURRENT_PHASOR_X){
            GPIO_WritePin(54, 1);
            if(number_IEDs != NO_FIBER){
                NEXT_STATE(SM_TENSAO_REQ_I1);
            }
            else{
                NEXT_STATE(RET_TO_POL_CONVERT);    
            }
        }else{
            if(number_IEDs != NO_FIBER){
                NEXT_STATE(SM_TENSAO_REQ_I1);
            }
            else{
                NEXT_STATE(RET_TO_POL_CONVERT);    
            }
        }        
    }   
}

STATE(SM_TENSAO_REQ_I1){
    //Wait to recieve data via fiber
    if(JUST_ARRIVED){

        START(9000000);
        start_tx_frame(&tx_Fibra1, DATA_REQUEST, 0x0, 0x0);

    }if(IS_FINISHED){

        if(number_IEDs != ONE_FIBER){
            NEXT_STATE(SM_TENSAO_REQ_I2);
        }
        else{
            NEXT_STATE(RET_TO_POL_CONVERT);
        }

    }
    if(rx_frameReceived(&rx_Fibra1, &RX_Bytes)){

        if(rx_getFrameType(&rx_Fibra1) == CURRENT_PHASOR_X){
            GPIO_WritePin(54, 1);
            if(number_IEDs != ONE_FIBER){
                NEXT_STATE(SM_TENSAO_REQ_I2);
            }
            else{
                NEXT_STATE(RET_TO_POL_CONVERT);    
            }
        }else{
            if(number_IEDs != ONE_FIBER){
                NEXT_STATE(SM_TENSAO_REQ_I2);
            }
            else{
                NEXT_STATE(RET_TO_POL_CONVERT);    
            }
        }        
    }
}


STATE(SM_TENSAO_REQ_I2){
    //Wait to recieve data via fiber
    if(JUST_ARRIVED){

        START(9000000);
        start_tx_frame(&tx_Fibra2, DATA_REQUEST, 0x0, 0x0);
        
    }if(IS_FINISHED){

        if(number_IEDs != TWO_FIBERS){
            NEXT_STATE(SM_TENSAO_REQ_I3);
        }
        else{
            NEXT_STATE(RET_TO_POL_CONVERT);
        }
       
    }
    if(rx_frameReceived(&rx_Fibra2, &RX_Bytes)){
        if(rx_getFrameType(&rx_Fibra2) == CURRENT_PHASOR_X)
            if(number_IEDs != TWO_FIBERS){
                NEXT_STATE(SM_TENSAO_REQ_I3);
            }
            else{
                NEXT_STATE(RET_TO_POL_CONVERT);    
            }
        else
             if(number_IEDs != TWO_FIBERS){
                NEXT_STATE(SM_TENSAO_REQ_I3);
            }
            else{
                NEXT_STATE(RET_TO_POL_CONVERT);    
            }
    }
}

STATE(SM_TENSAO_REQ_I3){
    //Wait to recieve data via fiber
    if(JUST_ARRIVED){
        
        START(9000000);
        start_tx_frame(&tx_Fibra3, DATA_REQUEST, 0x0, 0x0);
        
    }
    if(IS_FINISHED){
        if(number_IEDs != THREE_FIBERS){
            NEXT_STATE(SM_TENSAO_REQ_I4);
        }
        else{
            NEXT_STATE(RET_TO_POL_CONVERT);
        }
        
    }
    if(rx_frameReceived(&rx_Fibra3, &RX_Bytes)){
        if(rx_getFrameType(&rx_Fibra3) == CURRENT_PHASOR_X)
            if(number_IEDs != THREE_FIBERS){
                NEXT_STATE(SM_TENSAO_REQ_I4);
            }
            else{
                NEXT_STATE(RET_TO_POL_CONVERT);    
            }
        else
            if(number_IEDs != THREE_FIBERS){
                NEXT_STATE(SM_TENSAO_REQ_I4);
            }
            else{
                NEXT_STATE(RET_TO_POL_CONVERT);    
            }
    }
}

STATE(SM_TENSAO_REQ_I4){
    //Wait to recieve data via fiber
    if(JUST_ARRIVED){
        
        START(9000000);
        start_tx_frame(&tx_Fibra4, DATA_REQUEST, 0x0, 0x0);
        
    }if(IS_FINISHED){

         NEXT_STATE(RET_TO_POL_CONVERT);

    }
        
    if(rx_frameReceived(&rx_Fibra4, &RX_Bytes)){

        if(rx_getFrameType(&rx_Fibra4) == CURRENT_PHASOR_X)
        {
            NEXT_STATE(RET_TO_POL_CONVERT);
        }    
        else{
            NEXT_STATE(RET_TO_POL_CONVERT);
        }
    }                
}

STATE(RET_TO_POL_CONVERT){
    
    phasors_int_to_float(&rx_Fibra0,&board_data_0);
    phasors_ret_to_polar(&board_data_0);

    if(number_IEDs == ONE_FIBER)
    {
        phasors_int_to_float(&rx_Fibra1,&board_data_1);
        phasors_ret_to_polar(&board_data_1);
    }
    else if(number_IEDs == TWO_FIBERS)
    {
        phasors_int_to_float(&rx_Fibra1,&board_data_1);
        phasors_ret_to_polar(&board_data_1);

        phasors_int_to_float(&rx_Fibra2,&board_data_2);
        phasors_ret_to_polar(&board_data_2);
    }
    else if(number_IEDs == THREE_FIBERS)
    {
        phasors_int_to_float(&rx_Fibra1,&board_data_1);
        phasors_ret_to_polar(&board_data_1);
        
        phasors_int_to_float(&rx_Fibra2,&board_data_2);
        phasors_ret_to_polar(&board_data_2);

        phasors_int_to_float(&rx_Fibra3,&board_data_3);
        phasors_ret_to_polar(&board_data_3);
    }
    else if(number_IEDs == FOUR_FIBERS)
    {
        phasors_int_to_float(&rx_Fibra1,&board_data_1);
        phasors_ret_to_polar(&board_data_1);
        
        phasors_int_to_float(&rx_Fibra2,&board_data_2);
        phasors_ret_to_polar(&board_data_2);

        phasors_int_to_float(&rx_Fibra3,&board_data_3);
        phasors_ret_to_polar(&board_data_3);

        phasors_int_to_float(&rx_Fibra4,&board_data_4);
        phasors_ret_to_polar(&board_data_4);
    }
    NEXT_STATE(SM_CORRENTE_TEMPERATURA);
}

STATE(SM_CORRENTE_TEMPERATURA){
    
    ads1118_int_to_float(&rx_Fibra0,&board_data_0);

    if(number_IEDs == ONE_FIBER)
    {
        ads1118_int_to_float(&rx_Fibra1,&board_data_1);
    }
    else if(number_IEDs == TWO_FIBERS)
    {
        ads1118_int_to_float(&rx_Fibra1,&board_data_1);
        ads1118_int_to_float(&rx_Fibra2,&board_data_2);
    }
    else if(number_IEDs == THREE_FIBERS)
    {
        ads1118_int_to_float(&rx_Fibra1,&board_data_1);
        ads1118_int_to_float(&rx_Fibra2,&board_data_2);
        ads1118_int_to_float(&rx_Fibra3,&board_data_3);
    }
    else if(number_IEDs == FOUR_FIBERS)
    {
        ads1118_int_to_float(&rx_Fibra1,&board_data_1);
        ads1118_int_to_float(&rx_Fibra2,&board_data_2);
        ads1118_int_to_float(&rx_Fibra3,&board_data_3);
        ads1118_int_to_float(&rx_Fibra4,&board_data_4);
    }
    NEXT_STATE(SM_SEND_DATA);
    
}

STATE(SM_SEND_DATA){

    if(JUST_ARRIVED){

        memcpy(&(tms320_data.boards[0]), &board_data_0, sizeof(tms320_board_data_t));
        memcpy(&(tms320_data.boards[1]), &board_data_1, sizeof(tms320_board_data_t));
        memcpy(&(tms320_data.boards[2]), &board_data_2, sizeof(tms320_board_data_t));
        memcpy(&(tms320_data.boards[3]), &board_data_3, sizeof(tms320_board_data_t));
        memcpy(&(tms320_data.boards[4]), &board_data_4, sizeof(tms320_board_data_t));
        //Send data to STM32 through software serial
        //data_frame.pCurrent.acquisition_counter = acquisition_counter;
        tms320_frame_crc(&tms320_data, &tms320_uart_frame);
        serial_tx_to_stm_Init();
        start_tx_frame_software(&tx_USB, TMS320_DATA_CRC, (uint8_t*)&tms320_uart_frame, sizeof(tms320_uart_frame));//Montar pacote de transferencia

    }
    if(tx_end(&tx_USB)){
        DELAY_US(10000);

        memset(&board_data_0, 0, sizeof(board_data_0));
        memset(&board_data_1, 0, sizeof(board_data_1));
        memset(&board_data_2, 0, sizeof(board_data_2));
        memset(&board_data_3, 0, sizeof(board_data_3));
        memset(&board_data_4, 0, sizeof(board_data_4));

        rx_free_frame(&rx_Fibra0);
        rx_free_frame(&rx_Fibra1);
        rx_free_frame(&rx_Fibra2);
        rx_free_frame(&rx_Fibra3);
        rx_free_frame(&rx_Fibra4);

        NEXT_STATE(SM_TENSAO_WAIT);
    }
}


