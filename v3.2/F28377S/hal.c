#pragma DATA_SECTION (ADC_Results,".data_frame")

#include "common/hal.h"
#include "pins.h"
#include "F28x_Project.h"
#include "common/sm_serial_api.h"
#include "common/sm_tensao.h"
#include "common/sm_corrente.h"
#include "F28377S/LCD_I2C.h"

#define contador 0xFFFFFFFF;

volatile uint8_t bufferFull;
volatile uint8_t startCap = 0;
volatile uint16_t Temp_index;
volatile uint16_t edgeCount = 0;
volatile uint16_t timer_end = 0;
volatile uint32_t delay_T1 = 0;
volatile uint32_t delay_T2 = 0;
volatile uint32_t delay_T3 = 0;

uint16_t acquisition_counter = 0;

uint32_t delay_v_T1[syncMax];
uint32_t delay_v_T2[syncMax];
uint32_t delay_v_T3[syncMax];

volatile uint16_t delayF = 0;

volatile t_adc_results ADC_Results;
volatile t_temp_data Temp_Results;

CiseiRxChannel rx_USB;
CiseiTxChannel tx_USB;

CiseiRxChannel rx_Fibra1;
CiseiTxChannel tx_Fibra1;

CiseiRxChannel rx_Fibra2;
CiseiTxChannel tx_Fibra2;

struct I2CMSG I2cMsgOut1 = { I2C_MSGSTAT_SEND_WITHSTOP,
                             I2C_SLAVE_ADDR,
                             I2C_NUMBYTES };

void syncInt_ena(){
    DINT;
    PieVectTable.XINT1_INT = &syncPulse;
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
    XintRegs.XINT1CR.bit.ENABLE = 1;
    EINT;

}

float64 analogEq(Uint16 adcres, Uint16 bits){
    float64 aux1, aux2, aux3, aux4;
    if(bits == 16){
        aux1 = adcres;
        aux2 = aux1 * 2;
        aux3 = aux2/65536.0;
        aux4 = aux3 - 1.0;
    }else if(bits == 12){
        aux1 = adcres;
        aux4 = aux1/4096.0;
    }
    else
        aux4 = -2;

    return(3.3*(aux4));
}

void syncInt_dis(){
    DINT;
    PieVectTable.XINT1_INT = &doNothing;
    XintRegs.XINT1CR.bit.ENABLE = 0;
    EINT;
    __asm(" NOP");__asm(" NOP");__asm(" NOP");
    DINT;
    PieVectTable.XINT1_INT = &syncPulse;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    EINT;
}


//==============================================================================
// init_hal - Inicializacao do HAL para a Interface de Tensao
//==============================================================================
void init_hal(void){
    sysInit();
    GPIOInit();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    config_ADC();
    ConfigureEPWM();
    SetupADCsEpwm();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    configTimer();

    initInts();

    serial_A_Init();
    serial_B_Init();
    serial_C_Init();
    serial_D_Init();


    bufferFull = Temp_index = 0;
    resetResultBuffer();

//    tx_B_byte(0x01);
//    (tx_C_byte0x01);
//    tx_A_byte(0x01);
//    tx_D_byte(0x01);

    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x2044;
    SpiaRegs.SPIFFCT.all = 0x0;

    InitSpi();
}


//==============================================================================
// init_hal_C - Inicializacao do HAL para a Interface de Corrente
//==============================================================================
void init_hal_C(void){
    
    sysInit();
    GPIOInit();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    config_ADC();
    ConfigureEPWM();
    SetupADCsEpwm();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    configTimer();

    initInts();

#if defined (BOARD_NEW)
    serial_D_Init(); // Placa nova
#elif defined (BOARD_PREVIOUS)
    serial_A_Init(); // Placa anterior (que estava em uso)
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    bufferFull = Temp_index = 0;
    resetResultBuffer();

//    tx_A_byte(0x01);
//    tx_D_byte(0x01);

    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x2044;
    SpiaRegs.SPIFFCT.all = 0x0;

    InitSpi();
}

interrupt void syncPulse(){
    delay_T1 = CpuTimer1Regs.TIM.all;
    if(delayF){
#if defined (BOARD_NEW)
        GpioDataRegs.GPBDAT.bit.GPIO47 = 1; // Placa Nova
#elif defined (BOARD_PREVIOUS)
        GpioDataRegs.GPBDAT.bit.GPIO48 = 1; // Placa Anterior
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    }
    if(startCap){
        GPIO_WritePin(CONVST, 1);
        GPIO_WritePin(54, 0);
        acquisition_counter++;
//        startCapture();
        startCap = 0;
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Issue PIE ACK
}

interrupt void doNothing(){
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Issue PIE ACK
}

interrupt void xint2_isr(void){
    delay_T2 = CpuTimer1Regs.TIM.all;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Issue PIE ACK
}

interrupt void xint3_isr(void){
    delay_T3 = CpuTimer1Regs.TIM.all;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12; // Issue PIE ACK
}

void activateUART_Ints(){
//#define BOARD_NEW

#if defined (BOARD_NEW)
    ScidRegs.SCICTL2.bit.TXINTENA   = 1;
    ScidRegs.SCICTL2.bit.RXBKINTENA = 1;
#endif
#if defined (BOARD_PREVIOUS) || (defined(BOARD_NEW) && defined(TENSAO))
    SciaRegs.SCICTL2.bit.TXINTENA   = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    ScibRegs.SCICTL2.bit.TXINTENA   = 1;
    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

    ScicRegs.SCICTL2.bit.TXINTENA   = 1;
    ScicRegs.SCICTL2.bit.RXBKINTENA = 1;
}

void TXInts(uint8_t enable){
#if defined (BOARD_NEW)
    PieCtrlRegs.PIEIER8.bit.INTx8   = enable;   // SCI.D TX
#endif
#if defined (BOARD_PREVIOUS) || (defined(BOARD_NEW) && defined(TENSAO))
    PieCtrlRegs.PIEIER9.bit.INTx2   = enable;   // SCI.A TX
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    PieCtrlRegs.PIEIER9.bit.INTx4   = enable;   // SCI.B TX
    PieCtrlRegs.PIEIER8.bit.INTx6   = enable;   // SCI.C TX
}

void RXInts(uint8_t enable){
#if defined (BOARD_NEW)
    PieCtrlRegs.PIEIER8.bit.INTx7   = enable; // SCI.D RX
#endif
#if defined (BOARD_PREVIOUS) || (defined(BOARD_NEW) && defined(TENSAO))
    PieCtrlRegs.PIEIER9.bit.INTx1   = enable; // SCI.A RX
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    PieCtrlRegs.PIEIER9.bit.INTx3   = enable; // SCI.B RX
    PieCtrlRegs.PIEIER8.bit.INTx5   = enable; // SCI.C RX  
}

void resetResultBuffer(){
    uint16_t i;
    for(i = 0; i < RESULTS_BUFFER_SIZE; i++){
       setA138(i,0);
       setB138(i,0);
       setC138(i,0);
       setA230(i,0);
       setB230(i,0);
       setC230(i,0);
    }
    bufferFull = Temp_index = 0;
}

interrupt void timer1_isr(void){
//    togglePin(DEBUG1);
//    CpuTimer1Regs.TCR.bit.TRB = 1;
    CpuTimer1.InterruptCount++;
    CpuTimer1Regs.TCR.bit.TSS = 1;
    timer_end = 1;
}

void initInts(){
    DINT;
    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    EALLOW;
    PieVectTable.TIMER1_INT = &timer1_isr;

    PieVectTable.ADCB1_INT = &readADC;
//  PieVectTable.TIMER0_INT = &runADC;


#if defined (BOARD_NEW)
    PieVectTable.SCID_RX_INT = &smRX_D; // Interrupcao de Recepcao (RX) SCI D (por Almeida)
    PieVectTable.SCID_TX_INT = &smTX_D; // Interrupcao de Transmissao (TX) SCI D (por Almeida)
#endif
#if defined (BOARD_PREVIOUS) || (defined(BOARD_NEW) && defined(TENSAO))
    PieVectTable.SCIA_RX_INT = &smRX_A;
    PieVectTable.SCIA_TX_INT = &smTX_A;
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    PieVectTable.SCIB_RX_INT = &smRX_B;
    PieVectTable.SCIB_TX_INT = &smTX_B;

    PieVectTable.SCIC_RX_INT = &smRX_C;
    PieVectTable.SCIC_TX_INT = &smTX_C;

    PieVectTable.XINT1_INT = &syncPulse;
    PieVectTable.XINT2_INT = &xint2_isr;
    PieVectTable.XINT3_INT = &xint3_isr;
    EDIS;

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;  // Enable the PIE block

  PieCtrlRegs.PIEIER1.bit.INTx2 = 1;  // PIE Group 1, INT2 - ADCB1_INT
    //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // PIE Group 1, INT7 - TIMER0_INT
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;  // PIE Group 1, INT4 - XINT1_INT
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;  // PIE Group 1, INT5 - XINT2_INT

    PieCtrlRegs.PIEIER8.bit.INTx5 = 1;  // PIE Group 8, INT5 - SCIC_RX_INT
    PieCtrlRegs.PIEIER8.bit.INTx6 = 1;  // PIE Group 8, INT6 - SCIC_TX_INT

#if defined (BOARD_NEW)
    PieCtrlRegs.PIEIER8.bit.INTx7 = 1;  // PIE Group 8, INT7 - SCID_RX_INT
    PieCtrlRegs.PIEIER8.bit.INTx8 = 1;  // PIE Group 8, INT8 - SCID_TX_INT
#endif
#if defined (BOARD_PREVIOUS) || (defined(BOARD_NEW) && defined(TENSAO))
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;  // PIE Group 9, INT1 - SCIA_RX_INT
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;  // PIE Group 9, INT2 - SCIA_TX_INT
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    //NAO UTILIZADO NO MOMENTO
//    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;  // PIE Group 9, INT3 - SCIB_RX_INT
//    PieCtrlRegs.PIEIER9.bit.INTx4 = 1;  // PIE Group 9, INT4 - SCIB_TX_INT

//    PieCtrlRegs.PIEIER10.bit.INTx2 = 1; // PIE Group 10, INT2 - ADCA2_INT
//    PieCtrlRegs.PIEIER10.bit.INTx8 = 1; // PIE Group 10, INT8 - ADCB4_INT

    IER |=  M_INT1 | M_INT8 | M_INT9 | M_INT10 | M_INT13;
    EINT;

    EALLOW;

#if defined (BOARD_NEW)
    InputXbarRegs.INPUT4SELECT = RXD;     // X-Bar Input4 -> RXD
#elif defined (BOARD_PREVIOUS)
//  InputXbarRegs.INPUT4SELECT = TXA;     // X-Bar Input4 -> TXA
    InputXbarRegs.INPUT4SELECT = RXA;     // X-Bar Input4 -> RXA
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

//  InputXbarRegs.INPUT5SELECT = TXC;     // X-Bar Input5 -> TXC
    InputXbarRegs.INPUT5SELECT = RXC;     // X-Bar Input5 -> RXC
    InputXbarRegs.INPUT6SELECT = RXB;     // X-Bar Input6 -> RXB
    EDIS;

    XintRegs.XINT1CR.bit.ENABLE = 1;      // XINT1 - Enable
    XintRegs.XINT1CR.bit.POLARITY = 0x1;  // XINT1 - Polarity Conf.: 0x0, 0x2 -> Neg. Edge / 0x1 -> Pos. Edge / 0x3 -> Both

    XintRegs.XINT2CR.bit.ENABLE = 1;      // XINT2 - Enable
    XintRegs.XINT2CR.bit.POLARITY = 0x1;  // XINT2 - Polarity Conf.: 0x0, 0x2 -> Neg. Edge / 0x1 -> Pos. Edge / 0x3 -> Both

    XintRegs.XINT3CR.bit.ENABLE = 1;      // XINT3 - Enable
    XintRegs.XINT3CR.bit.POLARITY = 0x1;  // XINT3 - Polarity Conf.: 0x0, 0x2 -> Neg. Edge / 0x1 -> Pos. Edge / 0x3 -> Both
}

void togglePin(uint8_t pin){
    GPIO_WritePin(pin, !GPIO_ReadPin(pin));
}

void setPin(uint8_t pin, uint8_t value){
    GPIO_WritePin(pin, value);
}

uint8_t readPin(uint8_t pin){
    return GPIO_ReadPin(pin);
}

void sysInit(void){
    //System functions
    InitSysCtrl();
    DisablePeripheralClocks();

    EALLOW;

    //Speed up clock for faster ePWM 
    //ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0x0;

    //Initialize only clock of utilized peripherals 
    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER2 = 1;
    CpuSysRegs.PCLKCR0.bit.HRPWM     = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM1     = 1;

#if defined (BOARD_NEW)
    CpuSysRegs.PCLKCR7.bit.SCI_D     = 1;
#endif
#if defined (BOARD_PREVIOUS) || (defined(BOARD_NEW) && defined(TENSAO))
    CpuSysRegs.PCLKCR7.bit.SCI_A     = 1;
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    CpuSysRegs.PCLKCR7.bit.SCI_B     = 1;
    CpuSysRegs.PCLKCR7.bit.SCI_C     = 1;

    CpuSysRegs.PCLKCR8.bit.SPI_A     = 1;
    CpuSysRegs.PCLKCR9.bit.I2C_B     = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_A    = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_B    = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_C    = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_D    = 1;

    //Speed up clock for faster SCI communications
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;

    EDIS;
}

void GPIOInit(void){
    InitEPwm1Gpio();
    InitSpiaGpio();

    //Configura leds como out e apaga
    GPIO_SetupPinMux(LED1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(LED1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(LED2, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(LED2, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(LED3, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(LED3, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(54, GPIO_OUTPUT, GPIO_PUSHPULL);

    clr_LED1();clr_LED2();clr_LED3();

    //Configura pinos de debug (JP6)
    GPIO_SetupPinMux(DEBUG1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(DEBUG2, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG2, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(DEBUG3, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG3, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(DEBUG4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG4, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(DEBUG5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG5, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(DEBUG6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG6, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(DEBUG7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG7, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(DEBUG8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(DEBUG8, GPIO_OUTPUT, GPIO_PUSHPULL);

    clr_DEBUG1();clr_DEBUG2();clr_DEBUG3();clr_DEBUG4();
    clr_DEBUG5();clr_DEBUG6();clr_DEBUG7();clr_DEBUG8();

    //Configura pinos I2C LCD
    GPIO_SetupPinMux(SDAB, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(SDAB, GPIO_OUTPUT, GPIO_PULLUP | GPIO_ASYNC);
    GPIO_SetupPinMux(SCLB, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(SCLB, GPIO_OUTPUT, GPIO_PULLUP | GPIO_ASYNC);

#if defined (BOARD_NEW)
//SCI-D - FIBRA 2
    GPIO_SetupPinMux(RXD, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(RXD, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXD, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(TXD, GPIO_OUTPUT, GPIO_ASYNC);

    #if defined (TENSAO)
        GPIO_SetupPinMux(RXA, GPIO_MUX_CPU1, 1);
        GPIO_SetupPinOptions(RXA, GPIO_INPUT, GPIO_PUSHPULL);
        GPIO_SetupPinMux(TXA, GPIO_MUX_CPU1, 1);
        GPIO_SetupPinOptions(TXA, GPIO_OUTPUT, GPIO_ASYNC);
    #endif
#elif defined (BOARD_PREVIOUS)
//SCI-A - FIBRA 1
    GPIO_SetupPinMux(RXA, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(RXA, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXA, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(TXA, GPIO_OUTPUT, GPIO_ASYNC);
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    //SCI-B - USB
    GPIO_SetupPinMux(RXB, GPIO_MUX_CPU1, 3);
    GPIO_SetupPinOptions(RXB, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXB, GPIO_MUX_CPU1, 3);
    GPIO_SetupPinOptions(TXB, GPIO_OUTPUT, GPIO_ASYNC);

    //SCI-B - GPIO
//    GPIO_SetupPinMux(RXB, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(RXB, GPIO_INPUT, GPIO_PUSHPULL);
//    GPIO_SetupPinMux(TXB, GPIO_MUX_CPU1, 0);
//    GPIO_SetupPinOptions(TXB, GPIO_OUTPUT, GPIO_PULLUP);


#if defined (BOARD_NEW)
//SCI-C - FIBRA 2
    GPIO_SetupPinMux(RXC, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(RXC, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXC, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(TXC, GPIO_OUTPUT, GPIO_ASYNC);
#elif defined (BOARD_PREVIOUS)
//SCI-C - FIBRA 2
    GPIO_SetupPinMux(RXC, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(RXC, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXC, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(TXC, GPIO_OUTPUT, GPIO_ASYNC);
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    //GBIC 1
    GPIO_SetupPinMux(SFP1_TX_DIS, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(SFP1_TX_DIS, GPIO_OUTPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(SFP1_RX_FLT, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(SFP1_RX_FLT, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(SFP1_LOS, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(SFP1_LOS, GPIO_INPUT, GPIO_PUSHPULL);

    //GBIC 2
    GPIO_SetupPinMux(SFP2_TX_DIS, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(SFP2_TX_DIS, GPIO_OUTPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(SFP2_RX_FLT, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(SFP2_RX_FLT, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(SFP2_LOS, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(SFP2_LOS, GPIO_INPUT, GPIO_PUSHPULL);
}

void ConfigureEPWM(void){
    Uint32 bF = BASE_FREQ * 1E6;
    float32 spF = ( 1 / NUM_CYCLES ); //( TARGET_FREQ / NUM_CYCLES );
    float32 auux = spF * 100; // spF * RESULTS_BUFFER_SIZE

    float32 prd = bF/(2*auux);

    EALLOW;
    // Assumes ePWM clock is already enabled
    EPwm1Regs.ETSEL.bit.SOCAEN  = 0;   // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;   // Select SOCA on up-count
    EPwm1Regs.ETPS.bit.SOCAPRD  = 1;   // Generate pulse on 1st event

    EPwm1Regs.ETSEL.bit.SOCBEN  = 0;   // Disable SOC on B group
    EPwm1Regs.ETSEL.bit.SOCBSEL = 4;   // Select SOCB on up-count
    EPwm1Regs.ETPS.bit.SOCBPRD  = 2;   // Generate pulse on 2nd event

    EPwm1Regs.CMPA.bit.CMPA = (Uint16)(prd/2);
    EPwm1Regs.CMPB.bit.CMPB = (Uint16)(prd/2);
    EPwm1Regs.TBPRD =         (Uint16)(prd - 1);
    EPwm1Regs.TBCTL.bit.CTRMODE   = 3;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;

    prd = prd - (Uint32)prd;

    EPwm1Regs.CMPCTL.bit.LOADAMODE = 1;
    EPwm1Regs.HRCNFG.bit.HRLOAD    = 2;
    EPwm1Regs.HRCNFG.bit.AUTOCONV  = 1;
    EPwm1Regs.HRCNFG.bit.EDGMODE   = 3;
    EPwm1Regs.HRPCTL.bit.TBPHSHRLOADE = 1;
    EPwm1Regs.TBCTL.bit.PHSEN = 1;
    EPwm1Regs.HRPCTL.bit.HRPE = 1;
    EPwm1Regs.HRMSTEP.bit.HRMSTEP = 55;

    EPwm1Regs.TBPRDHR = (Uint16)(prd * 256) << 8;

    // freeze counter
    EDIS;
}

void config_ADC(void){
    EALLOW;

    //
    //write configurations
    //
    Uint16 PrescaleVal = 14; //set ADCCLK divider to /8
//    Uint16 PrescaleVal = 11; //set ADCCLK divider to /6.5
//    Uint16 PrescaleVal = 6; //set ADCCLK divider to /4
//    Uint16 PrescaleVal = 0; //set ADCCLK divider to /1

    AdcbRegs.ADCCTL2.bit.PRESCALE = PrescaleVal;
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    AdcdRegs.ADCCTL2.bit.PRESCALE = PrescaleVal;
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;

    Device_cal();
}

void SetupADCsEpwm(void){
        Uint16 acqps;

    //
    // Determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
//        acqps = 259; //1300ns
        acqps = 129; //650ns
//        acqps = 64;  //325ns
//        acqps = 44; //225ns
    }

    //
    //Select the channels to convert and end of conversion flag
    //
    EALLOW;

    //IP2 - PTP
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = acqps;   //sample window is acqps+1 SYSCLK cycles
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5;     //trigger on ePWM1 SOCA
//    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
//    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 0;   //enable INT1 flag
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //IN2 - PTN1
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1;
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = acqps;   //sample window is acqps+1 SYSCLK cycles
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 5;     //trigger on ePWM1 SOCA
//    AdcdRegs.ADCINTSEL1N2.bit.INT2SEL = 1; //end of SOC1 will set INT2 flag
//    AdcdRegs.ADCINTSEL1N2.bit.INT2E = 0;   //enable INT1 flag
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //make sure INT1 flag is cleared

    //IN1 - PTN2
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 3;
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = acqps;   //sample window is acqps+1 SYSCLK cycles
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 5;     //trigger on ePWM1 SOCB
//    AdcdRegs.ADCINTSEL3N4.bit.INT3SEL = 2; //end of SOC0 will set INT1 flag
//    AdcdRegs.ADCINTSEL3N4.bit.INT3E = 0;   //enable INT1 flag
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT3 = 1; //make sure INT1 flag is cleared

    //IP1 - 4-20
    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 2;
    AdcdRegs.ADCSOC3CTL.bit.ACQPS = acqps;   //sample window is acqps+1 SYSCLK cycles
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = 6;     //trigger on ePWM1 SOCB
//    AdcdRegs.ADCINTSEL3N4.bit.INT4SEL = 3; //end of SOC1 will set INT2 flag
//    AdcdRegs.ADCINTSEL3N4.bit.INT4E = 0;   //enable INT1 flag
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT4 = 1; //make sure INT1 flag is cleared

    //----------------------------------------------------------------------------------
    //ADC IC

    //IP4 - PTP
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 14;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;   //sample window is acqps+1 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;     //trigger on ePWM1 SOCA
//    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
//    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 0;   //enable INT1 flag
//    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //IN4 - PTN1
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 15;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;   //sample window is acqps+1 SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5;     //trigger on ePWM1 SOCA
//    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1; //end of SOC1 will set INT2 flag
//    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 0;   //enable INT1 flag
//    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //make sure INT1 flag is cleared

    //IN3 - PTN2
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 3;
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps;   //sample window is acqps+1 SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 5;     //trigger on ePWM1 SOCA
//    AdcbRegs.ADCINTSEL3N4.bit.INT3SEL = 2; //end of SOC0 will set INT1 flag
//    AdcbRegs.ADCINTSEL3N4.bit.INT3E = 0;   //enable INT1 flag
//    AdcbRegs.ADCINTFLGCLR.bit.ADCINT3 = 1; //make sure INT1 flag is cleared

    //IP3 - 4-20
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 2;
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is acqps+1 SYSCLK cycles
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 6;   //trigger on ePWM1 SOCB
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //end of SOC3 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    
    EDIS;
}

void tx_A_byte(uint8_t b){
    SciaRegs.SCITXBUF.all = b;
}

uint8_t rx_A_byte(){
    uint8_t rec;
    rec = SciaRegs.SCIRXBUF.all;
    return(rec);
}

void tx_B_byte(uint8_t b){
    ScibRegs.SCITXBUF.all = b;
}

uint8_t rx_B_byte(){
    uint8_t rec;
    rec = ScibRegs.SCIRXBUF.all;
    return(rec);
}

void tx_C_byte(uint8_t b){
    ScicRegs.SCITXBUF.all = b;
}

uint8_t rx_C_byte(){
    uint8_t rec;
    rec = ScicRegs.SCIRXBUF.all;
    return(rec);
}


//==============================================================================
// tx_D_byte - Transmite um byte pela SCI D
//==============================================================================
void tx_D_byte(uint8_t b)
{
    ScidRegs.SCITXBUF.all = b;
}

//==============================================================================
// rx_D_byte - Recebe um byte pela SCI D
//==============================================================================
uint8_t rx_D_byte()
{
    uint8_t rec;
    rec = ScidRegs.SCIRXBUF.all;
    return(rec);
}


void configTimer(void){
    InitCpuTimers();
    CpuTimer0Regs.PRD.all = 3600000000; //CPU Timer Period Register
    CpuTimer0Regs.TCR.bit.TRB = 1;      //CPU Timer Timer reload
    CpuTimer0Regs.TCR.bit.FREE = 1;     //CPU Timer Free Run
    CpuTimer0Regs.TCR.bit.TIE = 0;      //CPU Timer Interrupt Enable
    CpuTimer0Regs.TCR.bit.TIF = 1;      //CPU Timer Overflow Flag
    CpuTimer0Regs.TPR.all  = 200;       //CPU Timer Prescale Register
    CpuTimer0Regs.TPRH.all = 0;     //CPU Timer Prescale Register High
    CpuTimer0Regs.TCR.bit.TSS = 0;      //CPU Timer stop status bit


    CpuTimer1Regs.PRD.all = 200000; //CPU Timer Period Register
    CpuTimer1Regs.TCR.bit.TRB = 1;  //CPU Timer Timer reload
    CpuTimer1Regs.TCR.bit.FREE = 1; //CPU Timer Free Run
    CpuTimer1Regs.TCR.bit.TIE = 1;  //CPU Timer Interrupt Enable
    CpuTimer1Regs.TCR.bit.TIF = 1;  //CPU Timer Overflow Flag
    CpuTimer1Regs.TPR.all  = 0;     //CPU Timer Prescale Register
    CpuTimer1Regs.TPRH.all = 0;     //CPU Timer Prescale Register High
    CpuTimer1Regs.TCR.bit.TSS = 1;  //CPU Timer stop status bit

    CpuTimer2Regs.PRD.all = contador;
    CpuTimer2Regs.TCR.bit.TRB = 1;  //CPU Timer Timer reload
    CpuTimer2Regs.TCR.bit.FREE = 1; //CPU Timer Free Run
    CpuTimer2Regs.TCR.bit.TIE = 1;  //CPU Timer Interrupt Enable
    CpuTimer2Regs.TCR.bit.TIF = 1;  //CPU Timer Overflow Flag
    CpuTimer2Regs.TPR.all  = 0;     //CPU Timer Prescale Register
    CpuTimer2Regs.TPRH.all = 0;     //CPU Timer Prescale Register High
    CpuTimer2Regs.TCR.bit.TSS = 1;  //CPU Timer stop status bit

}

void resetTimer0(void){
    CpuTimer0Regs.TCR.bit.TRB = 1;
}

void configureSCI_sync(void){
#if defined (BOARD_NEW)
    GPIO_SetupPinMux(RXD, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RXD, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXD, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TXD, GPIO_OUTPUT, GPIO_PULLUP);

#if defined (TENSAO)
    GPIO_SetupPinMux(RXA, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RXA, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXA, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TXA, GPIO_OUTPUT, GPIO_PULLUP);
#endif

#elif defined (BOARD_PREVIOUS)
    GPIO_SetupPinMux(RXA, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RXA, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXA, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TXA, GPIO_OUTPUT, GPIO_PULLUP);

#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

    GPIO_SetupPinMux(RXC, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RXC, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXC, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TXC, GPIO_OUTPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(RXB, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RXC, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXB, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TXC, GPIO_OUTPUT, GPIO_PULLUP);
}

void configureSCI_UART(void){
#if defined (BOARD_NEW)
    GPIO_SetupPinMux(RXD, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(RXD, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXD, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(TXD, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(RXC, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(RXC, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXC, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(TXC, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(RXB, GPIO_MUX_CPU1, 3);
    GPIO_SetupPinOptions(RXB, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXB, GPIO_MUX_CPU1, 3);
    GPIO_SetupPinOptions(TXB, GPIO_OUTPUT, GPIO_ASYNC);


    #if defined (TENSAO)
        GPIO_SetupPinMux(RXA, GPIO_MUX_CPU1, 1);
        GPIO_SetupPinOptions(RXA, GPIO_INPUT, GPIO_PUSHPULL);
        GPIO_SetupPinMux(TXA, GPIO_MUX_CPU1, 1);
        GPIO_SetupPinOptions(TXA, GPIO_OUTPUT, GPIO_ASYNC);
    #endif

#elif defined (BOARD_PREVIOUS)
    GPIO_SetupPinMux(RXA, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(RXA, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXA, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(TXA, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(RXC, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(RXC, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXC, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(TXC, GPIO_OUTPUT, GPIO_ASYNC);
#else
    #error Necessario definir a placa - Nova (BOARD_NEW) ou Anterior (BOARD_PREVIOUS)
#endif

}

void startCapture(void){
    CPLD_WE(1);
    CPLD_AD_CONV();
}

uint32_t now(void){
// Temporizador de 64 bits
//
//    static uint64_t last_time = 0;
//    static uint32_t count_up = 0;
//    uint32_t actual_time = CpuTimer0Regs.TIM.all;
//    actual_time = actual_time & 0x0000FFFF;
//
//    uint64_t ret = (((uint64_t)count_up) << 32) | actual_time;
//
//    if (ret >= last_time)
//        last_time = ret;
//    else
//    {
//        count_up++;
//        ret = (((uint64_t)count_up) << 32) | actual_time;
//    }
//
//    return (ret);

    return (CpuTimer0Regs.TIM.all);
}

uint8_t endCapture(){
    return bufferFull;
}

void serial_A_Init(void){
    SciaRegs.SCICCR.all  = 0x0067;
    SciaRegs.SCICTL1.all = 0x0003;
//    SciaRegs.SCICTL2.bit.TXINTENA = 1;
//    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    // SCIA at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
//    SciaRegs.SCIHBAUD.all = 0x0002;
//    SciaRegs.SCILBAUD.all = 0x008B;

    // SCIA at 115,2k baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x36.
//    SciaRegs.SCIHBAUD.all = 0x0000;
//    SciaRegs.SCILBAUD.all = 0x0036;
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0xD9.
    SciaRegs.SCIHBAUD.all = 0x0000;
    SciaRegs.SCILBAUD.all = 0x00D9;

    // SCIA at 256k baud
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x62.
    //SciaRegs.SCIHBAUD.all = 0x0000;
    //SciaRegs.SCILBAUD.all = 0x0062;

    // SCIA at 1M baud
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x19.
//    SciaRegs.SCIHBAUD.all = 0x0000;
//    SciaRegs.SCILBAUD.all = 0x0019;

    // SCIA at 1,5M baud (1,563M)
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x10.
//    SciaRegs.SCIHBAUD.all = 0x0000;
//    SciaRegs.SCILBAUD.all = 0x0010;

    // SCIA at 2M baud (2,083M)
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x0C.
//    SciaRegs.SCIHBAUD.all = 0x0000;
//    SciaRegs.SCILBAUD.all = 0x000C;

    // SCIA at 12M baud
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) LSPCLKDIV = 0x00, HBAUD = 0x00 and LBAUD = 0x01.
//    SciaRegs.SCIHBAUD.all = 0x0000;
//    SciaRegs.SCILBAUD.all = 0x0001;
//
//    SciaRegs.SCICCR.bit.LOOPBKENA = 1; // Enable loop back

//    SciaRegs.SCIFFTX.all = 0xE040;
//    SciaRegs.SCIFFRX.all = 0x2044;
//    SciaRegs.SCIFFCT.all = 0x00;

    SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

void serial_B_Init(void){
    ScibRegs.SCICCR.all  = 0x0067;
    ScibRegs.SCICTL1.all = 0x0003;

//    ScibRegs.SCICTL2.bit.TXINTENA = 1;
//    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    // SCIB at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
//    ScibRegs.SCIHBAUD.all = 0x0002;
//    ScibRegs.SCILBAUD.all = 0x008B;
    // SCIB at 115,2k baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x36.
//    ScibRegs.SCIHBAUD.all = 0x0000;
//    ScibRegs.SCILBAUD.all = 0x0036;
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0xD9.
    ScibRegs.SCIHBAUD.all = 0x0000;
    ScibRegs.SCILBAUD.all = 0x00D9;
    // SCIB at 1M baud
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x19.
//    ScibRegs.SCIHBAUD.all = 0x0000;
//    ScibRegs.SCILBAUD.all = 0x0019;
    // SCIB at 2M baud (2,083M)
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x0C.
//    ScibRegs.SCIHBAUD.all = 0x0000;
//    ScibRegs.SCILBAUD.all = 0x000C;
    // SCIB at 12M baud
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) LSPCLKDIV = 0x00, HBAUD = 0x00 and LBAUD = 0x01.
//    ScibRegs.SCIHBAUD.all = 0x0000;
//    ScibRegs.SCILBAUD.all = 0x0001;
//
//    ScibRegs.SCICCR.bit.LOOPBKENA = 1; // Enable loop back

//    ScibRegs.SCIFFTX.all = 0xE040;
//    ScibRegs.SCIFFRX.all = 0x2044;
//    ScibRegs.SCIFFCT.all = 0x00;

    ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void serial_C_Init(void){
    ScicRegs.SCICCR.all  = 0x0067;
    ScicRegs.SCICTL1.all = 0x0003;

//    ScicRegs.SCICTL2.bit.TXINTENA = 1;
//    ScicRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    // SCIC at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
//    ScicRegs.SCIHBAUD.all = 0x0002;
//    ScicRegs.SCILBAUD.all = 0x008B;
    // SCIC at 115,2k baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x36.
//    ScicRegs.SCIHBAUD.all = 0x0000;
//    ScicRegs.SCILBAUD.all = 0x0036;
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0xD9.
    ScicRegs.SCIHBAUD.all = 0x0000;
    ScicRegs.SCILBAUD.all = 0x00D9;
    // SCIC at 1M baud
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x19.
//    ScicRegs.SCIHBAUD.all = 0x0000;
//    ScicRegs.SCILBAUD.all = 0x0019;
    // SCIC at 2M baud (2,083M)
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x0C.
//    ScicRegs.SCIHBAUD.all = 0x0000;
//    ScicRegs.SCILBAUD.all = 0x000C;
    // SCIC at 12M baud
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) LSPCLKDIV = 0x00, HBAUD = 0x00 and LBAUD = 0x01.
//    ScicRegs.SCIHBAUD.all = 0x0000;
//    ScicRegs.SCILBAUD.all = 0x0001;
//
//    ScicRegs.SCICCR.bit.LOOPBKENA = 1; // Enable loop back

//    ScicRegs.SCIFFTX.all = 0xE040;
//    ScicRegs.SCIFFRX.all = 0x2044;
//    ScicRegs.SCIFFCT.all = 0x00;

    ScicRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}


//==============================================================================
// serial_D_Init - Inicializacao da SCI D
//==============================================================================
void serial_D_Init(void)
{
    ScidRegs.SCICCR.all  = 0x0067;
    ScidRegs.SCICTL1.all = 0x0003;

//    ScicRegs.SCICTL2.bit.TXINTENA = 1;
//    ScicRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    // SCIC at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
//    ScicRegs.SCIHBAUD.all = 0x0002;
//    ScicRegs.SCILBAUD.all = 0x008B;
    // SCIC at 115,2k baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x36.
//    ScicRegs.SCIHBAUD.all = 0x0000;
//    ScicRegs.SCILBAUD.all = 0x0036;
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0xD9.
    ScidRegs.SCIHBAUD.all = 0x0000;
    ScidRegs.SCILBAUD.all = 0x00D9;
    // SCIC at 1M baud
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x19.
//    ScicRegs.SCIHBAUD.all = 0x0000;
//    ScicRegs.SCILBAUD.all = 0x0019;
    // SCIC at 2M baud (2,083M)
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) HBAUD = 0x00 and LBAUD = 0x0C.
//    ScicRegs.SCIHBAUD.all = 0x0000;
//    ScicRegs.SCILBAUD.all = 0x000C;
    // SCIC at 12M baud
    // @LSPCLK = 200 MHz (200 MHz SYSCLK) LSPCLKDIV = 0x00, HBAUD = 0x00 and LBAUD = 0x01.
//    ScicRegs.SCIHBAUD.all = 0x0000;
//    ScicRegs.SCILBAUD.all = 0x0001;
//
//    ScicRegs.SCICCR.bit.LOOPBKENA = 1; // Enable loop back

//    ScicRegs.SCIFFTX.all = 0xE040;
//    ScicRegs.SCIFFRX.all = 0x2044;
//    ScicRegs.SCIFFCT.all = 0x00;

    ScidRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}


interrupt void smTX_A(void){
#if defined (BOARD_NEW) && defined(TENSAO)
    tx_interrupt(&tx_USB);
#else
    tx_interrupt(&tx_Fibra1);
#endif
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9; // Issue PIE ACK
}

interrupt void smRX_A(void){
    if(SciaRegs.SCIRXST.bit.BRKDT || SciaRegs.SCIRXST.bit.RXERROR){
        SciaRegs.SCICTL1.bit.SWRESET = 0;
        SciaRegs.SCICTL1.bit.SWRESET = 1;
//      USBInit();
        tx_A_byte(0xFF);
    }
    if(SciaRegs.SCIRXST.bit.RXRDY){
        uint8_t Received = SciaRegs.SCIRXBUF.all;
#if defined (BOARD_NEW) && defined(TENSAO)
        rx_interrupt(&rx_USB, Received);
#else
        rx_interrupt(&rx_Fibra1, Received);
#endif
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9; // Issue PIE ACK
}

interrupt void smTX_B(void){
    tx_interrupt(&tx_USB);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9; // Issue PIE ACK
}

interrupt void smRX_B(void){
    if(ScibRegs.SCIRXST.bit.BRKDT || ScibRegs.SCIRXST.bit.RXERROR){
        ScibRegs.SCICTL1.bit.SWRESET = 0;
        ScibRegs.SCICTL1.bit.SWRESET = 1;
//        serialInit();
        tx_B_byte(0xFF);
    }
    if(ScibRegs.SCIRXST.bit.RXRDY){
        uint8_t Received = ScibRegs.SCIRXBUF.all;
        rx_interrupt(&rx_USB, Received);
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9; // Issue PIE ACK
}

interrupt void smTX_C(void){
    tx_interrupt(&tx_Fibra2);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8; // Issue PIE ACK
}

interrupt void smRX_C(void){
    if(ScicRegs.SCIRXST.bit.BRKDT || ScicRegs.SCIRXST.bit.RXERROR){
        ScicRegs.SCICTL1.bit.SWRESET = 0;
        ScicRegs.SCICTL1.bit.SWRESET = 1;

        tx_C_byte(0xFF);
    }
    if(ScicRegs.SCIRXST.bit.RXRDY){
        uint8_t Received = ScicRegs.SCIRXBUF.all;
        rx_interrupt(&rx_Fibra2, Received);
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8; // Issue PIE ACK
}


//==============================================================================
// smTX_D - Rotina tratamento interrupcao TX SCI D
//==============================================================================
interrupt void smTX_D(void)
{
    tx_interrupt(&tx_Fibra1);

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8; // Issue PIE ACK
}


//==============================================================================
// smRX_D - Rotina tratamento interrupcao RX SCI D
//==============================================================================
interrupt void smRX_D(void)
{
    uint8_t Received;

    if(ScidRegs.SCIRXST.bit.BRKDT || ScidRegs.SCIRXST.bit.RXERROR)
    {
        // Limpa o buffer...
        Received = ScidRegs.SCIRXBUF.all;

        ScidRegs.SCICTL1.bit.SWRESET = 0;
        ScidRegs.SCICTL1.bit.SWRESET = 1;

        tx_D_byte(0xFF);
    }

    if(ScidRegs.SCIRXST.bit.RXRDY)
    {
        Received = ScidRegs.SCIRXBUF.all;

        rx_interrupt(&rx_Fibra1, Received);
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8; // Issue PIE ACK
}


//volatile int estado_ant = 0, estado_at = 0, start_save = 0;

interrupt void readADC(void){
    if(Temp_index == 0){
        set_LED2();
    }

    Temp_Results.PTP_1[Temp_index]  = AdcdResultRegs.ADCRESULT0;
    Temp_Results.PTN1_1[Temp_index] = AdcdResultRegs.ADCRESULT1;
    Temp_Results.S420_1[Temp_index] = AdcdResultRegs.ADCRESULT2;
    Temp_Results.PTN2_1[Temp_index] = AdcdResultRegs.ADCRESULT3;

    Temp_Results.PTP_2[Temp_index]  = AdcbResultRegs.ADCRESULT0;
    Temp_Results.PTN1_2[Temp_index] = AdcbResultRegs.ADCRESULT1;
    Temp_Results.S420_2[Temp_index] = AdcbResultRegs.ADCRESULT2;
    Temp_Results.PTN2_2[Temp_index] = AdcbResultRegs.ADCRESULT3;

    Temp_index++;


    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    if(1 == AdcbRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    if(Temp_index == 100){
        bufferFull = 1;
        EPwm1Regs.TBCTL.bit.CTRMODE = 3; //freeze counter
        EPwm1Regs.ETSEL.bit.SOCAEN = 0;  //disable SOCA
        EPwm1Regs.ETSEL.bit.SOCBEN = 0;  //disable SOCA
        EPwm1Regs.TBCTR = 0x0000;
        Temp_index = 0;
        clr_LED2();
    }
}

void setA138(uint16_t index, uint16_t value){
    ADC_Results.A138[index] = value;
}
void setB138(uint16_t index, uint16_t value){
    ADC_Results.B138[index] = value;
}
void setC138(uint16_t index, uint16_t value){
    ADC_Results.C138[index] = value;
}
void setA230(uint16_t index, uint16_t value){
    ADC_Results.A230[index] = value;
}
void setB230(uint16_t index, uint16_t value){
    ADC_Results.B230[index] = value;
}
void setC230(uint16_t index, uint16_t value){
    ADC_Results.C230[index] = value;
}

void quickSort(uint16_t vet[], int16_t esq, int16_t dir) {
    int16_t pivo = esq, i, ch, j;
    for (i = esq + 1; i <= dir; i++) {
        j = i;
        if (vet[j] < vet[pivo]) {
            ch = vet[j];
            while (j > pivo) {
                vet[j] = vet[j - 1];
                j--;
            }
            vet[j] = ch;
            pivo++;
        }
    }
    if (pivo - 1 >= esq) {
        quickSort(vet, esq, pivo - 1);
    }
    if (pivo + 1 <= dir) {
        quickSort(vet, pivo + 1, dir);
    }
}

float64 tempNTC(float res, uint16_t adc1, uint16_t adc2){
    const float64 beta = 3450.0;
    const float64 r0 = 330.0;
    const float64 t0 = 273.0 + 25.0;
    const float64 rx = r0 * expl(-beta/t0);

    float64 vcc = analogEq(adc2, 12);
    float64 R = res;

    float64 v = analogEq(adc1, 12);
    float64 rt = (vcc*R)/v - R;

    float64 t = beta / logl(rt/rx);

    return (t-273.0);
}
