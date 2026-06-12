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
volatile uint32_t delay_T4 = 0;

uint16_t acquisition_counter = 0;

uint32_t delay_v_T1[syncMax];
uint32_t delay_v_T2[syncMax];
uint32_t delay_v_T3[syncMax];
uint32_t delay_v_T4[syncMax];

volatile uint16_t delayF = 0;

volatile t_adc_results ADC_Results;
volatile t_temp_data Temp_Results;

CiseiRxChannel rx_USB;
CiseiTxChannel tx_USB;

CiseiRxChannel rx_Fibra1;
CiseiTxChannel tx_Fibra1;

CiseiRxChannel rx_Fibra2;
CiseiTxChannel tx_Fibra2;

CiseiRxChannel rx_Fibra3;
CiseiTxChannel tx_Fibra3;

CiseiRxChannel rx_Fibra4;
CiseiTxChannel tx_Fibra4;

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


interrupt void syncPulse(){
    delay_T1 = CpuTimer1Regs.TIM.all;
    if(delayF){
        GpioDataRegs.GPBDAT.bit.GPIO47 = 1; // Placa Nova
    }
    if(startCap){
        GPIO_WritePin(CONVST, 1);
        GPIO_WritePin(54, 0);
        acquisition_counter++;
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

interrupt void xint4_isr(void){
    delay_T4 = CpuTimer1Regs.TIM.all;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12; // Issue PIE ACK
}

void activateUART_Ints(){

    ScidRegs.SCICTL2.bit.TXINTENA   = 1;
    ScidRegs.SCICTL2.bit.RXBKINTENA = 1;

    SciaRegs.SCICTL2.bit.TXINTENA   = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

    ScibRegs.SCICTL2.bit.TXINTENA   = 1;
    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;

    ScicRegs.SCICTL2.bit.TXINTENA   = 1;
    ScicRegs.SCICTL2.bit.RXBKINTENA = 1;
}

void TXInts(uint8_t enable){

    PieCtrlRegs.PIEIER8.bit.INTx8   = enable;   // SCI.D TX
    PieCtrlRegs.PIEIER9.bit.INTx2   = enable;   // SCI.A TX
    PieCtrlRegs.PIEIER9.bit.INTx4   = enable;   // SCI.B TX
    PieCtrlRegs.PIEIER8.bit.INTx6   = enable;   // SCI.C TX

}

void RXInts(uint8_t enable){

    PieCtrlRegs.PIEIER8.bit.INTx7   = enable; // SCI.D RX
    PieCtrlRegs.PIEIER9.bit.INTx1   = enable; // SCI.A RX
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

    PieVectTable.SCIA_RX_INT = &smRX_A;
    PieVectTable.SCIA_TX_INT = &smTX_A;

    PieVectTable.SCIB_RX_INT = &smRX_B;
    PieVectTable.SCIB_TX_INT = &smTX_B;

    PieVectTable.SCIC_RX_INT = &smRX_C;
    PieVectTable.SCIC_TX_INT = &smTX_C;

    PieVectTable.SCID_RX_INT = &smRX_D; // Interrupcao de Recepcao (RX) SCI D (por Almeida)
    PieVectTable.SCID_TX_INT = &smTX_D; // Interrupcao de Transmissao (TX) SCI D (por Almeida)

    PieVectTable.XINT1_INT = &syncPulse;
    PieVectTable.XINT2_INT = &xint2_isr;
    PieVectTable.XINT3_INT = &xint3_isr;
    PieVectTable.XINT4_INT = &xint4_isr;
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

    PieCtrlRegs.PIEIER8.bit.INTx7 = 1;  // PIE Group 8, INT7 - SCID_RX_INT
    PieCtrlRegs.PIEIER8.bit.INTx8 = 1;  // PIE Group 8, INT8 - SCID_TX_INT

    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;  // PIE Group 9, INT1 - SCIA_RX_INT
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;  // PIE Group 9, INT2 - SCIA_TX_INT

    //NAO UTILIZADO NO MOMENTO
//    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;  // PIE Group 9, INT3 - SCIB_RX_INT
//    PieCtrlRegs.PIEIER9.bit.INTx4 = 1;  // PIE Group 9, INT4 - SCIB_TX_INT

//    PieCtrlRegs.PIEIER10.bit.INTx2 = 1; // PIE Group 10, INT2 - ADCA2_INT
//    PieCtrlRegs.PIEIER10.bit.INTx8 = 1; // PIE Group 10, INT8 - ADCB4_INT

    IER |=  M_INT1 | M_INT8 | M_INT9 | M_INT10 | M_INT13;
    EINT;

    EALLOW;
    InputXbarRegs.INPUT4SELECT = RXA;     // X-Bar Input4 -> RXA
    InputXbarRegs.INPUT5SELECT = RXC;     // X-Bar Input5 -> RXC
    InputXbarRegs.INPUT4SELECT = RXD;     // X-Bar Input4 -> RXD
    InputXbarRegs.INPUT6SELECT = RXB;     // X-Bar Input6 -> RXB
    
    EDIS;

    XintRegs.XINT1CR.bit.ENABLE = 1;      // XINT1 - Enable
    XintRegs.XINT1CR.bit.POLARITY = 0x1;  // XINT1 - Polarity Conf.: 0x0, 0x2 -> Neg. Edge / 0x1 -> Pos. Edge / 0x3 -> Both

    XintRegs.XINT2CR.bit.ENABLE = 1;      // XINT2 - Enable
    XintRegs.XINT2CR.bit.POLARITY = 0x1;  // XINT2 - Polarity Conf.: 0x0, 0x2 -> Neg. Edge / 0x1 -> Pos. Edge / 0x3 -> Both

    XintRegs.XINT3CR.bit.ENABLE = 1;      // XINT3 - Enable
    XintRegs.XINT3CR.bit.POLARITY = 0x1;  // XINT3 - Polarity Conf.: 0x0, 0x2 -> Neg. Edge / 0x1 -> Pos. Edge / 0x3 -> Both

    XintRegs.XINT4CR.bit.ENABLE = 1;      // XINT4 - Enable
    XintRegs.XINT4CR.bit.POLARITY = 0x1;  // XINT4 - Polarity Conf.: 0x0, 0x2 -> Neg. Edge / 0x1 -> Pos. Edge / 0x3 -> Both
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

    CpuSysRegs.PCLKCR7.bit.SCI_A     = 1;
    CpuSysRegs.PCLKCR7.bit.SCI_B     = 1;
    CpuSysRegs.PCLKCR7.bit.SCI_C     = 1;
    CpuSysRegs.PCLKCR7.bit.SCI_D     = 1;
    
    //CpuSysRegs.PCLKCR8.bit.SPI_A     = 1;
    //CpuSysRegs.PCLKCR9.bit.I2C_B     = 1;

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


    GPIO_SetupPinMux(RXD, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(RXD, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXD, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(TXD, GPIO_OUTPUT, GPIO_ASYNC);
  
    GPIO_SetupPinMux(RXA, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(RXA, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXA, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(TXA, GPIO_OUTPUT, GPIO_ASYNC);
    
    GPIO_SetupPinMux(RXB, GPIO_MUX_CPU1, 3);
    GPIO_SetupPinOptions(RXB, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXB, GPIO_MUX_CPU1, 3);
    GPIO_SetupPinOptions(TXB, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(RXC, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(RXC, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXC, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(TXC, GPIO_OUTPUT, GPIO_ASYNC);

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

    GPIO_SetupPinMux(RXA, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RXA, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXA, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TXA, GPIO_OUTPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(RXB, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RXC, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXB, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TXC, GPIO_OUTPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(RXC, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RXC, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXC, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TXC, GPIO_OUTPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(RXD, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RXD, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXD, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TXD, GPIO_OUTPUT, GPIO_PULLUP);

}

void configureSCI_UART(void){

    GPIO_SetupPinMux(RXA, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(RXA, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXA, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(TXA, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(RXB, GPIO_MUX_CPU1, 3);
    GPIO_SetupPinOptions(RXB, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXB, GPIO_MUX_CPU1, 3);
    GPIO_SetupPinOptions(TXB, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(RXC, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(RXC, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXC, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(TXC, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(RXD, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(RXD, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXD, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(TXD, GPIO_OUTPUT, GPIO_ASYNC);

}

void startCapture(void){
    CPLD_WE(1);
    CPLD_AD_CONV();
}

uint32_t now(void){

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

    tx_interrupt(&tx_Fibra4);

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

        rx_interrupt(&rx_Fibra4, Received);

    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9; // Issue PIE ACK
}

interrupt void smTX_B(void){
    tx_interrupt(&tx_Fibra3);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9; // Issue PIE ACK
}

interrupt void smRX_B(void){
    if(ScibRegs.SCIRXST.bit.BRKDT || ScibRegs.SCIRXST.bit.RXERROR){
        ScibRegs.SCICTL1.bit.SWRESET = 0;
        ScibRegs.SCICTL1.bit.SWRESET = 1;
        tx_B_byte(0xFF);
    }
    if(ScibRegs.SCIRXST.bit.RXRDY){
        uint8_t Received = ScibRegs.SCIRXBUF.all;
        rx_interrupt(&rx_Fibra3, Received);
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


void phasors_int_to_float(CiseiRxChannel * rx_Fibra, tms320_board_data_t * board_data)
{
    if ((rx_getFrameType(&rx_Fibra) == CURRENT_PHASOR_X) && (rx_Fibra->frameReceived))
    {
        uint32_to_float_t conv_to_float;

        //Real part conversion to float of channel 1
        conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[3]) << 24) | 
                                       (((uint32_t)rx_Fibra->pBuffer[2]) << 16) |
                                       (((uint32_t)rx_Fibra->pBuffer[1]) << 8) | 
                                       (((uint32_t)rx_Fibra->pBuffer[0]));
        board_data->channel1[0] = conv_to_float.retangular_float;
        //CONVERTING TABLE LUTS OF FPGA TO FLOAT VALUE OF COS TO FINAL IMAGINARY PART OF PHASOR
        board_data->channel1[0] = board_data->channel1[0]/(LUT_FLOAT_FACTOR*RESULTS_BUFFER_SIZE);
        //Imaginary part conversion to float of channel 1
        conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[7]) << 24) | 
                                       (((uint32_t)rx_Fibra->pBuffer[6]) << 16) |
                                       (((uint32_t)rx_Fibra->pBuffer[5]) << 8) | 
                                       (((uint32_t)rx_Fibra->pBuffer[4]));
        board_data->channel1[1] = conv_to_float.retangular_float;
        //CONVERTING TABLE LUTS OF FPGA TO FLOAT VALUE OF SIN AND TO FINAL IMAGINARY PART OF PHASOR 
        board_data->channel1[1] = board_data->channel1[1]/(LUT_FLOAT_FACTOR*RESULTS_BUFFER_SIZE);
        
        //Real part conversion to float of channel 2
        conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[11]) << 24) | 
                                       (((uint32_t)rx_Fibra->pBuffer[10]) << 16) |
                                       (((uint32_t)rx_Fibra->pBuffer[9]) << 8) | 
                                       (((uint32_t)rx_Fibra->pBuffer[8]));
        board_data->channel2[0] = conv_to_float.retangular_float;
        //CONVERTING TABLE LUTS OF FPGA TO FLOAT VALUE OF COS TO FINAL IMAGINARY PART OF PHASOR
        board_data->channel2[0] = board_data->channel2[0]/(LUT_FLOAT_FACTOR*RESULTS_BUFFER_SIZE);
        //Imaginary part conversion to float of channel 2
        conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[15]) << 24) | 
                                       (((uint32_t)rx_Fibra->pBuffer[14]) << 16) |
                                       (((uint32_t)rx_Fibra->pBuffer[13]) << 8) | 
                                       (((uint32_t)rx_Fibra->pBuffer[12]));
        board_data->channel2[1] = conv_to_float.retangular_float;
        //CONVERTING TABLE LUTS OF FPGA TO FLOAT VALUE OF SIN AND TO FINAL IMAGINARY PART OF PHASOR 
        board_data->channel2[1] = board_data->channel2[1]/(LUT_FLOAT_FACTOR*RESULTS_BUFFER_SIZE);
        
        //Real part conversion to float of channel 3
        conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[19]) << 24) | 
                                       (((uint32_t)rx_Fibra->pBuffer[18]) << 16) |
                                       (((uint32_t)rx_Fibra->pBuffer[17]) << 8) | 
                                       (((uint32_t)rx_Fibra->pBuffer[16]));
        board_data->channel3[0] = conv_to_float.retangular_float;
        //CONVERTING TABLE LUTS OF FPGA TO FLOAT VALUE OF COS TO FINAL IMAGINARY PART OF PHASOR
        board_data->channel3[0] = board_data->channel3[0]/(LUT_FLOAT_FACTOR*RESULTS_BUFFER_SIZE);
        //Imaginary part conversion to float of channel 3
        conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[23]) << 24) | 
                                       (((uint32_t)rx_Fibra->pBuffer[22]) << 16) |
                                       (((uint32_t)rx_Fibra->pBuffer[21]) << 8) | 
                                       (((uint32_t)rx_Fibra->pBuffer[20]));
        board_data->channel3[1] = conv_to_float.retangular_float;
        //CONVERTING TABLE LUTS OF FPGA TO FLOAT VALUE OF SIN AND TO FINAL IMAGINARY PART OF PHASOR 
        board_data->channel3[1] = board_data->channel3[1]/(LUT_FLOAT_FACTOR*RESULTS_BUFFER_SIZE);

        //Real part conversion to float of channel 4
        conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[27]) << 24) | 
                                       (((uint32_t)rx_Fibra->pBuffer[26]) << 16) |
                                       (((uint32_t)rx_Fibra->pBuffer[25]) << 8) | 
                                       (((uint32_t)rx_Fibra->pBuffer[24]));
        board_data->channel4[0] = conv_to_float.retangular_float;
        //CONVERTING TABLE LUTS OF FPGA TO FLOAT VALUE OF COS TO FINAL IMAGINARY PART OF PHASOR
        board_data->channel4[0] = board_data->channel4[0]/(LUT_FLOAT_FACTOR*RESULTS_BUFFER_SIZE);
        //Imaginary part conversion to float of channel 4
        conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[31]) << 24) | 
                                       (((uint32_t)rx_Fibra->pBuffer[30]) << 16) |
                                       (((uint32_t)rx_Fibra->pBuffer[29]) << 8) | 
                                       (((uint32_t)rx_Fibra->pBuffer[28]));
        board_data->channel4[1] = conv_to_float.retangular_float;
        //CONVERTING TABLE LUTS OF FPGA TO FLOAT VALUE OF SIN AND TO FINAL IMAGINARY PART OF PHASOR 
        board_data->channel4[1] = board_data->channel4[1]/(LUT_FLOAT_FACTOR*RESULTS_BUFFER_SIZE);

        //Real part conversion to float of channel 5
        conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[35]) << 24) | 
                                       (((uint32_t)rx_Fibra->pBuffer[34]) << 16) |
                                       (((uint32_t)rx_Fibra->pBuffer[33]) << 8) | 
                                       (((uint32_t)rx_Fibra->pBuffer[32]));
        board_data->channel5[0] = conv_to_float.retangular_float;
        //CONVERTING TABLE LUTS OF FPGA TO FLOAT VALUE OF COS TO FINAL IMAGINARY PART OF PHASOR
        board_data->channel5[0] = board_data->channel5[0]/(LUT_FLOAT_FACTOR*RESULTS_BUFFER_SIZE);
        //Imaginary part conversion to float of channel 5
        conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[39]) << 24) | 
                                       (((uint32_t)rx_Fibra->pBuffer[38]) << 16) |
                                       (((uint32_t)rx_Fibra->pBuffer[37]) << 8) | 
                                       (((uint32_t)rx_Fibra->pBuffer[36]));
        board_data->channel5[1] = conv_to_float.retangular_float;
        //CONVERTING TABLE LUTS OF FPGA TO FLOAT VALUE OF SIN AND TO FINAL IMAGINARY PART OF PHASOR 
        board_data->channel5[1] = board_data->channel5[1]/(LUT_FLOAT_FACTOR*RESULTS_BUFFER_SIZE);

        //Real part conversion to float of channel 6
        conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[43]) << 24) | 
                                       (((uint32_t)rx_Fibra->pBuffer[42]) << 16) |
                                       (((uint32_t)rx_Fibra->pBuffer[41]) << 8) | 
                                       (((uint32_t)rx_Fibra->pBuffer[40]));
        board_data->channel6[0] = conv_to_float.retangular_float;
        //CONVERTING TABLE LUTS OF FPGA TO FLOAT VALUE OF COS TO FINAL IMAGINARY PART OF PHASOR
        board_data->channel6[0] = board_data->channel6[0]/(LUT_FLOAT_FACTOR*RESULTS_BUFFER_SIZE);
        //Imaginary part conversion to float of channel 6
        conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[47]) << 24) | 
                                       (((uint32_t)rx_Fibra->pBuffer[46]) << 16) |
                                       (((uint32_t)rx_Fibra->pBuffer[45]) << 8) | 
                                       (((uint32_t)rx_Fibra->pBuffer[44]));
        board_data->channel6[1] = conv_to_float.retangular_float;
        //CONVERTING TABLE LUTS OF FPGA TO FLOAT VALUE OF SIN AND TO FINAL IMAGINARY PART OF PHASOR 
        board_data->channel6[1] = board_data->channel6[1]/(LUT_FLOAT_FACTOR*RESULTS_BUFFER_SIZE);

        board_data->alarm = 0;
        board_data->status = 1;
    }

    else
    {
        //NO_FRAME
        if((rx_getFrameType(&rx_Fibra) != CURRENT_PHASOR_X) && (!rx_checkframeReceived(&rx_Fibra)))
        {
            board_data->alarm = NO_FRAME;
        }
        //INCOMPLETE_FRAME
        else if((rx_getFrameType(&rx_Fibra) == CURRENT_PHASOR_X) && (!rx_checkframeReceived(&rx_Fibra)))
        {
            board_data->alarm = INCOMPLETE_FRAME;
        }
        //WRONG_FRAME
        else if((rx_checkframeReceived(&rx_Fibra)) && (rx_getFrameType(&rx_Fibra) != CURRENT_PHASOR_X))
        {
            board_data->alarm = WRONG_FRAME;            
        }
    }
    
}

void phasors_ret_to_polar(tms320_board_data_t * board_data)
{

    float Xre1 = board_data->channel1[0];
    float Xim1 = board_data->channel1[1];
    float Xre2 = board_data->channel2[0]; 
    float Xim2 = board_data->channel2[1];
    float Xre3 = board_data->channel3[0]; 
    float Xim3 = board_data->channel3[1];
    float Xre4 = board_data->channel4[0]; 
    float Xim4 = board_data->channel4[1];
    float Xre5 = board_data->channel5[0];
    float Xim5 = board_data->channel5[1];
    float Xre6 = board_data->channel6[0];
    float Xim6 = board_data->channel6[1];

    //Store phasors modules
    board_data->channel1[0] =  (float)(sqrtl(powl(Xre1, 2.0) + powl(Xim1, 2.0)));
    board_data->channel2[0] =  (float)(sqrtl(powl(Xre2, 2.0) + powl(Xim2, 2.0)));
    board_data->channel3[0] =  (float)(sqrtl(powl(Xre3, 2.0) + powl(Xim3, 2.0)));
    board_data->channel4[0] =  (float)(sqrtl(powl(Xre4, 2.0) + powl(Xim4, 2.0)));
    board_data->channel5[0] =  (float)(sqrtl(powl(Xre5, 2.0) + powl(Xim5, 2.0)));
    board_data->channel6[0] =  (float)(sqrtl(powl(Xre6, 2.0) + powl(Xim6, 2.0)));

    //Store phasors phases
    board_data->channel1[1] = (float)(atan2l(Xim1, Xre1)*180)/M_PI;
    board_data->channel2[1] = (float)(atan2l(Xim2, Xre2)*180)/M_PI;
    board_data->channel3[1] = (float)(atan2l(Xim3, Xre3)*180)/M_PI;
    board_data->channel4[1] = (float)(atan2l(Xim4, Xre4)*180)/M_PI;
    board_data->channel5[1] = (float)(atan2l(Xim5, Xre5)*180)/M_PI;
    board_data->channel6[1] = (float)(atan2l(Xim6, Xre6)*180)/M_PI;

}

void ads1118_int_to_float(CiseiRxChannel * rx_Fibra, tms320_board_data_t * board_data)
{
    if ((rx_getFrameType(&rx_Fibra) == CURRENT_PHASOR_X) && (rx_Fibra->frameReceived))
    {
        crc16_init();
        uint16_t CRC_calc;

        uint16_to_float_t conv_to_float;
        uint8_t i;
        for(i = 0; i < TMS320_ANALOG_FLOAT_COUNT;i++){
        
            crc16_data(rx_Fibra->pBuffer[48 + 2*i]);
            CRC_calc = crc16_data(rx_Fibra->pBuffer[49 + 2*i]);

            conv_to_float.ads1118_int = (((uint16_t)rx_Fibra->pBuffer[49 + 2*i]) << 8) | 
                                       (((uint16_t)rx_Fibra->pBuffer[48 + 2*i]));
            board_data->analog[i] = conv_to_float.ads1118_float;

        }

        conv_to_float.ads1118_int = (((uint16_t)rx_Fibra->pBuffer[59]) << 8) | 
                                       (((uint16_t)rx_Fibra->pBuffer[58]));

        if(CRC_calc != conv_to_float.ads1118_int)
        {
            board_data->alarm = ADS1118_CRC_NACK;
        }
    }
    rx_checkframeReceived(&rx_Fibra);
}

void tms320_frame_crc(tms320_data_t * tms320_data, tms320_uart_frame_t * tms320_uart_frame)
{
    crc16_init();
    uint8_t i;
    uint8_t j;
    uint32_t tmp_1;
    uint32_t tmp_2;
    uint16_t CRC_calc;

    for(i = 0; i < TMS320_BOARD_COUNT; i++)
    {
        memcpy(&tmp_1, &(tms320_data->boards[i].channel1[0]), sizeof(tmp_1));
        memcpy(&tmp_2, &(tms320_data->boards[i].channel1[1]), sizeof(tmp_2));
        crc16_data((uint16_t)((0x000000FF)&(tmp_1)));
        crc16_data((uint16_t)((0x0000FF00)&(tmp_1)));
        crc16_data((uint16_t)((0x00FF0000)&(tmp_1)));
        crc16_data((uint16_t)((0xFF000000)&(tmp_1)));
        crc16_data((uint16_t)((0x000000FF)&(tmp_2)));
        crc16_data((uint16_t)((0x0000FF00)&(tmp_2)));
        crc16_data((uint16_t)((0x00FF0000)&(tmp_2)));
        crc16_data((uint16_t)((0xFF000000)&(tmp_2)));

        memcpy(&tmp_1, &(tms320_data->boards[i].channel2[0]), sizeof(tmp_1));
        memcpy(&tmp_2, &(tms320_data->boards[i].channel2[1]), sizeof(tmp_2));
        crc16_data((uint16_t)((0x000000FF)&(tmp_1)));
        crc16_data((uint16_t)((0x0000FF00)&(tmp_1)));
        crc16_data((uint16_t)((0x00FF0000)&(tmp_1)));
        crc16_data((uint16_t)((0xFF000000)&(tmp_1)));
        crc16_data((uint16_t)((0x000000FF)&(tmp_2)));
        crc16_data((uint16_t)((0x0000FF00)&(tmp_2)));
        crc16_data((uint16_t)((0x00FF0000)&(tmp_2)));
        crc16_data((uint16_t)((0xFF000000)&(tmp_2)));

        memcpy(&tmp_1, &(tms320_data->boards[i].channel3[0]), sizeof(tmp_1));
        memcpy(&tmp_2, &(tms320_data->boards[i].channel3[1]), sizeof(tmp_2));
        crc16_data((uint16_t)((0x000000FF)&(tmp_1)));
        crc16_data((uint16_t)((0x0000FF00)&(tmp_1)));
        crc16_data((uint16_t)((0x00FF0000)&(tmp_1)));
        crc16_data((uint16_t)((0xFF000000)&(tmp_1)));
        crc16_data((uint16_t)((0x000000FF)&(tmp_2)));
        crc16_data((uint16_t)((0x0000FF00)&(tmp_2)));
        crc16_data((uint16_t)((0x00FF0000)&(tmp_2)));
        crc16_data((uint16_t)((0xFF000000)&(tmp_2)));

        memcpy(&tmp_1, &(tms320_data->boards[i].channel4[0]), sizeof(tmp_1));
        memcpy(&tmp_2, &(tms320_data->boards[i].channel4[1]), sizeof(tmp_2));
        crc16_data((uint16_t)((0x000000FF)&(tmp_1)));
        crc16_data((uint16_t)((0x0000FF00)&(tmp_1)));
        crc16_data((uint16_t)((0x00FF0000)&(tmp_1)));
        crc16_data((uint16_t)((0xFF000000)&(tmp_1)));
        crc16_data((uint16_t)((0x000000FF)&(tmp_2)));
        crc16_data((uint16_t)((0x0000FF00)&(tmp_2)));
        crc16_data((uint16_t)((0x00FF0000)&(tmp_2)));
        crc16_data((uint16_t)((0xFF000000)&(tmp_2)));

        memcpy(&tmp_1, &(tms320_data->boards[i].channel5[0]), sizeof(tmp_1));
        memcpy(&tmp_2, &(tms320_data->boards[i].channel5[1]), sizeof(tmp_2));
        crc16_data((uint16_t)((0x000000FF)&(tmp_1)));
        crc16_data((uint16_t)((0x0000FF00)&(tmp_1)));
        crc16_data((uint16_t)((0x00FF0000)&(tmp_1)));
        crc16_data((uint16_t)((0xFF000000)&(tmp_1)));
        crc16_data((uint16_t)((0x000000FF)&(tmp_2)));
        crc16_data((uint16_t)((0x0000FF00)&(tmp_2)));
        crc16_data((uint16_t)((0x00FF0000)&(tmp_2)));
        crc16_data((uint16_t)((0xFF000000)&(tmp_2)));

        memcpy(&tmp_1, &(tms320_data->boards[i].channel6[0]), sizeof(tmp_1));
        memcpy(&tmp_2, &(tms320_data->boards[i].channel6[1]), sizeof(tmp_2));
        crc16_data((uint16_t)((0x000000FF)&(tmp_1)));
        crc16_data((uint16_t)((0x0000FF00)&(tmp_1)));
        crc16_data((uint16_t)((0x00FF0000)&(tmp_1)));
        crc16_data((uint16_t)((0xFF000000)&(tmp_1)));
        crc16_data((uint16_t)((0x000000FF)&(tmp_2)));
        crc16_data((uint16_t)((0x0000FF00)&(tmp_2)));
        crc16_data((uint16_t)((0x00FF0000)&(tmp_2)));
        crc16_data((uint16_t)((0xFF000000)&(tmp_2)));

        for(j = 0; j < TMS320_ANALOG_FLOAT_COUNT; j++)
        {
            memcpy(&tmp_1, &(tms320_data->boards[i].analog[j]), sizeof(tmp_1));
            crc16_data((uint16_t)((0x000000FF)&(tmp_1)));
            crc16_data((uint16_t)((0x0000FF00)&(tmp_1)));
            crc16_data((uint16_t)((0x00FF0000)&(tmp_1)));
            CRC_calc = crc16_data((uint16_t)((0xFF000000)&(tmp_1)));
        }
    }
    memcpy(&(tms320_uart_frame->payload), &tms320_data, sizeof(tms320_data_t));
    tms320_uart_frame->crc = CRC_calc; 
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

