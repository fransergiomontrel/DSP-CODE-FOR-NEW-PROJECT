#include "common/sm_serial_api.h"
#include "common/datatypes.h"
#include "common/hal.h"
#include "pins.h"
#include "F28x_Project.h"
#include "common/sm_tensao.h"


//#include "F28377S/LCD_I2C.h"

#define contador 0xFFFFFFFF;

#define BAUD_TIME_SOFT_SERIAL 1101
#define ONE_HALF_BIT_TIME 1708

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

CiseiRxChannel_stm32 rx_USB;
CiseiTxChannel tx_USB;

CiseiRxChannel rx_Fibra0;
CiseiTxChannel tx_Fibra0;

CiseiRxChannel rx_Fibra1;
CiseiTxChannel tx_Fibra1;

CiseiRxChannel rx_Fibra2;
CiseiTxChannel tx_Fibra2;

CiseiRxChannel rx_Fibra3;
CiseiTxChannel tx_Fibra3;

CiseiRxChannel rx_Fibra4;
CiseiTxChannel tx_Fibra4;


//Global variable to transmit through soft serial
volatile uint8_t byte_global;
volatile uint8_t parity_bit;
//Global variable to receive through soft serial
uint8_t b = 0x00;
uint8_t rec_parity_bit = 0;

void syncInt_ena(){
    DINT;
    PieVectTable.XINT1_INT = &syncPulse;
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
    XintRegs.XINT1CR.bit.ENABLE = 1;
    EINT;
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

    //config_ADC();

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
    //resetResultBuffer();

//    tx_B_byte(0x01);
//    (tx_C_byte0x01);
//    tx_A_byte(0x01);
//    tx_D_byte(0x01);

  

    //InitSpi();
}

interrupt void syncPulse(){
    delay_T1 = CpuTimer1Regs.TIM.all;
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

    //PieCtrlRegs.PIEIER1.bit.INTx2 = 1;  // PIE Group 1, INT2 - ADCB1_INT
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
    InputXbarRegs.INPUT1SELECT = RXA;     // X-Bar Input4 -> RXA
    InputXbarRegs.INPUT2SELECT = RXB;     // X-Bar Input6 -> RXB
    InputXbarRegs.INPUT3SELECT = RXC;     // X-Bar Input5 -> RXC
    InputXbarRegs.INPUT4SELECT = RXD;     // X-Bar Input4 -> RXD
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

    //InitEPwm1Gpio();
    //InitSpiaGpio();

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

//==============================================================================
// tx_byte_soft - Transmite um byte pela software serial
//==============================================================================

void tx_byte_soft(uint8_t b)
{
    byte_global = b;       
    uint8_t i;
    parity_bit = (byte_global >> 0) & 0x01;
    CpuTimer2.InterruptCount = 0;

    for(i = 1; i < 8; i++)
    {
        parity_bit ^= (byte_global >> i) & 0x01;
    }
    
    CpuTimer2Regs.TCR.bit.TSS = 0;
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

    GPIO_SetupPinMux(RXA, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(RXA, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXA, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(TXA, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(RXB, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(RXB, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXB, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(TXB, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(RXC, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(RXC, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXC, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(TXC, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(RXD, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(RXD, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(TXD, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(TXD, GPIO_OUTPUT, GPIO_ASYNC);

    //Pins for uart by software
    GPIO_SetupPinMux(TX_STM_SOFT, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TX_STM_SOFT, GPIO_OUTPUT, GPIO_PUSHPULL | GPIO_ASYNC);
    GPIO_WritePin(TX_STM_SOFT, 1);
    GPIO_SetupPinMux(RX_STM_SOFT, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RX_STM_SOFT, GPIO_INPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(TX_FPGA_SOFT, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(TX_FPGA_SOFT, GPIO_OUTPUT, GPIO_PUSHPULL | GPIO_ASYNC);
    GPIO_WritePin(TX_FPGA_SOFT, 1);
    GPIO_SetupPinMux(RX_FPGA_SOFT, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RX_FPGA_SOFT, GPIO_INPUT, GPIO_ASYNC);

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

interrupt void timer2_tx_stm_isr(void){
   //Start bit
   if(CpuTimer2.InterruptCount == 0)
   {
       GpioDataRegs.GPCDAT.bit.GPIO82 = (byte_global >> CpuTimer2.InterruptCount) & 0x00;
   }
   //Data byte
   else if(CpuTimer2.InterruptCount < 9)
   {
       GpioDataRegs.GPCDAT.bit.GPIO82 = (byte_global >> (CpuTimer2.InterruptCount - 1)) & 0x01;
   }
   //Parity bit
   else if(CpuTimer2.InterruptCount == 9)
   {
       GpioDataRegs.GPCDAT.bit.GPIO82 = parity_bit;
   }
   //Stop bit
   else if(CpuTimer2.InterruptCount == 10)
   {
       GpioDataRegs.GPCDAT.bit.GPIO82 = 1;
       tx_interrupt(&tx_USB);
       CpuTimer2Regs.TCR.bit.TSS = 1;
   }
    ++CpuTimer2.InterruptCount;
    CpuTimer2Regs.TCR.bit.TIF = 1; // limpa flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    CpuTimer2Regs.TCR.bit.TRB = 1;  //CPU Timer Timer reload
}

interrupt void timer2_tx_fpga_isr(void){
   //Start bit
   if(CpuTimer2.InterruptCount == 0)
   {
       GpioDataRegs.GPADAT.bit.GPIO28 = (byte_global >> CpuTimer2.InterruptCount) & 0x00;
   }
   //Data byte
   else if(CpuTimer2.InterruptCount < 9)
   {
       GpioDataRegs.GPADAT.bit.GPIO28 = (byte_global >> (CpuTimer2.InterruptCount - 1)) & 0x01;
   }
   //Parity bit
   else if(CpuTimer2.InterruptCount == 9)
   {
       GpioDataRegs.GPADAT.bit.GPIO28 = parity_bit;
   }
   //Stop bit
   else if(CpuTimer2.InterruptCount == 10)
   {
       GpioDataRegs.GPADAT.bit.GPIO28 = 1;
       tx_interrupt(&tx_Fibra0);
       CpuTimer2Regs.TCR.bit.TSS = 1;
   }
    ++CpuTimer2.InterruptCount;
    CpuTimer2Regs.TCR.bit.TIF = 1; // limpa flag
    CpuTimer2Regs.TCR.bit.TRB = 1;  //CPU Timer Timer reload
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void serial_tx_to_stm_Init(void){

    CpuTimer2Regs.PRD.all = BAUD_TIME_SOFT_SERIAL;
    CpuTimer2Regs.TCR.bit.TRB = 1;  //CPU Timer Timer reload
    CpuTimer2Regs.TCR.bit.FREE = 1; //CPU Timer Free Run
    CpuTimer2Regs.TCR.bit.TIE = 1;  //CPU Timer Interrupt Enable
    CpuTimer2Regs.TCR.bit.TIF = 1;  //CPU Timer Overflow Flag
    CpuTimer2Regs.TPR.all  = 0;     //CPU Timer Prescale Register
    CpuTimer2Regs.TPRH.all = 0;     //CPU Timer Prescale Register High
    CpuTimer2Regs.TCR.bit.TSS = 1;  //CPU Timer stop status bit

    EALLOW;
    PieVectTable.TIMER2_INT = &timer2_tx_stm_isr;
    EDIS;

}

void serial_tx_to_fpga_Init(void){

    CpuTimer2Regs.PRD.all = BAUD_TIME_SOFT_SERIAL;
    CpuTimer2Regs.TCR.bit.TRB = 1;  //CPU Timer Timer reload
    CpuTimer2Regs.TCR.bit.FREE = 1; //CPU Timer Free Run
    CpuTimer2Regs.TCR.bit.TIE = 1;  //CPU Timer Interrupt Enable
    CpuTimer2Regs.TCR.bit.TIF = 1;  //CPU Timer Overflow Flag
    CpuTimer2Regs.TPR.all  = 0;     //CPU Timer Prescale Register
    CpuTimer2Regs.TPRH.all = 0;     //CPU Timer Prescale Register High
    CpuTimer2Regs.TCR.bit.TSS = 1;  //CPU Timer stop status bit

    EALLOW;
    PieVectTable.TIMER2_INT = &timer2_tx_fpga_isr;
    EDIS;

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


interrupt void xint1_isr(void)
{
    //Tratamento da interrupção
    CpuTimer2Regs.TCR.bit.TSS = 0;
    CpuTimer2.InterruptCount = 0;
    b = (unsigned char)0x00;
    // Desabilita XINT1
    XintRegs.XINT1CR.bit.ENABLE = 0;
    // Desabilita XINT1 dentro do PIE Group 1 / INTx4
    PieCtrlRegs.PIEIER1.bit.INTx4 = 0;
    CpuTimer2.InterruptCount = 0;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void timer2_rx_stm_isr(void){
   
    if(CpuTimer2.InterruptCount == 0)
    {
        b |= (((unsigned char)GpioDataRegs.GPCDAT.bit.GPIO83) << CpuTimer2.InterruptCount);
        CpuTimer2Regs.PRD.all = BAUD_TIME_SOFT_SERIAL;
    }
    else if((0 < CpuTimer2.InterruptCount) && (CpuTimer2.InterruptCount < 8))
    {
        b |= (((unsigned char)GpioDataRegs.GPCDAT.bit.GPIO83) << CpuTimer2.InterruptCount);
    }
    else if(CpuTimer2.InterruptCount == 8)
    {
        rec_parity_bit = GpioDataRegs.GPCDAT.bit.GPIO83;
    
        CpuTimer2Regs.TCR.bit.TSS = 1;
        
        CpuTimer2Regs.PRD.all = ONE_HALF_BIT_TIME;
        //Habilita XINT1
        XintRegs.XINT1CR.bit.ENABLE = 1;
        //Habilita canal da XINT1 no PIE
        PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
        rx_interrupt_stm32(&rx_USB, b);
    }
    
    ++CpuTimer2.InterruptCount;
    CpuTimer2Regs.TCR.bit.TIF = 1; // limpa flag
    CpuTimer2Regs.TCR.bit.TRB = 1;  //CPU Timer Timer reload
}

interrupt void timer2_rx_fpga_isr(void){
   
    if(CpuTimer2.InterruptCount == 0)
    {
        b |= (((unsigned char)GpioDataRegs.GPADAT.bit.GPIO30) << CpuTimer2.InterruptCount);
        CpuTimer2Regs.PRD.all = BAUD_TIME_SOFT_SERIAL;
    }
    else if((0 < CpuTimer2.InterruptCount) && (CpuTimer2.InterruptCount < 8))
    {
        b |= (((unsigned char)GpioDataRegs.GPADAT.bit.GPIO30) << CpuTimer2.InterruptCount);
    }
    else if(CpuTimer2.InterruptCount == 8)
    {
        rec_parity_bit = GpioDataRegs.GPADAT.bit.GPIO30;
    
        CpuTimer2Regs.TCR.bit.TSS = 1;
        
        CpuTimer2Regs.PRD.all = ONE_HALF_BIT_TIME;
        //Habilita XINT1
        XintRegs.XINT1CR.bit.ENABLE = 1;
        //Habilita canal da XINT1 no PIE
        PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
        rx_interrupt(&rx_Fibra0, b);
    }
    
    ++CpuTimer2.InterruptCount;
    CpuTimer2Regs.TCR.bit.TIF = 1; // limpa flag
    CpuTimer2Regs.TCR.bit.TRB = 1;  //CPU Timer Timer reload
}

void serial_rx_to_stm_Init(void){
    CpuTimer2Regs.PRD.all = ONE_HALF_BIT_TIME;
    CpuTimer2Regs.TCR.bit.TRB = 1;  //CPU Timer Timer reload
    CpuTimer2Regs.TCR.bit.FREE = 1; //CPU Timer Free Run
    CpuTimer2Regs.TCR.bit.TIE = 1;  //CPU Timer Interrupt Enable
    CpuTimer2Regs.TCR.bit.TIF = 1;  //CPU Timer Overflow Flag
    CpuTimer2Regs.TPR.all  = 0;     //CPU Timer Prescale Register
    CpuTimer2Regs.TPRH.all = 0;     //CPU Timer Prescale Register High
    CpuTimer2Regs.TCR.bit.TSS = 1;  //CPU Timer stop status bit
  
    EALLOW;
    PieVectTable.TIMER2_INT = &timer2_rx_stm_isr;
    // Vetor da XINT1
    PieVectTable.XINT1_INT = &xint1_isr;

    XintRegs.XINT1CR.bit.ENABLE = 0;       // desabilita XINT1 temporariamente
    PieCtrlRegs.PIEIER1.bit.INTx4 = 0;     // limpa PIE channel

    // Seleciona GPIO83 como fonte da XINT1
    InputXbarRegs.INPUT1SELECT = RX_STM_SOFT;   // exemplo: conecta GPIO83 ao XINT1
    XintRegs.XINT1CR.bit.POLARITY = 0; // borda de descida
    EDIS;
}

void serial_rx_to_fpga_Init(void){
    CpuTimer2Regs.PRD.all = ONE_HALF_BIT_TIME;
    CpuTimer2Regs.TCR.bit.TRB = 1;  //CPU Timer Timer reload
    CpuTimer2Regs.TCR.bit.FREE = 1; //CPU Timer Free Run
    CpuTimer2Regs.TCR.bit.TIE = 1;  //CPU Timer Interrupt Enable
    CpuTimer2Regs.TCR.bit.TIF = 1;  //CPU Timer Overflow Flag
    CpuTimer2Regs.TPR.all  = 0;     //CPU Timer Prescale Register
    CpuTimer2Regs.TPRH.all = 0;     //CPU Timer Prescale Register High
    CpuTimer2Regs.TCR.bit.TSS = 1;  //CPU Timer stop status bit
  
    EALLOW;
    PieVectTable.TIMER2_INT = &timer2_rx_fpga_isr;
    // Vetor da XINT1
    PieVectTable.XINT1_INT = &xint1_isr;

    XintRegs.XINT1CR.bit.ENABLE = 0;       // desabilita XINT1 temporariamente
    PieCtrlRegs.PIEIER1.bit.INTx4 = 0;     // limpa PIE channel

    // Seleciona GPIO83 como fonte da XINT1
    InputXbarRegs.INPUT1SELECT = RX_FPGA_SOFT;   // exemplo: conecta GPIO83 ao XINT1
    XintRegs.XINT1CR.bit.POLARITY = 0; // borda de descida
    EDIS;
}


void rx_byte_soft(void)
{
    XintRegs.XINT1CR.bit.ENABLE = 1;   // habilita XINT1
    //Habilita canal da XINT1 no PIE
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
}


void phasors_int_to_float(CiseiRxChannel * rx_Fibra, float phasors_data[])
{
    if ((rx_getFrameType(rx_Fibra) == CURRENT_PHASOR_X) && (rx_Fibra->frameReceived))
    {
        uint32_to_float_t conv_to_float;
        uint16_t channel;
        uint16_t component;
        uint16_t buffer_index;
        uint16_t phasor_index;

        for (channel = 0; channel < TMS320_CHANNEL_COUNT; channel++)
        {
            for (component = 0; component < TMS320_CHANNEL_FLOAT_COUNT; component++)
            {
                phasor_index = (channel * TMS320_CHANNEL_FLOAT_COUNT) + component;
                buffer_index = phasor_index * 4U;

                conv_to_float.retangular_int = (((uint32_t)rx_Fibra->pBuffer[buffer_index + 3U]) << 24) |
                                               (((uint32_t)rx_Fibra->pBuffer[buffer_index + 2U]) << 16) |
                                               (((uint32_t)rx_Fibra->pBuffer[buffer_index + 1U]) << 8) |
                                               (((uint32_t)rx_Fibra->pBuffer[buffer_index]));

                phasors_data[phasor_index] = conv_to_float.retangular_float;
                phasors_data[phasor_index] = phasors_data[phasor_index]/(LUT_FLOAT_FACTOR);
                phasors_data[phasor_index] = phasors_data[phasor_index]/(RESULTS_BUFFER_SIZE);
            }
        }
    }
    
}

void phasors_ret_to_polar(float phasors_data[])
{
    uint16_t channel;
    uint16_t real_index;
    uint16_t imag_index;
    float Xre;
    float Xim;

    for (channel = 0; channel < TMS320_CHANNEL_COUNT; channel++)
    {
        real_index = channel * TMS320_CHANNEL_FLOAT_COUNT;
        imag_index = real_index + 1U;

        Xre = phasors_data[real_index];
        Xim = phasors_data[imag_index];

        phasors_data[real_index] = (float)(sqrtl(powl(Xre, 2.0) + powl(Xim, 2.0)));
        phasors_data[imag_index] = (float)(atan2l(Xim, Xre)*180)/M_PI;
    }
}

void ads1118_int_to_float(CiseiRxChannel * rx_Fibra, float phasors_data[], int8_t info_data[])
{
    if ((rx_getFrameType(rx_Fibra) == CURRENT_PHASOR_X) && (rx_Fibra->frameReceived))
    {
        crc16_init();
        uint16_t CRC_calc;

        uint16_to_float_t conv_to_float;
        uint8_t analog_channel;
        for(analog_channel = 0; analog_channel < TMS320_ANALOG_FLOAT_COUNT; analog_channel++){

            crc16_data(rx_Fibra->pBuffer[48 + 2*analog_channel]);
            CRC_calc = crc16_data(rx_Fibra->pBuffer[49 + 2*analog_channel]);

            conv_to_float.ads1118_int = (((uint16_t)rx_Fibra->pBuffer[49 + 2*analog_channel]) << 8) | 
                                       (((uint16_t)rx_Fibra->pBuffer[48 + 2*analog_channel]));
            phasors_data[analog_channel + MASTER_DATA_ANALOG_OFFSET] = conv_to_float.ads1118_float;
        }
        conv_to_float.ads1118_int = (((uint16_t)rx_Fibra->pBuffer[59]) << 8) | 
                                       (((uint16_t)rx_Fibra->pBuffer[58]));

        if(CRC_calc != conv_to_float.ads1118_int)
        {
            info_data[0] = ADS1118_CRC_NACK;
        }
    }
    
}

uint16_t tms320_frame_crc(float phasors_data[][MASTER_DATA_PHASORS_COLS], int8_t info_data[][BOARD_PARAMETERS])
{
    uint8_t i;
    uint8_t j;
    uint32_t tmp_1;
    uint16_t CRC_calc;

    for(i = 0; i < TMS320_BOARD_COUNT; i++)
    {
        for(j = 0; j < MASTER_DATA_ANALOG_OFFSET; i++)
        {   
            memcpy(&tmp_1, &phasors_data[i][j], sizeof(tmp_1));
            crc16_data((uint16_t)((0x000000FF)&(tmp_1)));
            crc16_data((uint16_t)((0x000000FF)&(tmp_1>>8)));
            crc16_data((uint16_t)((0x000000FF)&(tmp_1>>16)));
            crc16_data((uint16_t)((0x000000FF)&(tmp_1>>24)));
        }
        for(j = MASTER_DATA_ANALOG_OFFSET; j < MASTER_DATA_PHASORS_COLS; j++)
        {
            memcpy(&tmp_1, &phasors_data[i][j], sizeof(tmp_1));
            crc16_data((uint16_t)((0x000000FF)&(tmp_1)));
            crc16_data((uint16_t)((0x000000FF)&(tmp_1>>8)));
            crc16_data((uint16_t)((0x000000FF)&(tmp_1>>16)));
            crc16_data((uint16_t)((0x000000FF)&(tmp_1>>24)));
        }
        crc16_data((uint16_t)(info_data[i][0]));
        CRC_calc = crc16_data((uint16_t)(info_data[i][1]));
    }
   
    return CRC_calc;
    
}


