#include "../common/CPLD_Api.h"

cpld_modes cpld_mode = MODE_CFG_CPLD;
Uint32 ADC_frame[6];
Uint16 CRC_error = 0;
Uint16 AD[6][100];

void CPLD_GPIO_Config(){
    //Configura leds como out e apaga
    GPIO_SetupPinMux(LED1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(LED1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(LED2, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(LED2, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(LED3, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(LED3, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_WritePin(LED1, 1);
    GPIO_WritePin(LED2, 1);
    GPIO_WritePin(LED3, 1);

    GPIO_SetupPinMux(RST, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(RST, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(CONVST, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(CONVST, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(M0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(M0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(M1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(M1, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_WritePin(RST, 1);
    GPIO_WritePin(CONVST, 0);
    GPIO_WritePin(M0, 0);
    GPIO_WritePin(M1, 0);


    CPLD_RST();
}

void CPLD_RST(){
    GPIO_WritePin(RST, 0);
    DELAY_US(100000);
    GPIO_WritePin(RST, 1);
}

void CPLD_RST_AD(){
    GPIO_WritePin(RST, 1);
    DELAY_US(21000);
    GPIO_WritePin(RST, 0);
    DELAY_US(2);
    GPIO_WritePin(RST, 1);
}

Uint16 CPLD_Read_Write_SPI(Uint16 a){
    SpiaRegs.SPITXBUF = (a << 8);
    while(SpiaRegs.SPIFFTX.bit.TXFFST !=0) {}


    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) {}
    return SpiaRegs.SPIRXBUF;
}

Uint32 CPLD_Read_Write_SPI_32(Uint32 a){
    Uint32 ret;
    ret = CPLD_Read_Write_SPI(a >> 24);
    ret = (ret << 8) | CPLD_Read_Write_SPI(a >> 16);
    ret = (ret << 8) | CPLD_Read_Write_SPI(a >> 8);
    ret = (ret << 8) | CPLD_Read_Write_SPI(a);

    return ret;
}

void CPLD_Read_Write_AD(Data_Frame* in, Data_Frame* out){
    out->AD0 = CPLD_Read_Write_SPI_32(in->AD0);
    out->AD1 = CPLD_Read_Write_SPI_32(in->AD1);
    out->AD2 = CPLD_Read_Write_SPI_32(in->AD2);
    out->AD3 = CPLD_Read_Write_SPI_32(in->AD3);
    out->AD4 = CPLD_Read_Write_SPI_32(in->AD4);
    out->AD5 = CPLD_Read_Write_SPI_32(in->AD5);
}

void CPLD_Mode(cpld_modes a){
    DELAY_US(1000); //Pausa para estabiliza��o do CPLD
    GPIO_WritePin(M0, (a == MODE_CFG_CPLD)?1:0);
    DELAY_US(1);
    GPIO_WritePin(M1, (a == MODE_CFG_CPLD)?1:0);
    DELAY_US(1);

    GPIO_WritePin(M0, (((Uint16)a) & 1));
    DELAY_US(1);
    GPIO_WritePin(M1, (((Uint16)a) >> 1) & 1);
    cpld_mode = a;
    DELAY_US(1);
}

Uint32 
CPLD_Get_CFG(){
    CPLD_Mode(MODE_CFG_CPLD);

    return CPLD_Read_Write_SPI_32(0x00000000);
}

void CPLD_WE(Uint16 ena){

    Uint32 atual;
    atual = CPLD_Get_CFG();

    if(ena){
        atual |= 0xAC000001;
    }else{
        atual = (atual | 0xAC000000) & (0xFFFFFFFE);
    }

    CPLD_Mode(MODE_CFG_CPLD);
    CPLD_Read_Write_SPI_32(atual);
    
}

void CPLD_TestRAM(){

    Uint32 MaxPos, i;
    Uint16 data, rdata;

    MaxPos = 512L*1024;
    CPLD_WE(1);

    srand(0x21);
    CPLD_Mode(MODE_RW);
    GPIO_WritePin(LED1, 0);
    for(i=0; i < MaxPos; i++){
        data = (rand() % 0x100);
        CPLD_Read_Write_SPI(data);
    }
    GPIO_WritePin(LED1, 1);

    srand(0x21);
    CPLD_Mode(MODE_RW);
    GPIO_WritePin(LED2, 0);
    for(i=0; i < MaxPos; i++){
        data = (rand() % 0x100);
        rdata = CPLD_Read_Write_SPI(~data);
        if(rdata != data){
            CPLD_Error(1);
            return;
        }
    }
    GPIO_WritePin(LED2, 1);

    srand(0x21);
    CPLD_Mode(MODE_RW);
    GPIO_WritePin(LED3, 0);
    for(i=0; i < MaxPos; i++){
        data = (rand() % 0x100);
        rdata = CPLD_Read_Write_SPI(0x00);
        if(rdata != ((~data) & 0xFF)){
            CPLD_Error(1);
            return;
        }
    }
    GPIO_WritePin(LED3, 1);

    CPLD_WE(0);
}

void CPLD_Test_SPI_AD(){
    
    Uint32 MaxPos, i, data = 0, rdata = 0;

    MaxPos = 6;

    srand(0x21);
    CPLD_Mode(MODE_CFG);
    GPIO_WritePin(LED1, 0);
    for(i=0; i < MaxPos; i++){
        data = (rand() % 0x100);
        data = (data << 8) | (rand() % 0x100);
        data = (data << 8) | (rand() % 0x100);
        data = (data << 8) | (rand() % 0x100);
        CPLD_Read_Write_SPI_32(data);
    }
    GPIO_WritePin(LED1, 1);

    srand(0x21);
    data = 0;
    rdata = 0;
    GPIO_WritePin(LED2, 0);
    for(i=0; i < MaxPos; i++){
        data = (rand() % 0x100);
        data = (data << 8) | (rand() % 0x100);
        data = (data << 8) | (rand() % 0x100);
        data = (data << 8) | (rand() % 0x100);
        rdata = CPLD_Read_Write_SPI_32(~data);
        if(rdata != data){
            CPLD_Error(2);
            return;
        }
    }
    GPIO_WritePin(LED2, 1);

    srand(0x21);
    data = 0;
    rdata = 0;
    GPIO_WritePin(LED3, 0);
    for(i=0; i < MaxPos; i++){
        data = (rand() % 0x100);
        data = (data << 8) | (rand() % 0x100);
        data = (data << 8) | (rand() % 0x100);
        data = (data << 8) | (rand() % 0x100);
        rdata = CPLD_Read_Write_SPI_32(0x00);
        if(rdata != ((~data) & 0xFFFFFFFF)){
            CPLD_Error(2);
            return;
        }
    }
    GPIO_WritePin(LED3, 1);

}

void CPLD_Error(Uint16 a){
    Uint16 LED;
    GPIO_WritePin(LED1, 1);
    GPIO_WritePin(LED2, 1);
    GPIO_WritePin(LED3, 1);
    if(a == 1)
        LED = LED1;
    else if(a == 2)
        LED = LED2;
    else if(a == 3)
        LED = LED3;


    Uint16 count;
    for(count = 0; count < 30; count++){
        GPIO_WritePin(LED, 0);
        DELAY_US(100000);
        GPIO_WritePin(LED, 1);
        DELAY_US(100000);
    }
    count = 0;
}

void CPLD_CFG_AD(){

    Data_Frame inData, outData;
    inData.AD0 = inData.AD1 = inData.AD2 = inData.AD3 = inData.AD4 = inData.AD5 = 0xD0140040;
    CPLD_Read_Write_AD(&inData, &outData);

    GPIO_WritePin(CONVST, 1);
    DELAY_US(10);
    GPIO_WritePin(CONVST, 0);

}

void CPLD_AD_CONV(){

    GPIO_WritePin(CONVST, 0);
    CPLD_Mode(MODE_ACQ);
    DELAY_US(10);
//    GPIO_WritePin(CONVST, 1);

}

void Teste_Modo2(){
    CPLD_WE(1);
    CPLD_AD_CONV();
    DELAY_US(35000);
    GPIO_WritePin(CONVST, 0);
    CPLD_WE(0);

    CPLD_Mode(MODE_RW);

    Uint16 amostra = 0, canal = 0, byyte = 0;
    Uint16 V_read[2];

    crc16_init();

    for(amostra = 0; amostra < 100; amostra++){
        for(canal = 0; canal < 6; canal++){
            for(byyte = 0; byyte < 2; byyte++){
                V_read[byyte] = CPLD_Read_Write_SPI(0x00);
                crc16_data(V_read[byyte]);
            }
            AD[canal][amostra] = V_read[0] + (V_read[1] << 8);
        }
    }

    Uint16 trash[6], trash_i, CRC_calc;
    for(trash_i = 0; trash_i < 6; trash_i++){
        trash[trash_i] = CPLD_Read_Write_SPI(0x00);
        CRC_calc = crc16_data(trash[trash_i]);
    }

    Uint16 CRC_CPLD_RAM;
    CRC_CPLD_RAM = CPLD_Read_Write_SPI(0x00);
    CRC_CPLD_RAM = CRC_CPLD_RAM + (CPLD_Read_Write_SPI(0x00) << 8);

    if(CRC_CPLD_RAM != CRC_calc){
        CRC_error++;
        CPLD_Error(3);
    }

}
