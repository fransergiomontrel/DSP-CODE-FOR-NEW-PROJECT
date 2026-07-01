#include "sm_tx_serial_api.h"
#include "hal.h"
#include "CRC16.h"

uint16_t CRC;

uint16_t tx_end(CiseiTxChannel* tx){
    return tx->isFinished;
}

void tx_interrupt(CiseiTxChannel* tx) {
    EXEC(tx->sm_tx_serial_api);
}

void init_tx_serial(CiseiTxChannel* tx, tpTxByte pTxByteFunc, boolean dummy){
    
    TXInts(0);

    tx->tx_FF = dummy;
    tx->pTxByte = pTxByteFunc;
    tx->tx_start = 0;
	INIT(tx->sm_tx_serial_api, SM_TX_WAITING, tx);
	if(tx->tx_FF)
	    tx->pTxByte(0xFF);

	TXInts(1);

}

void init_tx_serial_software(CiseiTxChannel* tx, tpTxByte pTxByteFunc, boolean dummy){

    tx->tx_FF = dummy;
    tx->pTxByte = pTxByteFunc;
    tx->tx_start = 0;
	INIT(tx->sm_tx_serial_api, SM_TX_WAITING, tx);
	if(tx->tx_FF)
	    tx->pTxByte(0xFF);

}

uint8_t start_tx_frame(CiseiTxChannel* tx, teSerialFrameType type) {
    TXInts(0);

	if (!COMPARE(tx->sm_tx_serial_api, SM_TX_WAITING)){
	    TXInts(1);
        return 0;
	}

    tx->type = type;
    tx->index_to_tx = 0;
    tx->chksum = 0;
    tx->isFinished = 0;
    
    if(tx->tx_FF)
        tx->tx_start = 1;
    else{
        INIT(tx->sm_tx_serial_api, SM_START, tx);
        tx->pTxByte(SOH);
    }

    TXInts(1);
	return 1;
}

uint8_t start_tx_frame_software_fpga0(CiseiTxChannel* tx, teSerialFrameType type) {

	if (!COMPARE(tx->sm_tx_serial_api, SM_TX_WAITING)){
	    //TXInts(1);
        return 0;
	}

    tx->type = type;
    tx->index_to_tx = 0;
    tx->chksum = 0;
    tx->isFinished = 0;
    
    if(tx->tx_FF)
        tx->tx_start = 1;
    else{
        INIT(tx->sm_tx_serial_api, SM_START, tx);
        tx->pTxByte(SOH);
    }

	return 1;
}

uint8_t start_tx_frame_software_stm32(CiseiTxChannel* tx, uint8_t command, uint8_t* pPayload, uint32_t nBytes)
{
    if (!COMPARE(tx->sm_tx_serial_api, SM_TX_WAITING)){
	    //TXInts(1);
        return 0;
	}

    tx->ptr_payload = pPayload;
    tx->bytes_to_tx = nBytes;
    tx->index_to_tx = 0;
    tx->chksum = 0;
    tx->isFinished = 0;
    tx->command = command;
    tx->bytes_to_tx = nBytes;

    if(command == MEASURE)
    {
        crc16_init();
        crc16_data(SOH_STM32_LOW);
        crc16_data(SOH_STM32_HIGH);
        crc16_data(MEASURE);
        crc16_data(TMS_FRAME_LEN_LOW);
        crc16_data(TMS_FRAME_LEN_HIGH);
        tms320_frame_crc((tms320_uart_frame_t *)(tx->ptr_payload));
    }
    
    if(tx->tx_FF)
        tx->tx_start = 1;
    else{
        INIT(tx->sm_tx_serial_api, SM_START_STM32_HIGH, tx);
        tx->pTxByte(SOH_STM32_LOW);
    }

	return 1;
}

#define sm_tx   ((CiseiTxChannel*)SM_PARAM)

STATE(SM_TX_WAITING) {
    
    if(sm_tx->tx_FF){
        if(sm_tx->tx_start){
            sm_tx->tx_start = 0;
            sm_tx->pTxByte(SOH);
            NEXT_STATE(SM_START);
        }else{

            sm_tx->pTxByte(0xFF);
        }
    }
}

STATE(SM_START) {
    
    sm_tx->pTxByte((uint8_t)sm_tx->type); // Inicia a transmissao pelo SOF
	NEXT_STATE(SM_TX_FINALIZE);
}

// Eventos: TX_INT
STATE(SM_TX_FINALIZE) {
    
    sm_tx->isFinished = 1;
    if(sm_tx->tx_FF)
        sm_tx->pTxByte(0xFF);
	NEXT_STATE(SM_TX_WAITING);
}


STATE(SM_START_STM32_HIGH) {
    sm_tx->pTxByte(SOH_STM32_HIGH); // Inicia a transmissao pelo SOF
	NEXT_STATE(SM_TX_COMMAND);
}

STATE(SM_TX_COMMAND) {
    sm_tx->pTxByte(sm_tx->command); // Inicia a transmissao pelo SOF
	NEXT_STATE(SM_TX_DATA_STM32);
}

STATE(SM_TX_DATA_STM32) {
    if(sm_tx->index_to_tx == (sm_tx->bytes_to_tx-1))
    {
        NEXT_STATE(SM_TX_CHECKSUM_LOW);
    }
    sm_tx->pTxByte(sm_tx->ptr_payload[sm_tx->index_to_tx]);    
    sm_tx->index_to_tx = sm_tx->index_to_tx + 1; 
}

STATE(SM_TX_CHECKSUM_LOW) {
    
    tms320_uart_frame_t * temp_ptr = (tms320_uart_frame_t *)(sm_tx->ptr_payload); 
    sm_tx->pTxByte((temp_ptr->crc) & 0x00FF); 
    NEXT_STATE(SM_TX_CHECKSUM_HIGH);

}

STATE(SM_TX_CHECKSUM_HIGH) {
    
    tms320_uart_frame_t * temp_ptr = (tms320_uart_frame_t *)(sm_tx->ptr_payload); 
    sm_tx->pTxByte((temp_ptr->crc) >> 8);
    NEXT_STATE(SM_TX_FINALIZE_STM32);

}

STATE(SM_TX_FINALIZE_STM32) {
    
    sm_tx->isFinished = 1;
    
}
