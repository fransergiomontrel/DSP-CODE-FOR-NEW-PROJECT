#include "sm_tx_serial_api.h"
#include "hal.h"

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
    tx->estado = 0;
	INIT(tx->sm_tx_serial_api, SM_TX_WAITING, tx);
	if(tx->tx_FF)
	    tx->pTxByte(0xFF);

	TXInts(1);

}

void init_tx_serial_software(CiseiTxChannel* tx, tpTxByte pTxByteFunc, boolean dummy){

    tx->tx_FF = dummy;
    tx->pTxByte = pTxByteFunc;
    tx->tx_start = 0;
    tx->estado = 0;
	INIT(tx->sm_tx_serial_api, SM_TX_WAITING, tx);
	if(tx->tx_FF)
	    tx->pTxByte(0xFF);

}

uint8_t start_tx_frame(CiseiTxChannel* tx, teSerialFrameType type, uint8_t* pPayload, uint32_t nBytes) {
    TXInts(0);

	if (!COMPARE(tx->sm_tx_serial_api, SM_TX_WAITING)){
	    TXInts(1);
        return 0;
	}

    tx->type = type;
    tx->ptr_payload = pPayload;
    tx->bytes_to_tx = nBytes;
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

uint8_t start_tx_frame_software(CiseiTxChannel* tx, teSerialFrameType type, uint8_t* pPayload, uint32_t nBytes) {

	if (!COMPARE(tx->sm_tx_serial_api, SM_TX_WAITING)){
	    //TXInts(1);
        return 0;
	}

    tx->type = type;
    tx->ptr_payload = pPayload;
    tx->bytes_to_tx = nBytes;
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

#define sm_tx   ((CiseiTxChannel*)SM_PARAM)

STATE(SM_TX_WAITING) {
    sm_tx->estado = 1;
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
    sm_tx->estado = 2;
    sm_tx->pTxByte((uint8_t)sm_tx->type); // Inicia a transmissao pelo SOF
	NEXT_STATE(SM_TX_FINALIZE);
}


// Eventos: TX_INT
STATE(SM_TX_FINALIZE) {
    sm_tx->estado = 7;
    sm_tx->isFinished = 1;
    if(sm_tx->tx_FF)
        sm_tx->pTxByte(0xFF);
	NEXT_STATE(SM_TX_WAITING);
}
