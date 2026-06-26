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
    //INIT(tx->sm_tx_serial_api, SM_START, tx);	// Prepara a SM para iniciar a transmissao
//        tx->pTxByte(SOH); // Inicia a transmissao pelo SOF
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
	NEXT_STATE(SM_TX_DATA);
}

STATE(SM_TX_DATA) {

    sm_tx->estado = 3;
	if (sm_tx->index_to_tx == sm_tx->bytes_to_tx) {
        sm_tx->pTxByte(EOT); // Inicia a transmissao pelo SOF
		NEXT_STATE(SM_TX_CHKSUM_LSB);
		return;
	}
    
    uint8_t aux = sm_tx->ptr_payload[sm_tx->index_to_tx];
	if (aux == SOH || aux == EOT || aux == ESC) {
        sm_tx->pTxByte(ESC);
		NEXT_STATE(SM_TX_BYTE_STUFFING);
		return;
	}

    sm_tx->pTxByte(aux);
    sm_tx->chksum = sm_tx->chksum + aux;
    sm_tx->index_to_tx = sm_tx->index_to_tx + 1;	// A cada byte transmitido, diminui o numero de envios

}

// Eventos: TX_INT
STATE(SM_TX_BYTE_STUFFING) {
    sm_tx->estado = 4;
    #ifdef __TMS320C28X__
    uint8_t aux = readByte(sm_tx->ptr_payload,sm_tx->index_to_tx);
    #else
    uint8_t aux = sm_tx->ptr_payload[sm_tx->index_to_tx];
    #endif
    sm_tx->pTxByte(aux ^ 0xFF);
    sm_tx->chksum = sm_tx->chksum + aux;
    sm_tx->index_to_tx = sm_tx->index_to_tx + 1;	// A cada byte transmitido, diminui o numero de envios
	NEXT_STATE(SM_TX_DATA);
}

STATE(SM_TX_CHKSUM_LSB) {
    sm_tx->estado = 5;
    sm_tx->pTxByte(0x80 | (uint8_t)sm_tx->chksum);
	NEXT_STATE(SM_TX_CHKSUM_MSB);
}

STATE(SM_TX_CHKSUM_MSB) {
    sm_tx->estado = 6;
    sm_tx->pTxByte(0x80 | ((uint8_t)(sm_tx->chksum >> 8)));
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
