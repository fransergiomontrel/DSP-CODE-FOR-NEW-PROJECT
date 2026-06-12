#include "sm_rx_serial_api.h"
#include "hal.h"

teSerialFrameType rx_getFrameType(CiseiRxChannel* rx){
    return rx->rx_frame_type;
}

void rx_reset_buffer(CiseiRxChannel* rx) {
	rx->bytesReceived = 0;
	rx->chksum = 0;
	INIT(rx->sm_rx_serial_api, SM_RX_WAITING, rx);
}

void rx_interrupt(CiseiRxChannel* rx, uint8_t b) {
//    togglePin(DEBUG1);
	if (rx->lock)   // Se estiver travado, ignora o dado
        return;
	if (rx->frameReceived)  // Se a flag indicar que ha um frame recebido, nao permite a recepcao de nada
		return;

    rx->byte_received = b;
	// Sempre que receber SOH reseta a recepcao
	if (rx->byte_received == SOH)
		rx_reset_buffer(rx);

	EXEC(rx->sm_rx_serial_api);
}

void rx_free_frame(CiseiRxChannel* rx) {
	rx->frameReceived = 0;
}

uint8_t rx_frameReceived(CiseiRxChannel* rx, uint32_t* bytesReceived) {
	*bytesReceived = rx->bytesReceived;
	return rx->frameReceived;
}

uint8_t rx_checkframeReceived(CiseiRxChannel* rx) {
	return rx->frameReceived;
}

void init_rx_serial(CiseiRxChannel* rx, uint8_t* pBuffer, uint32_t nBytes) {
    rx->lock = 1;
	rx->pBuffer = pBuffer;
	rx->nBytes = nBytes;
	rx_free_frame(rx);
	rx_reset_buffer(rx);
	rx->lock = 0;
}

#define sm_rx   ((CiseiRxChannel*)SM_PARAM)

STATE(SM_RX_WAITING) {
	if (sm_rx->byte_received == SOH)
		NEXT_STATE(SM_RX_RECEIVE_TYPE);
}

STATE(SM_RX_RECEIVE_TYPE) {
    sm_rx->rx_frame_type = (teSerialFrameType)sm_rx->byte_received;
	NEXT_STATE(SM_RX_DATA);
}

STATE(SM_RX_DATA) {
	if (sm_rx->byte_received == ESC) {
		NEXT_STATE(SM_RX_BYTE_STUFFING);
		return;
	}

	if (sm_rx->byte_received == EOT) {
		NEXT_STATE(SM_RX_CHECKSUM_LSB);
		return;
	}

	if (sm_rx->bytesReceived >= sm_rx->nBytes) {
		// Buffer encheu...
		NEXT_STATE(SM_RX_FULL_BUFFER);
		return;
	}

    #ifdef __TMS320C28X__
    writeByte(sm_rx->pBuffer, sm_rx->bytesReceived, sm_rx->byte_received);
    #else
    sm_rx->pBuffer[sm_rx->bytesReceived] = sm_rx->byte_received;
    #endif
	sm_rx->chksum = sm_rx->chksum + (uint8_t)sm_rx->byte_received;
	sm_rx->bytesReceived = sm_rx->bytesReceived + 1;
}

STATE(SM_RX_BYTE_STUFFING) {
	uint8_t b = sm_rx->byte_received ^ 0xFF;
    #ifdef __TMS320C28X__
	writeByte(sm_rx->pBuffer, sm_rx->bytesReceived, b);
    #else
    sm_rx->pBuffer[sm_rx->bytesReceived] = b;
    #endif
	sm_rx->chksum = sm_rx->chksum + b;
	sm_rx->bytesReceived = sm_rx->bytesReceived + 1;
	NEXT_STATE(SM_RX_DATA);
}

STATE(SM_RX_CHECKSUM_LSB) {
	sm_rx->chksumReceived = sm_rx->byte_received;
	NEXT_STATE(SM_RX_CHECKSUM_MSB);
}

STATE(SM_RX_CHECKSUM_MSB) {
	sm_rx->chksumReceived = (((uint16_t)sm_rx->byte_received) << 8) | sm_rx->chksumReceived;
	if (sm_rx->chksumReceived == (sm_rx->chksum | 0x8080)) {
		sm_rx->frameReceived = 1;
		NEXT_STATE(SM_RX_WAITING);
	}
	else
		NEXT_STATE(SM_RX_CHKSUM_ERROR);
}

STATE(SM_RX_CHKSUM_ERROR) {
}

STATE(SM_RX_FULL_BUFFER) {
	// O buffer de recepcao ficou cheio. Sai aqui apenas quando receber um SOH
}
