#include "sm_rx_serial_api.h"
#include "hal.h"
#include "CRC16.h"

teSerialFrameType rx_getFrameType(CiseiRxChannel* rx){
    return rx->rx_frame_type;
}

void rx_reset_buffer(CiseiRxChannel* rx) {
	rx->bytesReceived = 0;
	rx->chksum = 0;
	INIT(rx->sm_rx_serial_api, SM_RX_WAITING, rx);
}

void rx_reset_buffer_stm32(CiseiRxChannel_stm32* rx) {
	rx->bytesReceived = 0;
	rx->chksum = 0;
	INIT(rx->sm_rx_serial_api, SM_RX_WAITING_STM32_LOW, rx);
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

void rx_interrupt_stm32(CiseiRxChannel_stm32* rx, uint8_t b) {
//    togglePin(DEBUG1);
	if (rx->lock)   // Se estiver travado, ignora o dado
        return;
	if (rx->frameReceived)  // Se a flag indicar que ha um frame recebido, nao permite a recepcao de nada
		return;

    rx->byte_received = b;
	// Sempre que receber SOH reseta a recepcao
	if (rx->byte_received == SOH_STM32_LOW)
		crc16_init();
		rx_reset_buffer_stm32(rx);

	EXEC(rx->sm_rx_serial_api);
}

void rx_free_frame(CiseiRxChannel* rx) {
	rx->frameReceived = 0;
}

void rx_free_frame_stm32(CiseiRxChannel_stm32* rx) {
	rx->frameReceived = 0;
}

uint8_t rx_frameReceived(CiseiRxChannel* rx, uint32_t* bytesReceived) {
	*bytesReceived = rx->bytesReceived;
	return rx->frameReceived;
}

uint8_t rx_frameReceived_stm32(CiseiRxChannel_stm32* rx, uint32_t* bytesReceived) {
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

void init_rx_serial_stm32(CiseiRxChannel_stm32* rx, uint8_t* pBuffer)
{
	rx->lock = 1;
	rx->pBuffer = pBuffer;
	rx_free_frame_stm32(rx);
	rx_reset_buffer_stm32(rx);
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

	if (sm_rx->bytesReceived == sm_rx->nBytes) {
		// Buffer encheu...
		NEXT_STATE(SM_RX_WAITING);
		return;
	}

    sm_rx->pBuffer[sm_rx->bytesReceived] = sm_rx->byte_received;
	sm_rx->bytesReceived = sm_rx->bytesReceived + 1;

}

#define sm_rx_stm32   ((CiseiRxChannel_stm32*)SM_PARAM)

STATE(SM_RX_WAITING_STM32_LOW) {
	if (sm_rx_stm32->byte_received == SOH_STM32_LOW)
		crc16_data(sm_rx_stm32->byte_received);
		NEXT_STATE(SM_RX_WAITING_STM32_HIGH);
}

STATE(SM_RX_WAITING_STM32_HIGH) {
	if (sm_rx_stm32->byte_received == SOH_STM32_HIGH)
		crc16_data(sm_rx_stm32->byte_received);
		NEXT_STATE(SM_RX_COMMAND);
}

STATE(SM_RX_COMMAND) {
	if (sm_rx_stm32->byte_received == MEASURE)
	{
		sm_rx_stm32->command = sm_rx_stm32->byte_received;
		crc16_data(sm_rx_stm32->byte_received);
		NEXT_STATE(SM_RX_LENGTH_LOW);
	}
	else if (sm_rx_stm32->byte_received == NOP)
	{
		sm_rx_stm32->command = sm_rx_stm32->byte_received;
		crc16_data(sm_rx_stm32->byte_received);
		NEXT_STATE(SM_RX_LENGTH_LOW);
	}
	else if (sm_rx_stm32->byte_received == IDENT_IED)
	{
		sm_rx_stm32->command = sm_rx_stm32->byte_received;
		crc16_data(sm_rx_stm32->byte_received);
		NEXT_STATE(SM_RX_LENGTH_LOW);
	}
	else
	{
		sm_rx_stm32->error = ERROR_UNAVAILABLE_COMMAND;
		return;
	}
}

STATE(SM_RX_LENGTH_LOW) {
	sm_rx_stm32->length = (uint16_t)(sm_rx_stm32->byte_received);
	crc16_data(sm_rx_stm32->byte_received);
	NEXT_STATE(SM_RX_LENGTH_HIGH);
}

STATE(SM_RX_LENGTH_HIGH) {
	sm_rx_stm32->length = (((uint16_t)(sm_rx_stm32->byte_received)) << 8) | sm_rx_stm32->length;
	crc16_data(sm_rx_stm32->byte_received);
	NEXT_STATE(SM_RX_DATA_STM32);
}

STATE(SM_RX_DATA_STM32) {

    sm_rx_stm32->pBuffer[sm_rx->bytesReceived] = sm_rx_stm32->byte_received;
	sm_rx_stm32->chksum = crc16_data(sm_rx_stm32->byte_received);
	sm_rx_stm32->bytesReceived = sm_rx_stm32->bytesReceived + 1;

	if (sm_rx_stm32->bytesReceived == sm_rx_stm32->length)
	{
		if(sm_rx_stm32->byte_received == MEASURE)
		{
			switch(sm_rx_stm32->pBuffer[0])
			{
				case NOM_FREQ_50HZ:
				break;
				case NOM_FREQ_60HZ:
				break;
				default:
				sm_rx_stm32->error = ERROR_NACK_COMMAND;
				return;
			}
			switch(sm_rx_stm32->pBuffer[1])
			{
				case NO_FIBER:
				break;
				case ONE_FIBER:
				break;
				case TWO_FIBERS:
				break;
				case THREE_FIBERS:
				break;
				case FOUR_FIBERS:
				break;
				default:
				sm_rx_stm32->error = ERROR_NACK_COMMAND;
				return;
			}
		}
		else if(sm_rx_stm32->byte_received == IDENT_IED)
		{
			switch(sm_rx_stm32->pBuffer[0])
			{
				case NO_FIBER:
				break;
				case ONE_FIBER:
				break;
				case TWO_FIBERS:
				break;
				case THREE_FIBERS:
				break;
				case FOUR_FIBERS:
				break;
				default:
				sm_rx_stm32->error = ERROR_NACK_COMMAND;
				return;
			}
		}
		NEXT_STATE(SM_RX_CHECKSUM_LOW);
	}

}

STATE(SM_RX_CHECKSUM_LOW) {
	sm_rx_stm32->chksumReceived = (uint16_t)(sm_rx_stm32->byte_received);
	NEXT_STATE(SM_RX_CHECKSUM_HIGH);
}

STATE(SM_RX_CHECKSUM_HIGH) {
	sm_rx_stm32->chksumReceived = (((uint16_t)(sm_rx_stm32->byte_received)) << 8) | sm_rx_stm32->chksumReceived;
	if(sm_rx_stm32->chksumReceived == sm_rx_stm32->chksum)
	{
		NEXT_STATE(SM_RX_WAITING_STM32_LOW);
		sm_rx_stm32->frameReceived = 1;
	}
	else
	{
		sm_rx_stm32->error = ERROR_INVALID_CHECKSUM;
		NEXT_STATE(SM_RX_WAITING_STM32_LOW);
	}
}






