/*
 * nrf24l01_plus.c
 *
 *  Created on: Jul 11, 2024
 *      Author: jeshs
 */

#include "nrf24l01_plus.h"


#define CSN_LOW(GPIOx, pin)    LL_GPIO_ResetOutputPin(GPIOx, pin)
#define CSN_HIGH(GPIOx, pin)   LL_GPIO_SetOutputPin(GPIOx, pin)

static uint8_t SPI_SendReceive(SPI_TypeDef *SPIx, uint8_t byte)
{
    while (!LL_SPI_IsActiveFlag_TXE(SPIx));
    LL_SPI_TransmitData8(SPIx, byte);
    while (!LL_SPI_IsActiveFlag_RXNE(SPIx));
    return LL_SPI_ReceiveData8(SPIx);
}


void NRF24L01_WriteRegister(SPI_TypeDef *SPIx, uint8_t reg, uint8_t value, GPIO_TypeDef *GPIOx, uint32_t CS)
{
	CSN_LOW(GPIOx, CS);
	SPI_SendReceive(SPIx, NRF24L01_CMD_W_REGISTER | reg);
	SPI_SendReceive(SPIx, value);
	CSN_HIGH(GPIOx, CS);
}

uint8_t NRF24L01_ReadRegister(SPI_TypeDef *SPIx, uint8_t reg, GPIO_TypeDef *GPIOx, uint32_t CS)
{
	uint8_t value = 0;
	CSN_LOW(GPIOx, CS);
	SPI_SendReceive(SPIx, NRF24L01_CMD_R_REGISTER | reg);
	value = SPI_SendReceive(SPIx, NRF24L01_CMD_NOP);
	CSN_HIGH(GPIOx, CS);
	return value;
}

void NRF24L01_ReadBuf(SPI_TypeDef *SPIx, uint8_t reg, uint8_t *buf, uint8_t len, GPIO_TypeDef *GPIOx, uint32_t CS)
{
	CSN_LOW(GPIOx, CS);
    SPI_SendReceive(SPIx, NRF24L01_CMD_R_REGISTER | reg); // Send command
    for (uint8_t i = 0; i < len; i++)
    {
        buf[len - (i + 1)] = SPI_SendReceive(SPIx, NRF24L01_CMD_NOP); // Read values
    }
	CSN_HIGH(GPIOx, CS);
}

void NRF24L01_WriteBuf(SPI_TypeDef *SPIx, uint8_t reg, uint64_t addr, uint8_t len, GPIO_TypeDef *GPIOx, uint32_t CS)
{
	CSN_LOW(GPIOx, CS);
    SPI_SendReceive(SPIx, NRF24L01_CMD_W_REGISTER | reg); // Send command
    for (uint8_t i = 0; i < len; i++)
    {
//        SPI_SendReceive(SPIx, *buf++); // Read values
    	uint8_t byte = addr & 0xff;
//        SPI_SendReceive(SPIx, (uint8_t)((addr >> (i * 8)) & 0xff)); // Read values
    	SPI_SendReceive(SPIx, byte); // Read values
        addr = addr >> 8;

    }
	CSN_HIGH(GPIOx, CS);
}

void NRF24L01_Init(SPI_TypeDef *SPIx, nrf24L01_InitTypeDef *NRFInitStruct)
{
	if(NRFInitStruct -> PRIM_RX)
	{
		NRFInitStruct -> EN_RX_ADDR_MASK = 0x03;
		NRFInitStruct -> EN_RX_AA_MASK = 0x3f;
		NRF24L01_WriteRegister(SPIx, NRF24L01_REG_CONFIG, NRF24L01_CONFIG_RX, NRFInitStruct -> NRF_GPIO, NRFInitStruct -> CS);
		NRF24L01_WriteRegister(SPIx, NRF24L01_REG_EN_RXADDR, NRFInitStruct -> EN_RX_ADDR_MASK, NRFInitStruct -> NRF_GPIO, NRFInitStruct -> CS);
		NRF24L01_WriteRegister(SPIx, NRF24L01_REG_EN_AA, NRFInitStruct -> EN_RX_AA_MASK, NRFInitStruct -> NRF_GPIO, NRFInitStruct -> CS);
		NRF24L01_WriteRegister(SPIx, NRF24L01_REG_RX_PW_P0, NRFInitStruct -> RX_PW_P0, NRFInitStruct -> NRF_GPIO, NRFInitStruct -> CS);
		NRF24L01_WriteBuf(SPIx, NRF24L01_REG_RX_ADDR_P0, NRFInitStruct -> RX_ADDR_P0, 5, NRFInitStruct -> NRF_GPIO, NRFInitStruct -> CS);

	}

	else
	{
		NRF24L01_WriteRegister(SPIx, NRF24L01_REG_CONFIG, NRF24L01_CONFIG_TX, NRFInitStruct -> NRF_GPIO, NRFInitStruct -> CS);
		NRF24L01_WriteBuf(SPIx, NRF24L01_REG_TX_ADDR, NRFInitStruct -> TX_ADDR, 5, NRFInitStruct -> NRF_GPIO, NRFInitStruct -> CS);
		NRF24L01_WriteBuf(SPIx, NRF24L01_REG_RX_ADDR_P0, NRFInitStruct -> TX_ADDR, 5, NRFInitStruct -> NRF_GPIO, NRFInitStruct -> CS);

	}

}
