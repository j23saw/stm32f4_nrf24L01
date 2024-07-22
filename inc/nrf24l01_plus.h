/*
 * nrf24l01_plus.h
 *
 *  Created on: Jul 11, 2024
 *      Author: jeshs
 */

#ifndef NRF24L01_PLUS_NRF24L01_PLUS_H_
#define NRF24L01_PLUS_NRF24L01_PLUS_H_

#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f411xe.h"

typedef struct
{
	uint8_t PRIM_RX;
	uint64_t TX_ADDR;
	uint8_t EN_RX_ADDR_MASK;
	uint8_t EN_RX_AA_MASK;
	uint64_t RX_ADDR_P0;
	uint8_t RX_PW_P0;
	uint32_t CE;
	uint32_t INT;
	uint32_t CS;
	GPIO_TypeDef *NRF_GPIO;

} nrf24L01_InitTypeDef;


// NRF24L01+ Command Definitions
#define NRF24L01_CMD_R_REGISTER    0x00
#define NRF24L01_CMD_W_REGISTER    0x20
#define NRF24L01_CMD_R_RX_PAYLOAD  0x61
#define NRF24L01_CMD_W_TX_PAYLOAD  0xA0
#define NRF24L01_CMD_FLUSH_TX      0xE1
#define NRF24L01_CMD_FLUSH_RX      0xE2
#define NRF24L01_CMD_NOP           0xFF

// NRF24L01+ Register Definitions
#define NRF24L01_REG_CONFIG        0x00
#define NRF24L01_REG_EN_AA         0x01
#define NRF24L01_REG_EN_RXADDR     0x02
#define NRF24L01_REG_SETUP_AW      0x03
#define NRF24L01_REG_SETUP_RETR    0x04
#define NRF24L01_REG_RF_CH         0x05
#define NRF24L01_REG_RF_SETUP      0x06
#define NRF24L01_REG_STATUS        0x07
#define NRF24L01_REG_RX_ADDR_P0    0x0A
#define NRF24L01_REG_TX_ADDR       0x10
#define NRF24L01_REG_RX_PW_P0      0x11

#define NRF24L01_CONFIG_RX         0x01
#define NRF24L01_CONFIG_TX         0x00

void NRF24L01_Init(SPI_TypeDef *SPIx, nrf24L01_InitTypeDef *NRFInitStruct);
void NRF24L01_WriteRegister(SPI_TypeDef *SPIx, uint8_t reg, uint8_t value, GPIO_TypeDef *GPIOx, uint32_t CS);
uint8_t NRF24L01_ReadRegister(SPI_TypeDef *SPIx, uint8_t reg, GPIO_TypeDef *GPIOx, uint32_t CS);
void NRF24L01_ReadBuf(SPI_TypeDef *SPIx, uint8_t reg, uint8_t *buf, uint8_t len, GPIO_TypeDef *GPIOx, uint32_t CS);
void NRF24L01_WriteBuf(SPI_TypeDef *SPIx, uint8_t reg, uint64_t addr, uint8_t len, GPIO_TypeDef *GPIOx, uint32_t CS);
void NRF24L01_Transmit(SPI_TypeDef *SPIx,  uint8_t data);
uint8_t NRF24L01_Receive(SPI_TypeDef *SPIx);

#endif /* NRF24L01_PLUS_NRF24L01_PLUS_H_ */
