/*
 * nrf.c
 *
 *  Created on: Dec 4, 2021
 *      Author: KIIT
 */
#include "stm32f1xx_hal.h"
#include "nrf.h"
// declaring the spi here as an extern

 extern SPI_HandleTypeDef hspi1;
#define CS_pin GPIO_PIN_10
#define CE_pin GPIO_PIN_11
//selecting the cs pin
void Cs_select(){
	HAL_GPIO_WritePin(GPIOA, CS_pin, GPIO_PIN_RESET);
}
//unselecting the CS pin
void Cs_unselect(){
	HAL_GPIO_WritePin(GPIOA, CS_pin, GPIO_PIN_SET);
}
//enable the CE pin
void CE_enable(){
	HAL_GPIO_WritePin(GPIOA, CE_pin, GPIO_PIN_SET);
}
// disable the CE pin
void CE_disable(){
	HAL_GPIO_WritePin(GPIOA, CE_pin, GPIO_PIN_RESET);
}
// write single byte to a particular register
void nrf_reg_single(uint8_t Reg, uint8_t data){
	uint8_t buff[2];
	buff[0] = Reg|1<<5;
	buff[1] = data;
	//pulling the CS pin between low to select the device
	Cs_select();
	HAL_SPI_Transmit(&hspi1, buff, 2, 1000);
	// pulling the CS pin high to release the decvice
	Cs_unselect();
}
//write multiple byte to a particular register
void nrf_reg_multiple(uint8_t Reg, uint8_t *data, int size){
	uint8_t buff[2];
	buff[0] = Reg|1<<5;
	//buff[1] = data;
	//pulling the CS pin between low to select the device
	Cs_select();
	HAL_SPI_Transmit(&hspi1, buff, 2, 100);
	HAL_SPI_Transmit(&hspi1, buff, size, 1000);
	// pulling the CS pin high to release the decvice
	Cs_unselect();
}

uint8_t read_nrf_reg_single(uint8_t Reg){
	uint8_t data=0;
	//pulling the CS pin between low to select the device
	Cs_select();
	HAL_SPI_Transmit(&hspi1, &Reg, 1, 100);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	// pulling the CS pin high to release the decvice
	Cs_unselect();
	return data;
}
void read_nrf_reg_multiple(uint8_t Reg, uint8_t *data, int size){
	//pulling the CS pin between low to select the device
	Cs_select();
	HAL_SPI_Transmit(&hspi1, &Reg, 1, 100);
	HAL_SPI_Transmit(&hspi1, data, size, 1000);
	// pulling the CS pin high to release the device
	Cs_unselect();
}
// send command to the NRF module
void sendcmd(uint8_t cmd){
	//pulling the CS pin between low to select the device
	Cs_select();
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	// pulling the CS pin high to release the decvice
	Cs_unselect();
}
// Initialize the NRF module

void init_device(){
	//disabling the chip before configuring the device
	CE_disable();
	Cs_unselect();
	nrf_reg_single(CONFIG, 0);
	nrf_reg_single(EN_AA, 0);// no auto ACK
	nrf_reg_single(EN_RXADDR, 0); // not enabling any data pipe right now
	nrf_reg_single(SETUP_AW , 0x03); // 5 byte TX/RX address
	nrf_reg_single(SETUP_RETR  , 0); // no retransmission
	nrf_reg_single(RF_CH, 0);// will be set during TX or RX
	nrf_reg_single(RF_SETUP, 0x0E); //Power=0db data rate =2mbps
	nrf_reg_single(RF_SETUP, 0x0E);

	//enabling the chip before configuring the device
	CE_enable();
	Cs_select();


}

// setting up the txmode
void enable_txmode(uint8_t *Address, uint8_t channel){
	//disabling the chip after configuring the device
	CE_disable();

	nrf_reg_single(RF_CH, channel);//select the channel
	nrf_reg_multiple(TX_ADDR, Address, 5);

	//power up the device
	uint8_t config = read_nrf_reg_single(CONFIG);
	config = config |(1<<1);
	nrf_reg_single(CONFIG, config);
	//enable the chip after configuring the device
	CE_enable();
}
//transmit the data
uint8_t data_transmit(uint8_t *data){
	uint8_t cmdtosend = 0;
	//select the device
	Cs_select();
	//payload command
	cmdtosend = W_TX_PAYLOAD ;
	HAL_SPI_Transmit(&hspi1, &cmdtosend, 1, 100);
	//send payload
	HAL_SPI_Transmit(&hspi1, data, 32, 1000);
	//unselecting the device
	Cs_unselect();
	HAL_Delay(1);
	uint8_t fifostatus = read_nrf_reg_single(FIFO_STATUS);
	if((fifostatus&(1<<4))&&(!(fifostatus&(1<<3))))
	{
		cmdtosend= FLUSH_TX;
		sendcmd(cmdtosend);
		return 1;
	}

	return 0;
}
 void nrf_rxmode(uint8_t *Address, uint8_t channel){
		//disabling the chip after configuring the device
		CE_disable();

		nrf_reg_single(RF_CH, channel);//select the channel
		//slecting the data pipe 1
		uint8_t en_rxaddr= read_nrf_reg_single(EN_RXADDR);
	    en_rxaddr =en_rxaddr|(1<<1);
	    nrf_reg_single(RF_CH, channel);
		nrf_reg_multiple(RX_ADDR_P1, Address, 5);
		nrf_reg_single(RX_PW_P1, 32); // 32 bits payload through pipe 1
		//power up the device
		uint8_t config = read_nrf_reg_single(CONFIG);
		config = config |(1<<1);
		nrf_reg_single(CONFIG, config);
		//enable the chip after configuring the device
		CE_enable();

 }
  uint8_t data_check(int pipenum){
	  uint8_t status = read_nrf_reg_single(STATUS);
	  if((status&(1<<6))&& (status&(pipenum<<1))){
		  nrf_reg_single(STATUS , (1<<6));
		  return 1;
	  }
	  return 0;

  }

  void receive_data(uint8_t *data){
		uint8_t cmdtosend = 0;
		//select the device
		Cs_select();
		//payload command
		cmdtosend = R_RX_PAYLOAD ;
		HAL_SPI_Transmit(&hspi1, &cmdtosend, 1, 100);
		//send payload
		HAL_SPI_Receive(&hspi1, data, 32, 1000);
		//unselecting the device
		Cs_unselect();
		HAL_Delay(1);
		cmdtosend= FLUSH_RX;
		sendcmd(cmdtosend);

  }





