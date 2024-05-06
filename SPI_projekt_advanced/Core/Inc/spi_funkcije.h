/*
 * spi_funkcije.h
 *
 *  Created on: May 6, 2024
 *      Author: user
 */

#ifndef INC_SPI_FUNKCIJE_H_
#define INC_SPI_FUNKCIJE_H_

extern SPI_HandleTypeDef hspi1;
/* ADRESE SVIH REGISTARA LIS302DL */
#define LIS302DL_WHO_AM_I_ADDR               0x0F
#define LIS302DL_CTRL_REG1_ADDR              0x20
#define LIS302DL_CTRL_REG2_ADDR              0x21
#define LIS302DL_CTRL_REG3_ADDR              0x22
#define LIS302DL_STATUS_REG_ADDR             0x27
#define LIS302DL_OUT_X_ADDR                  0x29
#define LIS302DL_OUT_Y_ADDR                  0x2B
#define LIS302DL_OUT_Z_ADDR                  0x2D
#define LIS302DL_FF_WU_CFG1_REG_ADDR         0x30
#define LIS302DL_FF_WU_SRC1_REG_ADDR         0x31
#define LIS302DL_FF_WU_THS1_REG_ADDR         0x32
#define LIS302DL_FF_WU_DURATION1_REG_ADDR    0x33

//Funkcija za pisanje na senzor
void MEMS_Write(uint8_t address, uint8_t data){
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1,&address,1,10);
    HAL_SPI_Transmit(&hspi1,&data,1,10);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

//Funkcija za čitanje sa senzora
void MEMS_Read(uint8_t address, uint8_t *data){
    address |= 0x80;
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1,&address,1,10);
    HAL_SPI_Receive(&hspi1,data,1,10);
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
}






#endif /* INC_SPI_FUNKCIJE_H_ */
