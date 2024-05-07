/*
 * poruke.h
 *
 *  Created on: May 4, 2024
 *      Author: Rio
 */

#ifndef INC_PORUKE_H_
#define INC_PORUKE_H_

CAN_TxHeaderTypeDef prvaPoruka;
CAN_TxHeaderTypeDef drugaPoruka;
CAN_TxHeaderTypeDef trecaPoruka;
CAN_TxHeaderTypeDef cetvrtaPoruka;
CAN_TxHeaderTypeDef masterprvaPoruka;
CAN_TxHeaderTypeDef masterdrugaPoruka;
CAN_TxHeaderTypeDef mastertrecaPoruka;
CAN_TxHeaderTypeDef mastercetvrtaPoruka;


extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef TxHeader;
extern uint8_t txData[8];
extern uint32_t TxMailbox[3];

void napraviPoruku(CAN_TxHeaderTypeDef *TxHeader,uint32_t broj) {

	 TxHeader -> StdId = broj;
	 TxHeader -> ExtId = 0;
	 TxHeader ->IDE = 0;
	 TxHeader -> RTR = 0;
	 TxHeader -> DLC = 8;


}
void posaljiPoruku (CAN_TxHeaderTypeDef *TxHeader,uint8_t brojMailboxa) {

	if(HAL_CAN_AddTxMessage(&hcan1, TxHeader, txData, &TxMailbox[brojMailboxa]) != HAL_OK) {
		Error_Handler();
	};

}






#endif /* INC_PORUKE_H_ */
