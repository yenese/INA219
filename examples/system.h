/*
 * system.h
 *
 *  Created on: Feb 3, 2025
 *      Author: yunus
 */

#ifndef INC_SYSTEM_H_
#define INC_SYSTEM_H_

void setup();
void loop();
int8_t i2cRead(void* intf, uint8_t reg, uint16_t *pRxData, uint8_t len);
int8_t i2cWrite(void* intf, uint8_t reg, uint16_t *pTxData, uint8_t len);


#endif /* INC_SYSTEM_H_ */
