#ifndef RX_H_
#define RX_H_

void configureReceiver();
void computeRC();
uint16_t readRawRC(uint8_t chan);
void readSerial_RX(void);

#endif /* RX_H_ */
