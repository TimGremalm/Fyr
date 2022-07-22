#ifndef DMX_H_
#define DMX_H_

#include <stdint.h>

extern uint8_t dmx_data[513];

void dmxtask(void *pvParameters);

#endif /* DMX_H_ */

