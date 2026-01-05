/*
 * evr.h
 *
 *  Created on: 02/10/25
 *      Author: clazo
 */
#ifndef EVR_H_
#define EVR_H_
#include <stdint.h>

typedef enum {
  EVR_SUCCESS = 0,
  EVR_NULL_ERROR,
  EVR_ERROR,
  EVR_CMD_NOT_READY,
} Evr_Status;

typedef enum {
  USER = 0,
  DEBUG
} Interface;

Evr_Status EVR_Init();
Evr_Status EVR(const char *format, ...);

#endif /* EVR_H_ */
