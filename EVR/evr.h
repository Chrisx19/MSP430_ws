/*
 * evr.h
 *
 *  Created on: Dec 30, 2024
 *      Author: clazo
 */
#ifndef EVR_EVR_H_
#define EVR_EVR_H_
#include <stdint.h>

typedef enum {
    EVR_SUCCESS = 0,
    EVR_NULL_ERROR,
    EVR_ERROR,
} EVR_Status;

typedef struct {
    int temp;
} EVR_t;

EVR_Status EVR_Init(EVR_t *evr);
EVR_Status EVR(const char *format, ...);
EVR_Status EVR_GetCommand(char *outBuffer);

#endif /* EVR_EVR_H_ */
