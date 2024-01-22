/*
 * utils.h
 *
 *  Created on: Feb 2, 2023
 *      Author: vienchau
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>



typedef struct ArrayData
{
    uint16_t pos1;
    uint16_t vel1;
    uint16_t acc1;
    uint16_t dir1;
    uint16_t dir2;
} ArrData_t;

ArrData_t ArrProcess(char *arr);

#endif /* INC_UTILS_H_ */
