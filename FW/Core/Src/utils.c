/*
 * utils.c
 *
 *  Created on: Feb 2, 2023
 *      Author: vienchau
 */

#include "utils.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>



ArrData_t ArrProcess(char *arr)
{
    ArrData_t data;
    char sampleString[19];
    strcpy(sampleString, arr);

    char *pos1 = (char *)malloc(sizeof(char) * 5);
    char *vel1 = (char *)malloc(sizeof(char) * 5);
    char *acc1 = (char *)malloc(sizeof(char) * 5);
    char *dir1 = (char *)malloc(sizeof(char) * 2);
    char *dir2 = (char *)malloc(sizeof(char) * 2);

    strncpy(pos1, sampleString, 4);
    pos1[4] = '\0';
    strncpy(vel1, &sampleString[5], 4);
    vel1[4] = '\0';
    strncpy(acc1, &sampleString[10], 4);
    acc1[4] = '\0';
    strncpy(dir1, &sampleString[15], 1);
    dir1[1] = '\0';
    strncpy(dir2, &sampleString[17], 1);
    dir2[1] = '\0';

    data.pos1 = atoi(pos1);
    data.vel1 = atoi(vel1);
    data.acc1 = atoi(acc1);
    data.dir1 = atoi(dir1);
    data.dir2 = atoi(dir2);

    free(pos1);
    free(vel1);
    free(acc1);
    free(dir1);
    free(dir2);

    return data;
}
