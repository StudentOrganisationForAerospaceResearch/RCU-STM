#ifndef SOAR_DATA_H_
#define SOAR_DATA_H_
/* Structs containing data primitives */

/*
 * IMPORTANT NOTE:
 *  Despite being typed as int32 or uint32 these are actually not integers.
 *  They represent fixed point decimal numbers.
 *
 * The specified precision is not consistent across all instruments,
 * please see the design manual for more information.
 */


typedef struct
{
    int32_t     pressure_1;
    int32_t		pressure_2;
    int32_t     pressure_3;
	int32_t		pressure_4;
} PressureTransducerData;

typedef struct
{
    PressureTransducerData* pressureTransducerData_;
} AllData;


#endif
