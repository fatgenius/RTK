#ifndef INCLUDE_CAN_H
#define INCLUDE_CAN_H

#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C"
{
#endif

struct can_device
{
    uint8_t devType;
    uint8_t devID;
    uint8_t devChN;
    uint16_t devBaudRate;
};

struct car_speed
{
    bool  isUpdate;
    float linSpeed;
    float angSpeed;
};

int  car_task(struct can_device *device);
bool car_spdIsUpdate(void);
void car_spdGetCurrent(struct car_speed *speed);
void car_spdSetCurrent(struct car_speed *speed);







#ifdef __cplusplus
}
#endif
#endif /* INCLUDE_CAN_H */