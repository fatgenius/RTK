#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "can.h"
#include "can_drv.h"
#include <ros/ros.h>

#define RX_WAIT_TIME  1000
#define RX_BUFF_SIZE  1000


static VCI_CAN_OBJ      g_SendCanObject;
static pthread_mutex_t  g_SendLock;
static struct car_speed g_SpdFeed;
static pthread_mutex_t  g_SpdFeedLock;

bool can_parse_frame(const VCI_CAN_OBJ *can)
{
    int i = 0;
    uint8_t sum = 0;

    sum += (can->ID >> 8) & 0xFF;
    sum += (can->ID >> 0) & 0xFF;
    sum += (can->DataLen) & 0xFF;
    for (i = 0; i < (can->DataLen - 1); i++)
    {
        sum += can->Data[i];
    }

    if ((sum & 0xFF) == can->Data[can->DataLen - 1])
    {
        return true;
    }

    return false;
}

void car_send_command(VCI_CAN_OBJ *can)
{
    uint8_t sum;

    can->SendType = 0;
    can->RemoteFlag = 0;
    can->ExternFlag = 0;

    sum = 0;
    sum += (can->ID >> 8) & 0xFF;
    sum += (can->ID >> 0) & 0xFF;
    sum += can->DataLen;
    for (int i = 0; i < (can->DataLen - 1); i++)
    {
        sum += can->Data[i];
    }

    sum &= 0xFF;
    can->Data[7] = sum;

    pthread_mutex_lock(&g_SendLock);
    memcpy(&g_SendCanObject, can, sizeof(g_SendCanObject));
    pthread_mutex_unlock(&g_SendLock);
}

void * tx_thread_move_main(void *data)
{
    int cnt;
    VCI_CAN_OBJ can;
    VCI_ERR_INFO errInfo;
    struct car_speed speed;
    struct can_device *device = (struct can_device *)data;

    speed.linSpeed = 0;
    speed.angSpeed = 0;
    car_spdSetCurrent(&speed);
    while (1)
    {
        pthread_mutex_lock(&g_SendLock);
        memcpy(&can, &g_SendCanObject, sizeof(can));
        pthread_mutex_unlock(&g_SendLock);

	ROS_INFO("can %d %d", can.Data[2], can.Data[3]);
        cnt = (int)VCI_Transmit(device->devType, device->devID, device->devChN, &can, 1);
        if (cnt <= 0)
        {
            VCI_ReadErrInfo(device->devType, device->devID, device->devChN, &errInfo);
        }

        usleep(20 * 1000);
    }
}

void * rx_thread_main(void *data)
{
   struct can_device *device = (struct can_device *)data;

    VCI_ERR_INFO errInfo;
    VCI_CAN_OBJ can[RX_BUFF_SIZE]; // buffer
    int cnt; // current received
    float fTmp;
    union value
    {
        int16_t data;
        struct {
            uint8_t lo;
            uint8_t hi;
        };
    };

    while (1)
    {
        cnt = (int)VCI_Receive(device->devType, device->devID, device->devChN, can, RX_BUFF_SIZE, RX_WAIT_TIME);
        if (cnt <= 0)
        {
            VCI_ReadErrInfo(device->devType, device->devID, device->devChN, &errInfo);
            continue;
        }

        for (int i = 0; i < cnt; i++)
        {
            if (!can_parse_frame(&can[i]))
            {
                ROS_INFO("wrong frame.\n");
                continue;
            }

            switch (can[i].ID)
            {
                case 0x131:
                    union value value;
                    pthread_mutex_lock(&g_SpdFeedLock);
                    value.hi = can[i].Data[0];
                    value.lo = can[i].Data[1];
                    fTmp = value.data;
                    g_SpdFeed.linSpeed = fTmp / 1000.0F;

                    value.hi = can[i].Data[2];
                    value.lo = can[i].Data[3];
                    fTmp = value.data;
                    g_SpdFeed.angSpeed = fTmp / 1000.0F;
                    
                    g_SpdFeed.isUpdate = true;
                    pthread_mutex_unlock(&g_SpdFeedLock);
                    break;
            }
        }

        usleep(100 * 1000);
    }
}

int car_task(struct can_device *device)
{
    void *ret;
    pthread_t rx_thread;
    pthread_t tx_thread;
    VCI_INIT_CONFIG config;

    if (!VCI_OpenDevice(device->devType, device->devID, 0))
    {
        printf("VCI_OpenDevice failed\n");
        return 0;
    }

    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter = 1;
    config.Mode = 0;
    config.Timing0 = 0x00;
    config.Timing1 = 0x1C;
    if (!VCI_InitCAN(device->devType, device->devID, device->devChN, &config))
    {
        printf("VCI_InitCAN failed\n");
        return 0;
    }

    if (!VCI_StartCAN(device->devType, device->devID, device->devChN))
    {
        printf("VCI_StartCAN failed");
        return 0;
    }

    g_SpdFeed.isUpdate = false;
    g_SpdFeed.linSpeed = 0;
    g_SpdFeed.angSpeed = 0;
    pthread_mutex_init(&g_SendLock, (const pthread_mutexattr_t *)0);
    pthread_mutex_init(&g_SpdFeedLock, (const pthread_mutexattr_t *)0);
    pthread_create(&rx_thread, (const pthread_attr_t *)0, rx_thread_main, device);
    pthread_create(&tx_thread, (const pthread_attr_t *)0, tx_thread_move_main, device);

 //   pthread_join(tx_thread, &ret);
 //   pthread_join(rx_thread, &ret);

//    VCI_CloseDevice(device->devType, device->devID);

    return 0;
}

bool car_spdIsUpdate(void)
{
    bool isUpdate;

    pthread_mutex_lock(&g_SpdFeedLock);
    isUpdate = g_SpdFeed.isUpdate;
    pthread_mutex_unlock(&g_SpdFeedLock);

    return isUpdate;
}

void car_spdGetCurrent(struct car_speed *speed)
{
    if (speed == (void *)0) 
    {
        return;
    }

    pthread_mutex_lock(&g_SpdFeedLock);
    speed->linSpeed = g_SpdFeed.linSpeed;
    speed->angSpeed = g_SpdFeed.angSpeed;
    speed->isUpdate = false;
    pthread_mutex_unlock(&g_SpdFeedLock);
}

void car_spdSetCurrent(struct car_speed *speed)
{
    int8_t moveSpdInt;
    int8_t angleSpdInt;
    VCI_CAN_OBJ can;

    if (speed->linSpeed > 1.5f) speed->linSpeed = 1.5f;
    if (speed->linSpeed < -1.5f) speed->linSpeed = -1.5f;

    if (speed->angSpeed > 0.7853f) speed->angSpeed = 0.7853F;
    if (speed->angSpeed < -0.7853f) speed->angSpeed = -0.7853F;

    moveSpdInt = (int8_t)(speed->linSpeed / 1.5f * 100);
    angleSpdInt = speed->angSpeed / 0.7853f * 100;
    can.ID = 0x0130;
    can.DataLen = 8;
    can.Data[0] = 0x01;
    can.Data[1] = 0x00;
    can.Data[2] = moveSpdInt;
    can.Data[3] = angleSpdInt;
    can.Data[4] = 0x00;
    can.Data[5] = 0x00;
    can.Data[6] = 0x00;

    car_send_command(&can);
}
