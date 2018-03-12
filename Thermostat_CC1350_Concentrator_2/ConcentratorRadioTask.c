/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
/* XDCtools Header files */ 
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include "ConcentratorRadioTask.h"

/* BIOS Header files */ 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

/* EasyLink API Header files */ 
#include "easylink/EasyLink.h"

/* Application Header files */ 
#include "RadioProtocol.h"


/***** Defines *****/
#define CONCENTRATORRADIO_TASK_STACK_SIZE 1024
#define CONCENTRATORRADIO_TASK_PRIORITY   3

#define RADIO_EVENT_ALL                  0xFFFFFFFF
#define RADIO_EVENT_VALID_PACKET_RECEIVED       (uint32_t)(1 << 0)
#define RADIO_EVENT_INVALID_PACKET_RECEIVED     (uint32_t)(1 << 1)
#define RADIO_EVENT_SEND_VENT_DATA              (uint32_t)(1 << 2)
#define RADIO_EVENT_DATA_ACK_RECEIVED           (uint32_t)(1 << 3)
#define RADIO_EVENT_ACK_TIMEOUT                 (uint32_t)(1 << 4)
#define RADIO_EVENT_SEND_FAIL                   (uint32_t)(1 << 5)
#ifdef FEATURE_BLE_ADV
#define NODE_EVENT_UBLE                         (uint32_t)(1 << 6)
#endif

#define CONCENTRATORRADIO_MAX_RETRIES 2
#define NODERADIO_ACK_TIMEOUT_TIME_MS (160)
#define NODERADIO_MAX_RETRIES 2


#define CONCENTRATOR_ACTIVITY_LED Board_PIN_LED0


////----------------------
//// New
///***** Type declarations *****/
//struct RadioOperation {
//    EasyLink_TxPacket easyLinkTxPacket;
//    uint8_t retriesDone;
//    uint8_t maxNumberOfRetries;
//    uint32_t ackTimeoutMs;
//    enum NodeRadioOperationStatus result;
//};

//--------------------
/***** Type declarations *****/
struct RadioOperation {
    EasyLink_TxPacket easyLinkTxPacket;
    uint8_t retriesDone;
    uint8_t maxNumberOfRetries;
    uint32_t ackTimeoutMs;
    enum NodeRadioOperationStatus result;
};

/***** Variable declarations *****/
static Task_Params concentratorRadioTaskParams;
Task_Struct concentratorRadioTask; /* not static so you can see in ROV */
static uint8_t concentratorRadioTaskStack[CONCENTRATORRADIO_TASK_STACK_SIZE];
Event_Struct radioOperationEvent;  /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle;

static uint16_t adcData;
static uint16_t ventData;
static uint8_t nodeAddress = 0;

static Semaphore_Handle radioAccessSemHandle;
static Semaphore_Handle radioResultSemHandle;
static struct RadioOperation currentRadioOperation;
static struct DualModeVentPacket dmVentPacket;

/* previous Tick count used to calculate uptime */
static uint32_t prevTicks;



static ConcentratorRadio_PacketReceivedCallback packetReceivedCallback;
static union ConcentratorPacket latestRxPacket;
static EasyLink_TxPacket txPacket;
static struct AckPacket ackPacket;
static uint8_t concentratorAddress;
static int8_t latestRssi;


/***** Prototypes *****/
static void concentratorRadioTaskFunction(UArg arg0, UArg arg1);
static void rxDoneCallbackSensor(EasyLink_RxPacket * rxPacket, EasyLink_Status status);
static void rxDoneCallbackVent(EasyLink_RxPacket * rxPacket, EasyLink_Status status);
static void notifyPacketReceived(union ConcentratorPacket* latestRxPacket);
static void sendAck(uint8_t latestSourceAddress);

static void sendDmPacket(struct DualModeVentPacket ventPacket, uint8_t maxNumberOfRetries, uint32_t ackTimeoutMs);
static void resendPacket(void);

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/* Configure LED Pin */
PIN_Config ledPinTable[] = {
        CONCENTRATOR_ACTIVITY_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// Old Functions before trying to send data
/***** Function definitions *****/
//void ConcentratorRadioTask_init(void) {
//
//    /* Open LED pins */
//    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
//    if (!ledPinHandle)
//    {
//        System_abort("Error initializing board 3.3V domain pins\n");
//    }
//
//    /* Create event used internally for state changes */
//    Event_Params eventParam;
//    Event_Params_init(&eventParam);
//    Event_construct(&radioOperationEvent, &eventParam);
//    radioOperationEventHandle = Event_handle(&radioOperationEvent);
//
//    /* Create the concentrator radio protocol task */
//    Task_Params_init(&concentratorRadioTaskParams);
//    concentratorRadioTaskParams.stackSize = CONCENTRATORRADIO_TASK_STACK_SIZE;
//    concentratorRadioTaskParams.priority = CONCENTRATORRADIO_TASK_PRIORITY;
//    concentratorRadioTaskParams.stack = &concentratorRadioTaskStack;
//    Task_construct(&concentratorRadioTask, concentratorRadioTaskFunction, &concentratorRadioTaskParams, NULL);
//}
//
//void ConcentratorRadioTask_registerPacketReceivedCallback(ConcentratorRadio_PacketReceivedCallback callback) {
//    packetReceivedCallback = callback;
//}
//
//static void concentratorRadioTaskFunction(UArg arg0, UArg arg1)
//{
//    /* Initialize EasyLink */
//    if(EasyLink_init(RADIO_EASYLINK_MODULATION) != EasyLink_Status_Success) {
//        System_abort("EasyLink_init failed");
//    }
//
//
//    /* If you wish to use a frequency other than the default use
//     * the below API
//     * EasyLink_setFrequency(868000000);
//     */
//
//    /* Set concentrator address */;
//    concentratorAddress = RADIO_CONCENTRATOR_ADDRESS;
//    EasyLink_enableRxAddrFilter(&concentratorAddress, 1, 1);
//
//    /* Set up Ack packet */
//    ackPacket.header.sourceAddress = concentratorAddress;
//    ackPacket.header.packetType = RADIO_PACKET_TYPE_ACK_PACKET;
//
//    /* Enter receive */
//    if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
//        System_abort("EasyLink_receiveAsync failed");
//    }
//
//    while (1) {
//        uint32_t events = Event_pend(radioOperationEventHandle, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);
//
//        /* If valid packet received */
//        if(events & RADIO_EVENT_VALID_PACKET_RECEIVED) {
//
//            /* Send ack packet */
//            sendAck(latestRxPacket.header.sourceAddress);
//
//            /* Call packet received callback */
//            notifyPacketReceived(&latestRxPacket);
//
//            /* Go back to RX */
//            if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
//                System_abort("EasyLink_receiveAsync failed");
//            }
//
//            /* toggle Activity LED */
//            PIN_setOutputValue(ledPinHandle, CONCENTRATOR_ACTIVITY_LED,
//                    !PIN_getOutputValue(CONCENTRATOR_ACTIVITY_LED));
//        }
//
//        /* If invalid packet received */
//        if(events & RADIO_EVENT_INVALID_PACKET_RECEIVED) {
//            /* Go back to RX */
//            if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success) {
//                System_abort("EasyLink_receiveAsync failed");
//            }
//        }
//    }
//}
//
//static void sendAck(uint8_t latestSourceAddress) {
//
//    /* Set destinationAdress, but use EasyLink layers destination address capability */
//    txPacket.dstAddr[0] = latestSourceAddress;
//
//    /* Copy ACK packet to payload, skipping the destination adress byte.
//     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
//    memcpy(txPacket.payload, &ackPacket.header, sizeof(ackPacket));
//    txPacket.len = sizeof(ackPacket);
//
//    /* Send packet  */
//    if (EasyLink_transmit(&txPacket) != EasyLink_Status_Success)
//    {
//        System_abort("EasyLink_transmit failed");
//    }
//}
//
//static void notifyPacketReceived(union ConcentratorPacket* latestRxPacket)
//{
//    if (packetReceivedCallback)
//    {
//        packetReceivedCallback(latestRxPacket, latestRssi);
//    }
//}
//
//static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
//{
//    union ConcentratorPacket* tmpRxPacket;
//
//    /* If we received a packet successfully */
//    if (status == EasyLink_Status_Success)
//    {
//        /* Save the latest RSSI, which is later sent to the receive callback */
//        latestRssi = (int8_t)rxPacket->rssi;
//
//        /* Check that this is a valid packet */
//        tmpRxPacket = (union ConcentratorPacket*)(rxPacket->payload);
//
//        /* If this is a known packet */
//        if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_ADC_SENSOR_PACKET)
//        {
//            /* Save packet */
//            latestRxPacket.header.sourceAddress = rxPacket->payload[0];
//            latestRxPacket.header.packetType = rxPacket->payload[1];
//            latestRxPacket.adcSensorPacket.adcValue = (rxPacket->payload[2] << 8) | rxPacket->payload[3];
//
//            /* Signal packet received */
//            Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
//        }
//        else if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_DM_SENSOR_PACKET)
//        {
////Old version
////            latestRxPacket.header.sourceAddress = rxPacket->payload[0];
////            latestRxPacket.header.packetType = rxPacket->payload[1];
////            latestRxPacket.dmSensorPacket.adcValue = (rxPacket->payload[2] << 8) | rxPacket->payload[3];
////            latestRxPacket.dmSensorPacket.batt = (rxPacket->payload[4] << 8) | rxPacket->payload[5];
////            latestRxPacket.dmSensorPacket.time100MiliSec = (rxPacket->payload[6] << 24) |
////                                                                       (rxPacket->payload[7] << 16) |
////                                                                       (rxPacket->payload[8] << 8) |
////                                                                        rxPacket->payload[9];
////            latestRxPacket.dmSensorPacket.button = rxPacket->payload[10];
//            /* Save packet */
//
//            //latestRxPacket.header.sourceAddress - NodeID
//            latestRxPacket.header.sourceAddress             = rxPacket->payload[0];
//            latestRxPacket.header.packetType                = rxPacket->payload[1];
//            latestRxPacket.dmSensorPacket.adcValue          = (rxPacket->payload[2] << 8) | rxPacket->payload[3];
//            latestRxPacket.dmSensorPacket.batt              = (rxPacket->payload[4] << 8) | rxPacket->payload[5];
//            latestRxPacket.dmSensorPacket.time100MiliSec    = (rxPacket->payload[6] << 24) |
//                                                              (rxPacket->payload[7] << 16) |
//                                                              (rxPacket->payload[8] << 8)  | rxPacket->payload[9];
//            latestRxPacket.dmSensorPacket.button            = rxPacket->payload[10];
//
//            /* Signal packet received */
//            Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
//        }
//        else
//        {
//            /* Signal invalid packet received */
//            Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
//        }
//    }
//    else
//    {
//        /* Signal invalid packet received */
//        Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
//    }
//}


//----------------------------------------
// Send Data

/***** Function definitions *****/
void ConcentratorRadioTask_init(void) {

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle)
    {
        System_abort("Error initializing board 3.3V domain pins\n");
    }

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&radioOperationEvent, &eventParam);
    radioOperationEventHandle = Event_handle(&radioOperationEvent);

    /* Create the concentrator radio protocol task */
    Task_Params_init(&concentratorRadioTaskParams);
    concentratorRadioTaskParams.stackSize = CONCENTRATORRADIO_TASK_STACK_SIZE;
    concentratorRadioTaskParams.priority = CONCENTRATORRADIO_TASK_PRIORITY;
    concentratorRadioTaskParams.stack = &concentratorRadioTaskStack;
    Task_construct(&concentratorRadioTask, concentratorRadioTaskFunction, &concentratorRadioTaskParams, NULL);
}

void ConcentratorRadioTask_registerPacketReceivedCallback(ConcentratorRadio_PacketReceivedCallback callback) {
    packetReceivedCallback = callback;
}

static void concentratorRadioTaskFunction(UArg arg0, UArg arg1)
{
    /* Initialize EasyLink */
    if(EasyLink_init(RADIO_EASYLINK_MODULATION) != EasyLink_Status_Success) {
        System_abort("EasyLink_init failed");
    }


    /* If you wish to use a frequency other than the default use
    * the below API
    * EasyLink_setFrequency(868000000);
    */

    /* Set concentrator address */;
    concentratorAddress = RADIO_CONCENTRATOR_ADDRESS;
    EasyLink_enableRxAddrFilter(&concentratorAddress, 1, 1);

    /* Set up Ack packet */
    ackPacket.header.sourceAddress = concentratorAddress;
    ackPacket.header.packetType = RADIO_PACKET_TYPE_ACK_PACKET;

    /* Enter receive */
    if(EasyLink_receiveAsync(rxDoneCallbackSensor, 0) != EasyLink_Status_Success) {
        System_abort("EasyLink_receiveAsync failed");
    }

    while (1) {
        uint32_t events = Event_pend(radioOperationEventHandle, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If valid packet received */
        if(events & RADIO_EVENT_VALID_PACKET_RECEIVED) {

            /* Send ack packet */
            sendAck(latestRxPacket.header.sourceAddress);

            /* Call packet received callback */
            notifyPacketReceived(&latestRxPacket);
            /* Go back to RX */
            if(EasyLink_receiveAsync(rxDoneCallbackSensor, 0) != EasyLink_Status_Success) {
                System_abort("EasyLink_receiveAsync failed");
            }

            /* toggle Activity LED */
            PIN_setOutputValue(ledPinHandle, CONCENTRATOR_ACTIVITY_LED,
                    !PIN_getOutputValue(CONCENTRATOR_ACTIVITY_LED));
        }
        /* If invalid packet received */
        if(events & RADIO_EVENT_INVALID_PACKET_RECEIVED) {
            /* Go back to RX */
            if(EasyLink_receiveAsync(rxDoneCallbackSensor, 0) != EasyLink_Status_Success) {
                System_abort("EasyLink_receiveAsync failed");
            }
        }

        if (events & RADIO_EVENT_SEND_VENT_DATA)
        {
            uint32_t currentTicks;

            currentTicks = Clock_getTicks();
            //check for wrap around
            if (currentTicks > prevTicks)
            {
                //calculate time since last reading in 0.1s units
                dmVentPacket.time100MiliSec += ((currentTicks - prevTicks) * Clock_tickPeriod) / 100000;
            }
            else
            {
                //calculate time since last reading in 0.1s units
                dmVentPacket.time100MiliSec += ((prevTicks - currentTicks) * Clock_tickPeriod) / 100000;
            }
            prevTicks = currentTicks;

            dmVentPacket.adcValue = ventData;


            sendDmPacket(dmVentPacket, NODERADIO_MAX_RETRIES, NODERADIO_ACK_TIMEOUT_TIME_MS);
        }
        /* If we get an ACK from the concentrator */
        if (events & RADIO_EVENT_DATA_ACK_RECEIVED)
        {
            returnRadioOperationStatus(NodeRadioStatus_Success);
        }

        /* If we get an ACK timeout */
        if (events & RADIO_EVENT_ACK_TIMEOUT)
        {

            /* If we haven't resent it the maximum number of times yet, then resend packet */
            if (currentRadioOperation.retriesDone < currentRadioOperation.maxNumberOfRetries)
            {
                resendPacket();
            }
            else
            {
                /* Else return send fail */
                Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_FAIL);
            }
        }
    }
}

static void sendAck(uint8_t latestSourceAddress) {

    /* Set destinationAdress, but use EasyLink layers destination address capability */
    txPacket.dstAddr[0] = latestSourceAddress;


    /* Copy ACK packet to payload, skipping the destination adress byte.
     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
    memcpy(txPacket.payload, &ackPacket.header, sizeof(ackPacket));
    txPacket.len = sizeof(ackPacket);

    /* Send packet  */
    if (EasyLink_transmit(&txPacket) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }

}

enum NodeRadioOperationStatus NodeRadioTask_sendVentData(uint16_t data, uint8_t address)
{
    enum NodeRadioOperationStatus status;

    /* Get radio access semaphore */
    Semaphore_pend(radioAccessSemHandle, BIOS_WAIT_FOREVER);

    /* Save data to send */
    ventData = data;

    /* Raise RADIO_EVENT_SEND_ADC_DATA event */
    Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_VENT_DATA);

    /* Wait for result */
    Semaphore_pend(radioResultSemHandle, BIOS_WAIT_FOREVER);

    /* Get result */
    status = currentRadioOperation.result;

    /* Return radio access semaphore */
    Semaphore_post(radioAccessSemHandle);

    return status;
}

static void notifyPacketReceived(union ConcentratorPacket* latestRxPacket)
{
    if (packetReceivedCallback)
    {
        packetReceivedCallback(latestRxPacket, latestRssi);
    }
}

static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    union ConcentratorPacket* tmpRxPacket;

    /* If we received a packet successfully */
    if (status == EasyLink_Status_Success)
    {
        /* Save the latest RSSI, which is later sent to the receive callback */
        latestRssi = (int8_t)rxPacket->rssi;

        /* Check that this is a valid packet */
        tmpRxPacket = (union ConcentratorPacket*)(rxPacket->payload);

        /* If this is a known packet */
        if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_ADC_SENSOR_PACKET)
        {
            /* Save packet */
            latestRxPacket.header.sourceAddress             = rxPacket->payload[0];
            latestRxPacket.header.packetType                = rxPacket->payload[1];
            latestRxPacket.adcSensorPacket.adcValue         = (rxPacket->payload[2] << 8) | rxPacket->payload[3];

            /* Signal packet received */
            Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
        }
        else if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_DM_SENSOR_PACKET)
        {
//Old version
//            latestRxPacket.header.sourceAddress = rxPacket->payload[0];
//            latestRxPacket.header.packetType = rxPacket->payload[1];
//            latestRxPacket.dmSensorPacket.adcValue = (rxPacket->payload[2] << 8) | rxPacket->payload[3];
//            latestRxPacket.dmSensorPacket.batt = (rxPacket->payload[4] << 8) | rxPacket->payload[5];
//            latestRxPacket.dmSensorPacket.time100MiliSec = (rxPacket->payload[6] << 24) |
//                                                                       (rxPacket->payload[7] << 16) |
//                                                                       (rxPacket->payload[8] << 8) |
//                                                                        rxPacket->payload[9];
//            latestRxPacket.dmSensorPacket.button = rxPacket->payload[10];
            /* Save packet */

            //latestRxPacket.header.sourceAddress - NodeID
            latestRxPacket.header.sourceAddress             = rxPacket->payload[0];
            latestRxPacket.header.packetType                = rxPacket->payload[1];
            latestRxPacket.dmSensorPacket.adcValue          = (rxPacket->payload[2] << 8) | rxPacket->payload[3];


            /* Signal packet received */
            Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
        }
        else
        {
            /* Signal invalid packet received */
            Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
        }
    }
    else
    {
        /* Signal invalid packet received */
        Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
    }
}


static void sendDmPacket(struct DualModeVentPacket ventPacket, uint8_t maxNumberOfRetries, uint32_t ackTimeoutMs){
    currentRadioOperation.easyLinkTxPacket.dstAddr[0] = RADIO_CONCENTRATOR_ADDRESS;
    /* Set destination address in EasyLink API */
    currentRadioOperation.easyLinkTxPacket.dstAddr[0] = RADIO_CONCENTRATOR_ADDRESS;

    /* Copy ADC packet to payload
    * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
    currentRadioOperation.easyLinkTxPacket.payload[0] = dmVentPacket.header.sourceAddress;

    currentRadioOperation.easyLinkTxPacket.len = sizeof(struct DualModeVentPacket);

    /* Setup retries */
    currentRadioOperation.maxNumberOfRetries = NODERADIO_MAX_RETRIES;
    currentRadioOperation.ackTimeoutMs = NODERADIO_ACK_TIMEOUT_TIME_MS;
    currentRadioOperation.retriesDone = 0;
    EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(ackTimeoutMs));

    /* Send packet  */
    if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
    {
      System_abort("EasyLink_transmit failed");
    }


    /* Enter RX */
    if (EasyLink_receiveAsync(rxDoneCallbackVent, 0) != EasyLink_Status_Success)
    {
      System_abort("EasyLink_receiveAsync failed");
    }
}

static void resendPacket(void){
    /* Send packet  */
    if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }


    /* Enter RX and wait for ACK with timeout */
    if (EasyLink_receiveAsync(rxDoneCallbackVent, 0) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_receiveAsync failed");
    }

    /* Increase retries by one */
    currentRadioOperation.retriesDone++;
}

static void rxDoneCallbackVent(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    struct PacketHeader* packetHeader;

    /* If this callback is called because of a packet received */
    if (status == EasyLink_Status_Success)
    {
        /* Check the payload header */
        packetHeader = (struct PacketHeader*)rxPacket->payload;

        /* Check if this is an ACK packet */
        if (packetHeader->packetType == RADIO_PACKET_TYPE_ACK_PACKET)
        {
            /* Signal ACK packet received */
            Event_post(radioOperationEventHandle, RADIO_EVENT_DATA_ACK_RECEIVED);
        }
        else
        {
            /* Packet Error, treat as a Timeout and Post a RADIO_EVENT_ACK_TIMEOUT
               event */
            Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);
        }
    }
    /* did the Rx timeout */
    else if(status == EasyLink_Status_Rx_Timeout)
    {
        /* Post a RADIO_EVENT_ACK_TIMEOUT event */
        Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);
    }
    else
    {
        /* The Ack receiption may have been corrupted causing an error.
         * Treat this as a timeout
         */
        Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);
    }
}


