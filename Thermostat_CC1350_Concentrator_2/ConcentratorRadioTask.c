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

/* Standard C Libraries */
#include <stdlib.h>

/* EasyLink API Header files */ 
#include "easylink/EasyLink.h"

/* Application Header files */ 
#include "RadioProtocol.h"
#include "ConcentratorRadioTask.h"


//---------------------------------------
// Defines

#define CONCENTRATORRADIO_TASK_STACK_SIZE 512
#define CONCENTRATORRADIO_TASK_PRIORITY_RX   3
#define CONCENTRATORRADIO_TASK_PRIORITY_TX   4


#define RADIO_EVENT_ALL                     0xFFFFFFFF

//RX Radio Events
#define RADIO_EVENT_VALID_PACKET_RECEIVED   (uint32_t)(1 << 0)
#define RADIO_EVENT_INVALID_PACKET_RECEIVED (uint32_t)(1 << 1)

//TX Radio Events
#define RADIO_EVENT_SEND_VENT_DATA          (uint32_t)(1 << 2)
#define RADIO_EVENT_DATA_ACK_RECEIVED       (uint32_t)(1 << 3)
#define RADIO_EVENT_ACK_TIMEOUT             (uint32_t)(1 << 4)
#define RADIO_EVENT_SEND_FAIL               (uint32_t)(1 << 5)


#define CONCENTRATORRADIO_MAX_RETRIES 2
#define CONCENTRATORRADIO_ACK_TIMEOUT_TIME_MS (160)


#define CONCENTRATOR_ACTIVITY_LED_RX Board_PIN_LED0
#define CONCENTRATOR_ACTIVITY_LED_TX Board_PIN_LED1


//---------------------------------------
//  Type Declaration

//RX Type

//TX Type
struct RadioOperation {
    EasyLink_TxPacket easyLinkTxPacket;
    uint8_t retriesDone;
    uint8_t maxNumberOfRetries;
    uint32_t ackTimeoutMs;
    enum ConcentratorRadioOperationStatus_TX result;
};


//---------------------------------------
//  Variable Declaration



//  RX Variables
static Task_Params concentratorRadioTaskParams_RX;
Task_Struct concentratorRadioTask_RX; /* not static so you can see in ROV */
static uint8_t concentratorRadioTaskStack_RX[CONCENTRATORRADIO_TASK_STACK_SIZE];
Event_Struct radioOperationEvent_RX;  /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle_RX;
static ConcentratorRadio_PacketReceivedCallback packetReceivedCallback;
static union ConcentratorPacket latestRxPacket;
static EasyLink_TxPacket txPacket_RX;
static struct AckPacket ackPacket_RX;
static uint8_t concentratorAddress;
static int8_t latestRssi;

//  TX Variables
static Task_Params concentratorRadioTaskParams_TX;
Task_Struct concentratorRadioTask_TX;        /* not static so you can see in ROV */
static uint8_t concentratorRadioTaskStack_TX[CONCENTRATORRADIO_TASK_STACK_SIZE];
Semaphore_Struct radioAccessSem_TX;  /* not static so you can see in ROV */
static Semaphore_Handle radioAccessSemHandle_TX;
Event_Struct radioOperationEvent_TX; /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle_TX;
Semaphore_Struct radioResultSem_TX;  /* not static so you can see in ROV */
static Semaphore_Handle radioResultSemHandle_TX;
static struct RadioOperation currentRadioOperation;
static uint16_t ventData;
static uint8_t ventAddress = 0;
static struct DualModeVentPacket dmVentPacket;

static uint32_t prevTicks;

//---------------------------------------
//  Prototypes

//  RX Prototypes
static void concentratorRadioTaskFunction_RX(UArg arg0, UArg arg1);
static void rxDoneCallback_RX(EasyLink_RxPacket * rxPacket_RX, EasyLink_Status status);
static void notifyPacketReceived(union ConcentratorPacket* latestRxPacket);
static void sendAck(uint8_t latestSourceAddress);


//  TX Prototypes
static void concentratorRadioTaskFunction_TX(UArg arg0, UArg arg1);
static void returnRadioOperationStatus_TX(enum ConcentratorRadioOperationStatus_TX status);
static void sendDmPacket(struct DualModeVentPacket ventPacket, uint8_t maxNumberOfRetries, uint32_t ackTimeoutMs);
static void resendPacket(void);
static void rxDoneCallback_TX(EasyLink_RxPacket * rxPacket_TX, EasyLink_Status status);


/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/* Configure LED Pin */
PIN_Config ledPinTable[] = {
        CONCENTRATOR_ACTIVITY_LED_RX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};


//-----------------------------------------------
// Concentrator Receive Task Functions
void ConcentratorRadioTask_init_RX(void) {

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle)
    {
        System_abort("Error initializing board 3.3V domain pins\n");
    }

    /* Create event used internally for state changes */
    Event_Params eventParam_RX;
    Event_Params_init(&eventParam_RX);
    Event_construct(&radioOperationEvent_RX, &eventParam_RX);
    radioOperationEventHandle_RX = Event_handle(&radioOperationEvent_RX);

    /* Create the concentrator radio protocol task */
    Task_Params_init(&concentratorRadioTaskParams_RX);
    concentratorRadioTaskParams_RX.stackSize = CONCENTRATORRADIO_TASK_STACK_SIZE;
    concentratorRadioTaskParams_RX.priority = CONCENTRATORRADIO_TASK_PRIORITY_RX;
    concentratorRadioTaskParams_RX.stack = &concentratorRadioTaskStack_RX;
    Task_construct(&concentratorRadioTask_RX, concentratorRadioTaskFunction_RX, &concentratorRadioTaskParams_RX, NULL);
}

void ConcentratorRadioTask_registerPacketReceivedCallback(ConcentratorRadio_PacketReceivedCallback callback) {
    packetReceivedCallback = callback;
}

static void concentratorRadioTaskFunction_RX(UArg arg0, UArg arg1)
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
    ackPacket_RX.header.sourceAddress = concentratorAddress;
    ackPacket_RX.header.packetType = RADIO_PACKET_TYPE_ACK_PACKET;

    /* Enter receive */
    if(EasyLink_receiveAsync(rxDoneCallback_RX, 0) != EasyLink_Status_Success) {
        System_abort("EasyLink_receiveAsync failed");
    }

    while (1) {
        uint32_t events = Event_pend(radioOperationEventHandle_RX, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If valid packet received */
        if(events & RADIO_EVENT_VALID_PACKET_RECEIVED) {

            /* Send ack packet */
            sendAck(latestRxPacket.header.sourceAddress);

            /* Call packet received callback */
            notifyPacketReceived(&latestRxPacket);
            /* Go back to RX */
            if(EasyLink_receiveAsync(rxDoneCallback_RX, 0) != EasyLink_Status_Success) {
                System_abort("EasyLink_receiveAsync failed");
            }

            /* toggle Activity LED */
            PIN_setOutputValue(ledPinHandle, CONCENTRATOR_ACTIVITY_LED_RX,
                    !PIN_getOutputValue(CONCENTRATOR_ACTIVITY_LED_RX));
        }

        /* If invalid packet received */
        if(events & RADIO_EVENT_INVALID_PACKET_RECEIVED) {
            /* Go back to RX */
            if(EasyLink_receiveAsync(rxDoneCallback_RX, 0) != EasyLink_Status_Success) {
                System_abort("EasyLink_receiveAsync failed");
            }
        }
    }
}

static void sendAck(uint8_t latestSourceAddress) {

    /* Set destinationAdress, but use EasyLink layers destination address capability */
    txPacket_RX.dstAddr[0] = latestSourceAddress;


    /* Copy ACK packet to payload, skipping the destination adress byte.
     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
    memcpy(txPacket_RX.payload, &ackPacket_RX.header, sizeof(ackPacket_RX));
    txPacket_RX.len = sizeof(ackPacket_RX);

    /* Send packet  */
    if (EasyLink_transmit(&txPacket_RX) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }

}

static void notifyPacketReceived(union ConcentratorPacket* latestRxPacket)
{
    if (packetReceivedCallback)
    {
        packetReceivedCallback(latestRxPacket, latestRssi);
    }
}

static void rxDoneCallback_RX(EasyLink_RxPacket * rxPacket_RX, EasyLink_Status status)
{
    union ConcentratorPacket* tmpRxPacket;

    /* If we received a packet successfully */
    if (status == EasyLink_Status_Success)
    {
        /* Save the latest RSSI, which is later sent to the receive callback */
        latestRssi = (int8_t)rxPacket_RX->rssi;

        /* Check that this is a valid packet */
        tmpRxPacket = (union ConcentratorPacket*)(rxPacket_RX->payload);

        /* If this is a known packet */
        if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_ADC_SENSOR_PACKET)
        {
            /* Save packet */
            latestRxPacket.header.sourceAddress             = rxPacket_RX->payload[0];
            latestRxPacket.header.packetType                = rxPacket_RX->payload[1];
            latestRxPacket.adcSensorPacket.adcValue         = (rxPacket_RX->payload[2] << 8) | rxPacket_RX->payload[3];

            /* Signal packet received */
            Event_post(radioOperationEventHandle_RX, RADIO_EVENT_VALID_PACKET_RECEIVED);
        }
        else if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_DM_SENSOR_PACKET)
        {


            //latestRxPacket.header.sourceAddress - NodeID
            latestRxPacket.header.sourceAddress             = rxPacket_RX->payload[0];
            latestRxPacket.header.packetType                = rxPacket_RX->payload[1];
            latestRxPacket.dmSensorPacket.adcValue          = (rxPacket_RX->payload[2] << 8) | rxPacket_RX->payload[3];
            latestRxPacket.dmSensorPacket.batt              = (rxPacket_RX->payload[4] << 8) | rxPacket_RX->payload[5];
            latestRxPacket.dmSensorPacket.time100MiliSec    = (rxPacket_RX->payload[6] << 24) |
                                                              (rxPacket_RX->payload[7] << 16) |
                                                              (rxPacket_RX->payload[8] << 8)  | rxPacket_RX->payload[9];
            latestRxPacket.dmSensorPacket.button            = rxPacket_RX->payload[10];

            /* Signal packet received */
            Event_post(radioOperationEventHandle_RX, RADIO_EVENT_VALID_PACKET_RECEIVED);
        }
        else
        {
            /* Signal invalid packet received */
            Event_post(radioOperationEventHandle_RX, RADIO_EVENT_INVALID_PACKET_RECEIVED);
        }
    }
    else
    {
        /* Signal invalid packet received */
        Event_post(radioOperationEventHandle_RX, RADIO_EVENT_INVALID_PACKET_RECEIVED);
    }
}


//-----------------------------------------------
// Concentrator Send Task
void ConcentratorRadioTask_init_TX(void) {

    /* Create semaphore used for exclusive radio access */
    Semaphore_Params semParam_TX;
    Semaphore_Params_init(&semParam_TX);
    Semaphore_construct(&radioAccessSem_TX, 1, &semParam_TX);
    radioAccessSemHandle_TX = Semaphore_handle(&radioAccessSem_TX);

    /* Create semaphore used for callers to wait for result */
    Semaphore_construct(&radioResultSem_TX, 0, &semParam_TX);
    radioAccessSemHandle_TX = Semaphore_handle(&radioResultSem_TX);

    /* Create event used internally for state changes */
    Event_Params eventParam_TX;
    Event_Params_init(&eventParam_TX);
    Event_construct(&radioOperationEvent_TX, &eventParam_TX);
    radioOperationEventHandle_TX = Event_handle(&radioOperationEvent_TX);

    /* Create the radio protocol task */
    Task_Params_init(&concentratorRadioTaskParams_TX);
    concentratorRadioTaskParams_TX.stackSize = CONCENTRATORRADIO_TASK_STACK_SIZE;
    concentratorRadioTaskParams_TX.priority = CONCENTRATORRADIO_TASK_PRIORITY_TX;
    concentratorRadioTaskParams_TX.stack = &concentratorRadioTaskStack_TX;
    Task_construct(&concentratorRadioTask_TX, concentratorRadioTaskFunction_TX, &concentratorRadioTaskParams_TX, NULL);
}


static void concentratorRadioTaskFunction_TX(UArg arg0, UArg arg1)
{
#ifdef FEATURE_BLE_ADV
    BleAdv_Params_t bleAdv_Params;
    /* Set mulitclient mode for EasyLink */
    EasyLink_setCtrl(EasyLink_Ctrl_MultiClient_Mode, 1);

#endif

    EasyLink_Params easyLink_Params;
    EasyLink_Params_init(&easyLink_Params);
    //EasyLink_TxPacket pkt;

#ifdef FEATURE_BLE_ADV
    easyLink_Params.pClientEventCb = &rfSwitchCallback;
    easyLink_Params.ui32ModType = RADIO_EASYLINK_MODULATION;
    easyLink_Params.nClientEventMask = RF_ClientEventSwitchClientEntered;

    /* Initialize EasyLink */
    if(EasyLink_init_multimode(&easyLink_Params) != EasyLink_Status_Success){
        System_abort("EasyLink_init failed");
    }
#else
    if (EasyLink_init(RADIO_EASYLINK_MODULATION) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_init failed");
    }
#endif

    /* If you wich to use a frequency other than the default use
     * the below API
     * EasyLink_setFrequency(868000000);
     */

    /* Use the True Random Number Generator to generate sensor node address randomly */;
//    Power_setDependency(PowerCC26XX_PERIPH_TRNG);
//    TRNGEnable();
    /* Do not accept the same address as the concentrator, in that case get a new random value */

//    do
//    {
//        while (!(TRNGStatusGet() & TRNG_NUMBER_READY))
//        {
//            //wait for random number generator
//        }
//        //nodeAddress = (uint8_t)TRNGNumberGet(TRNG_LOW_WORD);
//        nodeAddress = (uint8_t) 0x28;
//    } while (nodeAddress == RADIO_CONCENTRATOR_ADDRESS);


    // This could possibly cause problems since the concentrator address is usually 0x00
    // maybe keeping it 0x00 wont have any adverse effects
    concentratorAddress = (uint8_t) 0x00;


//    TRNGDisable();
//    Power_releaseDependency(PowerCC26XX_PERIPH_TRNG);

    /* Set the filter to the generated random address */
    if (EasyLink_enableRxAddrFilter(&concentratorAddress, 1, 1) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_enableRxAddrFilter failed");
    }

    /* Setup ADC sensor packet */
    dmVentPacket.header.sourceAddress = ventAddress;
    dmVentPacket.header.packetType = RADIO_PACKET_TYPE_DM_SENSOR_PACKET;

    /* Initialise previous Tick count used to calculate uptime for the TLM beacon */
    prevTicks = Clock_getTicks();

#ifdef FEATURE_BLE_ADV
    /* Initialize the Simple Beacon module wit default params */
    BleAdv_Params_init(&bleAdv_Params);
    bleAdv_Params.pfnPostEvtProxyCB = bleAdv_eventProxyCB;
    bleAdv_Params.pfnUpdateTlmCB = bleAdv_updateTlmCB;
    bleAdv_Params.pfnUpdateMsButtonCB = bleAdv_updateMsButtonCB;
    bleAdv_Params.pfnAdvStatsCB = NodeTask_advStatsCB;
    BleAdv_init(&bleAdv_Params);

    /* initialize BLE advertisements to default to MS */
    BleAdv_setAdvertiserType(BleAdv_AdertiserMs);
#endif

    /* Enter main task loop */
    while (1)
    {
        /* Wait for an event */
        uint32_t events = Event_pend(radioOperationEventHandle_TX, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If we should send ADC data */
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


            sendDmPacket(dmVentPacket, CONCENTRATORRADIO_MAX_RETRIES, CONCENTRATORRADIO_ACK_TIMEOUT_TIME_MS);
        }

        /* If we get an ACK from the concentrator */
        if (events & RADIO_EVENT_DATA_ACK_RECEIVED)
        {
            returnRadioOperationStatus_TX(ConcentratorRadioStatus_TX_Success);
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
                Event_post(radioOperationEventHandle_TX, RADIO_EVENT_SEND_FAIL);
            }
        }

        /* If send fail */
        if (events & RADIO_EVENT_SEND_FAIL)
        {
            returnRadioOperationStatus_TX(ConcentratorRadioStatus_TX_Failed);
        }

#ifdef FEATURE_BLE_ADV
        if (events & NODE_EVENT_UBLE)
        {
            uble_processMsg();
        }
#endif
    }
}

enum ConcentratorRadioOperationStatus_TX ConcentratorRadioTask_sendVentData(uint8_t currentTemp, uint8_t desiredTemp, uint8_t roomNum)
{
    enum ConcentratorRadioOperationStatus_TX status;

    /* Get radio access semaphore */
    Semaphore_pend(radioAccessSemHandle_TX, BIOS_WAIT_FOREVER);

    /* Save data to send */

    //bit-shifting two data points in
    ventData = (uint16_t)currentTemp + ((uint16_t)desiredTemp * 0x0100);

    /* Raise RADIO_EVENT_SEND_ADC_DATA event */
    Event_post(radioOperationEventHandle_TX, RADIO_EVENT_SEND_VENT_DATA);

    /* Wait for result */
    Semaphore_pend(radioResultSemHandle_TX, BIOS_WAIT_FOREVER);

    /* Get result */
    status = currentRadioOperation.result;

    /* Return radio access semaphore */
    Semaphore_post(radioAccessSemHandle_TX);

    return status;
}

static void returnRadioOperationStatus_TX(enum ConcentratorRadioOperationStatus_TX result)
{
    /* Save result */
    currentRadioOperation.result = result;

    /* Post result semaphore */
    Semaphore_post(radioResultSemHandle_TX);
}

static void sendDmPacket(struct DualModeVentPacket ventPacket, uint8_t maxNumberOfRetries, uint32_t ackTimeoutMs)
{
    /* Set destination address in EasyLink API */
    currentRadioOperation.easyLinkTxPacket.dstAddr[0] = ventAddress;

    /* Copy ADC packet to payload
     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
    currentRadioOperation.easyLinkTxPacket.payload[0] = dmVentPacket.header.sourceAddress;
    currentRadioOperation.easyLinkTxPacket.payload[1] = dmVentPacket.header.packetType;
    currentRadioOperation.easyLinkTxPacket.payload[2] = (dmVentPacket.ventData & 0xFF00) >> 8;
    currentRadioOperation.easyLinkTxPacket.payload[3] = (dmVentPacket.ventData & 0xFF);
    currentRadioOperation.easyLinkTxPacket.payload[4] = (dmVentPacket.time100MiliSec & 0xFF000000) >> 24;
    currentRadioOperation.easyLinkTxPacket.payload[5] = (dmVentPacket.time100MiliSec & 0x00FF0000) >> 16;
    currentRadioOperation.easyLinkTxPacket.payload[6] = (dmVentPacket.time100MiliSec & 0xFF00) >> 8;
    currentRadioOperation.easyLinkTxPacket.payload[7] = (dmVentPacket.time100MiliSec & 0xFF);

    currentRadioOperation.easyLinkTxPacket.len = sizeof(struct DualModeVentPacket);

    /* Setup retries */
    currentRadioOperation.maxNumberOfRetries = maxNumberOfRetries;
    currentRadioOperation.ackTimeoutMs = ackTimeoutMs;
    currentRadioOperation.retriesDone = 0;
    EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(ackTimeoutMs));

    /* Send packet  */
    if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }


    /* Enter RX */
    if (EasyLink_receiveAsync(rxDoneCallback_TX, 0) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_receiveAsync failed");
    }
}

static void resendPacket(void)
{
    /* Send packet  */
    if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }


    /* Enter RX and wait for ACK with timeout */
    if (EasyLink_receiveAsync(rxDoneCallback_TX, 0) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_receiveAsync failed");
    }

    /* Increase retries by one */
    currentRadioOperation.retriesDone++;
}


static void rxDoneCallback_TX(EasyLink_RxPacket * rxPacket_TX, EasyLink_Status status)
{
    struct PacketHeader* packetHeader;



    /* If this callback is called because of a packet received */
    if (status == EasyLink_Status_Success)
    {
        /* Check the payload header */
        packetHeader = (struct PacketHeader*)rxPacket_TX->payload;

        /* Check if this is an ACK packet */
        if (packetHeader->packetType == RADIO_PACKET_TYPE_ACK_PACKET)
        {
            /* Signal ACK packet received */
            Event_post(radioOperationEventHandle_TX, RADIO_EVENT_DATA_ACK_RECEIVED);
        }
        else
        {
            /* Packet Error, treat as a Timeout and Post a RADIO_EVENT_ACK_TIMEOUT
               event */
            Event_post(radioOperationEventHandle_TX, RADIO_EVENT_ACK_TIMEOUT);
        }
    }
    /* did the Rx timeout */
    else if(status == EasyLink_Status_Rx_Timeout)
    {
        /* Post a RADIO_EVENT_ACK_TIMEOUT event */
        Event_post(radioOperationEventHandle_TX, RADIO_EVENT_ACK_TIMEOUT);
    }
    else
    {
        /* The Ack receiption may have been corrupted causing an error.
         * Treat this as a timeout
         */
        Event_post(radioOperationEventHandle_TX, RADIO_EVENT_ACK_TIMEOUT);
    }
}
