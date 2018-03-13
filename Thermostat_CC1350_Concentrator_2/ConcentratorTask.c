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
#include <stdio.h>

/* BIOS Header files */ 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayExt.h>

/* Board Header files */
#include "Board.h"

/* Application Header files */ 
#include "ConcentratorRadioTask.h"
#include "ConcentratorTask.h"
#include "RadioProtocol.h"



/***** Defines *****/
#define CONCENTRATOR_TASK_STACK_SIZE 1024
#define CONCENTRATOR_TASK_PRIORITY   3

#define CONCENTRATOR_EVENT_ALL                      0xFFFFFFFF
#define CONCENTRATOR_EVENT_NEW_ADC_SENSOR_VALUE     (uint32_t)(1 << 0)
#define CONCENTRATOR_EVENT_SEND_VENT_DATA           (uint32_t)(1 << 2)

#define CONCENTRATOR_MAX_NODES 7

#define CONCENTRATOR_DISPLAY_LINES 8

/***** Type declarations *****/
struct AdcSensorNode {
    uint8_t address;
    uint16_t latestAdcValue;
    uint8_t button;
    int8_t latestRssi;
};

struct Room {
    uint8_t     RoomActive;
    uint8_t     SensorActive;
    uint8_t     VentActive;
    uint16_t    DesiredTemp;
    uint16_t    CurrentTemp;
    uint16_t    MotionDetected;
};
struct Room Room[9];


/***** Variable declarations *****/
static Task_Params concentratorTaskParams;
Task_Struct concentratorTask;    /* not static so you can see in ROV */
static uint8_t concentratorTaskStack[CONCENTRATOR_TASK_STACK_SIZE];
Event_Struct concentratorEvent;  /* not static so you can see in ROV */
static Event_Handle concentratorEventHandle;
static struct AdcSensorNode latestActiveAdcSensorNode;
struct AdcSensorNode knownSensorNodes[CONCENTRATOR_MAX_NODES];
static struct AdcSensorNode* lastAddedSensorNode = knownSensorNodes;
static Display_Handle hDisplayLcd;
static Display_Handle hDisplaySerial;


uint8_t n = 0x01;

uint8_t RoomActiveAddress = 0x00;
uint8_t RoomFilter = 0xF0;
uint8_t DeviceFilter = 0x0F;

uint8_t SpecificRoomFilter = 0x00;
uint8_t SensorFilter = (0x08);
uint8_t VentFilter = (0x07);

/***** Prototypes *****/
static void concentratorTaskFunction(UArg arg0, UArg arg1);
static void packetReceivedCallback(union ConcentratorPacket* packet, int8_t rssi);
static void updateLcd();
static void addNewNode(struct AdcSensorNode* node);
static void updateNode(struct AdcSensorNode* node);
static uint8_t isKnownNodeAddress(uint8_t address);

uint16_t TempFilter(uint16_t AdcValue);
uint16_t MotionFilter(uint16_t AdcValue);

// Old functions before trying to send data
/***** Function definitions *****/
//void ConcentratorTask_init(void) {
//
//    /* Create event used internally for state changes */
//    Event_Params eventParam;
//    Event_Params_init(&eventParam);
//    Event_construct(&concentratorEvent, &eventParam);
//    concentratorEventHandle = Event_handle(&concentratorEvent);
//
//    /* Create the concentrator radio protocol task */
//    Task_Params_init(&concentratorTaskParams);
//    concentratorTaskParams.stackSize = CONCENTRATOR_TASK_STACK_SIZE;
//    concentratorTaskParams.priority = CONCENTRATOR_TASK_PRIORITY;
//    concentratorTaskParams.stack = &concentratorTaskStack;
//    Task_construct(&concentratorTask, concentratorTaskFunction, &concentratorTaskParams, NULL);
//}
//
//uint16_t TempFilter(uint16_t AdcValue)
//{
//    //AdcValue includes two different superimposed values from the Sensor Controller Studio
//    //AdcValue is a uint16_t
//
//    // 0b 0000 0000 0000 0000
//    //         [000 0000 0000] - Temperature 0x07FF
//    //        [0]              - Motion Detection 0x0800
//    //
//
//
//    int Temp;
//    int TempAdc;
//    int TempFilter = 0x07FF;
//
//    // Note: Temperature ADC Values are 10X the actual temperature in Celcius
//    TempAdc = AdcValue & TempFilter;
//
//    // Hence the division by 10
//    Temp = TempAdc / 10;
//}
//
//uint16_t MotionFilter(uint16_t AdcValue)
//{
//    //AdcValue includes two different superimposed values from the Sensor Controller Studio
//    //AdcValue is a uint16_t
//
//    //0b 0000 0000 0000 0000
//    //        [000 0000 0000] - Temperature 0x07FF
//    //       [0]              - Motion Detection 0x0800
//
//    int Motion;
//    int MotionAdc;
//    int MotionFilter = 0x0800;
//
//    //To get either a 1 or a 0 for motion, must divide by the placeholder
//    MotionAdc = AdcValue & MotionFilter;
//    Motion = MotionAdc / 0x0800;
//}
//static void concentratorTaskFunction(UArg arg0, UArg arg1)
//{
//
//
//    /* Initialize display and try to open both UART and LCD types of display. */
//    Display_Params params;
//    Display_Params_init(&params);
//    params.lineClearMode = DISPLAY_CLEAR_BOTH;
//
//    /* Open both an available LCD display and an UART display.
//     * Whether the open call is successful depends on what is present in the
//     * Display_config[] array of the board file.
//     *
//     * Note that for SensorTag evaluation boards combined with the SHARP96x96
//     * Watch DevPack, there is a pin conflict with UART such that one must be
//     * excluded, and UART is preferred by default. To display on the Watch
//     * DevPack, add the precompiler define BOARD_DISPLAY_EXCLUDE_UART.
//     */
//    hDisplayLcd = Display_open(Display_Type_LCD, &params);
//    hDisplaySerial = Display_open(Display_Type_UART, &params);
//
//
//    /* Check if the selected Display type was found and successfully opened */
//    if (hDisplaySerial)
//    {
//        Display_printf(hDisplaySerial, 0, 0, "Waiting for nodes...");
//    }
//
//    /* Check if the selected Display type was found and successfully opened */
//    if (hDisplayLcd)
//    {
//        Display_printf(hDisplayLcd, 0, 0, "Waiting for nodes...");
//    }
//
//    /* Register a packet received callback with the radio task */
//    ConcentratorRadioTask_registerPacketReceivedCallback(packetReceivedCallback);
//
//    /* Enter main task loop */
//    while(1) {
//        uint32_t events = Event_pend(concentratorEventHandle, 0, CONCENTRATOR_EVENT_ALL, BIOS_WAIT_FOREVER);
//
//        /* If we got a new ADC sensor value */
//        if(events & CONCENTRATOR_EVENT_NEW_ADC_SENSOR_VALUE) {
//            /* If we knew this node from before, update the value */
//            if(isKnownNodeAddress(latestActiveAdcSensorNode.address)) {
//                updateNode(&latestActiveAdcSensorNode);
//            }
//            else {
//                /* Else add it */
//                addNewNode(&latestActiveAdcSensorNode);
//            }
//
//            //Sets the Active Room Address as the latest address pulled in from the connected node
//            RoomActiveAddress = latestActiveAdcSensorNode.address;
//
//            // room n is set by doing a bitwise and operation on the top bit of the
//            // node address, which, by convention is the room number
//            n = ((RoomActiveAddress & RoomFilter)/0x10);
//
//            // Specific Room Filter adds another filtering value for getting the rest of the important values
//            SpecificRoomFilter = (0x10*n);
//
//            // Checks if the room is active by testing the filtered Active Room Address and Specific Room Filter
//            Room[n].RoomActive = ((RoomActiveAddress & RoomFilter) == SpecificRoomFilter);
//
//
//            // Gets the type of node connected (Sensor Node or Vent Node)
//            // if it is a sensor node, Gets the current temperature and Motion data
//            if((RoomActiveAddress & RoomFilter) == SpecificRoomFilter){
//                if( ((RoomActiveAddress & DeviceFilter) == SensorFilter) & ((RoomActiveAddress & RoomFilter) == SpecificRoomFilter) ){
//                    Room[n].SensorActive = 1;
//                }
//                else if(  ((RoomActiveAddress & DeviceFilter) <= VentFilter) & ((RoomActiveAddress & RoomFilter) == SpecificRoomFilter) ){
//                    Room[n].VentActive = 1;
//                }
//                else {
//                    Room[n].SensorActive = 0;
//                    Room[n].VentActive = 0;
//                }
//                if(Room[n].RoomActive){
//                    Room[n].CurrentTemp = TempFilter(latestActiveAdcSensorNode.latestAdcValue);
//                    Room[n].MotionDetected = MotionFilter(latestActiveAdcSensorNode.latestAdcValue);
//                }
//            }
//
//            /* Update the values on the LCD */
//            updateLcd(n);
//
//        }
//        /* Wait for event */
//    }
//}
//
//static void packetReceivedCallback(union ConcentratorPacket* packet, int8_t rssi)
//{
//    /* If we recived an ADC sensor packet, for backward compatibility */
//    if (packet->header.packetType == RADIO_PACKET_TYPE_ADC_SENSOR_PACKET)
//    {
//        /* Save the values */
//        latestActiveAdcSensorNode.address = packet->header.sourceAddress;
//        latestActiveAdcSensorNode.latestAdcValue = packet->adcSensorPacket.adcValue;
//        latestActiveAdcSensorNode.button = 0; //no button value in ADC packet
//        latestActiveAdcSensorNode.latestRssi = rssi;
//
//        Event_post(concentratorEventHandle, CONCENTRATOR_EVENT_NEW_ADC_SENSOR_VALUE);
//    }
//    /* If we recived an DualMode ADC sensor packet*/
//    else if(packet->header.packetType == RADIO_PACKET_TYPE_DM_SENSOR_PACKET)
//    {
//
//        /* Save the values */
//        latestActiveAdcSensorNode.address = packet->header.sourceAddress;
//        latestActiveAdcSensorNode.latestAdcValue = packet->dmSensorPacket.adcValue;
//        latestActiveAdcSensorNode.button = packet->dmSensorPacket.button;
//        latestActiveAdcSensorNode.latestRssi = rssi;
//
//        Event_post(concentratorEventHandle, CONCENTRATOR_EVENT_NEW_ADC_SENSOR_VALUE);
//    }
//}
//
//static uint8_t isKnownNodeAddress(uint8_t address) {
//    uint8_t found = 0;
//    uint8_t i;
//    for (i = 0; i < CONCENTRATOR_MAX_NODES; i++)
//    {
//        if (knownSensorNodes[i].address == address)
//        {
//            found = 1;
//            break;
//        }
//    }
//    return found;
//}
//
//static void updateNode(struct AdcSensorNode* node) {
//    uint8_t i;
//    for (i = 0; i < CONCENTRATOR_MAX_NODES; i++) {
//        if (knownSensorNodes[i].address == node->address)
//        {
//            knownSensorNodes[i].latestAdcValue = node->latestAdcValue;
//            knownSensorNodes[i].latestRssi = node->latestRssi;
//            knownSensorNodes[i].button = node->button;
//            break;
//        }
//    }
//}
//
//static void addNewNode(struct AdcSensorNode* node) {
//    *lastAddedSensorNode = *node;
//
//    /* Increment and wrap */
//    lastAddedSensorNode++;
//    if (lastAddedSensorNode > &knownSensorNodes[CONCENTRATOR_MAX_NODES-1])
//    {
//        lastAddedSensorNode = knownSensorNodes;
//    }
//}
//
//static void updateLcd() {
//    struct AdcSensorNode* nodePointer = knownSensorNodes;
//    uint8_t currentLcdLine;
//
//    /* Clear the display and write header on first line */
//    Display_clear(hDisplayLcd);
//    Display_printf(hDisplayLcd, 0, 0, "Nodes Value SW  RSSI");
//
//    //clear screen, put cuser to beggining of terminal and print the header
//    Display_printf(hDisplaySerial, 0, 0, "\033[2J \033[0;0HNodes   Value   SW    RSSI");
//
//    /* Start on the second line */
//    currentLcdLine = 1;
//
//
//    /* Write one line per node */
//    while ((nodePointer < &knownSensorNodes[CONCENTRATOR_MAX_NODES]) &&
//          (nodePointer->address != 0) &&
//          (currentLcdLine < CONCENTRATOR_DISPLAY_LINES))
//    {
//        /* print to LCD */
//        Display_printf(hDisplayLcd, currentLcdLine, 0, "0x%02x  %04d  %d   %04d",
//                nodePointer->address, nodePointer->latestAdcValue, nodePointer->button,
//                nodePointer->latestRssi);
//
//        /* print to UART */
//        Display_printf(hDisplaySerial, 0, 0, "0x%02x    %04x    %d    %04d",
//                nodePointer->address, nodePointer->latestAdcValue, nodePointer->button,
//                nodePointer->latestRssi);
//        printf("Address: 0x%02x\n   Latest ADC Value: 0x%02x\n    Button: %d\n    Latest Rssi: %04d\n",
//               nodePointer->address, nodePointer->latestAdcValue, nodePointer->button,
//               nodePointer->latestRssi); nodePointer++;
//
//
//        Display_printf(hDisplaySerial, 0, 0, "Room  Active  Sensor  Vents   Temp    Motion");
//
//
////        for(n = 1; n <=8; n++){
//            Display_printf(hDisplaySerial, 0, 0 ,"%x     %x      %x       %x       %d        %x",
//            n, Room[n].RoomActive, Room[n].SensorActive, Room[n].VentActive,
//            Room[n].CurrentTemp, Room[n].MotionDetected);
////        }
//
//
//
//        nodePointer++;
//        currentLcdLine++;
//    }
//}




//Function Declaration
void ConcentratorTask_init(void) {

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&concentratorEvent, &eventParam);
    concentratorEventHandle = Event_handle(&concentratorEvent);

    /* Create the concentrator radio protocol task */
    Task_Params_init(&concentratorTaskParams);
    concentratorTaskParams.stackSize = CONCENTRATOR_TASK_STACK_SIZE;
    concentratorTaskParams.priority = CONCENTRATOR_TASK_PRIORITY;
    concentratorTaskParams.stack = &concentratorTaskStack;
    Task_construct(&concentratorTask, concentratorTaskFunction, &concentratorTaskParams, NULL);
}

uint16_t TempFilter(uint16_t AdcValue)
{
    //AdcValue includes two different superimposed values from the Sensor Controller Studio
    //AdcValue is a uint16_t

    // 0b 0000 0000 0000 0000
    //         [000 0000 0000] - Temperature 0x07FF
    //        [0]              - Motion Detection 0x0800
    //


    int Temp;
    int TempAdc;
    int TempFilter = 0x07FF;

    // Note: Temperature ADC Values are 10X the actual temperature in Celcius
    TempAdc = AdcValue & TempFilter;

    // Hence the division by 10
    Temp = TempAdc / 10;
}

uint16_t MotionFilter(uint16_t AdcValue)
{
    //AdcValue includes two different superimposed values from the Sensor Controller Studio
    //AdcValue is a uint16_t

    //0b 0000 0000 0000 0000
    //        [000 0000 0000] - Temperature 0x07FF
    //       [0]              - Motion Detection 0x0800

    int Motion;
    int MotionAdc;
    int MotionFilter = 0x0800;

    //To get either a 1 or a 0 for motion, must divide by the placeholder
    MotionAdc = AdcValue & MotionFilter;
    Motion = MotionAdc / 0x0800;
}

static void concentratorTaskFunction(UArg arg0, UArg arg1)
{


    /* Initialize display and try to open both UART and LCD types of display. */
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;

    /* Open both an available LCD display and an UART display.
     * Whether the open call is successful depends on what is present in the
     * Display_config[] array of the board file.
     *
     * Note that for SensorTag evaluation boards combined with the SHARP96x96
     * Watch DevPack, there is a pin conflict with UART such that one must be
     * excluded, and UART is preferred by default. To display on the Watch
     * DevPack, add the precompiler define BOARD_DISPLAY_EXCLUDE_UART.
     */
    hDisplayLcd = Display_open(Display_Type_LCD, &params);
    hDisplaySerial = Display_open(Display_Type_UART, &params);


    /* Check if the selected Display type was found and successfully opened */
    if (hDisplaySerial)
    {
        Display_printf(hDisplaySerial, 0, 0, "Waiting for nodes...");
    }

    /* Check if the selected Display type was found and successfully opened */
    if (hDisplayLcd)
    {
        Display_printf(hDisplayLcd, 0, 0, "Waiting for nodes...");
    }

    /* Register a packet received callback with the radio task */
    ConcentratorRadioTask_registerPacketReceivedCallback(packetReceivedCallback);

    Room[1].DesiredTemp = 20;
    Room[2].DesiredTemp = 20;
    Room[3].DesiredTemp = 20;
    Room[4].DesiredTemp = 20;
    Room[5].DesiredTemp = 20;
    Room[6].DesiredTemp = 20;
    Room[7].DesiredTemp = 20;
    Room[8].DesiredTemp = 20;

    /* Enter main task loop */
    while(1) {
        uint32_t events = Event_pend(concentratorEventHandle, 0, CONCENTRATOR_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If we got a new ADC sensor value */
        if(events & CONCENTRATOR_EVENT_NEW_ADC_SENSOR_VALUE) {
            /* If we knew this node from before, update the value */
            if(isKnownNodeAddress(latestActiveAdcSensorNode.address)) {
                updateNode(&latestActiveAdcSensorNode);
            }
            else {
                /* Else add it */
                addNewNode(&latestActiveAdcSensorNode);
            }

            //Sets the Active Room Address as the latest address pulled in from the connected node
            RoomActiveAddress = latestActiveAdcSensorNode.address;

            // room n is set by doing a bitwise and operation on the top bit of the
            // node address, which, by convention is the room number
            n = ((RoomActiveAddress & RoomFilter)/0x10);

            // Specific Room Filter adds another filtering value for getting the rest of the important values
            SpecificRoomFilter = (0x10*n);

            // Checks if the room is active by testing the filtered Active Room Address and Specific Room Filter
            Room[n].RoomActive = ((RoomActiveAddress & RoomFilter) == SpecificRoomFilter);


            // Gets the type of node connected (Sensor Node or Vent Node)
            // if it is a sensor node, Gets the current temperature and Motion data
            if((RoomActiveAddress & RoomFilter) == SpecificRoomFilter){
                if( ((RoomActiveAddress & DeviceFilter) == SensorFilter) & ((RoomActiveAddress & RoomFilter) == SpecificRoomFilter) ){
                    Room[n].SensorActive = 1;
                    Room[n].CurrentTemp = TempFilter(latestActiveAdcSensorNode.latestAdcValue);
                    Room[n].MotionDetected = MotionFilter(latestActiveAdcSensorNode.latestAdcValue);
                }
                else if(  ((RoomActiveAddress & DeviceFilter) <= VentFilter) & ((RoomActiveAddress & RoomFilter) == SpecificRoomFilter) ){
                    Room[n].VentActive = 1;
                }
                else {
                    Room[n].SensorActive = 0;
                    Room[n].VentActive = 0;
                }
//                if(Room[n].RoomActive){
//                    Room[n].CurrentTemp = TempFilter(latestActiveAdcSensorNode.latestAdcValue);
//                    Room[n].MotionDetected = MotionFilter(latestActiveAdcSensorNode.latestAdcValue);
//                }
            }

            /* Update the values on the LCD */
            updateLcd(n);

        }
        /* Wait for event */
    }
}

static void packetReceivedCallback(union ConcentratorPacket* packet, int8_t rssi)
{
    /* If we recived an ADC sensor packet, for backward compatibility */
    if (packet->header.packetType == RADIO_PACKET_TYPE_ADC_SENSOR_PACKET)
    {
        /* Save the values */
        latestActiveAdcSensorNode.address = packet->header.sourceAddress;
        latestActiveAdcSensorNode.latestAdcValue = packet->adcSensorPacket.adcValue;
        latestActiveAdcSensorNode.button = 0; //no button value in ADC packet
        latestActiveAdcSensorNode.latestRssi = rssi;

        Event_post(concentratorEventHandle, CONCENTRATOR_EVENT_NEW_ADC_SENSOR_VALUE);
    }
    /* If we recived an DualMode ADC sensor packet*/
    else if(packet->header.packetType == RADIO_PACKET_TYPE_DM_SENSOR_PACKET)
    {

        /* Save the values */
        latestActiveAdcSensorNode.address = packet->header.sourceAddress;
        latestActiveAdcSensorNode.latestAdcValue = packet->dmSensorPacket.adcValue;
        latestActiveAdcSensorNode.button = packet->dmSensorPacket.button;
        latestActiveAdcSensorNode.latestRssi = rssi;

        Event_post(concentratorEventHandle, CONCENTRATOR_EVENT_NEW_ADC_SENSOR_VALUE);
    }
}

static uint8_t isKnownNodeAddress(uint8_t address) {
    uint8_t found = 0;
    uint8_t i;
    for (i = 0; i < CONCENTRATOR_MAX_NODES; i++)
    {
        if (knownSensorNodes[i].address == address)
        {
            found = 1;
            break;
        }
    }
    return found;
}

static void updateNode(struct AdcSensorNode* node) {
    uint8_t i;
    for (i = 0; i < CONCENTRATOR_MAX_NODES; i++) {
        if (knownSensorNodes[i].address == node->address)
        {
            knownSensorNodes[i].latestAdcValue = node->latestAdcValue;
            knownSensorNodes[i].latestRssi = node->latestRssi;
            knownSensorNodes[i].button = node->button;
            break;
        }
    }
}

static void addNewNode(struct AdcSensorNode* node) {
    *lastAddedSensorNode = *node;

    /* Increment and wrap */
    lastAddedSensorNode++;
    if (lastAddedSensorNode > &knownSensorNodes[CONCENTRATOR_MAX_NODES-1])
    {
        lastAddedSensorNode = knownSensorNodes;
    }
}

static void updateLcd() {
    struct AdcSensorNode* nodePointer = knownSensorNodes;
    uint8_t currentLcdLine;

    /* Clear the display and write header on first line */
    Display_clear(hDisplayLcd);
    Display_printf(hDisplayLcd, 0, 0, "Nodes Value SW  RSSI");

    //clear screen, put cuser to beggining of terminal and print the header
    Display_printf(hDisplaySerial, 0, 0, "\033[2J \033[0;0HNodes   Value   SW    RSSI");

    /* Start on the second line */
    currentLcdLine = 1;


    /* Write one line per node */
    while ((nodePointer < &knownSensorNodes[CONCENTRATOR_MAX_NODES]) &&
          (nodePointer->address != 0) &&
          (currentLcdLine < CONCENTRATOR_DISPLAY_LINES))
    {
        /* print to LCD */
        Display_printf(hDisplayLcd, currentLcdLine, 0, "0x%02x  %04d  %d   %04d",
                nodePointer->address, nodePointer->latestAdcValue, nodePointer->button,
                nodePointer->latestRssi);

        /* print to UART */
        Display_printf(hDisplaySerial, 0, 0, "0x%02x    %04x    %d    %04d",
                nodePointer->address, nodePointer->latestAdcValue, nodePointer->button,
                nodePointer->latestRssi);
        printf("Address: 0x%02x\n   Latest ADC Value: 0x%02x\n    Button: %d\n    Latest Rssi: %04d\n",
               nodePointer->address, nodePointer->latestAdcValue, nodePointer->button,
               nodePointer->latestRssi); nodePointer++;


        Display_printf(hDisplaySerial, 0, 0, "Room  Active  Sensor  Vents   Temp    Motion");


//        for(n = 1; n <=8; n++){
            Display_printf(hDisplaySerial, 0, 0 ,"%x     %x      %x       %x       %d        %x",
            n, Room[n].RoomActive, Room[n].SensorActive, Room[n].VentActive,
            Room[n].CurrentTemp, Room[n].MotionDetected);
//        }



        nodePointer++;
        currentLcdLine++;
    }
}
