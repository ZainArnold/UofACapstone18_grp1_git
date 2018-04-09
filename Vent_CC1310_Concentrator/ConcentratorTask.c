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

//-------------------------------------------------------
//  Includes
/* XDCtools Header files */ 
#include <xdc/std.h>
#include <xdc/runtime/System.h>

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




//-------------------------------------------------------
//  Defines
#define CONCENTRATOR_TASK_STACK_SIZE 1024
#define CONCENTRATOR_TASK_PRIORITY   3

#define CONCENTRATOR_EVENT_ALL                      0xFFFFFFFF
#define CONCENTRATOR_EVENT_NEW_ADC_SENSOR_VALUE     (uint32_t)(1 << 0)
#define CONCENTRATOR_EVENT_NEW_THERMOSTAT_VALUE     (uint32_t)(1 << 1)

#define CONCENTRATOR_MAX_NODES 7



#define CONCENTRATOR_DISPLAY_LINES 8

//-------------------------------------------------------
//  Type Declarations
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
    uint16_t    TempFlag;
    uint16_t    MotionDetected;
};
struct Room Room[9];


struct ThermostatNode {
    uint8_t address;

    uint16_t roomTempC1;
    uint16_t roomTempD1;

    uint16_t roomTempC2;
    uint16_t roomTempD2;

    uint16_t roomTempC3;
    uint16_t roomTempD3;

    uint16_t roomTempC4;
    uint16_t roomTempD4;

    uint16_t roomTempC5;
    uint16_t roomTempD5;

    uint16_t roomTempC6;
    uint16_t roomTempD6;

    uint16_t roomTempC7;
    uint16_t roomTempD7;

    uint16_t roomTempC8;
    uint16_t roomTempD8;

    uint8_t button;
    int8_t latestRssi;
};


//-------------------------------------------------------
//  Variable Declarations
static Task_Params concentratorTaskParams;
Task_Struct concentratorTask;    /* not static so you can see in ROV */
static uint8_t concentratorTaskStack[CONCENTRATOR_TASK_STACK_SIZE];
Event_Struct concentratorEvent;  /* not static so you can see in ROV */
static Event_Handle concentratorEventHandle;
static struct AdcSensorNode latestActiveAdcSensorNode;
static struct ThermostatNode latestActiveThermostatNode;
struct AdcSensorNode knownSensorNodes[CONCENTRATOR_MAX_NODES];
struct ThermostatNode knownThermostatNodes[CONCENTRATOR_MAX_NODES];
static struct AdcSensorNode* lastAddedSensorNode = knownSensorNodes;
static struct ThermostatNode* lastAddedThermostatNode = knownThermostatNodes;
static Display_Handle hDisplayLcd;
static Display_Handle hDisplaySerial;

uint8_t ventAddress = 0x20;
uint8_t n = 0x01;

uint8_t RoomActiveAddress = 0x00;
uint8_t RoomFilter = 0xF0;
uint8_t DeviceFilter = 0x0F;

uint8_t SpecificRoomFilter = 0x00;
uint8_t SensorFilter = (0x08);
uint8_t VentFilter = (0x07);

/* Period and duty in microseconds */
//
//uint16_t   duty = 450;
//uint16_t   dutyInc = 50;
//
///* Sleep time in microseconds */
//uint32_t   slptime = 50000;


//-------------------------------------------------------
//  Prototypes
static void concentratorTaskFunction(UArg arg0, UArg arg1);
static void packetReceivedCallback(union ConcentratorPacket* packet, int8_t rssi);
static void updateLcd(void);
static void addNewNode(struct AdcSensorNode* AdcNode, struct ThermostatNode* ThermostatNode);
static void updateNode(struct AdcSensorNode* AdcNode, struct ThermostatNode* ThermostatNode);
static uint8_t isKnownNodeAddress(uint8_t AdcAddress, uint8_t ThermostatAddress);

uint16_t TempFilter(uint16_t AdcValue);
uint16_t MotionFilter(uint16_t AdcValue);

//-------------------------------------------------------
//  Function Definitions
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

//    PWM_init();
//    PWM_Params_init(&PWMparams);
//    PWMparams.dutyUnits = PWM_DUTY_US;
//    PWMparams.dutyValue = 0;
//    PWMparams.periodUnits = PWM_PERIOD_US;
//    PWMparams.periodValue = pwmPeriod;
//    pwmPeriod = 10000;

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
    Temp = TempAdc;
    return(Temp);
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
    return(Motion);
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

    /* Period and duty in microseconds */
    uint16_t   pwmPeriod = 10000;
    uint16_t   duty = 1000;
    uint16_t   dutyInc = 100;

    /* Sleep time in microseconds */
    uint32_t   time = 50000;
    PWM_Handle pwm1 = NULL;
//    PWM_Handle pwm2 = NULL;
    PWM_Params PWMparams;

    /* Call driver init functions. */
    PWM_init();

    PWM_Params_init(&PWMparams);
    PWMparams.dutyUnits = PWM_DUTY_US;
    PWMparams.dutyValue = 0;
    PWMparams.periodUnits = PWM_PERIOD_US;
    PWMparams.periodValue = pwmPeriod;
    pwm1 = PWM_open(Board_PWM0, &PWMparams);





//    if (Board_PWM1 != Board_PWM0) {
//        pwm2 = PWM_open(Board_PWM1, &params);
//
//        if (pwm2 == NULL) {
//            /* Board_PWM0 did not open */
//            while (1);
//        }
//
//        PWM_start(pwm2);
//    }

    if (pwm1 == NULL) {
        /* Board_PWM0 did not open */
        while (1);
    }
    PWM_start(pwm1);
//    duty = 1000;
//    PWM_setDuty(pwm1, duty);
//    usleep(time);



    /* Enter main task loop */
    while(1) {


        /* Wait for event */
        uint32_t events = Event_pend(concentratorEventHandle, 0, CONCENTRATOR_EVENT_ALL, BIOS_WAIT_FOREVER);


//        if (duty >= 1900 || (!duty)) {
//            //dutyInc = -dutyInc;
//            duty = 1000;
//        }
//        else if (duty <= 1000 || (!duty)) {
//            //dutyInc = -dutyInc;
//            duty = 1900;
//        }
//        else if ((duty > 1000 & duty < 1900) || (!duty))
//        {
//            duty = 1000;
//        }
//        PWM_setDuty(pwm1, duty);
//        usleep(time);

        /* If we got a new ADC sensor value */
        if(events & CONCENTRATOR_EVENT_NEW_ADC_SENSOR_VALUE) {

            /* If we knew this node from before, update the value */
            if(isKnownNodeAddress(latestActiveAdcSensorNode.address, NULL)) {
                updateNode(&latestActiveAdcSensorNode, NULL);
            }
            else {
                /* Else add it */
                addNewNode(&latestActiveAdcSensorNode, NULL);
            }


            //Sets the Active Room Address as the latest address pulled in from the connected node
            RoomActiveAddress = latestActiveAdcSensorNode.address;

            // room n is set by doing a bitwise and operation on the top bit of the
            // node address, which, by convention is the room number
            //n = ((RoomActiveAddress & RoomFilter)/0x10);
            n = (ventAddress / 0x10);

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
                    //ventData = Room[n].CurrentTemp + (Room[n].DesiredTemp * 0x0100);
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
            updateLcd();
        }
        if(events & CONCENTRATOR_EVENT_NEW_THERMOSTAT_VALUE) {
            /* If we knew this node from before, update the value */
            if(isKnownNodeAddress(NULL, latestActiveThermostatNode.address)) {
                updateNode(NULL, &latestActiveThermostatNode);

            }
            else {
                /* Else add it */
                addNewNode(NULL, &latestActiveThermostatNode);
            }
            //                Room[1].DesiredTemp = latestActiveThermostatNode.roomTempD1;
                            Room[2].DesiredTemp = latestActiveThermostatNode.roomTempD2;
            //                Room[3].DesiredTemp = latestActiveThermostatNode.roomTempD3;
            //                Room[4].DesiredTemp = latestActiveThermostatNode.roomTempD4;
            //                Room[5].DesiredTemp = latestActiveThermostatNode.roomTempD5;
            //                Room[6].DesiredTemp = latestActiveThermostatNode.roomTempD6;
            //                Room[7].DesiredTemp = latestActiveThermostatNode.roomTempD7;
            //                Room[8].DesiredTemp = latestActiveThermostatNode.roomTempD8;

//
//            for(n = 1; n < 9; n++){
//                if (Room[n].CurrentTemp < Room[n].DesiredTemp){
//                    Room[n].TempFlag = 1;
//                }
//            }
            /* Update the values on the LCD */
            updateLcd();
//            usleep(slptime);
        }
        if (Room[n].CurrentTemp < Room[n].DesiredTemp){
            duty = 1000;
            PWM_setDuty(pwm1, duty);
            usleep(time);
        }
        else if (Room[n].CurrentTemp >= Room[n].DesiredTemp){
            duty = 1900;
            PWM_setDuty(pwm1, duty);
            usleep(time);
        }
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
    else if(packet->header.packetType == RADIO_PACKET_TYPE_DM_THERMOSTAT_PACKET)
    {

        /* Save the values */
        latestActiveThermostatNode.address = packet->header.sourceAddress;
//
//        latestActiveThermostatNode.roomTempC1 = packet->dmThermostatPacket.roomTempC1;
//        latestActiveThermostatNode.roomTempD1 = packet->dmThermostatPacket.roomTempD1;

        latestActiveThermostatNode.roomTempC2 = packet->dmThermostatPacket.roomTempC2;
        latestActiveThermostatNode.roomTempD2 = packet->dmThermostatPacket.roomTempD2;

//        latestActiveThermostatNode.roomTempC3 = packet->dmThermostatPacket.roomTempC3;
//        latestActiveThermostatNode.roomTempD3 = packet->dmThermostatPacket.roomTempD3;
//
//        latestActiveThermostatNode.roomTempC4 = packet->dmThermostatPacket.roomTempC4;
//        latestActiveThermostatNode.roomTempD4 = packet->dmThermostatPacket.roomTempD4;
//
//        latestActiveThermostatNode.roomTempC5 = packet->dmThermostatPacket.roomTempC5;
//        latestActiveThermostatNode.roomTempD5 = packet->dmThermostatPacket.roomTempD5;
//
//        latestActiveThermostatNode.roomTempC6 = packet->dmThermostatPacket.roomTempC6;
//        latestActiveThermostatNode.roomTempD6 = packet->dmThermostatPacket.roomTempD6;
//
//        latestActiveThermostatNode.roomTempC7 = packet->dmThermostatPacket.roomTempC7;
//        latestActiveThermostatNode.roomTempD7 = packet->dmThermostatPacket.roomTempD7;
//
//        latestActiveThermostatNode.roomTempC8 = packet->dmThermostatPacket.roomTempC8;
//        latestActiveThermostatNode.roomTempD8 = packet->dmThermostatPacket.roomTempD8;

        latestActiveThermostatNode.button = packet->dmSensorPacket.button;
        latestActiveThermostatNode.latestRssi = rssi;

        Event_post(concentratorEventHandle, CONCENTRATOR_EVENT_NEW_THERMOSTAT_VALUE);
    }
}

static uint8_t isKnownNodeAddress(uint8_t AdcAddress, uint8_t ThermostatAddress) {
    uint8_t found = 0;
    uint8_t i;
    if(AdcAddress != NULL) {
        for (i = 0; i < CONCENTRATOR_MAX_NODES; i++)
        {
            if (knownSensorNodes[i].address == AdcAddress)
            {
                found = 1;
                break;
            }
        }
    }
    else if (ThermostatAddress != NULL) {
        for (i = 0; i < CONCENTRATOR_MAX_NODES; i++)
        {
            if (knownThermostatNodes[i].address == ThermostatAddress)
            {
                found = 1;
                break;
            }
        }
    }
    return found;
}

static void updateNode(struct AdcSensorNode* AdcNode, struct ThermostatNode* ThermostatNode) {
    uint8_t i;
    if (AdcNode != NULL){
        for (i = 0; i < CONCENTRATOR_MAX_NODES; i++) {
            if (knownSensorNodes[i].address == AdcNode->address)
            {
                knownSensorNodes[i].latestAdcValue = AdcNode->latestAdcValue;
                knownSensorNodes[i].latestRssi = AdcNode->latestRssi;
                knownSensorNodes[i].button = AdcNode->button;
                break;
            }
        }
    }
    else if (ThermostatNode != NULL){
        for (i = 0; i < CONCENTRATOR_MAX_NODES; i++) {
            if (knownThermostatNodes[i].address == ThermostatNode->address)
            {
                knownThermostatNodes[i].roomTempC1 = ThermostatNode->roomTempC1;
                knownThermostatNodes[i].roomTempD1 = ThermostatNode->roomTempD1;

                knownThermostatNodes[i].roomTempC2 = ThermostatNode->roomTempC2;
                knownThermostatNodes[i].roomTempD2 = ThermostatNode->roomTempD2;

                knownThermostatNodes[i].roomTempC3 = ThermostatNode->roomTempC3;
                knownThermostatNodes[i].roomTempD3 = ThermostatNode->roomTempD3;

                knownThermostatNodes[i].roomTempC4 = ThermostatNode->roomTempC4;
                knownThermostatNodes[i].roomTempD4 = ThermostatNode->roomTempD4;

                knownThermostatNodes[i].roomTempC5 = ThermostatNode->roomTempC5;
                knownThermostatNodes[i].roomTempD5 = ThermostatNode->roomTempD5;

                knownThermostatNodes[i].roomTempC6 = ThermostatNode->roomTempC6;
                knownThermostatNodes[i].roomTempD6 = ThermostatNode->roomTempD6;

                knownThermostatNodes[i].roomTempC7 = ThermostatNode->roomTempC7;
                knownThermostatNodes[i].roomTempD7 = ThermostatNode->roomTempD7;

                knownThermostatNodes[i].roomTempC8 = ThermostatNode->roomTempC8;
                knownThermostatNodes[i].roomTempD8 = ThermostatNode->roomTempD8;

                knownThermostatNodes[i].latestRssi = ThermostatNode->latestRssi;
                knownThermostatNodes[i].button = ThermostatNode->button;
                break;
            }
        }
    }
}

static void addNewNode(struct AdcSensorNode* AdcNode, struct ThermostatNode* ThermostatNode) {
    if (AdcNode != NULL){
        *lastAddedSensorNode = *AdcNode;

        /* Increment and wrap */
        lastAddedSensorNode++;
        if (lastAddedSensorNode > &knownSensorNodes[CONCENTRATOR_MAX_NODES-1])
        {
            lastAddedSensorNode = knownSensorNodes;
        }
    }
    else if (ThermostatNode != NULL){
        *lastAddedThermostatNode = *ThermostatNode;

        /* Increment and wrap */
        lastAddedThermostatNode++;
        if (lastAddedThermostatNode > &knownThermostatNodes[CONCENTRATOR_MAX_NODES-1])
        {
            lastAddedThermostatNode = knownThermostatNodes;
        }
    }
}

static void updateLcd(void) {
    struct AdcSensorNode* nodePointer = knownSensorNodes;
    struct ThermostatNode* nodePointer1 = knownThermostatNodes;
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
        Display_printf(hDisplaySerial, 0, 0, "0x%02x    %04d    %d    %04d",
                       nodePointer->address, nodePointer->latestAdcValue, nodePointer->button,
                       nodePointer->latestRssi);

//        Display_printf(hDisplaySerial, 0, 0,

        nodePointer++;
        currentLcdLine++;
    }
    while ((nodePointer1 < &knownThermostatNodes[CONCENTRATOR_MAX_NODES]) &&
          (nodePointer1->address != 0) &&
          (currentLcdLine < CONCENTRATOR_DISPLAY_LINES))
    {
        /* print to LCD */
        Display_printf(hDisplayLcd, currentLcdLine, 0, "0x%02x  %04d  %d   %04d",
                nodePointer1->address, nodePointer1->roomTempD2, nodePointer1->button,
                nodePointer1->latestRssi);

        /* print to UART */
        Display_printf(hDisplaySerial, 0, 0, "0x%02x    %04d    %d    %04d",
                nodePointer1->address, nodePointer1->roomTempD2, nodePointer1->button,
                nodePointer1->latestRssi);

//        Display_printf(hDisplaySerial, 0, 0,

        nodePointer1++;
        currentLcdLine++;
    }
}
