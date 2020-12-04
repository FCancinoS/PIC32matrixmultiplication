/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "definitions.h"
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
unsigned char row;
float matrix1[10][10];
float matrix2[10][10];
float matrix3[10][10];
/* Recieve data buffer */
uint8_t receiveDataBuffer[64] CACHE_ALIGN;

/* Transmit data buffer */
uint8_t  transmitDataBuffer[64] CACHE_ALIGN;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
int APP_USB_SWITCH_2;
int APP_USB_SWITCH_3;

int i;
int h;
int j;
int k;
int ans;
float ansf;
int *n1,*n2;
float *n1f, *n2f;
uint8_t n1temp[4];
uint8_t n2temp[4];
uint8_t *anstemp;




        // *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

USB_DEVICE_HID_EVENT_RESPONSE APP_USBDeviceHIDEventHandler
(
    USB_DEVICE_HID_INDEX iHID,
    USB_DEVICE_HID_EVENT event,
    void * eventData,
    uintptr_t userData
)
{
    USB_DEVICE_HID_EVENT_DATA_REPORT_SENT * reportSent;
    USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED * reportReceived;

    /* Check type of event */
    switch (event)
    {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_SENT
             * pointer type containing details about the report that was
             * sent. */
            reportSent = (USB_DEVICE_HID_EVENT_DATA_REPORT_SENT *) eventData;
            if(reportSent->handle == appData.txTransferHandle )
            {
                // Transfer progressed.
                appData.hidDataTransmitted = true;
            }
            
            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_RECEIVED
             * pointer type containing details about the report that was
             * received. */

            reportReceived = (USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED *) eventData;
            if(reportReceived->handle == appData.rxTransferHandle )
            {
                // Transfer progressed.
                appData.hidDataReceived = true;
            }
          
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* For now we just accept this request as is. We acknowledge
             * this request using the USB_DEVICE_HID_ControlStatus()
             * function with a USB_DEVICE_CONTROL_STATUS_OK flag */

            USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* Save Idle rate recieved from Host */
            appData.idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*)eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData.usbDevHandle, & (appData.idleRate),1);

            /* On successfully reciveing Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function drvier returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;
        default:
            // Nothing to do.
            break;
    }
    return USB_DEVICE_HID_EVENT_RESPONSE_NONE;
}

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Host has de configured the device or a bus reset has happened.
             * Device layer is going to de-initialize all function drivers.
             * Hence close handles to all function drivers (Only if they are
             * opened previously. */

            LED_Off(); 
            appData.deviceConfigured = false;
            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            break;

        case USB_DEVICE_EVENT_CONFIGURED:
            LED_On(); 
            /* Set the flag indicating device is configured. */
            appData.deviceConfigured = true;

            /* Save the other details for later use. */
            appData.configurationValue = ((USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData)->configurationValue;

            /* Register application HID event handler */
            USB_DEVICE_HID_EventHandlerSet(USB_DEVICE_HID_INDEX_0, APP_USBDeviceHIDEventHandler, (uintptr_t)&appData);


            break;

        case USB_DEVICE_EVENT_SUSPENDED:
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */

            USB_DEVICE_Attach (appData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:
            LED_Off(); 
            /* VBUS is not available */
            USB_DEVICE_Detach(appData.usbDevHandle);
            break;

        /* These events are not used in this demo */
        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    appData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    appData.deviceConfigured = false;
    appData.txTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    appData.rxTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    appData.hidDataReceived = false;
    appData.hidDataTransmitted = true;
    appData.receiveDataBuffer = &receiveDataBuffer[0];
    appData.transmitDataBuffer = &transmitDataBuffer[0];
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks (void )
{

    /* Check if device is configured.  See if it is configured with correct
     * configuration value  */

    switch(appData.state)
    {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

            if(appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.usbDevHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            if(appData.deviceConfigured == true)
            {
                /* Device is ready to run the main task */
                appData.hidDataReceived = false;
                appData.hidDataTransmitted = true;
                appData.state = APP_STATE_MAIN_TASK;

                /* Place a new read request. */
                USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                        &appData.rxTransferHandle, appData.receiveDataBuffer, 64);
            }
            break;

        case APP_STATE_MAIN_TASK:

            if(!appData.deviceConfigured)
            {
                /* Device is not configured */
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else if( appData.hidDataReceived )
            {
                /* Look at the data the host sent, to see what
                 * kind of application specific command it sent. */
                LED_Off();
                switch(appData.receiveDataBuffer[0])
                {
                   case 0x01:
                       if(appData.receiveDataBuffer[1]){
                           LED_On();
                       }else{
                           LED_Off();
                       }
                       appData.hidDataReceived = false;

                       /* Place a new read request. */
                       USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                               &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                   break;
                   case 0x02:
                         if(appData.receiveDataBuffer[1]){
                           LED2_On();
                       }else{
                           LED2_Off();
                       }
                       appData.hidDataReceived = false;

                       /* Place a new read request. */
                       USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                               &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                   break;

                   case 0x03:
                       if(appData.receiveDataBuffer[1]){
                           LED3_On();
                       }else{
                           LED3_Off();
                       }
                       appData.hidDataReceived = false;

                       /* Place a new read request. */
                       USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                               &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                   break;
                   
                    case 0x10:
                       for(i = 0; i<4; i++){
                           n1temp[i] = appData.receiveDataBuffer[i+1];
                       }
                       n1 = (int *)&n1temp[0];
                       for(i = 0; i<4; i++){
                           n2temp[i] = appData.receiveDataBuffer[i+5];
                       }
                       n2 = (int *)&n2temp[0];
                       ans = *n1 + *n2;
                       anstemp = (uint8_t *)&ans;
                       appData.transmitDataBuffer[0] = 0x10;
                       for(i = 0; i<4; i++){
                           appData.transmitDataBuffer[i+1] = *(anstemp+i);
                       }
                        appData.hidDataTransmitted = false;

                           /* Prepare the USB module to send the data packet to the host */
                           USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                   &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                           appData.hidDataReceived = false;

                           /* Place a new read request. */
                           USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                   &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                       break;
                   case 0x11:
                       for(i = 0; i<4; i++){
                           n1temp[i] = appData.receiveDataBuffer[i+1];
                       }
                       n1 = (int *)&n1temp[0];
                       for(i = 0; i<4; i++){
                           n2temp[i] = appData.receiveDataBuffer[i+5];
                       }
                       n2 = (int *)&n2temp[0];
                       ans = *n1 - *n2;
                       anstemp = (uint8_t *)&ans;
                       appData.transmitDataBuffer[0] = 0x11;
                       for(i = 0; i<4; i++){
                           appData.transmitDataBuffer[i+1] = *(anstemp+i);
                       }
                        appData.hidDataTransmitted = false;

                           /* Prepare the USB module to send the data packet to the host */
                           USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                   &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                           appData.hidDataReceived = false;

                           /* Place a new read request. */
                           USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                   &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                       break;
                   case 0x12:
                       for(i = 0; i<4; i++){
                           n1temp[i] = appData.receiveDataBuffer[i+1];
                       }
                       n1 = (int *)&n1temp[0];
                       for(i = 0; i<4; i++){
                           n2temp[i] = appData.receiveDataBuffer[i+5];
                       }
                       n2 = (int *)&n2temp[0];
                       ans = *n1 * *n2;
                       anstemp = (uint8_t *)&ans;
                       appData.transmitDataBuffer[0] = 0x12;
                       for(i = 0; i<4; i++){
                           appData.transmitDataBuffer[i+1] = *(anstemp+i);
                       }
                        appData.hidDataTransmitted = false;

                           /* Prepare the USB module to send the data packet to the host */
                           USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                   &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                           appData.hidDataReceived = false;

                           /* Place a new read request. */
                           USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                   &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                       break;
                   case 0x13:
                       for(i = 0; i<4; i++){
                           n1temp[i] = appData.receiveDataBuffer[i+1];
                       }
                       n1 = (int *)&n1temp[0];
                       for(i = 0; i<4; i++){
                           n2temp[i] = appData.receiveDataBuffer[i+5];
                       }
                       n2 = (int *)&n2temp[0];
                       ans = *n1 / *n2;
                       anstemp = (uint8_t *)&ans;
                       appData.transmitDataBuffer[0] = 0x13;
                       for(i = 0; i<4; i++){
                           appData.transmitDataBuffer[i+1] = *(anstemp+i);
                       }
                        appData.hidDataTransmitted = false;

                           /* Prepare the USB module to send the data packet to the host */
                           USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                   &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                           appData.hidDataReceived = false;

                           /* Place a new read request. */
                           USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                   &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                       break;
                     //coprosedador matematico Float
                        
                        
                        
                    //primera matriz
                    case 0x14:  
                        
                        //el segundo byte es el número de fila 
                        row = appData.receiveDataBuffer[1];
                        for(h = 0; h<10; h++){
                        for(i = 0; i<4; i++){
                            n1temp[i] = appData.receiveDataBuffer[h*4+i+2];
                        }
                        n1f = (float *)&n1temp[0];
                        matrix1[row][h] = *n1f;
                        }
                        appData.hidDataReceived = false;
                        USB_DEVICE_HID_ReportReceive( USB_DEVICE_HID_INDEX_0,&appData.rxTransferHandle, appData.receiveDataBuffer, 64);
                        
                        break;
                    //segunda matriz
                    case 0x15:
                        
                        //el segundo byte es el número de fila 
                        row = appData.receiveDataBuffer[1];
                        for(h = 0; h<10; h++){
                        for(i = 0; i<4; i++){
                            n2temp[i] = appData.receiveDataBuffer[h*4+i+2];
                        }
                        n2f = (float *)&n2temp[0];
                        matrix2[row][h] = *n2f;
                        }
                        appData.hidDataReceived = false;
                        USB_DEVICE_HID_ReportReceive( USB_DEVICE_HID_INDEX_0,&appData.rxTransferHandle, appData.receiveDataBuffer, 64);
                         
                        break;
                        
                        
                    case 0x16:
                        
                        for(i=0; i<10; i++){
                            for(j=0; j<10; j++){
                                matrix3[i][j] = 0;
                            }
                    }
                        for(i=0; i<10; i++){
                            for(j=0; j<10; j++){
                                for(k=0; k<10; k++){
                                    matrix3[i][j]+=matrix1[i][k]*matrix2[k][j];
                                    }
                                }
                            }
                        appData.hidDataReceived = false;
                        USB_DEVICE_HID_ReportReceive( USB_DEVICE_HID_INDEX_0,&appData.rxTransferHandle, appData.receiveDataBuffer, 64);
                        break;
                    case 0xFF: 
                        LED3_On();
                        if(appData.hidDataTransmitted){
                            row = appData.receiveDataBuffer[1];
                        
                        appData.transmitDataBuffer[0] = 0xFF;
                        appData.transmitDataBuffer[1] = (uint8_t)row;
 
                        for(i=0; i<10; i++){
                            ansf = matrix3[row][i];
                            anstemp = (uint8_t*)&ansf;
                            for(h=0; h<4;h++){
                             appData.transmitDataBuffer[(i*4+2)+h] = *(anstemp+h);   
                            }
                            
                        
                        }
                        
                        appData.hidDataTransmitted = false;
                        USB_DEVICE_HID_ReportSend(USB_DEVICE_HID_INDEX_0,&appData.txTransferHandle, appData.transmitDataBuffer, 64);
                        
                        appData.hidDataReceived= false;
                        USB_DEVICE_HID_ReportReceive(USB_DEVICE_HID_INDEX_0, &appData.rxTransferHandle,appData.receiveDataBuffer,64);
                        }
                        break;
                        
                        
                        
                        
                        
                        
                        
                        
                        case 0x20:
                        for(i = 0; i<4; i++){
                            n1temp[i] = appData.receiveDataBuffer[i+1];
                        }
                        n1f = (float *)&n1temp[0];
                        for(i = 0; i<4; i++){
                            n2temp[i] = appData.receiveDataBuffer[i+5];
                        }
                        n2f = (float *)&n2temp[0];
                        ansf = *n1f + *n2f;
                        anstemp = (uint8_t *)&ansf;
                        appData.transmitDataBuffer[0] = 0x20;
                        for(i = 0; i<4; i++){
                            appData.transmitDataBuffer[i+1] = *(anstemp+i);
                        }
                         appData.hidDataTransmitted = false;

                            /* Prepare the USB module to send the data packet to the host */
                            USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                    &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                            appData.hidDataReceived = false;

                            /* Place a new read request. */
                            USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                    &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                        break;
                    case 0x21:
                        for(i = 0; i<4; i++){
                            n1temp[i] = appData.receiveDataBuffer[i+1];
                        }
                        n1f = (float *)&n1temp[0];
                        for(i = 0; i<4; i++){
                            n2temp[i] = appData.receiveDataBuffer[i+5];
                        }
                        n2f = (float *)&n2temp[0];
                        ansf = (*n1f) - (*n2f);
                        anstemp = (uint8_t *)&ansf;
                        appData.transmitDataBuffer[0] = 0x21;
                        for(i = 0; i<4; i++){
                            appData.transmitDataBuffer[i+1] = *(anstemp+i);
                        }
                         appData.hidDataTransmitted = false;

                            /* Prepare the USB module to send the data packet to the host */
                            USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                    &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                            appData.hidDataReceived = false;

                            /* Place a new read request. */
                            USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                    &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                        break;
                    case 0x22:
                        for(i = 0; i<4; i++){
                            n1temp[i] = appData.receiveDataBuffer[i+1];
                        }
                        n1f = (float *)&n1temp[0];
                        for(i = 0; i<4; i++){
                            n2temp[i] = appData.receiveDataBuffer[i+5];
                        }
                        n2f = (float *)&n2temp[0];
                        ansf = *n1f * *n2f;
                        anstemp = (uint8_t *)&ansf;
                        appData.transmitDataBuffer[0] = 0x22;
                        for(i = 0; i<4; i++){
                            appData.transmitDataBuffer[i+1] = *(anstemp+i);
                        }
                         appData.hidDataTransmitted = false;

                            /* Prepare the USB module to send the data packet to the host */
                            USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                    &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                            appData.hidDataReceived = false;

                            /* Place a new read request. */
                            USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                    &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                        break;
                    case 0x23:
                        for(i = 0; i<4; i++){
                            n1temp[i] = appData.receiveDataBuffer[i+1];
                        }
                        n1f = (float *)&n1temp[0];
                        for(i = 0; i<4; i++){
                            n2temp[i] = appData.receiveDataBuffer[i+5];
                        }
                        n2f = (float *)&n2temp[0];
                        ansf = *n1f / *n2f;
                        anstemp = (uint8_t *)&ansf;
                        appData.transmitDataBuffer[0] = 0x23;
                        for(i = 0; i<4; i++){
                            appData.transmitDataBuffer[i+1] = *(anstemp+i);
                        }
                         appData.hidDataTransmitted = false;

                            /* Prepare the USB module to send the data packet to the host */
                            USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                    &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                            appData.hidDataReceived = false;

                            /* Place a new read request. */
                            USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                    &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                        break;
                    
                    case 0x80:

                        /* Toggle on board LED1 to LED2. */
                        LED_Toggle(  );

                        appData.hidDataReceived = false;

                        /* Place a new read request. */
                        USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );

                        break;

                    case 0x81:
                        
                        if(appData.hidDataTransmitted)
                        {
                            /* Echo back to the host PC the command we are fulfilling in
                             * the first byte.  In this case, the Get Push-button State
                             * command. */

                            appData.transmitDataBuffer[0] = 0x81;

                            if( SWITCH_Get() == SWITCH_STATE_PRESSED )
                            {
                                appData.transmitDataBuffer[1] = 0x00;
                            }
                            else
                            {
                                appData.transmitDataBuffer[1] = 0x01;
                            }

                            appData.hidDataTransmitted = false;

                            /* Prepare the USB module to send the data packet to the host */
                            USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                    &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                            appData.hidDataReceived = false;

                            /* Place a new read request. */
                            USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                    &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                        }
                        break;
                    case 0x82:
                        if(appData.hidDataTransmitted)
                        {
                            /* Echo back to the host PC the command we are fulfilling in
                             * the first byte.  In this case, the Get Push-button State
                             * command. */
                            
                            appData.transmitDataBuffer[0] = 0x82;//the code

                            appData.transmitDataBuffer[1] = 'A';
                            appData.transmitDataBuffer[2] = '0';
                            appData.transmitDataBuffer[3] = '1';
                            appData.transmitDataBuffer[4] = '7';
                            appData.transmitDataBuffer[5] = '3';
                            appData.transmitDataBuffer[6] = '0';
                            appData.transmitDataBuffer[7] = '6';
                            appData.transmitDataBuffer[8] = '9';
                            appData.transmitDataBuffer[8] = '8';
                            appData.transmitDataBuffer[10] = ' ';
                            appData.transmitDataBuffer[11] = 'A';
                            appData.transmitDataBuffer[12] = '0';
                            appData.transmitDataBuffer[13] = '1';
                            appData.transmitDataBuffer[14] = '3';
                            appData.transmitDataBuffer[15] = '8';
                            appData.transmitDataBuffer[16] = '1';
                            appData.transmitDataBuffer[17] = '3';
                            appData.transmitDataBuffer[18] = '3';
                            appData.transmitDataBuffer[19] = '4';
                            appData.transmitDataBuffer[20] = ' ';
                            appData.transmitDataBuffer[21] = 'A';
                            appData.transmitDataBuffer[22] = '0';
                            appData.transmitDataBuffer[23] = '1';
                            appData.transmitDataBuffer[24] = '3';
                            appData.transmitDataBuffer[25] = '8';
                            appData.transmitDataBuffer[26] = '1';
                            appData.transmitDataBuffer[27] = '4';
                            appData.transmitDataBuffer[28] = '8';
                            appData.transmitDataBuffer[29] = '7';


                            appData.hidDataTransmitted = false;

                            /* Prepare the USB module to send the data packet to the host */
                            USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                    &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                            appData.hidDataReceived = false;

                            /* Place a new read request. */
                            USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                    &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                        }
                        break;

                    default:

                        appData.hidDataReceived = false;

                        /* Place a new read request. */
                        USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                        break;
                }
            }
        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}
 

/*******************************************************************************
 End of File
 */

