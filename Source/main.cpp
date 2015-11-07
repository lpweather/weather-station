/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "mbed.h"
#include "board.h"
#include "LoRaMac.h"
#include "utilities.h"
#include "DigitDisplay.h"

/*!
 * When set to 1 the application uses the Over-the-Air activation procedure
 * When set to 0 the application uses the Personalization activation procedure
 */
#define OVER_THE_AIR_ACTIVATION                     0

/*!
 * Mote device IEEE EUI
 */
static uint8_t DevEui[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#if( OVER_THE_AIR_ACTIVATION != 0 )

#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE          7000000  // 7 [s] value in us

/*!
 * Application IEEE EUI
 */
static uint8_t AppEui[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*!
 * AES encryption/decryption cipher application key
 */
static uint8_t AppKey[] = 
{ 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#else

/*!
* AES encryption/decryption cipher network session key
*/
static uint8_t NwkSKey[] = 
{ 
 0x04, 0xca, 0xd8, 0x18, 0x24, 0x4c, 0x9f, 0x14,
 0x22, 0x82, 0x28, 0x39, 0x21, 0xf1, 0x92, 0x17
};

/*!
* AES encryption/decryption cipher application session key
*/
static uint8_t AppSKey[] = 
{ 
 0x97, 0x74, 0x11, 0x8b, 0xa8, 0xd8, 0x22, 0x1c,
 0x65, 0x8f, 0x70, 0xb8, 0x0e, 0x55, 0xbd, 0x1c
};

/*!
* Device address
*/
static uint32_t DevAddr = 0x80001600;


#endif

/*!
 * Indicates if the MAC layer has already joined a network.
 */
static bool IsNetworkJoined = false;

/*!
 * Defines the application data transmission duty cycle
 */
#define APP_TX_DUTYCYCLE                             10000000  // 10 [s] value in us
#define APP_TX_DUTYCYCLE_RND                         2000000  // 2 [s] value in us

/*!
 * User application data buffer size
 */
#define APP_DATA_SIZE                               5

/*!
 * User application data
 */
static uint8_t AppData[APP_DATA_SIZE];

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

Ticker TxNextPacketTimer;

#if( OVER_THE_AIR_ACTIVATION != 0 )

/*!
 * Defines the join request timer
 */
Ticker JoinReqTimer;

#endif

/*!
 * Indicates if a new packet can be sent
 */
static bool TxNextPacket = true;
static bool TxDone = false;

static uint8_t AppPort = 3;
static uint8_t AppDataSize = APP_DATA_SIZE;

static LoRaMacEvent_t LoRaMacEvents;

static float temperatureValue = 0;
static float luminanceValue = 0.0;
static float lightValue = 0.0;  

Ticker Led1Timer;
Ticker Led2Timer;
Ticker BuzTimer;

AnalogIn LightSens(A1);
AnalogIn TempretureSens(A2);
AnalogIn LuminanceSens ( A3 );
DigitDisplay display(D6, D7);


/*!
 *
 */
static int PrepareTxFrame( uint8_t port, uint8_t sensorIndex )
{
    AppData[0] = sensorIndex;
    float sensorValue;
    switch (sensorIndex)
    {
        case 0:
            debug("[Tx] Temperature=%f\n\r", temperatureValue);
            sensorValue = temperatureValue;
            sensorIndex++;
            break;
        case 1:
            debug("[Tx] Luminance=%f\n\r", luminanceValue);
            sensorValue = luminanceValue;
            sensorIndex++;
            break;
        case 2:
            debug("[Tx] Light=%f\n\r", lightValue);
            sensorValue = lightValue;
            sensorIndex = 0;
            break;
        default:
            break;
    }
    uint32_t value = sensorValue * 10000;
    for (int i = 4; i > 0; i--)
    {
        AppData[i] = value % 100;
        value /= 100;
    }
    return sensorIndex;
}

static void ProcessRxFrame( LoRaMacEventFlags_t *flags, LoRaMacEventInfo_t *info )
{
    debug( "[Rx] Port=%d\n\r" , info->RxPort);
    switch( info->RxPort ) // Check Rx port number
    {
        case 10: 
            display.write( 0, info->RxBuffer[0] );
            display.write( 1, info->RxBuffer[1] );
            display.write( 2, info->RxBuffer[2] );
            display.write( 3, info->RxBuffer[3] ); 
            break;
            
        default:
            break;
    }
}

static bool SendFrame( void )
{
    uint8_t sendFrameStatus = 0;

    debug( "[SendFrame]\n\r");

    sendFrameStatus = LoRaMacSendFrame( AppPort, AppData, AppDataSize );
 //   sendFrameStatus = LoRaMacSendConfirmedFrame( AppPort, AppData, AppDataSize, 8 );
    switch( sendFrameStatus )
    {
    case 5: // NO_FREE_CHANNEL
        // Try again later
        debug( "[SendFrame] no free channel\n\r");
        return true;
    default:
        return false;
    }
}


#if( OVER_THE_AIR_ACTIVATION != 0 )
/*!
 * \brief Function executed on JoinReq Timeout event
 */
static void OnJoinReqTimerEvent( void )
{
    TxNextPacket = true;
    JoinReqTimer.detach( );
}
#endif


/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{    
    TxNextPacket = true;
    TxNextPacketTimer.detach( );
}


/*!
 * \brief Function to be executed on MAC layer event
 */
static void OnMacEvent( LoRaMacEventFlags_t *flags, LoRaMacEventInfo_t *info )
{
    debug( "[OnMacEvent]\n\r");
    
    if( flags->Bits.JoinAccept == 1 )
    {
#if( OVER_THE_AIR_ACTIVATION != 0 )
        JoinReqTimer.detach( );
#endif
        IsNetworkJoined = true;
    }
    
    if( flags->Bits.Tx == 1 )
    {
    }

    if( flags->Bits.Rx == 1 )
    {
        if( flags->Bits.RxData == true )
        {
            ProcessRxFrame( flags, info );
        }
    }

    // Schedule a new transmission
    TxDone = true;
}

/**
 * Main application entry point.
 */
int main( void )
{
    #if( OVER_THE_AIR_ACTIVATION != 0 )
        uint8_t sendFrameStatus = 0;
    #endif
    bool trySendingFrameAgain = false; 

    debug( "\n\n\r    LoRaWAN Class A Demo code  \n\n\r" );
    
    BoardInitMcu( );
    BoardInitPeriph( );

    // Initialize LoRaMac device unique ID
    // BoardGetUniqueId( DevEui );
    
    LoRaMacEvents.MacEvent = OnMacEvent;
    LoRaMacInit( &LoRaMacEvents );

    IsNetworkJoined = false;

    #if( OVER_THE_AIR_ACTIVATION == 0 )
        // Random seed initialization
        srand( RAND_SEED );
        // Choose a random device address
        // NwkID = 0
        // NwkAddr rand [0, 33554431]
        if( ( DevAddr == 0 ) || ( DevAddr == 0xFFFFFFFF ) )
        {
            // Generate random DevAddr if it does not exist
            debug("Generate random DevAddr\n\r");
            DevAddr = randr( 0, 0x01FFFFFF );
        }
        debug( "- DevAddr = 0x%x\n\r" , DevAddr);    
        LoRaMacInitNwkIds( 0x000000, DevAddr, NwkSKey, AppSKey );
        IsNetworkJoined = true;
    #endif

    TxNextPacket = true;

    LoRaMacSetAdrOn( false );
    
    LoRaMacSetDutyCycleOn( false );    
        
    int sensorIndex = 0;
    int displayIndex = 0;
        
    while( 1 )
    {

        // Read light sensor
        lightValue = LightSens.read( ) * 1.65;

        int a;
        int B = 3975;                                                         //B value of the thermistor
        float resistance;
        a = TempretureSens * 1023;
        resistance =(float)(1023 - a) * 10000 / a;                         //get the resistance of the sensor;
        temperatureValue = (float)(1 / (log(resistance / 10000) / B + 1 / 298.15) - 273.15);
        luminanceValue = LuminanceSens.read();

        while( IsNetworkJoined == false )
        {
            #if( OVER_THE_AIR_ACTIVATION != 0 )
                if( TxNextPacket == true )
                {
                    TxNextPacket = false;
                    
                    sendFrameStatus = LoRaMacJoinReq( DevEui, AppEui, AppKey );
                    debug("Req Sent\n\r");
                    switch( sendFrameStatus )
                    {
                    case 1: // BUSY
                        break;
                    case 0: // OK
                    case 2: // NO_NETWORK_JOINED
                    case 3: // LENGTH_PORT_ERROR
                    case 4: // MAC_CMD_ERROR
                    case 6: // DEVICE_OFF
                    default:
                        // Relaunch timer for next trial
                        JoinReqTimer.attach_us( &OnJoinReqTimerEvent, OVER_THE_AIR_ACTIVATION_DUTYCYCLE );
                        break;
                    }
                }
    //            TimerLowPowerHandler( );
            #endif
        }

        if( TxDone == true )
        {

            display.clear();

            switch(displayIndex)
            {
                case 0:
                    display.write(0, ((int)temperatureValue)/10);
                    display.write(1, ((int)temperatureValue)%10);
                    display.writeRaw(2, (int)0x63);
                    display.writeRaw(3, (int)0x39);
                    displayIndex++;
                    break;

                case 1:
                    display.write(luminanceValue*1000);
                    displayIndex++;
                    break;
                case 2:
                    display.write(lightValue*1000);
                    displayIndex = 0;
                    break;
            }
            
            TxDone = false;
            
            debug( "TxDone \n\n\r" );
            // Schedule next packet transmission
            TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
            TxNextPacketTimer.attach_us( &OnTxNextPacketTimerEvent, TxDutyCycleTime );
        }

        if( trySendingFrameAgain == true )
        {
            trySendingFrameAgain = SendFrame( );
        }
        
        if( TxNextPacket == true )
        {       
            TxNextPacketTimer.detach( );
            
            TxNextPacket = false;
        
            sensorIndex = PrepareTxFrame( AppPort, sensorIndex );
            
            trySendingFrameAgain = SendFrame( );
        }

//       TimerLowPowerHandler( );
    }
}