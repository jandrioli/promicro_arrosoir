/*
* This is a watering system for garden plants.
* A promicro is on a wafer with 2 relays that activate water pumps. 
* Other equipment attached are:
* - buzzer
* - momentary switch indicating water level
* - analog voltage reading a battery level
* - thermometer (not currently connected)
*/

#include <SPI.h>
#include "RF24.h"
#include "printf.h"
//
// Physical connections 
//
#define HW_BATT   A3   // analog input
#define HW_TEMP   A0   // analog input
#define HW_BUZZ   4    // digital output
#define HW_WATE   6    // digital output
#define HW_RELAY1 7    // d output
#define HW_RELAY2 8    // digital output
#define HW_CSN    9    // icsp
#define HW_CE    10    // icsp
#define DEFAULT_ACTIVATION 600 // 10h from now we activate (in case radio is down and can't program)
#define DEFAULT_DURATION 10   // max 10s of activation time by default

/** 
 *  User Config
 *  This radio node is supposed to be "Node2", meaning it 
 *  listens for tx from mater on pipe#0. 
 *  The sketch rf24_RelayControl-master uses pipe#0 for
 *  writing so all other "slaves" should use other pipes,
 *  each node has their own pipe. This node here in this
 *  sketch "promicro_arrosoir" is using the pipe#2. 
 *  By doing this, the master can understand which node
 *  sent what message. 
 */
RF24 radio(HW_CE, HW_CSN);
const uint8_t addresses[][6] = {"0Node","1Node","2Node"};

/**
 * exchange data via radio more efficiently with data structures.
 * we can exchange max 32 bytes of data per msg. 
 * schedules are reset every 24h (last for a day) so an INTEGER is
 * large enough to store the maximal value of a 24h-schedule.
 * temperature threshold is rarely used
 */
struct relayctl {
  unsigned long uptime = 0;                      // current running time of the machine (millis())  4 bytes  
  unsigned long sched1 = DEFAULT_ACTIVATION;     // schedule in minutes for the relay output nbr1   4 bytes
  unsigned long sched2 = DEFAULT_ACTIVATION;     // schedule in minutes for the relay output nbr2   4 bytes
  unsigned int  maxdur1 = DEFAULT_DURATION;      // max duration nbr1 is ON                         2 bytes
  unsigned int  maxdur2 = DEFAULT_DURATION;      // max duration nbr2 is ON                         2 bytes
  unsigned int  temp_thres = 999;                // temperature at which the syatem is operational  4 bytes
  float         temp_now   = 20;                 // current temperature read on the sensor          4 bytes
  short         battery    =  0;                 // current temperature read on the sensor          2 bytes
  bool          state1 = false;                  // state of relay output 1                         1 byte
  bool          state2 = false;                  // "" 2                                            1 byte
  bool          waterlow = false;                // indicates whether water is low                  1 byte
} myData;

void setup() 
{
  /* real setup starts */
  pinMode(HW_RELAY1, OUTPUT);
  pinMode(HW_RELAY2, OUTPUT);
  digitalWrite(HW_RELAY1, HIGH);
  digitalWrite(HW_RELAY2, HIGH);
  pinMode(HW_BUZZ, OUTPUT);
  pinMode(HW_WATE, INPUT_PULLUP);
  pinMode(HW_BATT, INPUT);
  pinMode(HW_TEMP, INPUT);
  /*a little something to tell we're alive*/
  for (int ii = 0; ii<= 5; ii++) 
  {  
    /*blinks the LEDS on the micro*/
    RXLED1;
    TXLED0; //TX LED is not tied to a normally controlled pin
    delay(500);              // wait for a second
    TXLED1;
    RXLED0;
    delay(500);              // wait for a second
  }
  TXLED0; 
  RXLED0;
  
  Serial.begin(115200);
  
  radio.begin();
  radio.setCRCLength( RF24_CRC_16 ) ;
  radio.setRetries( 15, 5 ) ;
  radio.setAutoAck( true ) ;
  radio.setPALevel( RF24_PA_MAX ) ;
  radio.setDataRate( RF24_250KBPS ) ;
  radio.setChannel( 108 ) ;
  radio.enableDynamicPayloads(); //dont work with my modules :-/
  
  Serial.println(F("RF24 Slave - power socket controller -"));  
  
  radio.openWritingPipe(addresses[2]);
  radio.openReadingPipe(1,addresses[0]);

  // fun
  printf_begin();
  radio.printDetails();
  
  // Start the radio listening for data
  radio.powerUp();
  radio.write( &myData, sizeof(myData) ); 
  radio.startListening();
}

void loop() 
{ 
  byte pipeNo ; 
  if( radio.available(&pipeNo))
  {
    //radio.writeAckPayload(pipeNo,&gotByte, 1 );   
    
    bool done = false;
    uint8_t len = 0;
    String s1;
    
    while (radio.available()) 
    {
      len = radio.getDynamicPayloadSize();
      if ( len == sizeof(relayctl) )
      {
        radio.read( &myData, len );
      }
      else
      {
        char* rx_data = NULL;
        rx_data = (char*)calloc(len+1, sizeof(char));
        if (rx_data == NULL)
        {
          Serial.println("Cannot allocate enough memory to read payload");
          break;
        }
        radio.read( rx_data, len );
      
        // Put a zero at the end for easy printing
        rx_data[len+1] = 0;
      
          // Spew it
        Serial.write("Got msg size=%d value=",len);
        Serial.println(rx_data);
      
        s1 = String(rx_data);
        free(rx_data);
        rx_data = NULL;
      }
    }
    // First, stop listening so we can talk
    radio.stopListening();
    
    
    myData.uptime = millis() / 60000;
    readBatteryVoltage();
    readWaterLevelLow();
    
    
    if (s1.indexOf("stop")>=0)
    {
      myData.state1 = false;
      myData.state2 = false;
      myData.sched1 = 0;
      myData.sched2 = 0;
    }
    else 
    {
      Serial.print("Sending out status ");
      delay(250); // !! Careful: this delay is here because with multiple nodes on air,
                  //    my master always gets confused if they all respond too quickly
                  //    at the same time.
      Serial.println(sizeof(myData));
      radio.write( &myData, sizeof(myData) );
    }
    // Now, resume listening so we catch the next packets.
    radio.startListening();
  }




  
  // is it cold enough to turn justify heating the engine...
  if ( readTemperature() < myData.temp_thres )
  {
    if ( millis()/60000 >= myData.sched1  && myData.sched1 > 0  )
    {
      myData.state1 = true;
      radio.write( &myData, sizeof(myData) );
    }
    if ( millis()/60000 >= myData.sched2  && myData.sched2 > 0  )
    {
      myData.state2 = true;
      radio.write( &myData, sizeof(myData) );
    }
  }
    
  // switch relays off after max_duration, & sendout new status
  if ( myData.sched1 > 0 && (millis()/1000) > (myData.sched1*60)+myData.maxdur1 )
  { 
    myData.state1 = false;
    //automatically schedule relay1 to tomorrow
    myData.sched1 += (unsigned long)24*(unsigned long)60; 
    radio.write( &myData, sizeof(myData) );
  }
  if ( myData.sched2 > 0 && (millis()/1000) > (myData.sched2*60)+myData.maxdur2 )
  { 
    myData.state2 = false;
    //automatically schedule relay2 to tomorrow
    myData.sched2 += (unsigned long)24*(unsigned long)60; 
    radio.write( &myData, sizeof(myData) );
  }

  
  digitalWrite(HW_RELAY1, !myData.state1); // relays are npn-transistorized so have to reverse the logic
  digitalWrite(HW_RELAY2, !myData.state2); // of my program to de/activate each channel

  // if we`re low on water or battery voltage dangerously low, beep the buzzer
  if (myData.waterlow || myData.battery < 8)
  { 
    /*Serial.println("Water is low or battery empty"); 
    Serial.println(millis()/60000);
    Serial.println((unsigned long)last_buzz_at);*/
    if (millis()/60000 > last_buzz_at)
    {
      last_buzz_at = (millis()/60000)+BUZZER_ACTIVATIN_INTERVAL;
      writeBuzzer();
    }
  }
} // Loop




// Read analog pin to get value from temp sensor
int readTemperature()
{
  int value = analogRead(HW_TEMP);

  float millivolts = (value / 1024.0) * 5000;
  float celsius = millivolts / 10;  // sensor output is 10mV per degree Celsius
  myData.temp_now = celsius;
  return celsius;
}

// Read analog pin to get value from voltage divider 
int readBatteryVoltage()
{
  int value = analogRead(HW_BATT);
  myData.battery = map(value, 0, 1024, 0, 15);
  return myData.battery;
}


bool readWaterLevelLow()
{
  myData.waterlow = !digitalRead(HW_WATE);
  return myData.waterlow;
}

void writeBuzzer()
{
  int i = 0;
  int numOfLoops = 0;
  int noteDuration = 1000/8;  // an eighth note
  int pauseBetweenNotes = noteDuration * 0.2;


  // This outer for statement determines the number
  // of siren cycles that are played.
  for(numOfLoops =0; numOfLoops < 4; numOfLoops++) 
  {
    // Play low to high frequencies
    for(i=25;i<120;i++)
    {
      tone(HW_BUZZ, 20*i, noteDuration);
      delay(pauseBetweenNotes);
    }
    // Play high to low frequencies
    for(i=120;i>=25;i--)
    {
      tone(HW_BUZZ, 20*i, noteDuration);
      delay(pauseBetweenNotes);
    }
  }

 }
