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
#include <avr/wdt.h>
//
// Physical connections 
//
#define HW_BATT   A3       // a input
#define HW_TEMP   A0       // a input
#define HW_BUZZ   3        // d output
#define HW_BUTT1  A1       // d input
#define HW_BUTT2  A2       // d input
#define HW_WATE   6        // d output
#define HW_RELAY1 7        // d output
#define HW_RELAY2 8        // d output
#define HW_CSN    9        // icsp
#define HW_CE    10        // icsp
                           // 74hc595 pin 16 (VCC)
                           // 74hc595 pin 15 (Q0 - output 0)
#define HW_SHIFTDATA   2  // 74hc595 pin 14 (DS)
                           // 74hc595 pin 13 (OE! output enable active low)
#define HW_SHIFTLATCH  5  // 74hc595 pin 12 (SH_CP latch)
#define HW_SHIFTCLOCK  4  // 74hc595 pin 11 (ST_CP clock)
//disabled this coz ran out of pins (dont wanna use 0 and 1 coz theyre rx/tx)
//#define HW_SHIFTRESET  2  // 74hc595 pin 13 (MR! master reset active low)
// 
// SW Logic and firmware definitions
// 
#define THIS_NODE_ID 2                       // master is 0, unoR3 debugger is 1, promicro_arrosoir is 2, etc
#define DEFAULT_ACTIVATION 600               // 10h from now we activate (in case radio is down and can't program)
#define DEFAULT_DURATION 10                  // max 10s of activation time by default
#define BUZZER_ACTIVATIN_INTERVAL 18000      // buzz every 5h when sth is wrong
/** 
 *  User Config
 */
RF24 radio(HW_CE, HW_CSN);

// Radio pipe addresses for the nodes to communicate.
// WARNING!! 3Node and 4Node are used by my testing sketches ping/pong
const uint8_t addresses[][5] = {
  "0Node", // master writes broadcasts here
  "1Node", // unor3 writes here
  "2Node", // unor3 reads here
  "3Node", // arrosoir reads here
  "4Node", // arrosoir writes here
  "5Node"};// not yet used by anybody

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
  unsigned long sched2 = DEFAULT_ACTIVATION+1;   // schedule in minutes for the relay output nbr2   4 bytes
  unsigned int  maxdur1 = DEFAULT_DURATION;      // max duration nbr1 is ON                         2 bytes
  unsigned int  maxdur2 = DEFAULT_DURATION;      // max duration nbr2 is ON                         2 bytes
  unsigned int  temp_thres = 999;                // temperature at which the syatem is operational  4 bytes
  float         temp_now   = 20;                 // current temperature read on the sensor          4 bytes
  short         battery    =  0;                 // current temperature read on the sensor          2 bytes
  bool          state1 = false;                  // state of relay output 1                         1 byte
  bool          state2 = false;                  // "" 2                                            1 byte
  bool          waterlow = false;                // indicates whether water is low                  1 byte
  byte          nodeid = THIS_NODE_ID;           // nodeid is the identifier of the slave           1 byte
} myData;


uint64_t last_buzz_at = 0;
bool activation_notified = false;


byte leds = 0;
bool l = false;

// THESE ALL WORK OF SHITOUT WITH MSBFIRST AND THE 74HC595
// IS CONNECTED TO 7SEG DISPLAY'S LIKE THIS:
// Q0 - seg a
// ...
// Q7 - seg dp
//                     
//         segments:   acfe.dbg
byte dec_digits[] ={ 0b11110110,
                     0b01000010,
                     0b10010111,
                     0b11000111,
                     0b01100011,
                     0b11100101,
                     0b01110101,
                     0b11000010,
                     0b11110111,
                     0b11100011 };


void doShift(byte value)
{
  digitalWrite(HW_SHIFTLATCH, LOW);
  shiftOut(HW_SHIFTDATA, HW_SHIFTCLOCK, MSBFIRST, value);
  digitalWrite(HW_SHIFTLATCH, HIGH);
}
void displayOff()
{/*
  this code worked when we had access to the master reset pin of the 74hc595
  digitalWrite(HW_SHIFTLATCH, LOW);
  delay(1);
  digitalWrite(HW_SHIFTRESET, HIGH);
  delay(1);
  digitalWrite(HW_SHIFTRESET, LOW);
  digitalWrite(HW_SHIFTLATCH, HIGH);
  */
  
  digitalWrite(HW_SHIFTLATCH, LOW);
  shiftOut(HW_SHIFTDATA, HW_SHIFTCLOCK, MSBFIRST, 0b0000000000000000);
  digitalWrite(HW_SHIFTLATCH, HIGH);
  delay(1);
  digitalWrite(HW_SHIFTLATCH, LOW);
  shiftOut(HW_SHIFTDATA, HW_SHIFTCLOCK, MSBFIRST, 0b00000000);
  digitalWrite(HW_SHIFTLATCH, HIGH);
  delay(1);
}
void displayNumber(int toShow)
{
  String val = String(toShow);
  byte lower = val[0] - '0', upper = 0;
  if (val.length() > 1)
    upper = val[1] - '0';

  doShift(dec_digits[upper]);
  doShift(dec_digits[lower]);
}

void testIndividualSegments()
{
  byte b = 0;  
  for ( int x = 0; x < 8; x++ )
  {
    bitSet(b, x);
    Serial.print("Writing byte:");
    Serial.print(x);
    Serial.print(" ");
    Serial.println(b, BIN);
    
    delay(150);
    
    digitalWrite(HW_SHIFTLATCH, LOW);
    delay(1);
    shiftOut(HW_SHIFTDATA, HW_SHIFTCLOCK, LSBFIRST, b);
    delay(1);
    digitalWrite(HW_SHIFTLATCH, HIGH);

    
    delay(1000);
    b = 0;
  }  
  delay(2000);
}


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
  int noteDuration = 1000/12;  // an eighth note
  int pauseBetweenNotes = noteDuration * 0.1;


  // This outer for statement determines the number
  // of siren cycles that are played.
  for(numOfLoops =0; numOfLoops < 2; numOfLoops++) 
  {
    // Play low to high frequencies
    for(i=25;i<120;i++)
    {
      tone(HW_BUZZ, 20*i, noteDuration);
      delay(pauseBetweenNotes);
    }
    // Play high to low frequencies
    /*for(i=120;i>=25;i--)
    {
      tone(HW_BUZZ, 20*i, noteDuration);
      delay(pauseBetweenNotes);
    }*/
  }
}

void doBlink()
{ 
#if defined(ARDUINO_AVR_LEONARDO) 
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
#endif
}


void setup() 
{
  /* real setup starts */
  pinMode(HW_RELAY1, OUTPUT);
  pinMode(HW_RELAY2, OUTPUT);
  pinMode(HW_BUZZ, OUTPUT);
  pinMode(HW_WATE, INPUT_PULLUP);
  pinMode(HW_BUTT1, INPUT_PULLUP);
  pinMode(HW_BUTT2, INPUT_PULLUP);
  pinMode(HW_BATT, INPUT);
  pinMode(HW_TEMP, INPUT);
  
  pinMode(HW_SHIFTLATCH, OUTPUT);
  pinMode(HW_SHIFTDATA, OUTPUT);  
  pinMode(HW_SHIFTCLOCK, OUTPUT);
//  pinMode(HW_SHIFTRESET, OUTPUT);
  digitalWrite(HW_RELAY1, HIGH);
  digitalWrite(HW_RELAY2, HIGH);
//  digitalWrite(HW_SHIFTRESET, LOW);
  digitalWrite(HW_SHIFTLATCH, LOW);
  delay(1);
//  digitalWrite(HW_SHIFTRESET, HIGH);
  digitalWrite(HW_SHIFTLATCH, HIGH);

  testIndividualSegments();
  displayNumber(34);
  delay(2000);
  displayOff();
  
  doBlink();
  writeBuzzer();

  
  //
  // Print preamble
  //
  Serial.begin(115200);
  delay(500);
  Serial.println(F("RF24 Slave - power socket controller"));  
  delay(500);
  Serial.println(F("Warning! Always query the controller before attempting to program it!"));  
  delay(500);
  Serial.println(F("- - - - -"));  

  // using watchdog timer to signal this sketch is alive and well.
  // prevent getting hung for some reason
  wdt_enable(WDTO_8S); 
  
  radio.begin();
  radio.setCRCLength( RF24_CRC_16 ) ;
  radio.setRetries( 15, 15 ) ;
  radio.setAutoAck( true ) ;
  radio.setPALevel( RF24_PA_MAX ) ;
  radio.setDataRate( RF24_250KBPS ) ;
  radio.setChannel( 108 ) ;
  radio.enableDynamicPayloads(); 
  
  radio.openWritingPipe(addresses[4]);
  radio.openReadingPipe(1,addresses[0]);
  radio.openReadingPipe(2,addresses[3]);

  //
  // Dump the configuration of the rf unit for debugging
  //
  printf_begin();
  Serial.println(F("Radio setup:"));  
  radio.printDetails();
  Serial.println(F("- - - - -"));  
  
  // Start the radio listening for data
  radio.powerUp();
  radio.write( &myData, sizeof(myData) ); 
  radio.startListening();
}

void loop() 
{ 
  wdt_reset();
  byte pipeNo ;
  
  if( radio.available(&pipeNo))
  {
    uint8_t len = 0;
    String s1;
    
    while (radio.available()) 
    {
      len = radio.getDynamicPayloadSize();
      if ( len == sizeof(relayctl) )
      {
        radio.read( &myData, len );
        myData.nodeid = THIS_NODE_ID; // preserve my node id
        Serial.println("Got new program!");
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
        Serial.write("Got msg size= ",len);
        Serial.println(rx_data);
      
        s1 = String(rx_data);
        free(rx_data);
        rx_data = NULL;
      }
    }
    
    
    
    
    if (s1.indexOf("stop")>=0)
    {
      myData.state1 = false;
      myData.state2 = false;
      myData.sched1 = 0;
      myData.sched2 = 0;
      Serial.println("Stopping watering system. A valid program must be sent, or reboot.");
    }
    else 
    {
      Serial.print("Sending out status. Packet size: ");
      Serial.println(sizeof(myData));
      delay(250); // !! Careful: this delay is here because with multiple nodes on air,
                  //    my master always gets confused if they all respond too quickly
                  //    at the same time.
    }
    radio.stopListening();
    radio.write( &myData, sizeof(myData) );
    radio.startListening();      
  }


  myData.uptime = (float)millis() / (float)60000;
  readBatteryVoltage();
  readWaterLevelLow();


  
  // is it cold enough to turn justify heating the engine...
  if ( readTemperature() < myData.temp_thres )
  {
    if ( millis()/60000 >= myData.sched1  && myData.sched1 > 0  )
    {
      if (myData.state1 = false) Serial.println("Activating plug1");
      myData.state1 = true;
      if (!activation_notified)
      { 
        Serial.println("Notifying about activation of plug1"); 
        radio.stopListening();
        radio.write( &myData, sizeof(myData) );
        radio.startListening();
        activation_notified = true;
      }
    }
    if ( millis()/60000 >= myData.sched2  && myData.sched2 > 0  )
    {
      if (myData.state2 = false) Serial.println("Activating plug2");
      myData.state2 = true;
      if (!activation_notified)
      { 
        Serial.println("Notifying about activation of plug2"); 
        radio.stopListening();
        radio.write( &myData, sizeof(myData) );
        radio.startListening();
        activation_notified = true;
      }
    }
  }
    
  // switch relays off after max_duration, & sendout new status
  if ( myData.sched1 > 0 && (millis()/1000) > (myData.sched1*60)+myData.maxdur1 )
  { 
    Serial.println("Deactivating plug1"); 
    myData.state1 = false;
    //automatically schedule relay1 to tomorrow
    myData.sched1 += 1440;
    radio.stopListening();
    radio.write( &myData, sizeof(myData) );
    radio.startListening();
    activation_notified = false;
  }
  if ( myData.sched2 > 0 && (millis()/1000) > (myData.sched2*60)+myData.maxdur2 )
  { 
    Serial.println("Deactivating plug2"); 
    myData.state2 = false;
    //automatically schedule relay2 to tomorrow
    myData.sched2 += 1440; 
    radio.stopListening();
    radio.write( &myData, sizeof(myData) );
    radio.startListening();
    activation_notified = false;
  }

  
  digitalWrite(HW_RELAY1, !myData.state1); // relays are npn-transistorized so have to reverse the logic
  digitalWrite(HW_RELAY2, !myData.state2); // of my program to de/activate each channel



  if (myData.waterlow && last_buzz_at+BUZZER_ACTIVATIN_INTERVAL < (millis()/1000))
  {
    Serial.println((unsigned long)last_buzz_at);
    Serial.println(millis()/1000);
    last_buzz_at = millis()/1000;
    writeBuzzer();
    radio.stopListening();
    radio.write( &myData, sizeof(myData) ); 
    radio.startListening();
  }
  if (Serial.available() || !digitalRead(HW_BUTT1))
  {
    Serial.readString();
    Serial.read();
    Serial.println(F("Radio setup:"));  
    radio.printDetails();
    Serial.println(F("- - - - -"));  
    bool goodSignal = radio.testRPD();
    Serial.println(goodSignal ? "Strong signal > 64dBm" : "Weak signal < 64dBm" );
    Serial.println(F("- - - - -"));  
    Serial.print("Plug 1: ");
    Serial.print(myData.sched1);
    Serial.print("min, during ");
    Serial.print(myData.maxdur1);
    Serial.print("s(currently ");
    Serial.print(myData.state1);
    Serial.print(")\nPlug 2: ");
    Serial.print(myData.sched2);
    Serial.print("min, during ");
    Serial.print(myData.maxdur2);
    Serial.print("s(currently ");
    Serial.print(myData.state2);
    Serial.print(")\nTemperature: ");
    Serial.print(myData.temp_now);
    Serial.print("/");
    Serial.print(myData.temp_thres);
    Serial.print("\nUptime: ");
    Serial.print(myData.uptime);
    Serial.print("min\nBattery:");
    Serial.print(myData.battery);
    Serial.print("V\nWaterLow:");
    Serial.print(myData.waterlow);
    Serial.println(F("\n\r- - - - -"));  
    
    Serial.println("RF24-BLOB-BEGIN");
    Serial.write((uint8_t *)&myData, sizeof(myData));
    Serial.println();
    
    radio.stopListening();
    radio.write( &myData, sizeof(myData) ); 
    radio.startListening();
   
    doBlink(); 
  }
  if (analogRead(HW_BUTT2)<800) // theres a 312ohm resistor before ground on that board :-/
  {
    delay(500);
    digitalWrite(HW_RELAY1, LOW);
    Serial.print("Displaying hours countwodn...");
    int val = myData.sched1 / 60;
    Serial.println(val);
    if (val >=0)
      displayNumber(val);
    else
      doShift(0b00000001);
    delay(2000);
    digitalWrite(HW_RELAY1, HIGH);
    
    digitalWrite(HW_RELAY2, LOW);
    Serial.print("Displaying hours countwodn...");
    val = myData.sched2 / 60;
    Serial.println(val);
    if (val >=0)
      displayNumber(val);
    else
      doShift(0b00000001);
    delay(2000);
    displayOff();
    digitalWrite(HW_RELAY2, HIGH);
  }
  delay(50);
} // Loop


