#include "arduino_secrets.h"
/* 
  Sketch generated by the Arduino IoT Cloud Thing "Untitled"
  https://create.arduino.cc/cloud/things/926c0d8c-551b-471b-95ad-db8d88e2a675 

  Arduino IoT Cloud Variables description

  The following variables are automatically generated and updated when changes are made to the Thing

  float presion;
  float setpoint;
  bool terreno;
  bool terreno2;
  bool terreno3;

  Variables which are marked as READ/WRITE in the Cloud Thing will also have functions
  which are called when their values are changed from the Dashboard.
  These functions are generated with the Thing and added at the end of this sketch.
*/

#include "thingProperties.h"

//  Declaracion de variables extras
/* MCP2515 definitions */
#define WS_BUFFER_SIZE  13
#ifdef ARDUINO_ARCH_ESP32
#define TEST_PGN  0x18F00401
#else
#define TEST_PGN  0x18F00402
#endif
// CAN TX Variables
unsigned long txId = 0;
unsigned char txLen = 0;
unsigned char txBuf[8];
char msgString[128];
unsigned long prevTX = 0;                                        // Variable to store last execution time
const unsigned int invlTX = 1000;                                // One second interval constant
// Generic CAN data to send
byte data2[8] ={0x32, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// CAN RX Variables
unsigned long rxId;
unsigned char len;
unsigned char rxBuf[8];
unsigned char rxMsgBuffer[13];
// CAN0 INT and CS
#define CAN0_INT D2                            // Set INT to pin 2 on ESP8266
MCP_CAN CAN0(D8);                               // Set CS to pin 15 on ESP8266

//  Función hexToDec
void convertTxMessage(uint8_t juan );


void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(115200);
  
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500); 

  // Defined in thingProperties.h
  initProperties();

  /* Initialize MCP CAN */
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  // Since we do not set NORMAL mode, we are in loopback mode by default.
  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT);                           // Configuring pin for /INT input
  
  Serial.println("MCP2515 Loopback ...");



  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
 */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void loop() {
  ArduinoCloud.update();

  /*  Lectura de CAN  */

  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    
    if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

    
  
    Serial.print(msgString);
    if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }
        
    Serial.println();
  }
  /*  Transmission CAN  */
  if(millis() - prevTX >= invlTX){                    // Send this at a one second interval. 
    prevTX = millis();
    byte sndStat = CAN0.sendMsgBuf(0x04FEF9A3, 8, data2);
    
    if(sndStat == CAN_OK)
      Serial.println("CAN Message Sent Successfully!");
    else
      Serial.println("Error Sending CAN Message...");
  }
  delay(500);

  
  presion = rxBuf[0];
}


/*
  Since LED is READ_WRITE variable, onLEDChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onLEDChange()  {
  /*if(LED == 0)
  {
    digitalWrite(2, HIGH);
  }
  else
  {
    digitalWrite(2, LOW);
  }*/
  
  // Add your code here to act upon LED change
}




/*
  Since Setpoint is READ_WRITE variable, onSetpointChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onSetpointChange()  
{
  // Add your code here to act upon Setpoint change
   if(setpoint > 20)
  {
    digitalWrite(2, LOW);
    presion = 30;
  }
  else
  {
    digitalWrite(2, HIGH);
    presion = 80;
  }
  
}


/*
  Since Terreno is READ_WRITE variable, onTerrenoChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onTerrenoChange()  {
  // Add your code here to act upon Terreno change

  if(terreno = true)
  {
    terreno2 = false;
    terreno3 = false;
   // presion = 5;
    setpoint = 5;
    data2[0] = setpoint;
    
  }
  else
  {
    data2[0] = 0;
  }
  
  
}


/*
  Since Terreno2 is READ_WRITE variable, onTerreno2Change() is
  executed every time a new value is received from IoT Cloud.
*/
void onTerreno2Change()  {
  // Add your code here to act upon Terreno2 change
  
  if(terreno2 = true)
  {
    terreno = false;
    terreno3 = false;
    //presion = 40;
    setpoint = 50;
    //  Actualizamos el valor del setpoint para mandarlo en la transmisión
    data2[0] = setpoint;
    
  }
    else
  {
    data2[0] = 0;
  }
}


/*
  Since Terreno3 is READ_WRITE variable, onTerreno3Change() is
  executed every time a new value is received from IoT Cloud.
*/
void onTerreno3Change()  {
  // Add your code here to act upon Terreno3 change

  if(terreno3 = true)
  {
    terreno = false;
    terreno2 = false;
    //presion = 25;
    setpoint = 30;
    data2[0] = setpoint;
    
  }
  else
  {
    data2[0] = 0;
  }
}


void convertTxMessage(uint8_t juan )
{

  data2[0] = setpoint;
  int kucha;
  kucha = juan;
 
  
}
