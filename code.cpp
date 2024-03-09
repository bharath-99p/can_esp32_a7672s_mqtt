#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include<SoftwareSerial.h>

//_________________________________esp32--->mqtt____________________________________________________

SoftwareSerial mySerial(3,2);
String Publish = "/publish"; //Publish Topic
String Subscribe = "/subscribe"; //Subscribe Topic
int flag_mqtt =0;

//_________________________________CAN_CONFIGURATION______________________________________________

CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size
int flag_can = 0;

void setup() {

  //___________________________________CAN_SETUP_________________________________________  
  
  Serial.begin(115200);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");
  CAN_cfg.speed = CAN_SPEED_125KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();
   
  //____________________________________ESP32->mqtt server _______________________________________
  
  mySerial.begin(115200);

  //AT Commands for setting up the client id and Server
  //Need to be executed once -- Open serial terminal doe seeing the debug messages
  Serial.println("Connecting To Server........");
  mySerial.println("ATE0");
  delay(2000);
  mySerial.println("AT+CMQTTSTART"); //Establishing MQTT Connection
  delay(2000); 
  mySerial.println("AT+CMQTTACCQ=0,\"elementz/123\""); //Client ID - change this for each client as this need to be unique
  delay(2000);
  mySerial.println("AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",90,1"); //MQTT Server Name for connecting this client
  delay(2000);

  //SUBSCRIBE MESSAGE
  //Need to be executed once
  mySerial.println("AT+CMQTTSUBTOPIC=0,9,1"); //AT Command for Setting up the Subscribe Topic Name 
  delay(2000);
  mySerial.println(Subscribe); //Topic Name
  delay(2000);
  mySerial.println("AT+CMQTTSUB=0,4,1,1"); //Length of message
  delay(2000);
  mySerial.println("HAII"); //message
  delay(2000);
  Serial.println("Done"); 

}
void loop() 
{
  //_________________________DATA---->ESP32________________________________________________________________________
  CAN_frame_t rx_frame;

  unsigned long currentMillis = millis();

  // Receive next CAN frame from queue
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

    if (rx_frame.FIR.B.FF == CAN_frame_std) {
      printf("New standard frame");
    }
    else {
      printf("New extended frame");
    }

    if (rx_frame.FIR.B.RTR == CAN_RTR) {
      printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
    }
    else {
      printf(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
      for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
        printf("0x%02X ", rx_frame.data.u8[i]);
      }
      printf("\n");
    }
  }
  
 //Receiving MODEM Response
  while(mySerial.available()>0)
  {
    delay(10);
    a = mySerial.readString();
    if(flag==0)
    {
      //Serial.println(a);
    flag = 1;
    }
    //Serial.println(b);
    if(a.indexOf("PAYLOAD") != -1)
    {
       flag = 0;
       int new1 = a.indexOf("PAYLOAD");
       String neww = a.substring(new1);
       int new2 = neww.indexOf('\n');
       String new3 = neww.substring(new2+1);
       int new4 = new3.indexOf('\n');
       String new5 = new3.substring(0,new4);
       
       Serial.println("Topic: led/subscribe");
       Serial.print("Message is: ");
       Serial.println(new5);
       new5.remove(new5.length()-1);
       if(new5 == "a")
       {
        state=1;
        Serial.println("LED ON");
        digitalWrite(led, HIGH);
       }
       else if(new5 == "b")
       {
        state=0;
        flag1=0;
        Serial.println("LED OFF");
        digitalWrite(led, LOW);
       }
    }      
  }
  //_____________________________________________________ESP32---->DATA___________________________________
   
  // Send CAN Message
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x001;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = 0x00;
    tx_frame.data.u8[1] = 0x01;
    tx_frame.data.u8[2] = 0x02;
    tx_frame.data.u8[3] = 0x03;
    tx_frame.data.u8[4] = 0x04;
    tx_frame.data.u8[5] = 0x05;
    tx_frame.data.u8[6] = 0x06;
    tx_frame.data.u8[7] = 0x07;
    ESP32Can.CANWriteFrame(&tx_frame);
  }
  //Publish mqtt message
  String  a;
  if(state==0)
  {

    if(digitalRead(sw) == 0 && flag1 == 0)
    {
      //PUBLISH MESSAGE
      flag1 = 1;
      digitalWrite(led, HIGH);
      Serial.println("Publishing Message: LED ON");
      mySerial.println("AT+CMQTTTOPIC=0,8"); //AT Command for Setting up the Publish Topic Name
      delay(1000);
      mySerial.println(Publish); //Topic Name
      delay(1000);
      mySerial.println("AT+CMQTTPAYLOAD=0,1"); //Payload length
      delay(1000);
      mySerial.println("a"); //Payload message
      delay(1000);
      mySerial.println("AT+CMQTTPUB=0,1,60"); //Acknowledgment
      delay(1000);
    }
    else if(digitalRead(sw) == 0 && flag1 == 1)
    {
      flag1 = 0;
     digitalWrite(led, LOW); 
      Serial.println("Publishing Message: LED OFF");
      mySerial.println("AT+CMQTTTOPIC=0,8"); //AT Command for Setting up the Publish Topic Name
      delay(1000);
      mySerial.println(Publish); //Topic Name
      delay(1000);
      mySerial.println("AT+CMQTTPAYLOAD=0,1"); //Payload length
      delay(1000);
      mySerial.println("b"); //Payload message
      delay(1000);
      mySerial.println("AT+CMQTTPUB=0,1,60"); //Acknowledgment
      delay(1000);
    }
  }
  if(state==1)
  {
    if(digitalRead(sw) == 0 && flag1 == 0)
    {
      //PUBLISH MESSAGE
      flag1 = 1;
      digitalWrite(led, LOW);
      Serial.println("Publishing Message: LED OFF");
      mySerial.println("AT+CMQTTTOPIC=0,8"); //AT Command for Setting up the Publish Topic Name
      delay(1000);
      mySerial.println(Publish); //Topic Name
      delay(1000);
      mySerial.println("AT+CMQTTPAYLOAD=0,1"); //Payload length
      delay(1000);
      mySerial.println("b"); //Payload message
      delay(1000);
      mySerial.println("AT+CMQTTPUB=0,1,60"); //Acknowledgment
      delay(1000);
    }
    else if(digitalRead(sw) == 0 && flag1 == 1)
    {
      flag1 = 0;
      digitalWrite(led,HIGH); 
      Serial.println("Publishing Message: LED ON");
      mySerial.println("AT+CMQTTTOPIC=0,8"); //AT Command for Setting up the Publish Topic Name
      delay(1000);
      mySerial.println(Publish); //Topic Name
      delay(1000);
      mySerial.println("AT+CMQTTPAYLOAD=0,1"); //Payload length
      delay(1000);
      mySerial.println("a"); //Payload message
      delay(1000);
      mySerial.println("AT+CMQTTPUB=0,1,60"); //Acknowledgment
      delay(1000);
    }
  }

}
