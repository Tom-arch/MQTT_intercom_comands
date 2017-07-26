#include <UIPEthernet.h>
#include <PubSubClient.h>

//Define constants
#define LEDPIN           13
#define ARR_LENGTH       4

#define SHORT_TIME       500
#define LONG_TIME        2000

#define PUBLISH_INTERVAL 10000

//MQTT constants
const PROGMEM char* const SERVER = "192.168.1.4";
const PROGMEM uint16_t PORT = 1883;
const PROGMEM char* const CLIENT_ID = "your_client_id";
const PROGMEM char* const USERNAME = "test";
const PROGMEM char* const PASSWORD = "your_password";
/*
const PROGMEM char* STATOPIC1 = "/hass/living/small_gate/status";
const PROGMEM char* STATOPIC2 = "/hass/living/main_door/status";
const PROGMEM char* STATOPIC3 = "/hass/living/external_light/status";
const PROGMEM char* STATOPIC4 = "/hass/living/gate/status";
const PROGMEM char* CMDTOPIC1 = "/hass/living/small_gate/commands";
const PROGMEM char* CMDTOPIC2 = "/hass/living/main_door/commands";
const PROGMEM char* CMDTOPIC3 = "/hass/living/external_light/commands";
const PROGMEM char* CMDTOPIC4 = "/hass/living/gate/commands";
*/
const PROGMEM char* const PAYLOAD_ON = "ON";
const PROGMEM char* const PAYLOAD_OFF = "OFF";

const PROGMEM char* const stateTopics [4] = {"topic1/status", 
                                        "topic2/status",
                                        "topic3/status", 
                                        "topic4/status"};
const PROGMEM char* const commandTopics [4] = {"topic1/commands", 
                                        "topic2/commands",
                                        "topic3/commands", 
                                        "topic4/commands"};
const PROGMEM uint8_t pins [4] = {6, 5, 4, 3};
const PROGMEM long delays [4] = {SHORT_TIME, LONG_TIME, SHORT_TIME, SHORT_TIME};
bool states [4] = {false, false, false, false};



//Command constant error value
const PROGMEM uint8_t NONE = ARR_LENGTH+1;



const PROGMEM uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};
EthernetClient ethClient;
PubSubClient mqttClient;
volatile uint8_t command = NONE;

//Command handling variables
struct timer {
  uint8_t cmd = 0;
  long expireMillis = 0;
};
timer expireCmds [4];

//Blink and publish timers and control variables
bool publishStatus = false;
unsigned long prevPublishMillis = 0;


//Setup method
void setup() {

  // setup serial communication
  Serial.begin(9600);

  //Setup pins
  for (int i = 0; i < ARR_LENGTH; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], HIGH);
  }
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  
  //Setup ethernet communication using DHCP
  if(Ethernet.begin(mac) == 0) {
    //Serial.println(F("Unable to configure Ethernet using DHCP"));
    for(;;);
  }

  //Setup mqtt client
  mqttClient.setClient(ethClient);
  mqttClient.setServer(SERVER, PORT);
  mqttClient.setCallback(callback);

  //init timers
  prevPublishMillis = millis();
  Serial.println("READY");
}

//Loop method
void loop() {
  
  //First begin the command, if any
  uint8_t cmd = command; //Ensure that command does not change since it is volatile
  if(cmd != NONE && !states[cmd]) {
    //Set the status and create the timer
    for(int i = 0; i < ARR_LENGTH; i++) {
      if(expireCmds[i].expireMillis == 0) { //the timer is free to use
        digitalWrite(pins[cmd], LOW);
        states[i] = true;
        expireCmds[i].cmd = cmd;
        //If this overflows then it willi be the future millis value as if no overflow occurred
        expireCmds[i].expireMillis = millis() + delays[cmd];
        publishStatus = true;
        break;
      }
      //If the timers are all taken the command is ignored
    }
    command = NONE;
  }
  
  unsigned long currentMillis = millis();

  //Check if any timer has expired
  for (int i = 0; i < ARR_LENGTH; i++) {
    if (expireCmds[i].expireMillis != 0 && currentMillis >= expireCmds[i].expireMillis) {
      uint8_t cmd = expireCmds[i].cmd;
      digitalWrite(pins[cmd], HIGH);
      states[i] = false;
      expireCmds[i].expireMillis = 0; //Free the timer
      publishStatus = true;
    }
  }
  

  //Check for millis overflow
  
  if (currentMillis < prevPublishMillis) {
    prevPublishMillis = 0;
  }

  //Check if status timer has expired
  if ((currentMillis - prevPublishMillis) > PUBLISH_INTERVAL) {
    publishStatus = true;
    prevPublishMillis = currentMillis;
  }


//Check if MQTT is connected
  if (!mqttClient.connected()) {
    if(mqttClient.connect(CLIENT_ID, USERNAME, PASSWORD)) {
      for (int i = 0; i < ARR_LENGTH; i++) {
        mqttClient.subscribe(commandTopics[i]);
      }
    }
  } else {
    if(publishStatus) {
      for (int i = 0 ; i < ARR_LENGTH; i++) {
        if (states[i]) {
          mqttClient.publish(stateTopics[i], PAYLOAD_ON);
        } else {
          mqttClient.publish(stateTopics[i], PAYLOAD_OFF);
        }
      }
      publishStatus = false;
    }
    mqttClient.loop();
  }
  
}



void callback(char* topic, byte* payload, unsigned int length) {
  
  //Parse the message
  char message_buff[length+1];
  int i = 0;
  for(i=0; i<length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  String msgString = String(message_buff);
  Serial.println("Payload: " + msgString);

  
  //Choose the correct command
  if (msgString.equals(PAYLOAD_ON)) {
    for (int i = 0; i < ARR_LENGTH; i++) {
      if (strcmp(topic,commandTopics[i])==0) {
        command = i;
        break;
      }
    }
  }
  
}
