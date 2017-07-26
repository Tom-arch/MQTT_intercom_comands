#include <Ethernet.h>
#include <PubSubClient.h>

//Define constants
#define LEDPIN           7
#define ARR_LENGTH       4

#define SHORT_TIME       500
#define LONG_TIME        2000

#define PUBLISH_INTERVAL 10000
#define NO_WIFI   200
#define NO_MQTT   500
#define WORKING   5000

//MQTT constants
const PROGMEM char* const SERVER = "192.168.1.4";
const PROGMEM uint16_t PORT = 1883;
const PROGMEM char* const CLIENT_ID = "your_client_id";
const PROGMEM char* const USERNAME = "user";
const PROGMEM char* const PASSWORD = "password";
const PROGMEM char* const STATOPIC0 = "button0/state";
const PROGMEM char* const STATOPIC1 = "button1/state";
const PROGMEM char* const STATOPIC2 = "button2/state";
const PROGMEM char* const STATOPIC3 = "button3/state";
const PROGMEM char* const CMDTOPIC0 = "button0/cmd";
const PROGMEM char* const CMDTOPIC1 = "button1/cmd";
const PROGMEM char* const CMDTOPIC2 = "button2/cmd";
const PROGMEM char* const CMDTOPIC3 = "button3/cmd";

const PROGMEM char* const PAYLOAD_ON = "ON";
const PROGMEM char* const PAYLOAD_OFF = "OFF";

const uint8_t pins [4] = {6, 5, 4, 3};

const unsigned long delays [4] = {SHORT_TIME, LONG_TIME, SHORT_TIME, SHORT_TIME};
bool states [4] = {false, false, false, false};
const char* stateTopics [4] = {STATOPIC0, STATOPIC1, STATOPIC2, STATOPIC3};
const char* commandTopics [4] = {CMDTOPIC0, CMDTOPIC1, CMDTOPIC2, CMDTOPIC3};



//Command constant error value
const PROGMEM uint8_t NONE = ARR_LENGTH+1;
//Connectivity constants
const PROGMEM byte mac[6] = {0x00,0x01,0x02,0x03,0x04,0x06};
EthernetClient ethClient;
PubSubClient mqttClient;
volatile uint8_t command = NONE;

//Command handling variables
struct timer {
  uint8_t cmd = 0;
  long expireMillis = 0;
};

timer expireTimers [4];

//Blink and publish timers and control variables
bool publishStatus = false;
unsigned long prevPublishMillis = 0;
unsigned long prevBlinkMillis = 0;
unsigned long blinkInterval = NO_MQTT;
bool ledStatus = false;


//Setup method
void setup() {
  
  // setup Serial communication
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
    Serial.println(F("Unable to configure Ethernet using DHCP"));
    for(;;);
  }

  //Setup mqtt client
  mqttClient.setClient(ethClient);
  mqttClient.setServer(SERVER, PORT);
  mqttClient.setCallback(callback);

  //Init timers
  for(uint8_t i = 0; i < ARR_LENGTH; i++) {
    expireTimers[i].expireMillis = 0;
  }
  prevPublishMillis = millis();
  delay(1000);     //Delay to let the shield init
  ledStatus = false;
  blinkInterval = NO_MQTT;
  prevBlinkMillis = millis();

  Serial.println("READY");
}

//Loop method
void loop() {
  
  //First begin the command, if any
  uint8_t cmd = command; //Ensure that command does not change since it is volatile
  if(cmd < NONE && !states[cmd]) {
    Serial.println("CMD");
    //Set the status and create the timer
    for(int i = 0; i < ARR_LENGTH; i++) {
      if(expireTimers[i].expireMillis == 0) { //the timer is free to use
        Serial.print("Timer: ");
        Serial.println(i);
        digitalWrite(pins[cmd], LOW);
        states[cmd] = true;
        expireTimers[i].cmd = cmd;
        //If this overflows then it will be the future millis value as if no overflow occurred
        expireTimers[i].expireMillis = millis() + delays[cmd];
        publishStatus = true;
        Serial.print(expireTimers[i].cmd); Serial.print(" / "); Serial.println(expireTimers[i].expireMillis);
        break;
      }
      //If the timers are all taken the command is ignored
    }
    command = NONE;
  }
  
  unsigned long currentMillis = millis();

  //Check if any timer has expired
  for (uint8_t i = 0; i < ARR_LENGTH; i++) {
    if (expireTimers[i].expireMillis != 0 && currentMillis >= expireTimers[i].expireMillis) {
      Serial.print("Timer "); Serial.print(i); Serial.println(" has expired!");
      uint8_t cmd = expireTimers[i].cmd;
      digitalWrite(pins[cmd], HIGH);
      states[cmd] = false;
      expireTimers[i].expireMillis = 0; //Free the timer
      publishStatus = true;
    }
  }
  

  //Check for millis overflow
  if (currentMillis < prevPublishMillis) {
    prevPublishMillis = 0;
    prevBlinkMillis = 0;
  }

  //Check if status publishing timer and blink timer have expired
  if ((currentMillis - prevPublishMillis) > PUBLISH_INTERVAL) {
    publishStatus = true;
    prevPublishMillis = currentMillis;
  }
  if ((currentMillis - prevBlinkMillis) > blinkInterval) {
    Serial.println("blink");
    Serial.println(blinkInterval);
    triggerLed();
    prevBlinkMillis = currentMillis;
  }

  //Check if ethernet is connected
  int eth = Ethernet.maintain();
  if(eth==1 || eth ==3) {
    Serial.println("ETH DISCONNECTED");
    
    
  } 

//Check if MQTT is connected
  if (!mqttClient.connected()) {
    blinkInterval = NO_MQTT;
    if(mqttClient.connect(CLIENT_ID, USERNAME, PASSWORD)) {
      //Serial.println("CONN");
      for (uint8_t i = 0; i < ARR_LENGTH; i++) {
        mqttClient.subscribe(commandTopics[i]);
      }
    }
  } else {
    blinkInterval = WORKING;
    if(publishStatus) {
      Serial.println("STATUS");
      for (uint8_t i = 0 ; i < ARR_LENGTH; i++) {
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

//This method triggers the led
void triggerLed() {
  if(ledStatus) {
    digitalWrite(LEDPIN, HIGH);
    ledStatus = false;
  } else {
    digitalWrite(LEDPIN, LOW);
    ledStatus = true;
  }
}

//Callback for received messages invoked by mqttClient.loop()
void callback(char* topic, byte* payload, unsigned int len) {
  
  //Parse the message
  char message_buff[len+1];
  uint8_t i = 0;
  for(i=0; i<len; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  //Serial.println("Payload: " + *message_buff);

  
  //Choose the correct command
  if (strcmp(message_buff, PAYLOAD_ON)==0) {
    for (uint8_t i = 0; i < ARR_LENGTH; i++) {
      if (strcmp(topic, commandTopics[i])==0) {
        //Serial.println("COMMAND");
        Serial.println(i);
        command = i;
        break;
      }
    }
  }
  
}
