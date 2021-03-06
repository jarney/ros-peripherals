/*
 * Ros-friendly firmware for an array of 5 HC-SD04
 * sonar sensors.
 * (c) 2018 Ensor Robotics, Jonathan S. Arney
 * All Rights Reserved
 */

#include <ArduinoJson.h>
#include <NewPing.h>

#define MAX_RANGE_METERS (4.2f)

class NewPing3 : public NewPing {
private:
  unsigned long last5[5];
  uint8_t i;
  unsigned long ping_cm2();
public:
  NewPing3(uint8_t trigger_pin, uint8_t echo_pin, unsigned int max_cm_distance = MAX_SENSOR_DISTANCE);
  float ping_m();
};

NewPing3::NewPing3(uint8_t trigger_pin, uint8_t echo_pin, unsigned int max_cm_distance) :
  NewPing(trigger_pin, echo_pin, max_cm_distance)
  
{
  i = 0;
  last5[0] = 0;
  last5[1] = 0;
  last5[2] = 0;
  last5[3] = 0;
  last5[4] = 0;
}

unsigned long averagevalue(unsigned long *v, int n)
{
  unsigned long total = 0;
  for (int i = 0; i < n; i++) {
    total += v[i];
  }
  return total / n;

}

unsigned long NewPing3::ping_cm2()
{
  unsigned long pingvalue = NewPing::ping_cm();
  last5[i] = pingvalue;
  i++;
  i = i % 3;
  return averagevalue(last5, 3);
}

float NewPing3::ping_m()
{
  unsigned long pingvalue = ping_cm2();

  if (pingvalue == 0) {
    return MAX_RANGE_METERS;
  }
  
  float fv = ((float)pingvalue) / 100.0f;

  if (fv > MAX_RANGE_METERS) return MAX_RANGE_METERS;

  return fv;
  
}

// Supported Messages:
// {"msg":"subscribe"}
// {"msg":"unsubscribe"}
// {"msg":"device-discovery"}

NewPing3 sonar0(2,3);
NewPing3 sonar1(4,5);
NewPing3 sonar2(6,7);
NewPing3 sonar3(8,9);
NewPing3 sonar4(10,11);

const int STATE_IDLE = 0;
const int STATE_RUNNING = 1;

int state = STATE_IDLE;

boolean on;

void setup() {
  //Serial Port begin
  Serial.begin (38400);
  //Define inputs and outputs

  on=false;
  
}

void loop()
{

  handle_serial();
  
  switch (state) {
    case STATE_IDLE:
      handle_idle();
      break;
    case STATE_RUNNING:
      handle_running();
      break;
  }

}


typedef void (*msghandler_fn_t)(void);

typedef struct {
  const char *messageType;
  msghandler_fn_t handle;
} msghandler_t;

char serial_buffer[128];
int serial_buffer_ptr = 0;

void handle_serial() {
  
  while (Serial.available() > 0) {
    serial_buffer[serial_buffer_ptr] = Serial.read();
    
    if (serial_buffer[serial_buffer_ptr] == '\n') {
      processBuffer(serial_buffer);
      serial_buffer_ptr = 0;
    }
    else if (serial_buffer_ptr >= 128) {
      serial_buffer_ptr = 0;
    }
    else {
      serial_buffer_ptr++;
    }
  }
}

void processBuffer(char *serial_buffer) {
  char *p = serial_buffer;
  while (*p != '\n') {
    p++;
  }
  *p = '\0';
  
  msghandler_fn_t handler = (msghandler_fn_t)NULL;
  {
    StaticJsonBuffer<512> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(serial_buffer);
  
    // Test if parsing succeeds.
    if (!root.success()) {
      writeError(String("Parse error, could not parse message, invalid JSON format."));
      return;
    }
    handler = (msghandler_fn_t)getHandler(root);
  }

  handler();
  
}


msghandler_t messageHandlers[] = {
  {"device-discovery", handleDeviceDiscovery},
  {"subscribe", handleSubscribe},
  {"unsubscribe", handleUnsubscribe},
  {NULL, NULL}
};

void handleSubscribe() {
  state = STATE_RUNNING;
}

void handleUnsubscribe(){
  state = STATE_IDLE;
}

void handleDDHeader() {
  const char *msg = "{" 
"\"msg\": \"device-information\","
"\"id\":\"4c4020ce-879f-11e8-9a94-a6cf71072f73\","
"\"components\": [";
  Serial.print(msg);

}
void handleDDFooter() {
  const char *msg =
  "]"
"}";
  Serial.print(msg);
}

void handleDDDevice(const char*id, const char*loc, const char*orient) {
    Serial.print("{");
    Serial.print("\"id\":\"");
    Serial.print(id);
    Serial.print("\",");
    Serial.print(
        "\"type\": \"Range::ULTRASOUND\","
        "\"field_of_view\": 0.42,"
        "\"min_range\": 0.05,"
        "\"max_range\": 1.5,"
        );
    Serial.print("\"location\": { \"xyz\": ");
        Serial.print(loc);
        Serial.print(", \"rpy\": ");
        Serial.print(orient);
        Serial.print(", \"orient_type\": \"euler_degrees\"");
    Serial.print("}");
    Serial.print("}");
}

// This method sends the device description
// payload back to the host.
// We don't use the arduino json library
// here because it takes way too much stack space to send this message
// and we can just place the payload directly in code-space
// since it's a fixed payload.
void handleDeviceDiscovery() {

  handleDDHeader();
  handleDDDevice("0", "[-0.104,0.025,-0.015]", "[10, 0, -90]");
  Serial.print(",");
  handleDDDevice("1", "[-0.083,0.094,-0.015]", "[10, 0, -45]");
  Serial.print(",");
  handleDDDevice("2", "[0, 0.097,-0.015]", "[0, 0, 0]");
  Serial.print(",");
  handleDDDevice("3", "[0.070,0.094,-0.015]", "[10, 0, 45]");
  Serial.print(",");
  handleDDDevice("4", "[0.108,0.025,-0.015]", "[10, 0.24, 90]");

  handleDDFooter();
  
  Serial.println();
}

void handleError() {
  String errormsg = String("Unknown command type");
  writeError(errormsg);
}

void * getHandler(JsonObject& request) {
  
  JsonVariant msg = request["msg"];
  if (!msg.success()) {
    writeError("The 'msg' field is required");
    return (void*)handleError;
  }

  msghandler_t *messageHandler = messageHandlers;
  while (1) {
    if (messageHandler->messageType == NULL) {
      break;
    }
    if (strcmp(msg.as<char*>(), messageHandler->messageType) == 0) {
      return (void*)messageHandler->handle;
    }
    messageHandler++;
  }
  return (void*)handleError;
}



void writeError(String msg) {

  Serial.print("{\"msg\":\"");
  Serial.print(msg);
  Serial.print("\"}");
  Serial.println();
}


void handle_idle() {
    delay(500);
}





void handle_running() {
#ifdef DEBUG_PINGS
    sonar0.ping_m();
    delay(30);
    sonar1.ping_m();
    delay(30);
    sonar3.ping_m();
    delay(30);
    sonar4.ping_m();
    delay(30);
    float f = sonar2.ping_m();
    delay(30);

    int n = (int)((f/MAX_RANGE_METERS)*60.0f);

    for (int i = 0; i < n; i++) {
      Serial.print("#");
    }
    Serial.println();
#else

    StaticJsonBuffer<512> jsonBuffer;
    JsonObject& response = jsonBuffer.createObject();
    response["msg"] = "sensor-data";

    JsonArray & sensorData = response.createNestedArray("sensor_data");
    float f = sonar0.ping_m();
    delay(10);
    {
      JsonObject & sensor0 = sensorData.createNestedObject();
      sensor0["id"] = "0";
      sensor0["range"] = f;
    }
    f = sonar1.ping_m();
    delay(10);
    {
      JsonObject & sensor0 = sensorData.createNestedObject();
      sensor0["id"] = "1";
      sensor0["range"] = f;
    }
    f = sonar2.ping_m();
    delay(10);
    {
      JsonObject & sensor0 = sensorData.createNestedObject();
      sensor0["id"] = "2";
      sensor0["range"] = f;
    }
    f = sonar3.ping_m();
    delay(10);
    {
      JsonObject & sensor0 = sensorData.createNestedObject();
      sensor0["id"] = "3";
      sensor0["range"] = f;
    }
    f = sonar4.ping_m();
    delay(10);
    {
      JsonObject & sensor0 = sensorData.createNestedObject();
      sensor0["id"] = "4";
      sensor0["range"] = f;
    }

    response.printTo(Serial);
    Serial.println();
#endif    
}



