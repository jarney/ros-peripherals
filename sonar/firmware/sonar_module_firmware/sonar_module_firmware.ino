/*
 * Ros-friendly firmware for an array of 5 HC-SD04
 * sonar sensors.
 * (c) 2018 Ensor Robotics, Jonathan S. Arney
 * All Rights Reserved
 */

#include <ArduinoJson.h>

// TODO:
// * Place correct location and orientation
//   based on device's origin coordinate system.
// * Final acceptance test and assembly.
// * Write ROS driver software and deploy to PI.
// * Check whole thing into GitHub and upload designs to thingiverse.
// * Make a spare one and send to Ubiquity team.
// Supported Messages:
// {"msg":"subscribe"}
// {"msg":"unsubscribe"}
// {"msg":"device-discovery"}

class SonarDevice {
  private:
    int mTrigger;
    int mEcho;
  public:
    SonarDevice(int trigger, int echo);
    void setup();
    float read();
};

SonarDevice::SonarDevice(int trigger, int echo) {
    mTrigger = trigger;
    mEcho = echo;
}

void SonarDevice::setup() {
  pinMode(mTrigger, OUTPUT);
  pinMode(mEcho, INPUT_PULLUP);
}

float SonarDevice::read() {
    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(mTrigger, LOW);
  delayMicroseconds(5);
  digitalWrite(mTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(mTrigger, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  long duration = pulseIn(mEcho, HIGH, 100000);

  // convert the time into a distance
  // Duration/10 to get to the right units of time.
  // Divide by 2 because sound goes round-trip
  // for this type of sensor.
  // Divide by 331.3 meters per second because that's the speed of sound.
  // The result should be the distance in meters.
  float m = duration / (2.0 * 10.0 *  331.3);
  
  return m;
}

SonarDevice sonar0(2,3);
SonarDevice sonar1(4,5);
SonarDevice sonar2(6,7);
SonarDevice sonar3(8,9);
SonarDevice sonar4(10,11);

const int STATE_IDLE = 0;
const int STATE_RUNNING = 1;

int state = 0;

boolean on;

void setup() {
  //Serial Port begin
  Serial.begin (38400);
  //Define inputs and outputs

  sonar0.setup();
  sonar1.setup();
  sonar2.setup();
  sonar3.setup();
  sonar4.setup();

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
        "\"max_range\": 4.2,"
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

    StaticJsonBuffer<512> jsonBuffer;
    JsonObject& response = jsonBuffer.createObject();
    response["msg"] = "sensor-data";
    
    JsonArray & sensorData = response.createNestedArray("sensor_data");
    float f = sonar0.read();
    if (f > 0.05) {
      JsonObject & sensor0 = sensorData.createNestedObject();
      sensor0["id"] = "0";
      sensor0["range"] = f;
    }
    f = sonar1.read();
    if (f > 0.05) {
      JsonObject & sensor0 = sensorData.createNestedObject();
      sensor0["id"] = "1";
      sensor0["range"] = f;
    }
    f = sonar2.read();
    if (f > 0.05) {
      JsonObject & sensor0 = sensorData.createNestedObject();
      sensor0["id"] = "2";
      sensor0["range"] = f;
    }
    f = sonar3.read();
    if (f > 0.05) {
      JsonObject & sensor0 = sensorData.createNestedObject();
      sensor0["id"] = "3";
      sensor0["range"] = f;
    }
    f = sonar4.read();
    if (f > 0.05) {
      JsonObject & sensor0 = sensorData.createNestedObject();
      sensor0["id"] = "4";
      sensor0["range"] = f;
    }
    

    response.printTo(Serial);
    Serial.println();
    
    delay(50);
}



