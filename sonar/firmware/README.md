# Introduction

This is the firmware intended to run on an Atmel (arduino-like)
platform.  Its responsibility is to mind 5 independent sonar sensors
and communicate state information via the USB port (usb serial)
about the state.

## Protocol

The protocol for communicating via serial with the device is a
simple JSON-based system consisting of messages transferred between
a host computer and the serial device.

All messages are to be transmitted via JSON with a "\n" delmiter indicating
a new packet.  Therefore, the JSON must not be "pretty-print" style with a
'\n' in the middle of the payload.

Example : "{'msg': 'discovery'}\n"

Any newline in the middle of a JSON packet will be rejected as malformed JSON
and the device will respond with an error.

### Device Discovery (host->device)
In order for the host to discover the type of device, it must send
a device discovery message to the device.  The device will respond with a
'Device Information' message which contains the details of the device's
function.

Example:
```json
{
    'msg': 'device-discovery'
}
```


### Device Information (device->host)

This message is sent in response to a 'Device Discovery' message.  The payload
indicates detailed information about the device.

Message Description:

|Field       | Type  | Description                               |
|------------|-------|-------------------------------------------|
| msg        | string| Name of message type (device-information) |
| components | array | Array of component objects                |

Component Object:

|Field       | Type   | Description                               |
|------------|------- |-------------------------------------------|
| id         | string | Unique ID of component within the device  |
| type       | string | Type of device |
| field_of_view | number | Field of view in radians. |
| min_distance | number | Minimum distance in meters that the sensor can detect |
| max_distance | number | Maximum distance in meters that the sensor can detect|
| location | object | Location object giving the position and orientation of the object relative to the device's origin and orientation|

Location object:

|Field       | Type  | Description                               |
|------------|-------|-------------------------------------------|
| pos        | array | 3 floating point numbers indicating the location of the sensor relative to the device's origin with each coordinate expressed in meters |
| orientation | array | 3 floating point numbers indicating the orientation of the sensor relative to the device's origin in radians relative to the device's coordinate system | 


Example:

```json
{
    'msg': 'device-information',
    'components': [
        {
            'id': '0',
            'type': 'linear-sensor',
            'field_of_view': 0.42,
            'min_distance': 0.05,
            'max_distance': 4.2,
            'location': { pos: [0.2, 0.4, 0.02], orient: [0, 0.23, 0.1]}
        }
    ]
}
```


### Subscribe to updates (Host->Device)

This message is sent to tell the device to begin sending
regular status updates to the host.

Example:

```json
{'msg': 'subscribe'}
```
### Unsubscribe

This message is sent to tell the device that it is no longer interested
in receiving status updates.

Example:

```json
{'msg': 'subscribe'}
```


### Sensor Data (device->host)

When subscribed, the device sends these messages on a regular basis
so that the host can be aware of changes in the environment.

Example:

```json
{
    "msg": "sensor-data",
    "sensor_data": [
        {"id": "0", "range": 3.24},
        {"id": "1", "range": 1.43},
        {"id": "2", "range": 2.43},
        {"id": "3", "range": 2.43},
        {"id": "4", "range": 3.43},
    ]
}
```


### Error Message (device->host)

This message is sent whenever the device does not recognize or cannot
parse a message sent from a host.

```json
{
    'msg': 'error',
    'description': 'text indicating the nature of the error',
}
```


## Building the firmware.

The firmware is designed to be built and installed using the Arduino IDE.

The library depends on the ArduinoJson library.  To install it, refer to 
the installation manual here: https://arduinojson.org/v5/doc/installation/



