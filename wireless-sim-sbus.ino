#include "Arduino.h"
//#include <avr/interrupt.h>
#include <Joystick.h>

// Use to enable output of PPM values to serial
//#define SERIALOUT

// Minimal and maximal PPM-pulse * 2 for more precission, because grab 2xPPM-pulse. For real values divide half.
// Example: Minimal PPM-value 1110, 1110 * 2 = 2220
#define MIN_PULSE_WIDTH     204 //2000 // Minimal pulse
#define CENTER_PULSE_WIDTH 1020 //3000 // Middle pulse
#define MAX_PULSE_WIDTH    1836 //4000 // Maximal pulse
#define CENTER_PULSE_JITTER   0 // Dead zone. If possible, do not use it.

// Min and Max joystick value
#define USB_STICK_MIN -32767
#define USB_STICK_MAX  32767

// Number of channels. Between 4-8.
#define RC_CHANNELS_COUNT 8

// Create the Joystick
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK, 2, 0, true, true, true, true, true, true, false, false, false, false, false);

// Enum defines the order of channels
enum {
  ROLL,
  PITCH,
  THROTTLE,
  YAW,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};

// ********** SBUS ************
#define port Serial1
//#define SBUS_BAUDRATE         98000
#define SBUS_BAUDRATE         100000
//#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)
#define SBUS_PORT_OPTIONS     SERIAL_8E2

//#define ALL_CHANNELS
#define SBUS_MAX_CHANNELS     18
#define SBUS_FRAME_SIZE       25

//#define SBUS_FRAME_BEGIN_BYTE 0x0F
#define SBUS_START_BYTE       0x0F
#define SBUS_END_BYTE         0x00

#define SBUS_DIGITAL_CHANNEL_MIN   MIN_PULSE_WIDTH //173
#define SBUS_DIGITAL_CHANNEL_MAX   MAX_PULSE_WIDTH //1812

#define SBUS_SIGNAL_OK             0x00
#define SBUS_SIGNAL_LOST           0x01
#define SBUS_SIGNAL_FAILSAFE       0x03

#define SBUS_STATE_FAILSAFE        (1 << 0)
#define SBUS_STATE_SIGNALLOSS      (1 << 1)

#define SBUS_FLAG_CHANNEL_17       (1 << 0)
#define SBUS_FLAG_CHANNEL_18       (1 << 1)
#define SBUS_FLAG_SIGNAL_LOSS      (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE  (1 << 3)

// 16 channel (11 bit) + 2 digital channel
int16_t channels[SBUS_MAX_CHANNELS] = {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
uint8_t failsafeStatus = SBUS_SIGNAL_FAILSAFE; // ?SBUS_SIGNAL_OK
int toChannels = 0;
// private variables
uint8_t inBuffer[SBUS_FRAME_SIZE];
int bufferIndex = 0;
uint8_t inData;
int feedState = 0;

struct sbusFrame_s {
    uint8_t syncByte;
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int ch0 : 11;
    unsigned int ch1 : 11;
    unsigned int ch2 : 11;
    unsigned int ch3 : 11;
    unsigned int ch4 : 11;
    unsigned int ch5 : 11;
    unsigned int ch6 : 11;
    unsigned int ch7 : 11;
    unsigned int ch8 : 11;
    unsigned int ch9 : 11;
    unsigned int ch10 : 11;
    unsigned int ch11 : 11;
    unsigned int ch12 : 11;
    unsigned int ch13 : 11;
    unsigned int ch14 : 11;
    unsigned int ch15 : 11;
    uint8_t flags;
    /**
     * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
     *
     * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
     * and
     * https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
     */
    uint8_t endByte;
} __attribute__ ((__packed__));

typedef union {
    uint8_t bytes[SBUS_FRAME_SIZE];
    struct sbusFrame_s frame;
} sbusFrame_t;

static sbusFrame_t ssbusFrame;


// SETUP
void setup() {
  sbusBegin();

  Joystick.setXAxisRange(USB_STICK_MIN, USB_STICK_MAX);
  Joystick.setYAxisRange(USB_STICK_MIN, USB_STICK_MAX);
  Joystick.setZAxisRange(USB_STICK_MIN, USB_STICK_MAX);
  Joystick.setRxAxisRange(USB_STICK_MIN, USB_STICK_MAX);
  Joystick.setRyAxisRange(USB_STICK_MIN, USB_STICK_MAX);
  Joystick.setRzAxisRange(USB_STICK_MIN, USB_STICK_MAX);

  Joystick.begin(false);

#ifdef SERIALOUT
  Serial.begin(115200);
#endif
}

// LOOP
void loop(){
  feedLine();
  if (toChannels == 1) {
    updateChannels();
//    updateServos();
    toChannels = 0;
  }

  setControllerDataJoystick();
  Joystick.sendState();

#ifdef SERIALOUT
  Serial.print(channels[ROLL]);
  Serial.print("\t");
  Serial.print(channels[PITCH]);
  Serial.print("\t");
  Serial.print(channels[THROTTLE]);
  Serial.print("\t");
  Serial.print(channels[YAW]);
  Serial.print("\t");
  Serial.print(channels[AUX1]);
  Serial.print("\t");
  Serial.print(channels[AUX2]);
  Serial.print("\r\n");
#endif
}

// ********** Joystick ************

// Set joystick data in HID descriptor. Use functions: setXAxis, setYAxis, setZAxis, setRxAxis, setRyAxis, setRzAxis, setRudder, setThrottle.
void setControllerDataJoystick(){
  Joystick.setXAxis(stickValue(channels[ROLL]));
  Joystick.setYAxis(stickValue(channels[PITCH]));
  Joystick.setZAxis(stickValue(channels[THROTTLE]));
  Joystick.setRxAxis(stickValue(channels[YAW]));
  Joystick.setRyAxis(stickValue(channels[AUX1]));
  Joystick.setRzAxis(stickValue(channels[AUX2]));
  Joystick.setButton(0, channels[AUX3] > CENTER_PULSE_WIDTH);
  Joystick.setButton(1, channels[AUX4] > CENTER_PULSE_WIDTH);
}

// Convert a value in the range of [Min Pulse - Max Pulse] to [USB_STICK_MIN/USB_STICK_MAX]
uint16_t stickValue(uint16_t rcVal) {
  if(rcVal > (CENTER_PULSE_WIDTH + CENTER_PULSE_JITTER)) {return constrain( map(rcVal, CENTER_PULSE_WIDTH, MAX_PULSE_WIDTH, (USB_STICK_MAX + USB_STICK_MIN) / 2, USB_STICK_MAX ), (USB_STICK_MAX + USB_STICK_MIN) / 2, USB_STICK_MAX);}
  if(rcVal < (CENTER_PULSE_WIDTH - CENTER_PULSE_JITTER)) {return constrain( map(rcVal, MIN_PULSE_WIDTH, CENTER_PULSE_WIDTH, USB_STICK_MIN, (USB_STICK_MAX + USB_STICK_MIN) / 2 ), USB_STICK_MIN, (USB_STICK_MAX + USB_STICK_MIN) / 2);}
  return (USB_STICK_MAX + USB_STICK_MIN) / 2;
}


// ********** SBUS ************

void sbusBegin() {
  /*, SP_2_STOP_BIT | SP_EVEN_PARITY | SP_8_BIT_CHAR */
  port.begin(SBUS_BAUDRATE, SBUS_PORT_OPTIONS); 
  failsafeStatus = SBUS_SIGNAL_OK;
  toChannels = 0;
  bufferIndex = 0;
  feedState = 0; 
}

// Read channel data
int16_t channel(uint8_t ch) {
  if ((ch > 0) && (ch <= 16)) {
    return channels[ch-1];
  } else {
    return 1023;
  }
}

// Read digital channel data
uint8_t digiChannel(uint8_t ch) {
  if ((ch > 0) && (ch <= 2)) {
    return channels[15+ch];
  } else {
    return 0;
  }
}

uint8_t failsafe(void) {
  return failsafeStatus;
}

void updateChannels(void) {
  // using structure
  channels[0]  = sbusFrame.frame.ch0;
  channels[1]  = sbusFrame.frame.ch1;
  channels[2]  = sbusFrame.frame.ch2;
  channels[3]  = sbusFrame.frame.ch3;
  channels[4]  = sbusFrame.frame.ch4;
  channels[5]  = sbusFrame.frame.ch5;
  channels[6]  = sbusFrame.frame.ch6;
  channels[7]  = sbusFrame.frame.ch7;
#ifdef ALL_CHANNELS
  // & the other 8 + 2 channels if you need them
  channels[8]  = sbusFrame.frame.ch8;
  channels[9]  = sbusFrame.frame.ch9;
  channels[10] = sbusFrame.frame.ch10;
  channels[11] = sbusFrame.frame.ch11;
  channels[12] = sbusFrame.frame.ch12;
  channels[13] = sbusFrame.frame.ch13;
  channels[14] = sbusFrame.frame.ch14;
  channels[15] = sbusFrame.frame.ch15;

  // DigiChannel 1
  if (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_17) {
    channels[16] = SBUS_DIGITAL_CHANNEL_MAX;
  } else {
    channels[16] = SBUS_DIGITAL_CHANNEL_MIN;
  }

  // DigiChannel 2
  if (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_18) {
    channels[17] = SBUS_DIGITAL_CHANNEL_MAX;
  } else {
    channels[17] = SBUS_DIGITAL_CHANNEL_MIN;
  }
#endif

  // Failsafe
  failsafeStatus = SBUS_SIGNAL_OK;
  if (sbusFrame.frame.flags & SBUS_FLAG_SIGNAL_LOSS) {
    failsafeStatus = SBUS_SIGNAL_LOST;
  }
  if (sbusFrame.frame.flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
    // internal failsafe enabled and rx failsafe flag set
    failsafeStatus = SBUS_SIGNAL_FAILSAFE;
  }

}

void feedLine() {
  if (port.available() >= SBUS_FRAME_SIZE) {
    while (port.available() > 0) {
      inData = port.read();
      if (0 == feedState) {
        // feedState == 0
        if (inData != SBUS_START_BYTE){
          //read the contents of in buffer this should resync the transmission
          while (port.available() > 0){
            inData = port.read();
          }
          return;
        } else {
          bufferIndex = 0;
          inBuffer[bufferIndex] = inData;
          inBuffer[SBUS_FRAME_SIZE-1] = 0xff;
          feedState = 1;
        }
      } else {
        // feedState == 1
        bufferIndex ++;
        inBuffer[bufferIndex] = inData;
        if (bufferIndex < (SBUS_FRAME_SIZE-1) 
         && port.available() == 0) {
          feedState = 0;
        }
        if (bufferIndex == (SBUS_FRAME_SIZE-1)) {
          feedState = 0;
          if (inBuffer[0] == SBUS_START_BYTE 
           && inBuffer[SBUS_FRAME_SIZE-1] == SBUS_END_BYTE) {
            memcpy(sbusFrame.bytes, inBuffer, SBUS_FRAME_SIZE);
            toChannels = 1;
          }
        }
      }
    }
  }
}

