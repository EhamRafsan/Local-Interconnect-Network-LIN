#include "JRLinSlave.h"
#include "JRLinFrameBuffer.h"
#include <AltSoftSerial.h>


#define DEBUG_MODE_MAIN (2) // 0 no debug, 1 debug pin, 2 add basic serial, 3 extended serial
#define DEBUG_SERIAL_BAUD_RATE (19200)

#define NODE_ID   ("HW UART LIN Slave Node 1")
#define LIN_BAUD_RATE (9600)
#define FAULT_PIN (8)
#define CS_PIN    (7)
#define ONBOARD_LED (13)
#define INT_PIN   (2)
#define DEBUG_PIN (10)

#define PIN_BREAK_TIME (1400)  // in microseconds
#define SLOT_TIMEOUT (25000)   // 25 000 microsec = 25 ms

#define SCHEDULE_ENTRIES (1)

volatile unsigned long _pin_down_time = 0;
volatile unsigned long frame_start = 0;

JRLINSlave lin;
JRLINFrameBuffer framelist[SCHEDULE_ENTRIES]; //shedule entry memory

int sRetStatus = -1;

AltSoftSerial altSerial;

void setup() {
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW); //dark
  
  // put your setup code here, to run once:
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  pinMode(FAULT_PIN, OUTPUT);
  digitalWrite(FAULT_PIN, HIGH);

#if DEBUG_MODE_MAIN >= 1
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, LOW);
#endif

  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), pinchange, CHANGE);

  lin.breakSymbol = waiting;

  altSerial.begin(DEBUG_SERIAL_BAUD_RATE);
#if DEBUG_MODE_MAIN >= 2
  altSerial.println(NODE_ID);
#endif
  lin.config(LIN_BAUD_RATE, &altSerial);
framelist[0].init(0x30, fb_id_receive, 4);

//  framelist[0].init(0x27, fb_id_receive, 8);
//  framelist[1].init(0x30, fb_id_receive, 4);
//
//  framelist[2].init(0x32, fb_id_send, 4);
//  framelist[2].data[0] = 0x17;
//  framelist[2].data[1] = 0x11;
//  framelist[2].data[2] = 0x19;
//  framelist[2].data[3] = 0x77;
//  framelist[2].len = 4;

#if DEBUG_MODE_MAIN >= 2
  JRLINFrameBuffer::debug_buffer_print(&altSerial, &framelist[0]);
  JRLINFrameBuffer::debug_buffer_print(&altSerial, &framelist[1]);
  JRLINFrameBuffer::debug_buffer_print(&altSerial, &framelist[2]);
#endif

  lin.setFrameList(framelist, SCHEDULE_ENTRIES);
}

void loop()
{
 
  if (lin.breakSymbol != valid) {
    compute();
    return;
  } else {
    
    static SlaveStates ret;
    ret = lin.slave_receive(frame_start, SLOT_TIMEOUT );

    if ( ret == frameProcessing) {
      sRetStatus = 1000; 
      
    } else if ( ret == frameValid) {
      sRetStatus = 2000;
#if DEBUG_MODE_MAIN >= 1      
      digitalWrite(DEBUG_PIN, LOW); // frame finished
#endif      
#if DEBUG_MODE_MAIN >= 3
      altSerial.println("V");
#endif
    } else if ( ret == frameSlotTimeout) {
      sRetStatus = 3000;
#if DEBUG_MODE_MAIN >= 1      
      digitalWrite(DEBUG_PIN, LOW); 
#endif      
#if DEBUG_MODE_MAIN >= 3
      altSerial.println("T");
#endif
    } else if ( ret == frameWrongChecksum) {
      sRetStatus = 4000;
#if DEBUG_MODE_MAIN >= 1        
      digitalWrite(DEBUG_PIN, LOW); 
#endif 
#if DEBUG_MODE_MAIN >= 3
      altSerial.println("C");
#endif
    } else if ( ret == frameNoIdMatch) {
      sRetStatus = 5000;
#if DEBUG_MODE_MAIN >= 1        
      digitalWrite(DEBUG_PIN, LOW); 
#endif
#if DEBUG_MODE_MAIN >= 3
      altSerial.println("NID");
#endif
    } else if ( ret == frameIDReplied) {
      sRetStatus = 6000;
#if DEBUG_MODE_MAIN >= 1        
      digitalWrite(DEBUG_PIN, LOW); 
#endif
#if DEBUG_MODE_MAIN >= 3
      altSerial.println("REP");
#endif

    } else {
      sRetStatus = 121;
      
    }
  }
}

void compute() {
  if (sRetStatus != -1) {
#if DEBUG_MODE_MAIN >= 2
    altSerial.print("#R: ");
    altSerial.println(sRetStatus);
#endif
    if (sRetStatus == 2000)
    {
      //framelist[2].data[3] ++  ;     //Test Counter
      if (framelist[0].data[3] == 0x01) {
        digitalWrite(ONBOARD_LED, HIGH);
      } else {
        digitalWrite(ONBOARD_LED, LOW);
      }
    }
    sRetStatus = -1;
  }
  return;
}

// Interrupt routine
void pinchange() {
  if (digitalRead(INT_PIN) == LOW) { //pin logic moved to LOW
    _pin_down_time = micros();
    if (lin.breakSymbol != valid) lin.breakSymbol = pindown_indicated;
  } else { // pin logic moved to HIGH, check if valid break
    if ((micros() - _pin_down_time) >= PIN_BREAK_TIME) {
      lin.breakSymbol = valid;
      frame_start = micros(); // frame has started
#if DEBUG_MODE_MAIN >= 1        
      digitalWrite(DEBUG_PIN, HIGH); // frame started
#endif      
    }
  }
}
