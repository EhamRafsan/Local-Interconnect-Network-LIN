#ifndef __JRLIN_SLAVE_H
#define __JRLIN_SLACE_H

#include "Arduino.h"
#include "JRLinFrameBuffer.h"

#define DEBUG_MODE_SLAVE (0) // 0 no debug, 1 standard debug, 2 extended debug


#define LIN_BREAK_DURATION    (15)   // Number of bits in the break.
#define LIN_TIMEOUT_IN_FRAMES (2)    // Wait this many max frame times before declaring a read timeout. In microseconds!



enum LINStates {
  unknown //shall be never reached
  , off
  , detected_break
  , wait_sync_byte
  , get_id
  , get_data
  , get_chksm
  , reply_to_id_empty_and_terminate
  , empty_and_terminate
  , empty_and_terminate_wrongchecksum
  , empty_and_terminate_timeout
  , empty_and_terminate_no_id_match
};

enum SlaveStates {
  frameProcessing
  , frameValid
  , frameSlotTimeout
  , frameWrongChecksum
  , frameNoIdMatch
  , frameIDReplied
};

enum BREAKState {
  waiting
  , pindown_indicated
  , valid
};

class JRLINSlave {
  protected:
    AltSoftSerial *_ptrDbgSerial;
    
    bool _serialOn;  // UART is on or off
    uint32_t _speed; // UART speed

    uint8_t _buffer_count;

    LINStates _linStateMachine;

    JRLINFrameBuffer *_framelist; //pointer to the schedule list to process
    JRLINFrameBuffer *_ptrCurrentFrame;
    uint16_t _fcount; //framebuffer size

    void _terminate();
    void _slave_send(uint8_t addr, const uint8_t* message, uint8_t nBytes, uint8_t proto = 2);

  public:
    JRLINSlave();
    volatile BREAKState breakSymbol; //externally set through the interrupt routine

    /** Configuration API */
    void config(uint32_t speed, AltSoftSerial *dbgSerial = NULL); // merge config with begin
    void setFrameList(JRLINFrameBuffer *framelist, uint16_t fcount);
    
    /** Control API */
    void begin();
    SlaveStates slave_receive(unsigned long frame_start, unsigned long max_frame_duration);  // only public API

};

#endif
