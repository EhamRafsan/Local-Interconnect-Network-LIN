#ifndef __JRLIN_MASTER_H
#define __JRLIN_MASTER_H

#include <AltSoftSerial.h>

#include "Arduino.h"
#include "JRLinFrameBuffer.h"

#define DEBUG_MODE_MASTER (0) // 0 no debug, 1 standard debug, 2 extended debug

#define LIN_BREAK_DURATION    (15)   // Number of bits in the break.
#define LIN_TIMEOUT_IN_FRAMES (2)    // Wait in max frame times before declaring a read timeout.


enum LINMasterStates {
  lm_unknown //shall be never reached
  , lm_sync  // can be removed in the future, once readback is implemented
  , lm_id    // can be removed in the future, once readback is implemented
  , lm_get_data
  , lm_get_chksm
};

// Read Return
enum LINMasterRead {
  lm_ret_unknown //never
  , lm_ret_processing //processing
  , lm_ret_frame_valid //ok
  , lm_ret_frame_timeout //error
  , lm_ret_wrong_checksum //error
  , lm_ret_frame_buffer_not_found //error
};

class JRLINMaster {
  protected:

    // UART Config
    bool _serialOn;
    uint32_t _speed;

    // Break Pin Config
    uint8_t _txPin;

    // LIN configuration parameters
    uint32_t _tbit; //32 bit instead of 16bit int, convert to uint32_t!
    uint32_t _nominalFrameTime;
    uint32_t _timeout;
    uint32_t _brk_end;    // duration, not time period
    uint32_t _brk_begin;  // duration, not time period

    // State Machine for Master
    LINMasterStates _linMasterStateMachine;

    // Frame List
    JRLINFrameBuffer *_framelist; //pointer to the schedule list to process
    uint32_t _fcount; // used for what?

    // Debug Serial Config
    AltSoftSerial *_ptrDbgSerial;
    uint8_t  _charcounter; //useless? use _buffer_count instead?
     
    void serialBreak();
        
  public:
    JRLINMaster();
    void setFrameList(JRLINFrameBuffer *framelist, int fcount);

    void begin(uint32_t speed, uint8_t break_pin = 1, AltSoftSerial *dbgSerial = NULL);

    void master_send(JRLINFrameBuffer *ptrFrame);
    void slave_send(JRLINFrameBuffer *ptrFrame, uint8_t proto = 2);
    
    LINMasterRead master_receive(JRLINFrameBuffer *ptrFrame, unsigned long frame_start, unsigned long max_frame_duration);
};

#endif
