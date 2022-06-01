#ifndef __JRLIN_FRAME_BUFFER_H
#define __JRLIN_FRAME_BUFFER_H

#include <AltSoftSerial.h>
#include "Arduino.h"


enum JRLINFrameBufferType {
    fb_id_receive // receive into buffer, 
  , fb_id_send    // send with content, 
  , fb_id_only    // only trigger id
};


// not identical to the master frame buffer!
class JRLINFrameBuffer {
  public:
    JRLINFrameBuffer();
    JRLINFrameBufferType   frametype; 

    uint8_t   addr;
    uint8_t   calc_addr;

    // needed for receive & sending frames
    uint8_t   len;
    uint8_t   data[8];
    uint8_t   cksm;
    uint8_t   exp_cksm;
    uint8_t   updated;
    unsigned long timestamp;

    void init( uint8_t buf_addr, JRLINFrameBufferType buf_frametype, uint8_t buf_len);
    static void debug_buffer_print(AltSoftSerial *_dbgSerial, JRLINFrameBuffer *buf);    //static ?
};


#endif
