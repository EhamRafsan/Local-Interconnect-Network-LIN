#include "Arduino.h"
#include "JRLinSlave.h"
#include "JRLinHelper.h"

JRLINSlave::JRLINSlave()
{
  _speed = 0;
  _ptrDbgSerial = NULL;

  _framelist = NULL;
  _ptrCurrentFrame = NULL;

  _buffer_count = 0;
  _serialOn = false;

  _linStateMachine = off;
}

void JRLINSlave::setFrameList(JRLINFrameBuffer *framelist, uint16_t fcount) {
  _framelist = framelist;
  _fcount = fcount;
}

void JRLINSlave::config(uint32_t speed, AltSoftSerial *dbgSerial)
{
  _speed = speed;
  _ptrDbgSerial = dbgSerial;
  _linStateMachine = off;
}

void JRLINSlave::begin() // could be inlined for performance
{
  Serial.begin(_speed);
  _serialOn = true;

#if DEBUG_MODE_SLAVE >= 1
  if (_ptrDbgSerial) {
//    _ptrDbgSerial->begin(DEBUG_SERIAL_BAUD_RATE);
    _ptrDbgSerial->println("Lin:S");
  }
#endif
}

void JRLINSlave::_terminate()  // could be inlined for performance
{
  breakSymbol = waiting; // reset to wait for a new break symbol

  // flush the serial buffers and end the UART
  Serial.flush();
  Serial.end();
  _serialOn = false;

#if DEBUG_MODE_SLAVE >= 2
  if (_ptrDbgSerial) {
    _ptrDbgSerial->println("Lin:E");
    _ptrDbgSerial->end(); //  can create some garbage characters if multiple times started!
  }
#endif

  // reset all buffers
  _buffer_count = 0;

  // reset the state machine to start
  _linStateMachine = wait_sync_byte;
}

void JRLINSlave::_slave_send(uint8_t addr, const uint8_t* message, uint8_t nBytes, uint8_t proto) {
  uint8_t addrbyte = (addr & 0x3f) | JRLINHelper::addrParity(addr);
  uint8_t cksum = JRLINHelper::dataChecksum(message, nBytes, (proto == 1) ? 0 : addrbyte);

  Serial.write(message, nBytes); // data bytes
  Serial.write(cksum);  // checksum
}

SlaveStates JRLINSlave::slave_receive(unsigned long frame_start, unsigned long max_frame_duration)

{
  if ((micros() - frame_start) > max_frame_duration) {
    _linStateMachine = empty_and_terminate_timeout;

    breakSymbol = waiting; // prepare for another frame in the slot
    _terminate();
    return frameSlotTimeout; // cannot fall through ! otherwise the serial code will trigger
  }

  /***
      Serial State Machine
  */
  if (!_serialOn) {
    _serialOn = true; //redundant!
    begin(); //turn on the serial
    _linStateMachine = wait_sync_byte;
    return frameProcessing;
  }

  if (!Serial.available()) return frameProcessing; // true; // do nothing.
 
  static uint8_t c = -1;

  while (Serial.available()) {
    c = Serial.read();

    if ((micros() - frame_start) > max_frame_duration) _linStateMachine = empty_and_terminate_timeout;

    /*
        LIN state machine
    */
    switch (_linStateMachine) {
      case  wait_sync_byte:

        if (c == 0x55)
        {
          _linStateMachine = get_id;
#if DEBUG_MODE_SLAVE >= 2
          if (_ptrDbgSerial) _ptrDbgSerial->println("[x]SYNC");
#endif
        } else {
          _linStateMachine = empty_and_terminate;
        }
        break;

      case  get_id:
        
#if DEBUG_MODE_SLAVE >= 2
        if (_ptrDbgSerial) {
          _ptrDbgSerial->print("[x]ID:");
          _ptrDbgSerial->println(c, HEX);
        }
#endif
        // Extract Extended Frame Information

        _ptrCurrentFrame = NULL;
        for (uint8_t i = 0; i < _fcount; i++) {
          if (_framelist[i].calc_addr ==  c) {

            _ptrCurrentFrame = &_framelist[i];
#if DEBUG_MODE_SLAVE >= 2
            if (_ptrDbgSerial) {
              _ptrDbgSerial->print("=>L: 0x");
              _ptrDbgSerial->println(_framelist[i].len, HEX);
            }
#endif
            break; 
          }
        } //for

        if (_ptrCurrentFrame == NULL) {
          _linStateMachine = empty_and_terminate_no_id_match; // no len found
#if DEBUG_MODE_SLAVE >= 1
          if (_ptrDbgSerial)  _ptrDbgSerial->println("! _ptrCurrentFrame == NULL");
#endif         
        } else {
          //frame was found
          // reply accordingly

          if (_ptrCurrentFrame->frametype == fb_id_receive) {
            // init buffer, set it to empty
            _buffer_count = 0;

            _linStateMachine = get_data;
          } else if (_ptrCurrentFrame->frametype == fb_id_send) {
#if DEBUG_MODE_SLAVE >= 2
            if (_ptrDbgSerial)_ptrDbgSerial->println("!R");
#endif
            _linStateMachine = reply_to_id_empty_and_terminate;

          } else {
            // should never happen! some garbage frame
          }
        }
        break;

      case  get_data:
        if (_ptrCurrentFrame) {

          // keep filling up the buffer
          _ptrCurrentFrame->data[_buffer_count] = c;
#if DEBUG_MODE_SLAVE >= 2
          if (_ptrDbgSerial) {
            _ptrDbgSerial->print("[x]D:");
            _ptrDbgSerial->println(_ptrCurrentFrame->data[_buffer_count], HEX);
          }
#endif
        }

        _buffer_count++;
        if (_buffer_count >= _ptrCurrentFrame->len) {
          _linStateMachine = get_chksm;
        }
        break;

      case  get_chksm:
#if DEBUG_MODE_SLAVE >= 2
        if (_ptrDbgSerial) {
          _ptrDbgSerial->print("[x]C:");
          _ptrDbgSerial->println(c, HEX);
        }
#endif

        if (_ptrCurrentFrame) {
          _ptrCurrentFrame->exp_cksm = c;
          _ptrCurrentFrame->cksm = JRLINHelper::dataChecksum(_ptrCurrentFrame->data, _ptrCurrentFrame->len, _ptrCurrentFrame->calc_addr);

          if (_ptrCurrentFrame->exp_cksm != _ptrCurrentFrame->cksm) {
            _linStateMachine = empty_and_terminate_wrongchecksum;
            break;
          }
        }
        _linStateMachine = empty_and_terminate;
        break;

      default:
        break;
    }  /* End: LIN State Machine */

    /**
        LIN Error State Machine
    */
    switch (_linStateMachine) {
      case empty_and_terminate:
        //flush buffers and terminate parsing to recover CPU time
        _terminate();
        return frameValid; //true; //return code
        break;
      case reply_to_id_empty_and_terminate:
        if (_ptrCurrentFrame != NULL) {
          _slave_send(_ptrCurrentFrame->addr, _ptrCurrentFrame->data, _ptrCurrentFrame->len);
        } else {
          //shall never happen!
        }
        //flush buffers and terminate parsing to recover CPU time
        _terminate();
        return frameIDReplied; //true; //return code
        break;

      case empty_and_terminate_wrongchecksum:
        //flush buffers and terminate parsing to recover CPU time
        _terminate();
        return frameWrongChecksum; //true; //return code
        break;

      case empty_and_terminate_timeout:
        _terminate();
#if DEBUG_MODE_SLAVE >= 2
        if (_ptrDbgSerial) _ptrDbgSerial->println("#!T");
#endif
        return frameSlotTimeout; //false; //return code
        break;

      case empty_and_terminate_no_id_match:
        //flush buffers and terminate parsing to recover CPU time
        _terminate();
        return frameNoIdMatch; //true; //return code
        break;

      default:
        return frameProcessing; // no error, fall throug, continue, keep on processing!
        break;
    }
    /* End: LIN Error State Machine*/
  } //while
}
