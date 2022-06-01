
#include "JRLinFrameBuffer.h"
#include "JRLinHelper.h"

/***
    JRLin Filter Class

*/
JRLINFrameBuffer::JRLINFrameBuffer(): addr(-1), len(-1), updated(0), timestamp(0)
{
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
}

void JRLINFrameBuffer::init( uint8_t buf_addr, JRLINFrameBufferType buf_frametype, uint8_t buf_len,uint8_t myslot) {
  if (buf_len > 8) buf_len = 8;

  addr = buf_addr;
  calc_addr = (addr & 0x3f) | JRLINHelper::addrParity(addr); 
  frametype = buf_frametype;
  len = buf_len;
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  updated = 0;
  timestamp = 0;
  slot = myslot;
}

void JRLINFrameBuffer::debug_buffer_print(AltSoftSerial *_dbgSerial, JRLINFrameBuffer *buf) {
  if (!_dbgSerial) return;

  _dbgSerial->println("<= Filter Dump =>");
  _dbgSerial->print("Addr:0x"); _dbgSerial->println(buf->addr, HEX);
  _dbgSerial->print("CalcAddr:0x"); _dbgSerial->println(buf->calc_addr, HEX);
  _dbgSerial->print("Len:0x");
  _dbgSerial->println(buf->len, HEX);
  _dbgSerial->print("D0:0x");
  _dbgSerial->println(buf->data[0], HEX);
  _dbgSerial->print("D1:0x");
  _dbgSerial->println(buf->data[1], HEX);
  _dbgSerial->print("D2:0x");
  _dbgSerial->println(buf->data[2], HEX);
  _dbgSerial->print("D3:0x");
  _dbgSerial->println(buf->data[3], HEX);
  _dbgSerial->print("D4:0x");
  _dbgSerial->println(buf->data[4], HEX);
  _dbgSerial->print("D5:0x");
  _dbgSerial->println(buf->data[5], HEX);
  _dbgSerial->print("D6:0x");
  _dbgSerial->println(buf->data[6], HEX);
  _dbgSerial->print("D7:0x");
  _dbgSerial->println(buf->data[7], HEX);
  _dbgSerial->print("CKSM:0x");
  _dbgSerial->println(buf->cksm, HEX);
  _dbgSerial->print("ECKSM:0x");
  _dbgSerial->println(buf->exp_cksm, HEX);
  _dbgSerial->print("U:");  
  _dbgSerial->println(buf->updated);
  _dbgSerial->print("T:");
  _dbgSerial->println(buf->timestamp);

}
