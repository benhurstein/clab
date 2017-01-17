#ifndef _CLAB_MSG_H_
#define _CLAB_MSG_H_

#include <stdint.h>
#include <stdbool.h>

extern const uint8_t node_id;

#ifdef __cplusplus
extern "C" {
#endif

//#define NODE_ID                 0xf2 // who am I?

#ifdef ARDUINO_ARCH_AVR
#define SLAVE
#else
#define MASTER
#endif

#define BAUDRATE 115200

#define NODE_BROADCAST          0xff // node id for sending to all nodes
#define NODE_FIRMWARE           0xfe // node id for firmware writing
#define LAST_NODE_ID            0xfe //0xfe
#define FIRST_NODE_ID           0xf1
#define MSG_END                 0xf0 // end of DATA block - cannot be used as node id
#define ESCAPE_CHAR             0xef

// messages from master to slave - urgent
#define MSG_TALK                'T'   // master allowing slave to send
#define MSG_SYNC                'Y'   // broadcast to sync time
#define MSG_REBOOT              'R'   // reboot slave -- will run bootloader
// messages from slave to master - urgent
#define MSG_EMPTY               'E'   // slave has nothing to say
// messages for data exchange
#define MSG_DATA0               'D'   // even data block (D bytes CRC END)
#define MSG_DATA1               '1'   // odd data block (1 bytes CRC END)
#define MSG_ACK                 'A'   // ACK to DATA
#define MSG_NACK                'N'   // NACK to DATA


#ifdef SLAVE
typedef unsigned long timestamp_t;
#else
typedef double timestamp_t;
#endif

#ifdef MASTER
#define MSG_MAX_LEN 256
#else
#define MSG_MAX_LEN 64
#endif
typedef struct {
#ifdef MASTER
  uint8_t sender;
  timestamp_t timestamp;
#endif
  union {
    uint8_t type;
    uint8_t ix;
  };
  uint8_t data[MSG_MAX_LEN];
} msg;

uint8_t msg_getbyte(msg *m);
uint16_t msg_getword(msg *m);

void msg_begin(void);

bool msg_get_msg(msg *m);

bool msg_start(void);
#ifdef MASTER
bool msg_start_to_node(uint8_t node);
#endif
void msg_putbyte(uint8_t b);
#define msg_putword(w) do {msg_putbyte(w); msg_putbyte((w)>>8);} while(0)
//void msg_putword(uint16_t w);
bool msg_end(void);

#ifdef MASTER
void msg_send_reboot(uint8_t node);
#endif

bool msg_send_b(uint8_t b1);
bool msg_send_bbb(uint8_t b1, uint8_t b2, uint8_t b3);
bool msg_send_bbbw(uint8_t b1, uint8_t b2, uint8_t b3, uint16_t w1);
bool msg_send_bbw(uint8_t b1, uint8_t b2, uint16_t w1);
bool msg_send_w(uint16_t w1);
bool msg_send_ww(uint16_t w1, uint16_t w2);

// Dallas crc8
uint8_t crc8(uint8_t crc, uint8_t val);
uint8_t crc8_blk(const uint8_t *addr, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif
