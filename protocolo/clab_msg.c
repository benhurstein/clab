#include "clab_msg.h"


static void msg_got_byte(uint8_t ch);
static void msg_got_start_byte(void);
static void msg_got_talk_msg(void);
static void msg_got_sync_msg(void);
static void msg_got_empty_msg(void);
static void msg_got_ack_msg(void);
static void msg_got_nack_msg(void);
static void msg_got_reboot_msg(void);

timestamp_t rcv_msg_timestamp;
timestamp_t last_sync_timestamp;
uint8_t rcv_msg_buf[256];
volatile uint8_t rcv_msg_ctr;
// valid data from o to i-1 ; receiving fro i to r; i==o when empty
volatile uint8_t rcv_msg_iix;
uint8_t rcv_msg_oix;
uint8_t rcv_msg_rix;

typedef struct {
  uint8_t snd_ack_to_send;
  uint8_t snd_msg_buf[256];
  // valid data from s to i-i
  //   msg being sent from s to e-1
  //   next byte to send in o (s -> e-1)
  //   unsent data from e to i-1
  // incomplete data (being buffered) from i to p-1
  // s==o when empty; p==s-1 when full
  uint8_t snd_msg_six;
  volatile uint8_t snd_msg_oix;
  uint8_t snd_msg_eix;
  uint8_t snd_msg_iix;
  uint8_t snd_msg_pix;

  bool overflow;
  bool snd_msg_escaped;
  uint8_t snd_msg_crc;
  uint8_t snd_MSG_DATA;
  uint8_t rcv_MSG_DATA;
  timestamp_t last_sent_timestamp;
} snd_buf_t;


#ifdef ARDUINO_ARCH_AVR

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

snd_buf_t snd_buf;

static uint8_t getch(void)
{
  uint8_t ch;
  ch = UDR0;
  return ch;
}

static void putch(uint8_t ch)
{
  while (!(UCSR0A & _BV(UDRE0)));////
  UDR0 = ch;
}

static void enable_tx(void)
{
  UCSR0B |= (_BV(TXEN0) | _BV(UDRIE0)); 
}

static void disable_tx(void)
{
//  while((UCSR0A & _BV(TXC0)) == 0)
//    ; // wait for last byte to leave
  UCSR0B &= ~(_BV(TXEN0) | _BV(UDRIE0));
//  pinMode(1, INPUT);
}

// initialize UART
// this is OK for ATmega328
void msg_begin(void)
{
#define BAUD_CFG (((F_CPU / 4 / BAUDRATE - 1)) / 2)
  UCSR0A = 1 << U2X0; // double speed (if this is changed, change 4 to 8 above)
#if BAUD_CFG > 255
  UBRR0H = (BAUD_CFG >> 8); // higher bits of baud rate reg
#endif
  UBRR0L = BAUD_CFG; // lower bits of baud rate register
  UCSR0B = _BV(RXEN0) | _BV(RXCIE0);   // only RX enabled
  UCSR0C = _BV(UCSZ00) | _BV(UCSZ01); // 8 bits, no parity, 1 stop bit
  pinMode(1, INPUT); // put TX pin in high impedance

  snd_buf.snd_MSG_DATA = MSG_DATA0;
  snd_buf.rcv_MSG_DATA = MSG_DATA0;
}


// interrupt -- a new byte has been received by UART
ISR(USART_RX_vect)
{
  uint8_t ch;
  ch = getch();
  msg_got_byte(ch);
}

// interrupt -- UART send register is empty
ISR(USART_UDRE_vect)
{
  if (snd_buf.snd_ack_to_send && snd_buf.snd_msg_oix == snd_buf.snd_msg_six) {
    putch(snd_buf.snd_ack_to_send);
    snd_buf.snd_ack_to_send = 0;
  } else if (snd_buf.snd_msg_oix != snd_buf.snd_msg_eix) {
    uint8_t ch;
    ch = snd_buf.snd_msg_buf[snd_buf.snd_msg_oix++];
    putch(ch);
    if (ch == MSG_END) {
      // we're at the end of the message
      snd_buf.snd_msg_eix = snd_buf.snd_msg_oix;
      disable_tx();
    }
  } else {
    putch(MSG_END);
    disable_tx();
  }
}



// TREAT low level (internal) incoming messages

static void msg_got_start_byte(void)
{
  rcv_msg_timestamp = micros();
  // master only sends a start byte when we are not transmitting or we took too long to answer
  // -- in any case, we should not be transmitting
  disable_tx();
}


static void msg_got_talk_msg(void)
{
  // master misses me -- let's say something
  enable_tx();
  if (snd_buf.snd_msg_six == snd_buf.snd_msg_eix) {
    // last msg has been ack'd - send a new one
    snd_buf.snd_msg_eix = snd_buf.snd_msg_iix;        
  }
  // start at first byte of msg
  snd_buf.snd_msg_oix = snd_buf.snd_msg_six;
  putch(node_id);
}

static void msg_got_sync_msg(void)
{
  last_sync_timestamp = rcv_msg_timestamp;
}

static void msg_got_empty_msg(void)
{
  // ignore
}

static void msg_got_ack_msg(void)
{
  // forget ack'd message
  snd_buf.snd_msg_six = snd_buf.snd_msg_eix;
}

static void msg_got_nack_msg(void)
{
  snd_buf.snd_msg_oix = snd_buf.snd_msg_six;
}

//#include <avr/wdt.h>

static void reboot(void)
{
  // jump to bootloader (address is fixed for 2k bootloader in 32k memory) 
#define clab_bootflag (*(uint8_t *)0x503)
  clab_bootflag = 'R';
  ((void(*)(void))(0x7800/2))();
  //wdt_enable(WDTO_15MS);
  //for(;;) ;
} 

static void msg_got_reboot_msg(void)
{
  reboot();
}

//uint8_t pisca=LOW;


// process a byte that has been received
static void msg_got_byte(uint8_t ch)
{
  static enum {idle, starting, receiving} rcv_msg_status = idle;

//digitalWrite(13, pisca ^= LOW^HIGH);
//enable_tx();
//putch(ch);
//return;
  if (rcv_msg_status == idle) {
    if (ch == node_id) { // a message for me is starting!
      msg_got_start_byte();
      rcv_msg_status = starting;
      //rcv_msg_buf[rcv_msg_rix++] = ch;
    }
    // not for us -- ignore
  } else if (rcv_msg_status == starting) {
    // we have got the message type -- see if it is urgent
    if (ch == MSG_TALK) {
      msg_got_talk_msg();
      rcv_msg_status = idle;
    } else if (ch == MSG_SYNC) {
      msg_got_sync_msg();
      rcv_msg_status = idle;
    } /*else if (ch == MSG_REBOOT) {
      msg_got_reboot_msg();
      rcv_msg_status = idle;
    } */else {
      // non-urgent message starting -- put it in buffer
      rcv_msg_status = receiving;
      goto receiving;
    }
  } else { // rcv_msg_status == receiving
receiving:
    rcv_msg_buf[rcv_msg_rix++] = ch;
    if (ch == MSG_END) { // end of message
      rcv_msg_ctr++;
      rcv_msg_iix = rcv_msg_rix;
      rcv_msg_status = idle;
    }
  }
}


#else // !ARDUINO_ARCH_AVR

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>

//#define DEBUG_UART_CHARS

#define ACK_TIMEOUT 1

#define TTY "/dev/ttyUSB0"
int serial;

uint8_t rcv_msg_sender;

snd_buf_t snd_bufs[LAST_NODE_ID - FIRST_NODE_ID + 1];
int current_slave;
#define snd_buf snd_bufs[current_slave - FIRST_NODE_ID]

int getchu(void)
{
  uint8_t ch;
  int n;
  n = read(serial, &ch, 1);
  if (n == 1) {
#ifdef DEBUG_UART_CHARS
    fprintf(stderr, "<%02x%c>", ch, ch>=' '&&ch<='~'?ch:'.'); fflush(stderr);
#endif
    return ch;
  }
  else return -1;
}

// BUG: can loop forever
uint8_t getch(void)
{
  while (true) {
    int n = getchu();
    if (n != -1) {
      return n;  
    }
  }
}

void putch(uint8_t ch)
{
  write(serial, &ch, 1);
#ifdef DEBUG_UART_CHARS
  fprintf(stderr, "[%02x%c]", ch, ch>=' '&&ch<='~'?ch:'.'); fflush(stderr);
#endif
}


void msg_begin(void)
{
#define xstr(s) str(s)
#define str(s) #s
  system("stty -F " TTY " " xstr(BAUDRATE) 
         " raw -echo -echoe -echok -echoctl -echoke");
  serial = open(TTY, O_RDWR | O_NONBLOCK);
  if (serial == -1) {
    fprintf(stderr, "Nao consegui abrir %s\n", TTY);
    exit(1);
  }

  for (current_slave = FIRST_NODE_ID;
       current_slave <= LAST_NODE_ID;
       current_slave++) {
    snd_buf.snd_MSG_DATA = MSG_DATA0;
    snd_buf.rcv_MSG_DATA = MSG_DATA0;
  }

  //sleep(3);
}

extern timestamp_t now;
uint8_t who_can_talk = 0;
uint8_t next_to_talk = FIRST_NODE_ID;
timestamp_t last_talk_timestamp;
#define TALK_TIMEOUT 0.02

void verify_uart(void)
{
  do {
    int ch;
    ch = getchu();
    if (ch == -1) {
      break;
    }
    msg_got_byte(ch);
  } while (true/*false*//*true*/);

  if (who_can_talk != 0) {
    if ((now - last_talk_timestamp) > TALK_TIMEOUT) {
      putch(who_can_talk);
      putch(MSG_END);
      who_can_talk = 0;
    }
  }
  if (who_can_talk == 0) {
    next_to_talk++;
    if (next_to_talk > LAST_NODE_ID) {
      next_to_talk = FIRST_NODE_ID;
#ifdef DEBUG_UART_CHARS
fprintf(stderr, "\n%.1f ", now);
#endif
    }
    who_can_talk = next_to_talk;
    putch(who_can_talk);
    putch(MSG_TALK);
    last_talk_timestamp = now;
  }

  for (current_slave = FIRST_NODE_ID;
       current_slave <= LAST_NODE_ID;
       current_slave++) {

    // do not bother the talking slave
    if (current_slave == who_can_talk) continue;

    if (snd_buf.snd_msg_six == snd_buf.snd_msg_eix) {
      // last msg has been ack'd - send a new one
      snd_buf.snd_msg_eix = snd_buf.snd_msg_iix;
    } else {
      if ((now - snd_buf.last_sent_timestamp) > ACK_TIMEOUT) {
        // resend message - start at first byte of msg
        snd_buf.snd_msg_oix = snd_buf.snd_msg_six;
      }
    }

    if ((snd_buf.snd_ack_to_send != 0) 
        || (snd_buf.snd_msg_oix != snd_buf.snd_msg_eix)) {
      putch(current_slave);
      if (snd_buf.snd_ack_to_send) {
        putch(snd_buf.snd_ack_to_send);
        snd_buf.snd_ack_to_send = 0;
      }
      int bytes_sent = 0;
      while (snd_buf.snd_msg_oix != snd_buf.snd_msg_eix) {
        uint8_t ch;
        ch = snd_buf.snd_msg_buf[snd_buf.snd_msg_oix++];
        putch(ch);
        bytes_sent++;
        if (ch == MSG_END) {
          // we're at the end of the message
          snd_buf.snd_msg_eix = snd_buf.snd_msg_oix;
	}
      }
      if (bytes_sent == 0) {
        putch(MSG_END);
      }
      snd_buf.last_sent_timestamp = now;
    }
  }
}



// TREAT low level (internal) incoming messages

static void msg_got_start_byte(void)
{
  rcv_msg_timestamp = now;
  // TODO - cancel transmission if transmitting
}


static void msg_got_talk_msg(void)
{
  // master should not receive this message -- ignore
}

static void msg_got_sync_msg(void)
{
  // master should not receive this message -- ignore
}

static void msg_got_empty_msg(void)
{
  // ignore
}

static void msg_got_ack_msg(void)
{
  // forget ack'd message
  snd_bufs[rcv_msg_sender - FIRST_NODE_ID].snd_msg_six = snd_bufs[rcv_msg_sender - FIRST_NODE_ID].snd_msg_eix;
}

static void msg_got_nack_msg(void)
{
  snd_bufs[rcv_msg_sender - FIRST_NODE_ID].snd_msg_oix = snd_bufs[rcv_msg_sender - FIRST_NODE_ID].snd_msg_six;
}

static void msg_got_reboot_msg(void)
{
  // master should not receive this message -- ignore
}


// process a byte that has been received
static void msg_got_byte(uint8_t ch)
{
  static enum {idle, starting, receiving} rcv_msg_status = idle;

  if (rcv_msg_status == idle) {
    if (ch >= FIRST_NODE_ID && ch <= LAST_NODE_ID) { // a message for me is starting!
      // TODO testar who_can_talk
      rcv_msg_sender = ch;
      msg_got_start_byte();
      rcv_msg_status = starting;
      rcv_msg_buf[rcv_msg_rix++] = ch;
    }
    // error - stray byte received -- ignore
  } else if (rcv_msg_status == starting) {
    // we have got the message type -- see if it is urgent
    if (ch == MSG_TALK) {
      msg_got_talk_msg();
      rcv_msg_status = idle;
    } else if (ch == MSG_SYNC) {
      msg_got_sync_msg();
      rcv_msg_status = idle;
    } else {
      // non-urgent message starting -- put it in buffer
      rcv_msg_status = receiving;
      goto receiving;
    }
  } else { // rcv_msg_status == receiving
receiving:
    rcv_msg_buf[rcv_msg_rix++] = ch;
    if (ch == MSG_END) { // end of message
      rcv_msg_ctr++;
      rcv_msg_iix = rcv_msg_rix;
      rcv_msg_status = idle;
      who_can_talk = 0;
    }
  }
}


#endif // !ARDUINO_ARCH_AVR



// ENCODING / DECODING data in messages

// The start of a message is identified by a single byte, corresponding to the recipient's id.
// If one such value is to be transmitted inside a message, it must be escaped.
// These are helper functions to do the escaping and unescaping process.

// return true if ch shouldn't be inside a message
static bool invalid_char(uint8_t ch)
{
  return ch >= 0xf0;
}

// return true if b cannot be sent in unescaped mode
static bool invalid_unescaped_byte(uint8_t b)
{
  return b >= 0xef;
}

// return true if b cannot be sent in escaped mode
static bool invalid_escaped_byte(uint8_t b)
{
  return b <= 0x8f;
}

// return the char needed to send byte b in escaped mode
// The flag is unset. If it is not the last escaped char, the flag must be set.
static uint8_t get_escaped_char(uint8_t b)
{
  return (b - 16) & 0x7f;
}

// b is an escaped byte. return true if next byte is escaped too, false if b is the last one.
static bool get_escaped_flag(uint8_t ch)
{
  return ch >= 0x80;
}

// return the byte corresponding to the escaped char ch
static uint8_t get_escaped_byte(uint8_t ch)
{
  return (ch + 16) | 0x80;
}


// SENDING

uint8_t snd_free_bytes(void)
{
  return snd_buf.snd_msg_six - snd_buf.snd_msg_pix - 1;
}

// start the construction of a DATA msg to send
bool msg_start(void)
{
  snd_buf.snd_msg_pix = snd_buf.snd_msg_iix;
  if (snd_free_bytes() < 4) {
    snd_buf.overflow = true;
    return false;
  }
  snd_buf.snd_msg_escaped = false;
  snd_buf.snd_msg_buf[snd_buf.snd_msg_pix++] = snd_buf.snd_MSG_DATA;
  snd_buf.snd_msg_crc = crc8(0, snd_buf.snd_MSG_DATA);
  snd_buf.overflow = false;
  return true;
}

#ifdef MASTER
bool msg_start_to_node(uint8_t node)
{
  current_slave = node;
  return msg_start();
}
#endif

// append one byte to DATA msg being constructed
void msg_putbyte(uint8_t b)
{
  if (snd_buf.overflow) {
    return;
  }
  if (snd_free_bytes() < 3) {
    snd_buf.overflow = true;
    return;
  }
  snd_buf.snd_msg_crc = crc8(snd_buf.snd_msg_crc, b);
  if (!snd_buf.snd_msg_escaped) {
    if (invalid_unescaped_byte(b)) {
      snd_buf.snd_msg_buf[snd_buf.snd_msg_pix] = ESCAPE_CHAR;
      snd_buf.snd_msg_escaped = true;
      snd_buf.snd_msg_buf[++snd_buf.snd_msg_pix] = get_escaped_char(b);
    } else {
      snd_buf.snd_msg_buf[snd_buf.snd_msg_pix++] = b;
    }
  } else {
    if (invalid_escaped_byte(b)) {
      snd_buf.snd_msg_pix++;
      snd_buf.snd_msg_escaped = false;
      snd_buf.snd_msg_buf[snd_buf.snd_msg_pix++] = b;
    } else {
      snd_buf.snd_msg_buf[snd_buf.snd_msg_pix] |= 0x80;
      snd_buf.snd_msg_buf[++snd_buf.snd_msg_pix] = get_escaped_char(b);
    }
  }
}

/*
// append one word to DATA msg being constructed
void msg_putword(uint16_t w)
{
  msg_putbyte(w);
  msg_putbyte(w>>8);
}
*/

// end message construction, put in sending queue
bool msg_end(void)
{
  msg_putbyte(snd_buf.snd_msg_crc);
  if (snd_buf.snd_msg_escaped) {
    snd_buf.snd_msg_pix++;
    snd_buf.snd_msg_escaped = false;
  }
  if (snd_buf.overflow) {
    return false;
  }
  snd_buf.snd_msg_buf[snd_buf.snd_msg_pix++] = MSG_END;
  snd_buf.snd_msg_iix = snd_buf.snd_msg_pix;
  // toggle DATA type for next message
  snd_buf.snd_MSG_DATA ^= (MSG_DATA0 ^ MSG_DATA1);
  return true;
}

#ifdef MASTER
// send a REBOOT message to node
void msg_send_reboot(uint8_t node)
{
/*
  current_slave = node;
  snd_buf.snd_msg_buf[snd_buf.snd_msg_pix++] = MSG_REBOOT;
  snd_buf.snd_msg_buf[snd_buf.snd_msg_pix++] = MSG_END;
  snd_buf.snd_msg_iix = snd_buf.snd_msg_pix;
*/
  putch(node); putch(MSG_REBOOT); putch(MSG_END);
  // reset DATA message type for this node
  snd_buf.snd_MSG_DATA = MSG_DATA0;
}
#endif

// send a DATA message with byte b1 as payload
bool msg_send_b(uint8_t b1)
{
  if (!msg_start()) return false;
  msg_putbyte(b1);
  return msg_end();
}

// send a DATA message with bytes b1, b2, b3 as payload
bool msg_send_bbb(uint8_t b1, uint8_t b2, uint8_t b3)
{
  if (!msg_start()) return false;
  msg_putbyte(b1);
  msg_putbyte(b2);
  msg_putbyte(b3);
  return msg_end();
}

bool msg_send_bbbw(uint8_t b1, uint8_t b2, uint8_t b3, uint16_t w1)
{
  if (!msg_start()) return false;
  msg_putbyte(b1);
  msg_putbyte(b2);
  msg_putbyte(b3);
  msg_putword(w1);
  return msg_end();
}

// send a DATA message with bytes b1, b2 and word w1 as payload
bool msg_send_bbw(uint8_t b1, uint8_t b2, uint16_t w1)
{
  if (!msg_start()) return false;
  msg_putbyte(b1);
  msg_putbyte(b2);
  msg_putword(w1);
  return msg_end();
}

// send a DATA message with word w1 as payload
bool msg_send_w(uint16_t w1)
{
  if (!msg_start()) return false;
  msg_putword(w1);
  return msg_end();
}

// send a DATA message with words w1 and w2 as payload
bool msg_send_ww(uint16_t w1, uint16_t w2)
{
  if (!msg_start()) return false;
  msg_putword(w1);
  msg_putword(w2);
  return msg_end();
}


// send an ACK message
static void msg_send_ack(void)
{
  // ACK's are inserted before other messages
  snd_buf.snd_ack_to_send = MSG_ACK;
}

// send a NACK message
static void msg_send_nack(void)
{
  snd_buf.snd_ack_to_send = MSG_NACK;
}



// RECEIVING

bool rcv_msg_escaped = false;
uint8_t rcv_msg_crc = 0;
bool rcv_msg_ended = false;

// get one byte from rcv buffer, unescaping if necessary
// return 0 if message ended
// end message if at end of buffer or if MSG_END byte found
static uint8_t getbyte(void)
{
  uint8_t ch;
  if (rcv_msg_ended) return 0;
  if (rcv_msg_oix == rcv_msg_iix) {
    rcv_msg_ended = true;
    return 0;
  }
  ch = rcv_msg_buf[rcv_msg_oix++];
  if (ch == MSG_END) {
    rcv_msg_ended = true;
    return ch;
  }
  if (invalid_char(ch)) {
    // protocol error -- abort
    return 0;
  }
  if (rcv_msg_escaped) {
    rcv_msg_escaped = get_escaped_flag(ch);
    ch = get_escaped_byte(ch);
  } else {
    if (ch == ESCAPE_CHAR) {
      rcv_msg_escaped = true;
      return getbyte();
    }
  }
  rcv_msg_crc = crc8(rcv_msg_crc, ch);
  return ch;
}

// retrieve a DATA message from receiving buffer into m
// first message byte is already in m->type
// send ACK and return true if successfull
// return false if not successfull (repeated message, incorrect CRC)
static bool msg_get_data_msg(msg *m)
{
  uint8_t crc = crc8(0, m->type);
  uint8_t ix = 0;
  while (true) {
    uint8_t b = getbyte();
    if (rcv_msg_ended) {
      break;
    }
    if (ix < MSG_MAX_LEN) {
      crc = crc8(crc, b);
      m->data[ix++] = b;
    } else {
      crc = 1; // invalid crc
    }
  }
#ifdef DEBUG_UART_CHARS
fprintf(stderr, "(C%02x,%02x)", crc, ix);
#endif

  if (crc != 0) {
//msg_start();int i; for(i=0; i<ix;i++){msg_putbyte(0x20|(m->data[i])>>4);msg_putbyte(0x30|(m->data[i])&15);} msg_putbyte(crc); msg_end(); msg_send_ack();
    msg_send_nack();
  } else {
    msg_send_ack();
    if (m->type == snd_buf.rcv_MSG_DATA) {
      // message is OK -- toggle DATA type for next message
      snd_buf.rcv_MSG_DATA ^= (MSG_DATA0 ^ MSG_DATA1);
      m->ix = 0;
#ifdef DEBUG_UART_CHARS
fprintf(stderr, "(OK%02x)", current_slave);
#endif
      return true;
    }
  }
#ifdef DEBUG_UART_CHARS
fprintf(stderr, "(NOK%02x %02x)", current_slave, snd_buf.rcv_MSG_DATA);
#endif
  return false;
}

// retrieve a DATA message from receiving buffer into m
// treat any internal message found in buffer
// return true if succesfull (message is in m)
// return false if not (m may have rubbish)
bool msg_get_msg(msg *m)
{
  if (rcv_msg_iix == rcv_msg_oix) return false;
#ifdef MASTER
  if (rcv_msg_buf[rcv_msg_oix] >= FIRST_NODE_ID 
      && rcv_msg_buf[rcv_msg_oix] <= LAST_NODE_ID) {
    rcv_msg_sender = rcv_msg_buf[rcv_msg_oix++];
  }
  m->timestamp = rcv_msg_timestamp;
  m->sender = rcv_msg_sender;
  current_slave = m->sender;
#endif
  rcv_msg_escaped = false;
  rcv_msg_crc = 0;
  rcv_msg_ended = false;
  m->type = getbyte();
#ifdef DEBUG_UART_CHARS
fprintf(stderr, "(t%02x<-%02x)",m->type, m->sender);
#endif
  switch(m->type) {
/*
// messages from master to slave - urgent
#define MSG_TALK                'T'   // master allowing slave to send
#define MSG_SYNC                'Y'   // broadcast to sync time
#define MSG_REBOOT              'R'   // reboot slave -- will run bootloader
// messages from slave to master - urgent
#define MSG_EMPTY               'E'   // slave has nothing to say
// messages for data exchange
#define MSG_DATA0               'D'   // data block (D bytes CRC END)
#define MSG_DATA1               '1'   // data block (1 bytes CRC END)
#define MSG_ACK                 'A'   // ACK to DATA1
#define MSG_NACK                'N'   // NACK to DATA1
#define MSG_END                 '\xf0' // end of DATA block
*/
    case MSG_END: // empty message (i'm alive) -- should not happen
      m->type = MSG_EMPTY;
      // fall thru
    case MSG_EMPTY:
      msg_got_empty_msg();
      break;
    case MSG_ACK:
      msg_got_ack_msg();
      break;
    case MSG_NACK:
      msg_got_nack_msg();
      break;
    case MSG_REBOOT:
      msg_got_reboot_msg();
      break;
    case MSG_DATA0:
    case MSG_DATA1:
      return msg_get_data_msg(m);
      break;
    default:
      // unknown message type -- ignore
      while (!rcv_msg_ended) getbyte();
  }
  return false;
}

// return next byte from message m
uint8_t msg_getbyte(msg *m)
{
  return m->data[m->ix++];
}

// return next word from message m
uint16_t msg_getword(msg *m)
{
  uint8_t b;
  b = msg_getbyte(m);
  return b | ((uint16_t)msg_getbyte(m) << 8);
}



// CRC Code adapted from OneWire library

// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"

// This table comes from Dallas sample code where it is freely reusable,
// though Copyright (C) 2000 Dallas Semiconductor Corporation
#ifdef ARDUINO_ARCH_AVR
#include <avr/pgmspace.h>
static const uint8_t dscrc_table[] PROGMEM = {
#else
static const uint8_t dscrc_table[] = {
#endif
    0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
  157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
   35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
  190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
   70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
  219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
  101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
  248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
  140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
   17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
  175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
   50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
  202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
   87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
  233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
  116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
};

uint8_t crc8(uint8_t crc, uint8_t val)
{
#ifdef ARDUINO_ARCH_AVR
  return pgm_read_byte(dscrc_table + (crc ^ val));
#else
  if (current_slave != 0xfe) // 0xfe is the bootloader, it uses the old CRC
    return dscrc_table[crc ^ val];
  else
    return crc ^ val;
#endif
}

uint8_t crc8_blk(const uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;

  while (len--) {
    crc = crc8(crc, *addr++);
  }
  return crc;
}

