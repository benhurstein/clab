#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <limits.h>
#include <errno.h>

#define data_dirname "/home/benhur/clab/SENSOR_DATA"

uint8_t getch(void);
void putch(uint8_t ch);

typedef double timestamp_t;
timestamp_t now, t0=-1;

void set_now(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  now = ts.tv_sec + ts.tv_nsec/1e9;
  if (t0==-1) t0=now;
  //now -= t0;
}

typedef struct {
  int min;
  int max;
  int cur_address;
  bool complete;
  uint8_t d[64*1024];
} buffer;

int octet(char *line, int i)
{
  char d[3] = "00";
  d[0] = line[i];
  d[1] = line[i+1];
  if (!isxdigit(d[0]) || !isxdigit(d[1])) return -1;
  int v;
  sscanf(d, "%x", &v);
  return v;
}

uint8_t checksum(char *line)
{
  int i;
  uint8_t s = 0;

  for (i=1; ; i+=2) {
    int v = octet(line, i);
    if (v < 0) return s;
    s += v;
  }
  return 0;
}

bool lelinha(FILE *f, buffer *buf)
{
  int aux;
  int size;
  int addr;
  int type;

  char line[1+2+4+2+255*2+2+2+1];
  if(fgets(line, sizeof(line), f) != line)
    return false;
  if (line[0] != ':') return false;
  if (checksum(line) != 0) return false;

  size = octet(line, 1);
  if (size < 0) return false;
  aux = octet(line, 3);
  if (aux < 0) return false;
  addr = octet(line, 5);
  if (addr < 0) return false;
  addr |= aux << 8;
  type = octet(line, 7);
  if (type < 0) return false;
  // only data and eof types supported
  if (type != 0 && type != 1) return false;
  if (type == 1) { // eof
    buf->complete = true;
    return false;
  }
  for (int i = 0; i < size; i++) {
    int b;
    b = octet(line, 9 + i * 2);
    if (b < 0) return false;
    buf->d[addr + i] = b;
  }
  if (addr < buf->min) buf->min = addr;
  if (addr+size > buf->max) buf->max = addr+size;
  return true;
}

bool lehex(char *nome, buffer *buf)
{
  FILE *f;

  f = fopen(nome, "r");
  if (f == NULL) return false;

  buf->min = sizeof(buf->d);
  buf->max = 0;
  buf->complete = false;

  while(lelinha(f, buf))
    ;

  fclose(f);
  return buf->complete;
}


#include "clab_msg.h"

void verify_uart(void);

buffer buf;

static void send_block(buffer *buf, int address, int size)
{
  msg_start_to_node(NODE_FIRMWARE);
  msg_putword(address);
  msg_putword(size);
  for(int i = 0; i < size; i++) {
    msg_putbyte(buf->d[address+i]);
  }
  msg_end();
}

bool send_next_firmware_block(void)
{
    int size = 128;
    if (buf.cur_address+size > buf.max) {
//      size = buf.max - address;
    }
    send_block(&buf, buf.cur_address, size);
    buf.cur_address += size;
    if (buf.cur_address < buf.max) {
      return false;
    } else {
      return true;
    }
}

#include <stdarg.h>

void log_error(char *format, ...)
{
  va_list ap;
  va_start(ap, format);
  fprintf(stderr, "ERROR: ");
  vfprintf(stderr, format, ap);
  fprintf(stderr, "\n");
  va_end(ap);
}


typedef timestamp_t deltatime_t;

typedef struct {
  uint8_t node_id;
  uint8_t sensor_id;
  char *name;
  char *topic;
  uint8_t type;
  float scale;
  float offset;
  deltatime_t store_period;
  int filedaynumber; // timestamp in days for the open file
  int fd;
} sensor_t;

sensor_t *sensors;
int nsensors;

bool sensor_from_str(char *str, sensor_t *s)
{
  char c = 0;
  if (sscanf(str, " %c", &c) != 1 || c == '#') {
    return false;
  }
  int n;
  n = sscanf(str, "%hhd %hhd %ms %ms %c %f %f %lf",
                  &s->node_id,
                  &s->sensor_id,
                  &s->name,
                  &s->topic,
                  &s->type,
                  &s->scale,
                  &s->offset,
                  &s->store_period);
  if (n == 8) {
    s->node_id += 0xf1;
    //s->store_period = (24 * 3600) / s->store_period;
    s->filedaynumber = 0;
    s->fd = -1;
    return true;
  }
  if (n > 3) free(s->topic);
  if (n > 2) free(s->name);
  log_error("ignored sensors.cfg line [%d] \"%s\"", n, str);
  return false;
}

void read_sensors_config(void)
{
  FILE *f;
  char fname[PATH_MAX];

  sprintf(fname, "%s/sensors.cfg", data_dirname);
  f = fopen(fname, "r");
  if (f == NULL) {
    //TODO log error
    log_error("sensors.cfg: %s", strerror(errno));
    return;
  }
  while (!feof(f)) {
    char line[1000];
    sensor_t sensor;
    if (fgets(line, 1000, f) != NULL) {
      if (sensor_from_str(line, &sensor)) {
        sensors = realloc(sensors, (nsensors+1) * sizeof(sensor_t));
        sensors[nsensors++] = sensor;
      }
    }
  }
  fclose(f);
}

sensor_t *sensor_from_id(uint8_t node_id, uint8_t sensor_id)
{
  int s;
  for (s = 0; s < nsensors; s++) {
    if (sensors[s].node_id == node_id && sensors[s].sensor_id == sensor_id) {
      return &sensors[s];
    }
  }
  return NULL;
}

float cook(int16_t raw, uint8_t type, float scale, float offset)
{
  float cooked;
  switch (type) {
    case 'T': // DS18B20 temperature sensor
      cooked = raw / 16.0f * scale + offset;
      break;
    case 't': // AM2320 or AM2301, temperature
      cooked = raw / 10.0f * scale + offset;
      break;
    case 'H': // AM2320 or AM2301, humidity
      cooked = raw / 10.0f * scale + offset;
      break;
    case 'F': // water flow sensor, raw = ticks/s
      cooked = raw * scale + offset;
      break;
    default:  // unknown sensor type
      //TODO log error
      cooked = raw * scale + offset;
      break;
  }
  return cooked;
}

#include <mosquitto.h>

struct mosquitto *mosquiton;

void command_received(char *topic, char *payload);

void mosquitto_message_callback(struct mosquitto *mosq, void *userdata,
                                const struct mosquitto_message *message)
{
  if (message->payloadlen > 0) {
    command_received(message->topic, message->payload);
  }
}

void mqtt_start(void)
{
  //TODO -- verify errors
  mosquitto_lib_init();
  mosquiton = mosquitto_new(NULL, true, NULL);
  mosquitto_connect(mosquiton, "localhost", 1883, 10);
  mosquitto_message_callback_set(mosquiton, mosquitto_message_callback);
  mosquitto_subscribe(mosquiton, NULL, "/clab/actuator/#", 0);
}

void mqtt_stop(void)
{
  mosquitto_disconnect(mosquiton);
  mosquitto_destroy(mosquiton);
  mosquitto_lib_cleanup();
}

void mqtt_loop(void)
{
  mosquitto_loop(mosquiton, 0, 1);
}

void publish(char *topic, float data)
{
  //TODO
  char val[30];
  sprintf(val, "%.2f", data);
  mosquitto_publish(mosquiton, NULL, topic, strlen(val), val, 0, true);
}

void sensor_publish_data(sensor_t *sensor, uint16_t raw_data)
{
  float cooked_data;
  cooked_data = cook(raw_data, sensor->type, sensor->scale, sensor->offset);
  publish(sensor->topic, cooked_data);
}



int openfile(timestamp_t timestamp, char *sensorname)
{
  struct tm *t;
  time_t seconds;
  char name[PATH_MAX];
  char cmd[PATH_MAX+9];

  seconds = timestamp;
  t = gmtime(&seconds);
  sprintf(name, "%s/%d/%02d/%02d", 
                 data_dirname, t->tm_year+1900, t->tm_mon+1, t->tm_mday);
  sprintf(cmd, "mkdir -p %s", name);
  system(cmd);

  int fd;
  strcat(name, "/");
  strcat(name, sensorname);
  strcat(name, ".raw");
  fd = open(name, O_RDWR|O_CREAT, 0777);
  if (fd < 0) {
    log_error("opening \"%s\": %s", name, strerror(errno));
  }
  return fd;
}

void sensor_setfilepos(sensor_t *sensor, timestamp_t timestamp)
{
  int days = timestamp / (24 * 3600);
  if (days != sensor->filedaynumber) {
    if (sensor->fd != -1) {
      close(sensor->fd);
    }
    sensor->fd = openfile(timestamp, sensor->name);
    sensor->filedaynumber = days;
  }
  int slot = (timestamp - days * (24 * 3600)) / sensor->store_period;
  off_t pos = slot * 2;
//fprintf(stderr, "ts=%f days=%d fd=%d slot=%d p=%f\n", timestamp, days, sensor->fd, slot, sensor->store_period);
  lseek(sensor->fd, pos, SEEK_SET);
}

void write_word(int fd, uint16_t w)
{
  if (fd != -1) {
    uint8_t b[2];
    b[0] = w;
    b[1] = w>>8;
    write(fd, b, 2);
  }
}

void sensor_write_data(sensor_t *sensor,
                      uint16_t raw_data, timestamp_t timestamp)
{
  sensor_setfilepos(sensor, timestamp);
  if (raw_data == 0) raw_data = 0x8000;
  write_word(sensor->fd, raw_data);
}

void received_sensor_data(timestamp_t timestamp, 
                          uint8_t sender, uint8_t sensor_id, int16_t data)
{
  sensor_t *sensor;

  sensor = sensor_from_id(sender, sensor_id);
  if (sensor == NULL) {
    log_error("Got data from unknown sensor %02x/%02x", sender, sensor_id);
    return;
  }
  sensor_publish_data(sensor, data);
  sensor_write_data(sensor, data, timestamp);  
}

void received_message(msg m)
{
  if (m.sender == NODE_FIRMWARE) {
    static bool fw_ended = false;
    uint8_t type = msg_getbyte(&m);
    if (!buf.complete) {
      fprintf(stderr, "\nFW REBOOT\n");
      msg_send_reboot(NODE_FIRMWARE);
    } else if (type == MSG_ACK) {
      fprintf(stderr, "\nFW ACK: %04x\n", msg_getword(&m));
      if (fw_ended) {
        // reboot
        msg_send_reboot(NODE_FIRMWARE);
      } else {
        fw_ended = send_next_firmware_block();
      }
    } else if (type == MSG_NACK) {
      fprintf(stderr, "\nFW_NACK: ");
      fprintf(stderr, "%02x ", msg_getbyte(&m));
      fprintf(stderr, "%04x ", msg_getword(&m));
      fprintf(stderr, "%04x\n", msg_getword(&m));
    } else if (type == 'I') {
      fprintf(stderr, "\nFW_ID: ");
      fprintf(stderr, "%02x ", msg_getbyte(&m));
      fprintf(stderr, "%04x ", msg_getword(&m));
      fprintf(stderr, "%04x\n", msg_getword(&m));
      fw_ended = send_next_firmware_block();
    } else {
      fprintf(stderr, "\nFW_INV: %02x\n", type);
    }
  } else {
    uint8_t type = msg_getbyte(&m);
    if (type == '2') {
      int nsensors = msg_getbyte(&m);
      int i;
      for (i=0; i<nsensors; i++) {
        uint8_t sensor;
        int16_t data;
        sensor = msg_getbyte(&m);
        data = msg_getword(&m);
        received_sensor_data(m.timestamp, m.sender, sensor, data);
      }
    } else if (type == 'i') { // sensor id
      fprintf(stderr, "\n{sensor id %d=", msg_getbyte(&m));
      for (int i=0; i<7; i++) {
        fprintf(stderr, "%02x ", msg_getbyte(&m));
      }
      fprintf(stderr, "%02x}\n", msg_getbyte(&m));
    }
  }
}


typedef struct {
  uint8_t node_id;
  uint8_t actuator_id;
  char *name;
  char *topic;
  uint8_t type;
} actuator_t;

actuator_t *actuators;
int nactuators;

bool actuator_from_str(char *str, actuator_t *a)
{
  char c = 0;
  if (sscanf(str, " %c", &c) != 1 || c == '#') {
    return false;
  }
  int n;
  n = sscanf(str, "%hhd %hhd %ms %ms %c",
                  &a->node_id,
                  &a->actuator_id,
                  &a->name,
                  &a->topic,
                  &a->type);
  if (n == 5) {
    a->node_id += 0xf1;
    return true;
  }
  if (n > 3) free(a->topic);
  if (n > 2) free(a->name);
  log_error("ignored actuators.cfg line \"%s\"", str);
  return false;
}

void read_actuators_config(void)
{
  FILE *f;
  char fname[PATH_MAX];

  sprintf(fname, "%s/actuators.cfg", data_dirname);
  f = fopen(fname, "r");
  if (f == NULL) {
    //TODO log error
    log_error("actuators.cfg: %s", strerror(errno));
    return;
  }
  while (!feof(f)) {
    char line[1000];
    actuator_t actuator;
    if (fgets(line, 1000, f) != NULL) {
      if (actuator_from_str(line, &actuator)) {
        actuators = realloc(actuators, (nactuators+1) * sizeof(actuator_t));
        actuators[nactuators++] = actuator;
      }
    }
  }
  fclose(f);
}

actuator_t *actuator_from_topic(char *topic)
{
  int i;
  for (i = 0; i < nactuators; i++) {
    if (strcmp(topic, actuators[i].topic) == 0) {
      return &actuators[i];
    }
  }
  return NULL;
}

void actuator_execute_command(char *command)
{
  char cmd[100];
  uint8_t node_id;
  char filename[PATH_MAX];

  if (sscanf(command, "%s %hhx %s", cmd, &node_id, filename) == 3) {
    if (strcmp(cmd, "WRITE_FIRMWARE") == 0) {
      if (node_id < FIRST_NODE_ID || node_id > LAST_NODE_ID) {
        log_error("Invalid node id in command [%s]", command);
      } else {
        lehex(filename, &buf);
        if (!buf.complete) {
          log_error("Invalid hex file in command [%s]", command);
        } else {
          fprintf(stderr, "HEX: %x->%x\n", buf.min, buf.max);
          buf.cur_address = buf.min;
          msg_send_reboot(node_id);
        }
      }
    } else {
      log_error("Unrecognized command: [%s]", command);
    }
  } else if (sscanf(command, "%s %hhx", cmd, &node_id) == 2) {
    if (strcmp(cmd, "RESET") == 0) {
      if (node_id < FIRST_NODE_ID || node_id > LAST_NODE_ID) {
        log_error("Invalid node id in command [%s]", command);
      } else {
        fprintf(stderr, "RESET: %x\n", node_id);
        buf.complete = false;
        msg_send_reboot(node_id);
      }
    } else {
      log_error("Unrecognized command: [%s]", command);
    }
  } else {
    log_error("Unrecognized command: [%s]", command);
  }
}

void actuator_send_command(actuator_t *actuator, char *value)
{
  if (actuator->node_id == 0xf0) {
    actuator_execute_command(value);
  } else {
    msg_start_to_node(actuator->node_id);
    msg_putbyte('1');
    msg_putbyte(actuator->actuator_id);
    msg_putbyte(atoi(value));
    msg_end();
    //msg_send_bbb('1', actuator->actuator_id, atoi(value));
  }
}

void command_received(char *topic, char *payload)
{
  actuator_t *actuator;
  actuator = actuator_from_topic(topic);
  if (actuator == NULL) {
    // TODO log error
    return;
  }
  actuator_send_command(actuator, payload);
}

int main()
{
  msg m;

  read_sensors_config();
  read_actuators_config();
  msg_begin();
  mqtt_start();

  while(true) {
    set_now();
    verify_uart();
    if (msg_get_msg(&m)) {
      received_message(m);
    }
    mqtt_loop();
    usleep(10);
  }
}
