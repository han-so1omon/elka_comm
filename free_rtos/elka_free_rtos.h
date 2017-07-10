#ifndef ELKA_FREE_RTOS_H
#define ELKA_FREE_RTOS_H
#include <elka/common/elka.h>

#define MAX_SERIAL_MSGS 0x16 // Hold 21 messages of MAX_SERIAL_BUF_LEN

// Create struct to pass pointers to elka msgs
// as FreeRTOS pvParameters
struct elka_msg_ptrs_s {
  elka_msg_s rcv_msg, snd_msg;
  elka_msg_ack_s rcv_ack, snd_ack;
};

#endif
