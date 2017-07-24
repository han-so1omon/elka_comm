#ifndef ELKA_POSIX_DEVICES_H
#define ELKA_POSIX_DEVICES_H

#include <map>
#include <stdlib.h>
#include <elka_comm/common/elka.h>
#include <elka_comm/common/elka_comm.h>
#include <uORB/uORB.h>
#include <uORB/topics/elka_msg.h>
#include <uORB/topics/elka_msg_ack.h>
#include <utility>

#include "inet_comm.h"

namespace elka {
class GroundPort;
}

// Manages connection with an ELKA device
// Determines mode of input (autonomous or Spektrum)
// Sends elka_msg_s messages to snapdragon_uart
class elka::GroundPort : public elka::CommPort {
public:

  // elka_ack_snd is ack to be sent after parsing next cmd from rx_buf
  // elka_ack_rcv is ack received after sending msg from tx_buf
  struct elka_msg_ack_s _elka_ack_rcv,
                        _elka_ack_snd,
                        _elka_ack_rcv_cmd;
  // elka_snd is msg to send from tx buf
  // elka_rcv is msg to push to rx buf
  // elka_rcv_cmd is msg to be parsed from rx buf
  struct elka_msg_s _elka_snd,
                    _elka_rcv,
                    _elka_rcv_cmd;

  orb_advert_t _elka_msg_pub;
  orb_advert_t _elka_ack_pub;

  GroundPort(uint8_t port_num, uint8_t port_type, uint8_t buf_type,
      uint8_t size, char *dev_name);

  ~GroundPort();

  static int initialize(
      uint8_t port_num, uint8_t port_type, uint8_t buf_type,
      uint8_t queue_sz, char *dev_name);

  static elka::GroundPort *get_instance() {
    return _instance;
  }

  /**
   * Print statistics
   * @param reset, if true reset statistics afterwards
   * @return true if something printed, false otherwise
   */
  bool print_statistics(bool reset);

  // Add message to buffer
  // Adds message to buffer for all applicable
  // devices unless target_dev is specified
  // If num_retries and msg_num are set to zero, then this
  // message's msg_num will be set to the next msg_num of 
  // tx_buf
  // @return msg_type
  uint8_t add_msg(uint8_t msg_type,
                  uint8_t len,
                  uint8_t num_retries,
                  uint16_t msg_num,
                  uint8_t *data,
                  dev_id_t *target_dev);

  // Remove and send front message from buffer
  uint8_t send_msg(elka_msg_s &elka_msg);
  uint8_t send_msg(elka_msg_ack_s &elka_msg);
  
  // Parse elka msg and set ack if applicable 
  // If control message, perform appropriate actions on serial port.
  // If from ELKA device, set message in RX buffer.
  // @return msg_type if msg is meant for u
  //         MSG_NULL if msg not meant for u
  //         MSG_FAILED If msg meant for u and incorrect
  uint8_t parse_elka_msg(elka_msg_s &elka_ret);
  uint8_t parse_elka_msg(elka_msg_ack_s &elka_msg);

  // Check ack for sent message
  // Check ack with respect to port number from elka_ack.msg_id
  uint8_t check_ack(struct elka_msg_ack_s &elka_ack);

  // Set elka state in elka_msg. May push this to a buffer after
  uint8_t set_dev_state_msg(
      elka_msg_s &elka_snd,
      dev_id_t rcv_id,
      uint8_t state,
      bool elka_ctl);

  // Update _now variable with current time
  void update_time();

  uint8_t start_port() override;
  uint8_t stop_port() override;
  uint8_t pause_port() override;
  uint8_t resume_port() override;

  uint8_t remote_ctl_port() override;
  uint8_t autopilot_ctl_port() override;


private:

  /*
  // Map from port id to port num
  // IDs correspond to _ports[i]->_id
  std::map<dev_id_t, uint8_t> _port_num_map;
  */

  // Data members
  static GroundPort *_instance; // Singleton port instance

  // This must be updated frequently thru callback or otherwise!
  char _dev_name[MAX_NAME_LEN];
  uint8_t _inet_role;
  Child _inet_proc;

  void wait_for_child(Child *child);

  // Class methods
  int deinitialize();

  // Helper functions for parsing returned elka message based on current state
  // @return msg type:
  //         parse_motor_cmd() always returns MSG_NULL
  //         parses_port_ctl and parse_elka_ctl return:
  //         MSG_FAILED if msg parsing failed
  //         MSG_NULL if msg not for u
  //         Can return all other types of msgs except for MSG_ACK
  // Resumes elka if paused or started. Starts elka if stopped
  uint8_t parse_motor_cmd(elka_msg_s &elka_ret,
                          elka_msg_ack_s &elka_ack,
                          struct elka_msg_id_s &msg_id);
  uint8_t parse_port_ctl(elka_msg_s &elka_ret,
                         elka_msg_ack_s &elka_ack,
                         struct elka_msg_id_s &msg_id);
  uint8_t parse_elka_ctl(elka_msg_s &elka_ret,
                         elka_msg_ack_s &elka_ack,
                         struct elka_msg_id_s &msg_id);

};
#endif
