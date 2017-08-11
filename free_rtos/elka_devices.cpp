#include <cstring>
#include <errno.h>
#include <elka_log.h>

#include "elka_devices.h"

elka::ELKAPort::ELKAPort(uint8_t port_num, uint8_t port_type,
    uint8_t buf_type, uint8_t size, char *dev_name) 
    : elka::CommPort(port_num, port_type, buf_type, size) {

  memset(&_elka_ack_snd, 0, sizeof(_elka_ack_snd));
  memset(&_elka_ack_rcv, 0, sizeof(_elka_ack_rcv));
  memset(&_elka_ack_rcv_cmd, 0, sizeof(_elka_ack_rcv_cmd));
  memset(&_elka_snd, 0, sizeof(_elka_snd));
  memset(&_elka_rcv, 0, sizeof(_elka_rcv));
  memset(&_elka_rcv_cmd, 0, sizeof(_elka_rcv_cmd));

  _routing_table[_id].add_prop(DEV_PROP_ELKA_SIDE);
  _routing_table[_id].add_prop(DEV_PROP_PERFORM_LOCALIZATION);
  _routing_table[_id].add_prop(DEV_PROP_SENSE_LOCATION);
  _routing_table[_id].add_prop(DEV_PROP_SPIN_MOTORS);
  _routing_table[_id].add_prop(DEV_PROP_USE_CAMERA);
  _routing_table[_id].add_prop(DEV_PROP_HAS_CAMERA);
  _routing_table[_id].add_prop(DEV_PROP_TRANSMISSION_CTL);

  strcpy(_dev_name, dev_name);

  get_snd_params(&_snd_params, port_num, port_type,
    DEV_PROP_POSIX_SIDE);

  start_port();
}

elka::ELKAPort::~ELKAPort() {
  if (!(deinitialize() == ELKA_OK)) {
    LOG_ERR("Unable to deinitialize elka device %s",
    _dev_name);
    errno = ECANCELED;
  };

  delete _tx_buf;
  delete _rx_buf;
}

int elka::ELKAPort::initialize(
      uint8_t port_num, uint8_t port_type, uint8_t buf_type,
      uint8_t queue_sz, char *dev_name) {
  if (_instance == nullptr) {
    _instance = new ELKAPort(port_num, port_type,
        buf_type, queue_sz, dev_name);
  }

  return _instance != nullptr;
}

bool elka::ELKAPort::print_statistics(bool reset) {
  return false;
}

// msg_num and num_retries are useful parameters only
// with MSG_ACK.
// In all other cases these parameters are handled upon
// message pushing/getting
uint8_t elka::ELKAPort::add_msg(
    uint8_t msg_type,
    uint8_t len,
    uint8_t num_retries,
    uint16_t msg_num,
    uint8_t *data,
    dev_id_t *target_dev) {
  std::vector<dev_id_t> target_devs;
  
  // If msg_type is MSG_ROUTE_DEV_PROPS
  // and there is no target device,
  // then just push message in all possible ways,
  // because this is supposed to be a broadcast message.
  // If msg_type is not MSG_ROUTE_DEV_PROPS
  // or there is a target device,
  // then form vector of target devices.
  // TODO
  if (msg_type == MSG_ROUTE_DEV_PROPS && !target_dev) {
    elka_msg_s elka_msg;

    get_elka_msg_id(&elka_msg.msg_id,
      _id, 0, _snd_params,
      msg_type, len);

    memcpy(elka_msg.data, data, len);

    push_msg(elka_msg, true);

    return msg_type;
  } else if (target_dev) {
    target_devs.push_back(*target_dev); 
  } else {
    std::map<dev_id_t,
             DeviceRoute,
             dev_id_tCmp>::iterator
        dev_routes = _routing_table.begin();
    for (; dev_routes != _routing_table.end(); dev_routes++) {
      if ( cmp_dev_id_t(dev_routes->first, _id) &&  
           check_dev_compatible(msg_type,
                               dev_routes->first) )
        target_devs.push_back(dev_routes->first);
    }
  }

  std::vector<dev_id_t>::iterator curr_dev =
    target_devs.begin();;
  if (msg_type == MSG_ACK) {
    elka_msg_ack_s elka_msg;

    for (; curr_dev != target_devs.end(); curr_dev++) {
      get_elka_msg_id(&elka_msg.msg_id,
        _id, *curr_dev, _snd_params,
        msg_type, len);

      elka_msg.num_retries = num_retries;
      elka_msg.msg_num = msg_num;
      elka_msg.result = *data;

      push_msg(elka_msg, true);
    }
  } else {
    elka_msg_s elka_msg;
    
    for (; curr_dev != target_devs.end(); curr_dev++) {
      get_elka_msg_id(&elka_msg.msg_id,
        _id, *curr_dev, _snd_params,
        msg_type, len);

      elka_msg.num_retries = num_retries;
      elka_msg.msg_num = msg_num;
      memcpy(elka_msg.data, data, len);

      push_msg(elka_msg, true);
    }
  }

  return msg_type;
}

//FIXME is this necessary? It isn't in the other implementations
/*
uint8_t add_msg(msg_id_t msg_id,
                uint16_t msg_num,
                uint8_t num_retries,
                uint8_t *data,
                bool tx) {
  if (msg_type == MSG_ACK) {
    elka_msg_ack_s elka_msg;

    elka_msg.msg_id = msg_id;
    elka_msg.num_retries = num_retries;
    elka_msg.msg_num = msg_num;
    elka_msg.result = *data;

    push_msg(elka_msg, tx);
  } else {
    elka_msg_s elka_msg;
    
    elka_msg.num_retries = num_retries;
    elka_msg.msg_num = msg_num;
    elka_msg.msg_id = msg_id;
    memcpy(elka_msg.data, data, len);

    push_msg(elka_msg, tx);
  }

  return msg_type;
}
*/

// TODO eventually put this in common elka library
// TODO get rid of parameter, as _elka_snd should be set
// TODO send message (start with UART)
uint8_t elka::ELKAPort::send_msg(elka_msg_s &elka_msg) {
  uint8_t msg_type;
  get_elka_msg_id_attr(NULL, NULL, NULL, &msg_type, NULL,
      elka_msg.msg_id);

  return msg_type;
}

uint8_t elka::ELKAPort::send_msg(elka_msg_ack_s &elka_msg) {
  uint8_t msg_type;
  get_elka_msg_id_attr(NULL, NULL, NULL, &msg_type, NULL,
      elka_msg.msg_id);

  return msg_type;
}

// For now: Do nothing with elka_msg.msg_num field
uint8_t elka::ELKAPort::parse_elka_msg(elka_msg_s &elka_msg) {
  elka_msg_ack_s elka_ack;
  struct elka_msg_id_s msg_id;
  uint8_t parse_res;
  bool in_route;

  get_elka_msg_id_attr(&msg_id, elka_msg.msg_id);

  // Check that device can be reached.
  // If message is not meant for you, then push it thru.
  // If message is meant for you, then check message type
  // to determine correct method of parsing message.
  if (( !(in_route = check_route(msg_id.rcv_id)) &&
        !broadcast_msg(msg_id.rcv_id) ) ||
      initial_msg(elka_msg.msg_id,
                  elka_msg.msg_num,
                  elka_msg.num_retries)) {
    return MSG_NULL;
  } else if (in_route &&
             cmp_dev_id_t(msg_id.rcv_id, _id) &&
             cmp_dev_id_t(msg_id.snd_id, _id)) {
    // Push message along if:
    //    It can be reached 
    //    It is not for you
    //    It is not from you (avoid creating cycle in graph)
    return push_msg(elka_msg, true);
  }
  
  switch(msg_id.type) {
    case MSG_MOTOR_CMD:
      parse_res = parse_motor_cmd(elka_msg, elka_ack, msg_id);
      break;
    case MSG_PORT_CTL:
      parse_res = parse_port_ctl(elka_msg, elka_ack, msg_id);
      break;
    case MSG_ELKA_CTL:
      parse_res = parse_elka_ctl(elka_msg, elka_ack, msg_id);
      break;
    case MSG_ROUTE_DEV_PROPS:
    case MSG_ROUTE_REQUEST_HB:
    case MSG_ROUTE_HB:
    case MSG_ROUTE_CHANGED:
    case MSG_ROUTE_TABLE:
      elka_msg_s ret_routing_msg;
      parse_res = parse_routing_msg(
          elka_msg, msg_id, elka_ack, ret_routing_msg);
      break;
    default:
      return MSG_FAILED;
      break;
  }

  // Send ack if necessary 
  if (parse_res & TYPE_EXPECTING_ACK) {
    push_msg(elka_ack, true);
  }

  return parse_res;
}

uint8_t elka::ELKAPort::parse_motor_cmd(elka_msg_s &elka_msg,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id) {
  uint8_t state = _sw_state;

  switch (_sw_state) {
    case SW_CTL_SPEKTRUM:
      return MSG_FAILED;
    case SW_CTL_REMOTE:
      return MSG_FAILED;
      break;
    case SW_CTL_AUTOPILOT:
      return MSG_FAILED;
      break;
    default:
      return MSG_FAILED;
      break;
  }
}

uint8_t elka::ELKAPort::parse_port_ctl(elka_msg_s &elka_msg,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id) {
  uint8_t action = elka_msg.data[1];
  bool request = elka_msg.data[0];

  elka_ack.msg_num = elka_msg.msg_num;
  elka_ack.num_retries = elka_msg.num_retries;

  get_elka_msg_id(&elka_ack.msg_id,
      msg_id.rcv_id, msg_id.snd_id,
      msg_id.snd_params,
      MSG_ACK, MSG_ACK_LENGTH);

  if (request) {
    switch (action) {
      case HW_CTL_START:
        elka_ack.result = start_port();
        break;
      case HW_CTL_STOP:
        elka_ack.result = stop_port();
        break;
      case HW_CTL_PAUSE:
        elka_ack.result = pause_port();
        break;
      case HW_CTL_RESUME:
        elka_ack.result = resume_port();
        break;
      default:
        elka_ack.result = MSG_UNSUPPORTED;
        return MSG_FAILED;
        break;
    }
  } else {
    //TODO keep track of device states
  }

  return msg_id.type;

}

uint8_t elka::ELKAPort::parse_elka_ctl(elka_msg_s &elka_msg,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id) {
  uint8_t action = elka_msg.data[1];
  bool request = elka_msg.data[0];
  
  elka_ack.msg_num = elka_msg.msg_num;
  elka_ack.num_retries = elka_msg.num_retries;

  get_elka_msg_id(&elka_ack.msg_id,
      msg_id.rcv_id, msg_id.snd_id,
      msg_id.snd_params,
      MSG_ACK, MSG_ACK_LENGTH);

  if (request) {
    switch (action) {
      case SW_CTL_REMOTE:
        elka_ack.result = remote_ctl_port();
        break;
      case SW_CTL_AUTOPILOT:
        elka_ack.result = autopilot_ctl_port();
        break;
      default:
        elka_ack.result = MSG_UNSUPPORTED;
        return MSG_FAILED;
        break;
    }
  } else {
    //TODO keep track of device states
  }
  return msg_id.type;
}

// Check ack for sent message
// Check ack with respect to port number from elka_ack.msg_id
uint8_t elka::ELKAPort::check_ack(struct elka_msg_ack_s &elka_ack) {
  elka::SerialBuffer *sb;
  struct elka_msg_id_s ack_id;
  uint8_t ret;

  get_elka_msg_id_attr(&ack_id, elka_ack.msg_id);

  // Check that ack is meant for this device
  if (cmp_dev_id_t(ack_id.rcv_id, _id)) {
    return MSG_NULL;
  } else {
    sb = _tx_buf;
  }
 
  // First check if message number has recently been acked
  // If message has recently been acked then
  // return MSG_NULL
  // Else then check if message exists in buffer
  //    If message exists, then return check_elka_ack(...)
  //        Then add message number to list of recently acked messages
  //        Then remove message from buffer
  //    Else return MSG_NULL
  if ((ret = sb->check_recent_acks(elka_ack.msg_num)) !=
              MSG_NULL) {
    msg_id_t erase_msg_id;
    ElkaBufferMsg *ebm;
   
    if ((ebm = sb->get_buffer_msg(
            ack_id.rcv_id, ack_id.snd_id, elka_ack.msg_num))) {
      if ( (ret = check_elka_ack(elka_ack,
                  ebm->_msg_id,
                  ebm->_rmv_msg_num,
                  ebm->_num_retries)) ==
           MSG_FAILED) {
        LOG_WARN("Ack failed msg_id %" PRMIT "msg_num %" PRIu16
". %d retries",
            ebm->_msg_id, ebm->_rmv_msg_num,
            ebm->_num_retries);
      } else if (ret != MSG_NULL) {
        // Message received and processed fine, so erase it
        erase_msg_id = ebm->_msg_id;
        //sb->erase_msg(elka_ack.msg_id, elka_ack.msg_num);
        sb->erase_msg(erase_msg_id, elka_ack.msg_num);
        sb->push_recent_acks(elka_ack.msg_num);
      }
    }
  }

  return ret;

}

uint8_t elka::ELKAPort::set_dev_state_msg(
    elka_msg_s &elka_snd,
    dev_id_t rcv_id,
    uint8_t state,
    bool elka_ctl) {
  uint8_t msg_t;
  if (elka_ctl) {
    msg_t = MSG_ELKA_CTL;
  } else {
    msg_t = MSG_PORT_CTL;
  }

  set_state_msg(elka_snd, state,
                _id, rcv_id,
                _snd_params, msg_t, 1);

  elka_snd.msg_num = 0;
  elka_snd.num_retries = 0;

  return state;
}

//TODO
void elka::ELKAPort::update_time() {
}

int elka::ELKAPort::deinitialize() {
  _routing_table.clear();
  stop_port();
  return ELKA_OK;
}

uint8_t elka::ELKAPort::start_port() {
  uint8_t tmp_state = _hw_state;

  _hw_state = HW_CTL_START;
  _prev_hw_state = tmp_state;

  _serial_port_state = CONNECTION_OPEN;

  return resume_port();
}

uint8_t elka::ELKAPort::stop_port() {
  uint8_t tmp_state = _hw_state;

  _hw_state = HW_CTL_STOP;
  _prev_hw_state = tmp_state;
  

  return MSG_ACCEPTED;
}

uint8_t elka::ELKAPort::pause_port() {
  uint8_t tmp_state = _hw_state;

  _hw_state = HW_CTL_PAUSE;
  _prev_hw_state = tmp_state;

  return MSG_ACCEPTED;
}

uint8_t elka::ELKAPort::resume_port() {
  uint8_t tmp_state = _hw_state;
  
  _hw_state = HW_CTL_RESUME;
  _prev_hw_state = tmp_state;

  return MSG_ACCEPTED;
}

uint8_t elka::ELKAPort::remote_ctl_port() {
  uint8_t tmp_state = _sw_state;

  if (_sw_state != SW_CTL_KILL && _sw_state != SW_CTL_SPEKTRUM) {
    _sw_state = SW_CTL_REMOTE;
    _prev_sw_state = tmp_state;
  } else return SW_CTL_FAILED;

  return MSG_ACCEPTED;

}

uint8_t elka::ELKAPort::autopilot_ctl_port() {
  uint8_t tmp_state = _sw_state;
  
  if (_sw_state != SW_CTL_KILL && _sw_state != SW_CTL_SPEKTRUM) {
    _sw_state = SW_CTL_AUTOPILOT;
    _prev_sw_state = tmp_state;
  } else return SW_CTL_FAILED;

  return MSG_ACCEPTED;
}