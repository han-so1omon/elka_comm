#include <algorithm>
#include <functional>
#include <errno.h>

#if defined(__PX4_QURT) || defined(__PX4_POSIX)

#include <px4_defines.h>
#include <px4_log.h>

#endif

#include <elka_log.h>

#include "elka_comm.h"

//-----------------ElkaBufferMsg Methods---------------------

/* TODO See if we don't need these
elka::ElkaBufferMsg::ElkaBufferMsg() {
  _msg_id = 0;
  _msg_num = 0;
  _expecting_ack = 0;
  _num_retries = 0;
}

elka::ElkaBufferMsg::ElkaBufferMsg(elka_msg_s &msg) {
  uint8_t msg_len;
  _msg_id = msg.msg_id;
  get_elka_msg_id_attr(NULL,NULL,NULL,NULL,
      &msg_len,_msg_id);
  _push_msg_num = msg.msg_num;
  _rmv_msg_num = msg.msg_num;
  memcpy(_data, msg.data, msg_len);
  _expecting_ack = msg.msg_id & ID_EXPECTING_ACK;
  _num_retries = 0;
}

ElkaBufferMsg(elka_msg_ack_s &msg) {
  uint8_t msg_len;
  _msg_id = msg.msg_id;
  get_elka_msg_id_attr(NULL,NULL,NULL,NULL,
      &msg_len,_msg_id);
  _push_msg_num = msg.msg_num;
  _rmv_msg_num = 0;
  memset(_data, msg.msg.result, msg_len);
  _expecting_ack = msg.msg_id & ID_EXPECTING_ACK;
  _num_retries = 0;
}
*/

elka::ElkaBufferMsg::ElkaBufferMsg(msg_id_t msg_id,
    uint16_t push_msg_num, uint16_t rmv_msg_num,
    uint8_t num_retries, uint8_t *data) {
  uint8_t msg_len;
  _msg_id = msg_id;
  get_elka_msg_id_attr(NULL,NULL,NULL,NULL,
      &msg_len,msg_id);
  _push_msg_num = push_msg_num;
  _rmv_msg_num = rmv_msg_num;
  memcpy(_data, data, msg_len);
  _num_retries = num_retries;
  _expecting_ack = msg_id & ID_EXPECTING_ACK;
}

elka::ElkaBufferMsg::~ElkaBufferMsg() {
}

void elka::ElkaBufferMsg::clear_contents() {
  uint8_t msg_len;
  get_elka_msg_id_attr(NULL, NULL, NULL, NULL, &msg_len,
      _msg_id);
  memset(_data,0,msg_len);
}

uint8_t elka::ElkaBufferMsg::get_result() {
  uint8_t msg_type;
  get_elka_msg_id_attr(NULL, NULL, NULL, &msg_type, NULL,
    _msg_id);
  if (msg_type == MSG_ACK) 
    return _data[0];
  else
    return MSG_NULL;
}

//-----------------SerialBuffer Methods---------------------

elka::SerialBuffer::SerialBuffer(dev_id_t port_id,
    uint8_t buf_type, uint16_t size) {
  _port_id = port_id;
  _comp = Compare();
  _type = buf_type;
  _max_size = size;

  _recent_acks_end = 0;
  _recent_acks_len = 0;

  _push_msg_num=0;
  _rmv_msg_num=0;

  if (_type == ARRAY) {
  } else if (_type == PRIORITY_QUEUE) {
  } else {
    LOG_ERR("Unsupported buffer type");
    errno = EINVAL;
  }

  _buf_mutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_init(&_buf_mutex,NULL);
}

elka::SerialBuffer::~SerialBuffer() {
  pthread_mutex_destroy(&_buf_mutex);
}

uint8_t elka::SerialBuffer::check_recent_acks(uint16_t msg_num) {
  int idx = cb_bin_search(msg_num, _recent_acks,
    _recent_acks_end, _recent_acks_len, RECENT_ACKS_LEN);
  if (idx < 0) return MSG_FAILED;
  else return MSG_NULL;
}

void elka::SerialBuffer::push_recent_acks(uint16_t msg_num) {
  cb_push(msg_num, _recent_acks, _recent_acks_end,
      _recent_acks_len, RECENT_ACKS_LEN);
}

uint8_t elka::SerialBuffer::push_msg(elka_msg_ack_s &msg) {
  uint8_t data[1];
  memset(data, msg.result, 1);
  return push_msg(msg.msg_id, data,
                  msg.msg_num, msg.num_retries);
}

uint8_t elka::SerialBuffer::push_msg(elka_msg_s &msg) {
  return push_msg(msg.msg_id, msg.data,
                  msg.msg_num, msg.num_retries);
}

uint8_t elka::SerialBuffer::push_msg(msg_id_t msg_id,
    uint8_t *data) {
  // This is a new message (not retried)
  // Set num_retries to 0
  return push_msg(msg_id, data, 0, 0);
}

// Must have num_retries so that retried messages won't be
// redundantly added to receive buffer
uint8_t elka::SerialBuffer::push_msg(
    msg_id_t msg_id,
    uint8_t *data,
    uint16_t rmv_msg_num,
    uint8_t num_retries) {
  ElkaBufferMsg *ebm;
  uint8_t msg_type;


  get_elka_msg_id_attr(NULL, NULL, NULL, &msg_type, NULL,
      msg_id);

  // If message w/matching msg_id & rmv_msg_num is already in
  // buffer, change num_retries and return msg_type
  // TODO What if data is different?
  if ((ebm = get_buffer_msg(msg_id, rmv_msg_num)) != nullptr) {
    ebm->_num_retries = num_retries;
    return msg_type;
  }

  pthread_mutex_lock(&_buf_mutex);

  if (_type == ARRAY) {
    _buffer.push_back(ElkaBufferMsg(
          msg_id, _push_msg_num++, rmv_msg_num,
          num_retries, data));
    if (_buffer.size() > _max_size) {
      _buffer.pop_back();
    }
  } else if (_type == PRIORITY_QUEUE) {
    _buffer.push_back(ElkaBufferMsg(
          msg_id, _push_msg_num++, rmv_msg_num,
          num_retries, data));
    std::push_heap(_buffer.begin(), _buffer.end(), _comp);
    // Priority queue is fixed size
    if (_buffer.size() > _max_size) {
      std::pop_heap(_buffer.begin(), _buffer.end(), _comp);
      _buffer.pop_back();
    }
  } else {
    pthread_mutex_unlock(&_buf_mutex);
    return MSG_FAILED;
  }

  pthread_mutex_unlock(&_buf_mutex);

  return msg_type;
}

uint8_t elka::SerialBuffer::get_msg(
    elka_msg_s &elka_msg,
    elka_msg_ack_s &elka_msg_ack) {
  dev_id_t snd_id; 
  uint8_t ret, msg_len, front_msg_type;

  // Ensure that buffer isn't empty 
  // Pop messages that have been tried too many times
  if (_buffer.empty()) return MSG_NULL;
  while (_buffer.front()._num_retries > MAX_NUM_RETRIES) {
    pop_msg();
    if (_buffer.empty()) return MSG_NULL;
  }

  pthread_mutex_lock(&_buf_mutex);

  // Get message type for message at front of buffer
  front_msg_type = buffer_front_type();

  if (front_msg_type != MSG_ACK) {
    elka_msg.msg_id = _buffer.front()._msg_id;
    get_elka_msg_id_attr(&snd_id,NULL,NULL,&ret,&msg_len,
        elka_msg.msg_id);
  } else {
    elka_msg_ack.msg_id = _buffer.front()._msg_id;
    get_elka_msg_id_attr(&snd_id,NULL,NULL,&ret,&msg_len,
        elka_msg_ack.msg_id);
  }

  // Only assign and increment _rmv_msg_num if _num_retries for msg at
  // front of buffer is 0 and msg originated at this port and msg doesn't
  // already have a _rmv_msg_num.
  // Msg num must be the same for a msg so that it can be correctly
  // checked for ack.
  if (_buffer.front()._num_retries == 0 &&
      _buffer.front()._rmv_msg_num == 0 &&
      !cmp_dev_id_t(_port_id, snd_id)) {
    _buffer.front()._rmv_msg_num = get_nxt_msg_num();
  }

  if (front_msg_type != MSG_ACK) {
    elka_msg.msg_num = _buffer.front()._rmv_msg_num;
    elka_msg.num_retries = _buffer.front()._num_retries++;
    memcpy(elka_msg.data, _buffer.front()._data, msg_len);

    /*
    LOG_INFO("in get\tmsg_num: %" PRIu16 " num_retries: %d",
      elka_msg.msg_num,
      elka_msg.num_retries);
      */
  } else {
    elka_msg_ack.msg_num = _buffer.front()._rmv_msg_num;
    elka_msg_ack.num_retries = _buffer.front()._num_retries;
    // This will return MSG_NULL if the message is not of MSG_ACK type
    elka_msg_ack.result = _buffer.front().get_result();

    /*
    LOG_INFO("in get ack\tmsg_num: %" PRIu16 " num_retries: %d",
      elka_msg_ack.msg_num,
      elka_msg_ack.num_retries);
      */
  }

  pthread_mutex_unlock(&_buf_mutex);

  return ret;
}

//FIXME fix qurt side
elka::ElkaBufferMsg *elka::SerialBuffer::get_buffer_msg(
    msg_id_t msg_id, uint16_t msg_num) {
//#if defined(__PX4_QURT)

  pthread_mutex_lock(&_buf_mutex);

  for (std::vector<ElkaBufferMsg>::iterator it =
        _buffer.begin();
       it != _buffer.end(); it++) {
    if (msg_num != 0 &&
        it->_rmv_msg_num == msg_num &&
        !cmp_msg_id_t(msg_id, it->_msg_id)) {
      pthread_mutex_unlock(&_buf_mutex);
      return &(*it);
    }
  }

  pthread_mutex_unlock(&_buf_mutex);
  return nullptr;

  /*
#elif defined(__PX4_POSIX)
  std::vector<ElkaBufferMsg>::iterator it;
  if ( (it = std::find_if(_buffer.begin(),_buffer.end(),
            std::function<bool(ElkaBufferMsg const&)>(
              [msg_num](ElkaBufferMsg const &ebm) {
              return ebm._rmv_msg_num==msg_num;
            }) ) ) != _buffer.end()) {
    return &(*it);
  } else
    return NULL;
#endif
*/
}

elka::ElkaBufferMsg *elka::SerialBuffer::get_buffer_msg(
    dev_id_t snd_id,
    dev_id_t rcv_id,
    uint16_t msg_num) {
  dev_id_t msg_snd_id, msg_rcv_id;
  pthread_mutex_lock(&_buf_mutex);

  for (std::vector<ElkaBufferMsg>::iterator it =
        _buffer.begin();
       it != _buffer.end(); it++) {
    get_elka_msg_id_attr(
        &msg_snd_id, &msg_rcv_id,
        NULL, NULL, NULL,
        it->_msg_id);

    if (msg_num != 0 &&
        it->_rmv_msg_num == msg_num &&
        !cmp_dev_id_t(snd_id, msg_snd_id) &&
        !cmp_dev_id_t(rcv_id, msg_rcv_id)) {
      pthread_mutex_unlock(&_buf_mutex);
      return &(*it);
    }
  }

  pthread_mutex_unlock(&_buf_mutex);
  return nullptr;
}

// TODO necessary to set time stamps?
uint8_t elka::SerialBuffer::remove_msg(
    elka_msg_s &elka_msg,
    elka_msg_ack_s &elka_msg_ack) {
  msg_id_t msg_id;
  uint16_t msg_num;
  uint8_t ret;

  ret = get_msg(elka_msg,elka_msg_ack);

  if (ret == MSG_ACK) {
    msg_id = elka_msg_ack.msg_id;
    msg_num = elka_msg_ack.msg_num;
  } else {
    msg_id = elka_msg.msg_id;
    msg_num = elka_msg.msg_num;
  }

  erase_msg(msg_id, msg_num);
  return ret;
}

uint8_t elka::SerialBuffer::pop_msg() {
  uint8_t ret;

  // Ensure that buffer isn't empty
  if (_buffer.empty()) return MSG_NULL;

  pthread_mutex_lock(&_buf_mutex);

  if (_type == ARRAY) {
    _buffer.erase(_buffer.begin());
    ret = MSG_NULL;
  } else if (_type == PRIORITY_QUEUE) {
    _buffer.erase(_buffer.begin());
    std::make_heap(_buffer.begin(), _buffer.end(), _comp);
    ret = MSG_NULL;
  } else {
    ret = MSG_FAILED;
  }

  pthread_mutex_unlock(&_buf_mutex);

  return ret;
}

//FIXME fix qurt side
void elka::SerialBuffer::erase_msg(
    msg_id_t msg_id, uint16_t msg_num) {
//#if defined(__PX4_QURT)
  // Search beginning at end of array bc this should be closer to
  // msg_num in most expected use cases
  pthread_mutex_lock(&_buf_mutex);
  for (std::vector<ElkaBufferMsg>::reverse_iterator it =
       _buffer.rbegin();
       it != _buffer.rend(); it++) {
    if (it->_rmv_msg_num == msg_num &&
        !cmp_msg_id_t(msg_id, it->_msg_id)) {
      _buffer.erase(--(it.base()));
      break;
    }
  }
  pthread_mutex_unlock(&_buf_mutex);

  /*
#elif defined(__PX4_POSIX)
  _buffer.erase(std::remove_if(_buffer.begin(),_buffer.end(),
        std::function<bool(ElkaBufferMsg const&)>(
          [msg_num](ElkaBufferMsg const &ebm) {
    return ebm._rmv_msg_num==msg_num;
  })), _buffer.end());
#endif
*/
}

uint8_t elka::SerialBuffer::buffer_front_type() {
  uint8_t ret;
  get_elka_msg_id_attr(NULL, NULL, NULL, &ret, NULL,
      _buffer.front()._msg_id);
  return ret; 
}

uint8_t elka::SerialBuffer::Compare::msg_priority(
    uint8_t msg_type) {
  uint8_t ret = 0;
  switch (msg_type) {
    case MSG_NULL:
      ret = 254;
      break;
    case MSG_FAILED:
      ret = 9;
      break;
    case MSG_MOTOR_CMD:
      ret = 8;
      break;
    case MSG_PORT_CTL:
      ret = 7;
      break;
    case MSG_ELKA_CTL:
      ret = 6;
      break;
    case MSG_ROUTE_HB:
      ret = 1;
      break;
    case MSG_ROUTE_REQUEST_HB:
      ret = 2;
      break;
    case MSG_ROUTE_CHANGED:
      ret = 3;
      break;
    case MSG_ROUTE_TABLE:
      ret = 4;
      break;
    case MSG_ROUTE_DEV_PROPS:
      ret = 5;
      break;
    case MSG_ACK:
      ret = 0; // highest priority
      break;
    default:
      ret = 255; // lowest priority
      break;
  }
  return ret;
}

bool elka::SerialBuffer::Compare::operator()
  (ElkaBufferMsg p, ElkaBufferMsg q) {
  uint8_t p_type, q_type;

  get_elka_msg_id_attr(NULL,NULL,NULL,&p_type,NULL,
      p._msg_id);
  get_elka_msg_id_attr(NULL,NULL,NULL,&q_type,NULL,
      q._msg_id);

  /*
  LOG_INFO("push up? %s", ((msg_priority(p_type) < msg_priority(q_type)) &&
          p._push_msg_num < q._push_msg_num) ? "true" : "false");
         */
  return msg_priority(p_type) > msg_priority(q_type) ?
          true : p._push_msg_num > q._push_msg_num;
}

//-----------------DeviceRoute Methods---------------------
elka::DeviceRoute::DeviceRoute() {
  _heartbeat = true;
  _new_dev = true;
}

elka::DeviceRoute::DeviceRoute(std::vector<dev_id_t> *route,
            std::vector<dev_prop_t> *props) {
  if (route) {
    _route = std::vector<dev_id_t>(*route);
  }

  if (props) {
    _props = std::vector<dev_prop_t>(*props);
  }

  _heartbeat = true;
  _new_dev = true;
}

elka::DeviceRoute::DeviceRoute(DeviceRoute *dr) {
  if (dr) {
    _route = std::vector<dev_id_t>(dr->_route);
    _props = std::vector<dev_prop_t>(dr->_props);
  }

  _heartbeat = true;
  _new_dev = true;
}

void elka::DeviceRoute::add_prop(dev_prop_t prop) {
  if (!check_prop(prop)) {
    _props.push_back(prop);
    std::sort(_props.begin(), _props.end());
  }
}

void elka::DeviceRoute::remove_prop(dev_prop_t prop) {
  if (check_prop(prop)) {
    std::vector<dev_prop_t>::iterator pos;
    for(pos = _props.begin(); pos < _props.end(); pos++) {
      if (!cmp_dev_prop_t(*pos, prop)) {
        _props.erase(pos);
        std::sort(_props.begin(), _props.end());
      }
    }
  }
}

bool elka::DeviceRoute::check_prop(dev_prop_t prop) {
  return std::binary_search(_props.begin(), _props.end(), prop);
}

bool elka::DeviceRoute::check_alive() {
  return _heartbeat;
}

int8_t elka::DeviceRoute::route_cmp(std::vector<dev_id_t> *r1,
                                    std::vector<dev_id_t> *r2) {
  if (r1->size() > r2->size() || (!r1 && r2))
    return -1;
  else if (r1->size() < r2->size() || (r1 && !r2))
    return 1;
  else
    return 0;
}

bool elka::DeviceRoute::dev_props_cmp(
    std::vector<dev_prop_t> *p1,
    std::vector<dev_prop_t> *p2) {
  return ( p1 && p2 &&
           std::equal(p1->begin(), p2->end(), p1->begin()) );
}

void elka::DeviceRoute::print_dev_props(
      std::vector<dev_prop_t> &props) {
  std::vector<dev_prop_t>::iterator it_props;
  char dev_props[1024], nxt_el[10];
  memset(dev_props,0,1024);
  memset(nxt_el,0,10);

  // Form device properties cstring 
  for(it_props = props.begin();
      it_props != props.end();
      it_props++) {
    sprintf(nxt_el, "%" PRDPT "; ", *it_props);
    strcat(dev_props, nxt_el);
  }
  LOG_INFO("\tDevice props: %s", dev_props);
}

void elka::DeviceRoute::print_dev_route(
      std::vector<dev_id_t> &route) {
  std::vector<dev_id_t>::iterator it_route;
  char dev_route[1024], nxt_el[10];
  memset(dev_route,0,1024);
  memset(nxt_el,0,10);

  // Form device route cstring
  for(it_route = route.begin();
      it_route != route.end();
      it_route++) {
    sprintf(nxt_el, "-> %" PRDIT " ", *it_route);
    strcat(dev_route, nxt_el);
  }
  LOG_INFO("\tDevice route: %s", dev_route);
}

//-----------------CommPort Methods---------------------

elka::CommPort::CommPort(uint8_t port_n, uint8_t port_t,
    uint8_t buf_t, uint8_t size) {
  get_dev_id_t(&_id);

  // Add _id to _routing_table with empty route (size 0)
  // and no properties
  _routing_table[_id] = DeviceRoute();

  _port_num = port_n;

  _tx_buf = new SerialBuffer(_id,buf_t,size);
  _rx_buf = new SerialBuffer(_id,buf_t,size);

  _hw_state = HW_CTL_STOP;
  _sw_state = SW_CTL_NULL;

  _prev_hw_state = HW_CTL_NULL;
  _prev_sw_state = SW_CTL_NULL;

  // Set spektrum killswitch to channel 5 (back left)
  _spektrum_channel_kill = 4;
  // Set spektrum switch (switch to spektrum) to channel 6
  _spektrum_channel_switch = 5;

}

elka::CommPort::~CommPort() {
}

uint8_t elka::CommPort::get_state(bool hw) {
  if (hw) return _hw_state;
  else return _sw_state;
}

/*
uint8_t elka::CommPort::push_msg(
    dev_id_t &dst,
    uint8_t msg_type,
    uint8_t len,
    uint8_t *data) {
  elka::SerialBuffer *sb;
  struct elka_msg_id_s msg_id;
  dev_id_t my_id; 
  uint8_t port_num;

  // TODO what if device sending this message is deleted
  //      from routing table as message is being sent to you?
  // Find out if this device can reach the destination
  access_comm_port(msg_id.rcv_id, &my_id);
  if (my_id)
    port_num = _port_num_map.find(my_id)->second;
  else
    return MSG_NULL;

  // Check that port exists. This should always pass.
  // If not, this means that port was added incorrectly
  if (check_port(port_num) == PORT_NONE) {
    return MSG_FAILED;
  }

  return msg_type;
}
*/

uint8_t elka::CommPort::push_msg(
    elka_msg_s &elka_msg,
    bool tx) {
  elka::SerialBuffer *sb;
  struct elka_msg_id_s msg_id;

  get_elka_msg_id_attr(&msg_id, elka_msg.msg_id);

  // TODO what if device sending this message is deleted
  //      from routing table as message is being sent to you?
  if ((!broadcast_msg(msg_id.rcv_id) &&
       !check_route(msg_id.rcv_id))
      || initial_msg(elka_msg.msg_id,
                     elka_msg.msg_num,
                     elka_msg.num_retries)) {
    return MSG_NULL;
  }    

  if (tx) {
    sb = _tx_buf;

    /*
    LOG_INFO("in commport tx push\tmsg_num: %" PRIu16 " num_retries: %d",
      elka_msg.msg_num,
      elka_msg.num_retries);
      */

    return sb->push_msg(elka_msg);
  } else  {
    // Specify msg_num and num_retries
    sb = _rx_buf;

    /*
    LOG_INFO("in commport rx push\tmsg_num: %" PRIu16 " num_retries: %d",
      elka_msg.msg_num,
      elka_msg.num_retries);
      */

    return sb->push_msg(elka_msg.msg_id, elka_msg.data,
        elka_msg.msg_num, elka_msg.num_retries);
  }
}

uint8_t elka::CommPort::push_msg(
    elka_msg_ack_s &elka_msg,
    bool tx) {
  elka::SerialBuffer *sb;
  struct elka_msg_id_s msg_id;
  uint8_t data[1];

  get_elka_msg_id_attr(&msg_id, elka_msg.msg_id);

  // TODO what if device sending this message is deleted
  //      from routing table as message is being sent to you?
  if ((!broadcast_msg(msg_id.rcv_id) &&
       !check_route(msg_id.rcv_id))
      || initial_msg(elka_msg.msg_id,
                     elka_msg.msg_num,
                     elka_msg.num_retries))
    return MSG_NULL;

  if (tx) {
    sb = _tx_buf;

    /*
   LOG_INFO("in commport tx ack push\tmsg_num: %" PRIu16 " num_retries: %d",
      elka_msg.msg_num,
      elka_msg.num_retries);
      */
  } else  {
    // Specify msg_num and num_retries
    sb = _rx_buf;

    /*
    LOG_INFO("in commport rx ack push\tmsg_num: %" PRIu16 " num_retries: %d",
      elka_msg.msg_num,
      elka_msg.num_retries);
      */
  }

  memset(data,elka_msg.result,1);

  return sb->push_msg(elka_msg.msg_id, data,
      elka_msg.msg_num, elka_msg.num_retries);
}

uint8_t elka::CommPort::get_msg(
    elka_msg_s &elka_msg,
    elka_msg_ack_s &elka_msg_ack,
    bool tx) {
  elka::SerialBuffer *sb;

  if (tx) {
    sb = _tx_buf;
  } else {
    sb = _rx_buf;
  }

  return sb->get_msg(elka_msg, elka_msg_ack);
}

// Message numbers must be incremented upon message removal b/c
// messages are sorted upon adding into buffers
uint8_t elka::CommPort::remove_msg(
    elka_msg_s &elka_msg,
    elka_msg_ack_s &elka_msg_ack,
    bool tx) {
  elka::SerialBuffer *sb;

  if (tx) {
    sb = _tx_buf;
  } else {
    sb = _rx_buf;
  }

  return sb->remove_msg(elka_msg, elka_msg_ack);
}

uint8_t elka::CommPort::pop_msg(bool tx) {
  elka::SerialBuffer *sb;

  if (tx) {
    sb = _tx_buf;
  } else {
    sb = _rx_buf;
  }

  return sb->pop_msg();
}

void elka::CommPort::erase_msg(
    msg_id_t msg_id, uint16_t msg_num, bool tx) {
  elka::SerialBuffer *sb;

  if (tx) {
    sb = _tx_buf;
  } else {
    sb = _rx_buf;
  }

  sb->erase_msg(msg_id, msg_num);
}

uint8_t elka::CommPort::parse_routing_msg(
    elka_msg_s &elka_msg,
    struct elka_msg_id_s &msg_id,
    elka_msg_ack_s &elka_ack,
    elka_msg_s &ret_routing_msg) {

  //FIXME debugging
  //elka::CommPort::print_elka_route_msg(elka_msg);

  if (msg_id.type == MSG_ROUTE_DEV_PROPS) {
    std::vector<dev_prop_t> props;
    std::vector<dev_id_t> route;
    bool req_resp = elka_msg.data[0];
    uint8_t dev_props_len = elka_msg.data[1];

    // Determine device properties and add to _routing_table
    parse_dev_props(dev_props_len, &elka_msg.data[2], props);

    // Add self to route
    route.insert(route.begin(), _id);

    DeviceRoute dr = DeviceRoute(&route, &props);
    change_route(msg_id.snd_id, &dr);

    if (req_resp) {
      set_dev_props_msg(_id,
                        msg_id.snd_id, 
                        false,
                        ret_routing_msg);
      _tx_buf->push_msg(ret_routing_msg);
    }

  } else if (msg_id.type == MSG_ROUTE_REQUEST_HB) {
    //TODO form heartbeat message
    
  } else if (msg_id.type == MSG_ROUTE_HB) {
    //TODO reset heartbeat timer

  } else if (msg_id.type == MSG_ROUTE_CHANGED) {
    std::vector<dev_prop_t> props;
    std::vector<dev_id_t> route;
    uint16_t i = 0, j = 0;
    bool req_resp = elka_msg.data[0],
         dev_props_cmp_res;
    int8_t route_cmp_res;
    // num devices to delete
    uint8_t devs_idx = elka_msg.data[1];
    uint8_t dev_props_len, dev_route_len;
    dev_id_t nxt_dev, snd_id;

    get_elka_msg_id_attr(&snd_id, NULL, NULL, NULL, NULL,
        elka_msg.msg_id);

    // Delete devices
    for (; i < 2*devs_idx; i+=2) {
      nxt_dev = (elka_msg.data[i] << 8) | elka_msg.data[i+1];
      change_route(nxt_dev, NULL);
    }
    
    // Get num devices to change
    // FIXME not necessarily true that message ends up smaller than
    //       MAX_MSG_LEN
    devs_idx += elka_msg.data[i];
    for (; i < 2*devs_idx && j < MAX_MSG_LEN; i+=2) {
      j++;
      nxt_dev = (elka_msg.data[j] << 8) | elka_msg.data[j+1];
      j+=2;
      dev_route_len = elka_msg.data[j];
      j++;
      j += parse_dev_route(dev_route_len, &elka_msg.data[j], route);
      dev_props_len = elka_msg.data[j]; 
      j+=1;
      j += parse_dev_props(dev_props_len, &elka_msg.data[j], props);

      // Add self to route
      route.insert(route.begin(), _id);

      // Compare routes and props in order to decide what to change
      route_cmp_res = route_cmp(nxt_dev, route);
      dev_props_cmp_res = dev_props_cmp(nxt_dev, props);     
      
      // Change route to nxt_dev if it contains snd_id
      // Else choose better route
      // If changing route,
      // change device properties if they have changed
      if (check_route_contains(nxt_dev, snd_id)) {
        if (dev_props_cmp_res) {
          DeviceRoute dr = DeviceRoute(&route, &props);
          change_route(nxt_dev, &dr);
        } else {
          DeviceRoute dr = DeviceRoute(&route, NULL);
          change_route(nxt_dev, &dr);
        }
      } else if (route_cmp_res < 0) {
        if (dev_props_cmp_res) {
          DeviceRoute dr = DeviceRoute(&route, &props);
          change_route(nxt_dev, &dr);
        } else {
          DeviceRoute dr = DeviceRoute(&route, NULL);
          change_route(nxt_dev, &dr);
        }
      }
    }

    elka_ack.msg_num = elka_msg.msg_num;
    
    get_elka_msg_id(&elka_ack.msg_id,
        msg_id.rcv_id, msg_id.snd_id,
        _snd_params,
        MSG_ACK, MSG_ACK_LENGTH);

    elka_ack.result = MSG_UNSUPPORTED;

    // Set and push response message if requested
    if (req_resp) {
      set_route_changed_msg(_id,
                            msg_id.snd_id,
                            false,
                            ret_routing_msg);
      _tx_buf->push_msg(ret_routing_msg);
    }
  } else if (msg_id.type == MSG_ROUTE_TABLE) {
    std::vector<dev_prop_t> props;
    std::vector<dev_id_t> route;
    uint16_t i, j=0;
    bool req_resp = elka_msg.data[0], dev_props_cmp_res;
    // Num devices in route table
    uint8_t devs_idx = elka_msg.data[1];
    uint8_t dev_props_len, dev_route_len;
    dev_id_t nxt_dev, snd_id;
    int8_t route_cmp_res;

    // Get num devices to change
    // FIXME not necessarily true that message ends up smaller than
    //       MAX_MSG_LEN
    j=1;
    for (i=0; i < devs_idx && j < MAX_MSG_LEN; i++) {
      j++;
      nxt_dev = (elka_msg.data[j] << 8) | elka_msg.data[j+1];
      j+=2;
      dev_route_len = elka_msg.data[j];
      j++;
      j += parse_dev_route(dev_route_len,
                           &elka_msg.data[j],
                           route);
      dev_props_len = elka_msg.data[j]; 
      j++;
      j += parse_dev_props(dev_props_len,
                           &elka_msg.data[j],
                           props);

      // Add self to route
      route.insert(route.begin(), _id);

      // Compare routes and props in order to decide what to change
      route_cmp_res = route_cmp(nxt_dev, route);
      dev_props_cmp_res = dev_props_cmp(nxt_dev, props);     
      
      // Change route to nxt_dev if it contains snd_id
      // Else choose better route
      // If changing route,
      // change device properties if they have changed
      if (check_route_contains(nxt_dev, snd_id)) {
        if (dev_props_cmp_res) {
          DeviceRoute dr = DeviceRoute(&route, &props);
          change_route(nxt_dev, &dr);
        } else {
          DeviceRoute dr = DeviceRoute(&route, NULL);
          change_route(nxt_dev, &dr);
        }
      } else if (route_cmp_res < 0) {
        if (dev_props_cmp_res) {
          DeviceRoute dr = DeviceRoute(&route, &props);
          change_route(nxt_dev, &dr);
        } else {
          DeviceRoute dr = DeviceRoute(&route, NULL);
          change_route(nxt_dev, &dr);
        }
      }
    }
 
    elka_ack.msg_num = elka_msg.msg_num;
    
    get_elka_msg_id(&elka_ack.msg_id,
        msg_id.rcv_id, msg_id.snd_id,
        _snd_params,
        MSG_ACK, MSG_ACK_LENGTH);

    elka_ack.result = MSG_ACCEPTED;
    
    // Set and push response message if requested
    if (req_resp) {
      set_route_changed_msg(_id,
                            msg_id.snd_id,
                            false,
                            ret_routing_msg);
      _tx_buf->push_msg(ret_routing_msg);
    }
  }

  return msg_id.type;
}

int8_t elka::CommPort::route_cmp(
    dev_id_t &d,
    std::vector<dev_id_t> &r1) {
  if (check_route(d)) {
    return elka::DeviceRoute::route_cmp(
        &_routing_table[d]._route,
        &r1);
  } else
    return elka::DeviceRoute::route_cmp(
        NULL,
        &r1);
}

bool elka::CommPort::dev_props_cmp(
    dev_id_t &d,
    std::vector<dev_prop_t> &p1) {
  if (check_route(d))
    return elka::DeviceRoute::dev_props_cmp(
        &_routing_table[d]._props,
        &p1);
  else
    return elka::DeviceRoute::dev_props_cmp(
        NULL,
        &p1);
}

bool elka::CommPort::check_route_contains(dev_id_t &dst, dev_id_t &el) {
  if (check_route(dst)) {
    std::vector<dev_id_t>::iterator it =
      _routing_table[dst]._route.begin();
    std::vector<dev_id_t>::iterator end =
      _routing_table[dst]._route.end();
    for (; it != end; it++) {
      if (!cmp_dev_id_t(*it, el))
        return true;
    }
  }

  return false;
}

//FIXME
bool elka::CommPort::check_route(dev_id_t &d) {
  std::map<dev_id_t,DeviceRoute, dev_id_tCmp>::iterator iter =
    _routing_table.find(d);
  return (iter != _routing_table.end() &&
          iter->second.check_alive());
}

void elka::CommPort::change_route(
    dev_id_t &dev,
    DeviceRoute *dr) {
  if (!dr) { // Case remove route
    _routing_table.erase(dev);
  } else { // Case add or change route
    _routing_table[dev] = DeviceRoute(dr);
  }
}

void elka::CommPort::change_route(dev_id_t &dev,
                  std::vector<dev_id_t> *route,
                  std::vector<dev_prop_t> *props) {
  if (!route && !props) {
    _routing_table.erase(dev);
  } else {
    _routing_table[dev] = DeviceRoute(route,props);
  }
}

bool elka::CommPort::get_next_dev(dev_id_t &end, dev_id_t *nxt) {
  // If searching for self, return true
  if (!cmp_dev_id_t(_id,end))
    return true;
  else if (_routing_table.find(end) != _routing_table.end() &&
      _routing_table.size() > 1) {
    // Get second element bc first element is self
    *nxt = _routing_table[end]._route[1];
  } else
    // end device not found
    nxt = NULL;

  return false;
}

int16_t elka::CommPort::parse_route_table(
    uint8_t len,
    uint8_t *data,
    std::map<dev_id_t, DeviceRoute, dev_id_tCmp> &routing_table) {
  std::vector<dev_prop_t> props;
  std::vector<dev_id_t> route;
  uint16_t i, j=0;
  dev_id_t nxt_dev;
  uint8_t dev_props_len, dev_route_len;

  for (i=0; i < len && j < MAX_MSG_LEN; i++) {
    nxt_dev = (*(data+j) << 8) | *(data+j+1);
    j+=2;
    dev_route_len = *(data+j);
    j++;
    j += parse_dev_route(dev_route_len,
                         (data+j),
                         route);
    dev_props_len = *(data+j); 
    j++;
    j += parse_dev_props(dev_props_len,
                         (data+j),
                         props);
    routing_table[nxt_dev] = DeviceRoute(&route,&props);
  }

  return j;
}

// FIXME must filter down device props from props read from data
int16_t elka::CommPort::parse_dev_props(
    uint8_t len,
    uint8_t *data,
    std::vector<dev_prop_t> &props) {
  uint16_t i = 0;
  dev_prop_t nxt_prop;
  while (i < len) {
    nxt_prop = *(data + i);
    props.push_back(nxt_prop);
    i++;
  }
  std::sort(props.begin(), props.end());

  return i;
}

int16_t elka::CommPort::parse_dev_route(
    uint8_t len,
    uint8_t *data,
    std::vector<dev_id_t> &route) {
  uint16_t i = 0;
  dev_id_t nxt_id;
  while (i < 2*len) {
    nxt_id = (*(data+i) << 8) | *(data+i+1);
    route.push_back(nxt_id);
    i += 2;
  }

  return i;
}

void elka::CommPort::set_dev_props_msg(
    dev_id_t snd_id,
    dev_id_t rcv_id,
    bool req_resp,
    elka_msg_s &ret_routing_msg) {
  uint8_t len = 0, i = 0;
  std::vector<dev_prop_t> dev_props = _routing_table[_id]._props;

  ret_routing_msg.data[i++] = req_resp;
  ret_routing_msg.data[i++] = dev_props.size();

  len++;

  for (std::vector<dev_prop_t>::iterator it = dev_props.begin();
       it != dev_props.end(); it++) {
    ret_routing_msg.data[i++] = *it;
  }
  
  len += i - 1;

  get_elka_msg_id(&ret_routing_msg.msg_id,
    snd_id, rcv_id, _snd_params,
    MSG_ROUTE_DEV_PROPS, len);

  // Set ret_routing_msg.msg_num and ret_routing_msg.num_retries
  // to 0 so that message is assigned the next msg number upon
  // removal from buffer
  ret_routing_msg.msg_num = 0;
  ret_routing_msg.num_retries = 0;
}

//FIXME need some internal caching/mapping of which
//      routes are changed
//      This is not useful right now.
void elka::CommPort::set_route_changed_msg(
    dev_id_t &snd_id,
    dev_id_t &rcv_id,
    bool req_resp,
    elka_msg_s &ret_routing_msg) {
  uint8_t len=0;

  get_elka_msg_id(&ret_routing_msg.msg_id,
    snd_id, rcv_id, _snd_params,
    MSG_ROUTE_CHANGED, len);
}

void elka::CommPort::set_route_table_msg(
    dev_id_t &snd_id,
    dev_id_t &rcv_id,
    bool req_resp,
    elka_msg_s &ret_routing_msg) {
  uint8_t i = 0;

  ret_routing_msg.data[i++] = req_resp;
  ret_routing_msg.data[i++] = _routing_table.size(); 
  std::map<dev_id_t,DeviceRoute, dev_id_tCmp>::iterator iter =
    _routing_table.begin();
  for (; iter != _routing_table.end(); iter++) {
    i += set_dev_msg_part(iter->first,
                          &ret_routing_msg.data[i]);
  }

  get_elka_msg_id(&ret_routing_msg.msg_id,
    snd_id, rcv_id, _snd_params,
    MSG_ROUTE_CHANGED, i);
}

uint16_t elka::CommPort::set_dev_msg_part(
    dev_id_t dst,
    uint8_t *data) {
  uint16_t i = 2;
  uint8_t route_len, props_len;
  std::vector<dev_id_t>::iterator curr_route_dev,
                                  curr_route_end;
  std::vector<dev_prop_t>::iterator curr_dev_prop,
                                    curr_prop_end;

  if (check_route(dst)) {
    // Serialize dst dev id
    *data = (dst >> 8);
    *(data + 1) = (dst & 0xff);
    
    // Serialize dev route len and dev route
    route_len = _routing_table[dst]._route.size();
    *(data + i) = route_len;
    i++;
    curr_route_dev = _routing_table[dst]._route.begin();
    curr_route_end = _routing_table[dst]._route.end();
    // FIXME not necessarily true that message ends up smaller than
    //       MAX_MSG_LEN
    while (curr_route_dev != curr_route_end &&
        i < MAX_MSG_LEN) {
      *(data + i) = *curr_route_dev >> 8;
      *(data + i + 1) = *curr_route_dev & 0xff;
      i += 2;
      curr_route_dev++;
    }

    // Serialize dev props len and dev props
    props_len = _routing_table[dst]._props.size();
    *(data + i) = props_len;
    i++;

    // FIXME not necessarily true that message ends up smaller than
    //       MAX_MSG_LEN
    curr_dev_prop = _routing_table[dst]._props.begin();
    curr_prop_end = _routing_table[dst]._props.end();
    while (curr_dev_prop != curr_prop_end &&
        i < MAX_MSG_LEN) {
      *(data + i) = *curr_dev_prop;
      i++;
      curr_dev_prop++;
    }
    
  } else
    return 0;

  return i;
}
bool elka::CommPort::check_dev_compatible(
    uint8_t msg_type,
    dev_id_t dst) {
  // Check route to see that dst can be reached
  // and that dst is not this device
  if (!check_route(dst) || !cmp_dev_id_t(_id, dst))
    return false;

  //TODO set static variable to map device properties
  //to message types
  switch (msg_type) {
    case MSG_NULL:
      return false;
      break;
    case MSG_FAILED:
    case MSG_ACK:
    case MSG_ROUTE_DEV_PROPS:
    case MSG_ROUTE_REQUEST_HB:
    case MSG_ROUTE_HB:
    case MSG_ROUTE_CHANGED:
    case MSG_ROUTE_TABLE:
      return true;
      break;
    case MSG_MOTOR_CMD:
      return (_routing_table[_id].check_prop(
                DEV_PROP_SPIN_MOTORS) &&
              _routing_table[dst].check_prop(
                DEV_PROP_HAS_MOTORS));
      break;
    case MSG_PORT_CTL:
      return (_routing_table[_id].check_prop(
                DEV_PROP_TRANSMISSION_CTL));
      break;
    case MSG_ELKA_CTL:
      return (_routing_table[_id].check_prop(
                DEV_PROP_TRANSMISSION_CTL));
      break;
    default:
      return false;
  }
}

void elka::CommPort::print_elka_route_msg(elka_msg_s &elka_msg) {
  std::map<dev_id_t, DeviceRoute, dev_id_tCmp> routing_table;
  std::vector<dev_prop_t> props;
  uint8_t msg_type, num_els;
  bool req_resp;

  get_elka_msg_id_attr(NULL, NULL, NULL, &msg_type, NULL,
      elka_msg.msg_id);
  
  LOG_INFO("-----ELKA route message-----\n\
# %" PRIu16 ", retries: %" PRIu16 "",
           elka_msg.msg_num, elka_msg.num_retries);
  print_elka_msg_id(elka_msg.msg_id);

  switch(msg_type) {
    case MSG_ROUTE_TABLE:
      LOG_INFO("\tRouting table message");
      num_els = elka_msg.data[0];
      req_resp = elka_msg.data[1];
      parse_route_table(
          num_els,
          &(elka_msg.data[2]), routing_table);
      LOG_INFO("\tRequesting response: %d", req_resp);
      print_routing_table(routing_table);
      break;
    case MSG_ROUTE_DEV_PROPS:
      LOG_INFO("\tDev props message");
      req_resp = elka_msg.data[0];
      num_els = elka_msg.data[1];
      LOG_INFO("\tRequesting response: %d", req_resp);
      parse_dev_props(num_els,
          &(elka_msg.data[2]),
          props);
      elka::DeviceRoute::print_dev_props(props);
    default:
      break;
  }

  LOG_INFO("----- End route message\n");
}

void elka::CommPort::print_routing_table(
    std::map<dev_id_t,DeviceRoute, dev_id_tCmp> &routing_table) {
  std::map<dev_id_t,DeviceRoute, dev_id_tCmp>::iterator
      it_route_table = routing_table.begin();
  for(; it_route_table != routing_table.end(); it_route_table++) {
    LOG_INFO("\tDevice id: %d",it_route_table->first);

    elka::DeviceRoute::print_dev_route(
        it_route_table->second._route);

    elka::DeviceRoute::print_dev_props(
        it_route_table->second._props);
  }
}

void elka::CommPort::print_elka_ctl_msg(elka_msg_s &elka_msg) {
  char msg_type_name[42], request[42], state[42];
  uint8_t msg_type;

  get_elka_msg_id_attr(NULL, NULL, NULL, &msg_type, NULL,
      elka_msg.msg_id);
  
  PX4_INFO("-----ELKA ctl message-----\n\
# %" PRIu16 ", retries: %" PRIu16 "",
           elka_msg.msg_num, elka_msg.num_retries);
  print_elka_msg_id(elka_msg.msg_id);

  switch(msg_type) {
    case MSG_PORT_CTL:
      sprintf(msg_type_name, "PORT_CTL");

        switch(elka_msg.data[1]) {
          case HW_CTL_NULL:
            sprintf(state, "NULL");
            break;
          case HW_CTL_FAILED:
            sprintf(state, "FAILED");
            break;
          case HW_CTL_START:
            sprintf(state, "START");
            break;
          case HW_CTL_STOP:
            sprintf(state, "STOP");
            break;
          case HW_CTL_PAUSE:
            sprintf(state, "PAUSE");
            break;
          case HW_CTL_RESUME:
            sprintf(state, "RESUME");
            break;
          default:
            break;
        }

      break;
    case MSG_ELKA_CTL:
      sprintf(msg_type_name, "ELKA_CTL");

        switch(elka_msg.data[1]) {
          case SW_CTL_NULL:
            sprintf(state, "NULL");
            break;
          case SW_CTL_FAILED:
            sprintf(state, "FAILED");
            break;
          case SW_CTL_REMOTE:
            sprintf(state, "REMOTE");
            break;
          case SW_CTL_SPEKTRUM:
            sprintf(state, "SPEKTRUM");
            break;
          case SW_CTL_AUTOPILOT:
            sprintf(state, "AUTOPILOT");
            break;
          default:
            break;
        }

      break;
    default:
      PX4_INFO("------Not control message type-------");
      return;
      break;
  }

  switch(elka_msg.data[0]) {
    case true:
      sprintf(request, "request");
      break;
    case false:
      sprintf(request, "statement");
      break;
    default:
      break;
  }

  PX4_INFO("%s %s %s", msg_type_name, request, state);

  PX4_INFO("----- End ctl message\n");
}

void elka::CommPort::print_elka_state() {
  char hw_state[42], sw_state[42];
  switch(_hw_state) {
     case HW_CTL_NULL:
      sprintf(hw_state, "NULL");
      break;
    case HW_CTL_FAILED:
      sprintf(hw_state, "FAILED");
      break;
    case HW_CTL_KILL:
      sprintf(hw_state, "KILL");
      break;
    case HW_CTL_START:
      sprintf(hw_state, "START");
      break;
    case HW_CTL_STOP:
      sprintf(hw_state, "STOP");
      break;
    case HW_CTL_PAUSE:
      sprintf(hw_state, "PAUSE");
      break;
    case HW_CTL_RESUME:
      sprintf(hw_state, "RESUME");
      break;
    default:
      break;
  }

  switch(_sw_state) {
    case SW_CTL_NULL:
      sprintf(sw_state, "NULL");
      break;
    case SW_CTL_FAILED:
      sprintf(sw_state, "FAILED");
      break;
    case SW_CTL_KILL:
      sprintf(sw_state, "KILL");
      break;
    case SW_CTL_REMOTE:
      sprintf(sw_state, "REMOTE");
      break;
    case SW_CTL_SPEKTRUM:
      sprintf(sw_state, "SPEKTRUM");
      break;
    case SW_CTL_AUTOPILOT:
      sprintf(sw_state, "AUTOPILOT");
      break;
    default:
      break;
  }

  PX4_INFO("ELKA HW state: %s\tSW state: %s",
      hw_state, sw_state);
}
