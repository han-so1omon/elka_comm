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
    uint8_t *data) {
  uint8_t msg_len;
  _msg_id = msg_id;
  get_elka_msg_id_attr(NULL,NULL,NULL,NULL,
      &msg_len,msg_id);
  _push_msg_num = push_msg_num;
  _rmv_msg_num = rmv_msg_num;
  memcpy(_data, data, msg_len);
  _num_retries = 0;
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
}

elka::SerialBuffer::~SerialBuffer() {
}

uint8_t elka::SerialBuffer::check_recent_acks(uint16_t msg_num) {
  int idx = cb_bin_search(msg_num, _recent_acks,
    _recent_acks_end, _recent_acks_len, RECENT_ACKS_LEN);
  if (idx < 0) return elka_msg_ack_s::ACK_FAILED;
  else return elka_msg_ack_s::ACK_NULL;
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
  uint8_t msg_type;
  // Check if message is already in the buffer.
  // If so just update num_retries
  // This prevents redundant messages from being added
  // FIXME
  /*
  ElkaBufferMsg *ebm;
  if ((ebm = get_buffer_msg(msg_num))) {
    ebm->_num_retries = num_retries;
  } else {
  */

    if (_type == ARRAY) {
      _buffer.push_back(ElkaBufferMsg(
            msg_id, _push_msg_num++, rmv_msg_num, data));
      if (_buffer.size() > _max_size) {
        _buffer.pop_back();
      }
    } else if (_type == PRIORITY_QUEUE) {
      _buffer.push_back(ElkaBufferMsg(
            msg_id, _push_msg_num++, rmv_msg_num, data));
      std::push_heap(_buffer.begin(), _buffer.end(), _comp);
      // Priority queue is fixed size
      if (_buffer.size() > _max_size) {
        std::pop_heap(_buffer.begin(), _buffer.end(), _comp);
        _buffer.pop_back();
      }
    } else {
      return MSG_FAILED;
    }

    // Set _push_msg_num to follow msg_num
    // for next time that a message is pushed
    // Useful for sorting in the buffer
    // FIXME This may cause a lot of redundant message numbers
    //       It should be the case that no message has the same
    //       msg_num and msg_id (except for msg_num rollover)
    // _push_msg_num = msg_num + 1;
  //}

  get_elka_msg_id_attr(NULL, NULL, NULL, &msg_type, NULL,
      msg_id);
  return msg_type;
}

uint8_t elka::SerialBuffer::get_msg(
    elka_msg_s &elka_msg,
    elka_msg_ack_s &elka_msg_ack,
    bool tx) {
  dev_id_t snd_id; 
  uint8_t ret, msg_len, front_msg_type;
  // Ensure that buffer isn't empty and that front of buffer is not ACK
  if (_buffer.empty()) return MSG_NULL;

  // Get message type for message at front of buffer
  front_msg_type = buffer_front_type();

  // Pop messages that have been tried too many times
  while (_buffer.front()._num_retries > MAX_NUM_RETRIES)
    pop_msg();

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
    //if (tx)
      _buffer.front()._rmv_msg_num = _rmv_msg_num++;
    /*
    else
      _buffer.front()._rmv_msg_num = _buffer.front()._push_msg_num;
      */
  }

  if (front_msg_type != MSG_ACK) {
    elka_msg.msg_num = _buffer.front()._rmv_msg_num;
    elka_msg.num_retries = _buffer.front()._num_retries++;
    memcpy(elka_msg.data, _buffer.front()._data, msg_len);
  } else {
    elka_msg_ack.msg_num = _buffer.front()._rmv_msg_num;
    elka_msg_ack.num_retries = _buffer.front()._num_retries++;
    // This will return MSG_NULL if the message is not of MSG_ACK type
    elka_msg_ack.result = _buffer.front().get_result();
  }

  return ret;
}

//FIXME fix qurt side
elka::ElkaBufferMsg *elka::SerialBuffer::get_buffer_msg(
    msg_id_t msg_id, uint16_t msg_num, bool tx) {
//#if defined(__PX4_QURT)
  dev_id_t buf_snd_id, msg_snd_id;

  if (tx)
    get_elka_msg_id_attr(&msg_snd_id, NULL, NULL, NULL, NULL, msg_id);
  else
    get_elka_msg_id_attr(NULL, &msg_snd_id, NULL, NULL, NULL, msg_id);

  for (std::vector<ElkaBufferMsg>::iterator it = _buffer.begin();
       it != _buffer.end(); it++) {
    if (it->_rmv_msg_num == msg_num) {
      get_elka_msg_id_attr(&buf_snd_id,
          NULL, NULL, NULL, NULL, it->_msg_id);
      if (!cmp_dev_id_t(buf_snd_id, msg_snd_id))
        return &(*it);
    }
  }
  return NULL;

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

// TODO necessary to set time stamps?
uint8_t elka::SerialBuffer::remove_msg(
    elka_msg_s &elka_msg,
    elka_msg_ack_s &elka_msg_ack,
    bool tx) {
  uint8_t ret;

  ret = get_msg(elka_msg,elka_msg_ack,tx);
  if (pop_msg() != MSG_FAILED)
    return ret;
  else
    return MSG_FAILED;
}

uint8_t elka::SerialBuffer::pop_msg() {
  uint8_t ret;
  // Ensure that buffer isn't empty
  if (_buffer.empty()) return MSG_NULL;

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

  return ret;
}

//FIXME fix qurt side
void elka::SerialBuffer::erase_msg(msg_id_t msg_id, uint16_t msg_num,
    bool tx) {
  dev_id_t buf_snd_id, msg_snd_id;

  if (tx)
    get_elka_msg_id_attr(&msg_snd_id, NULL, NULL, NULL, NULL, msg_id);
  else
    get_elka_msg_id_attr(NULL, &msg_snd_id, NULL, NULL, NULL, msg_id);

//#if defined(__PX4_QURT)
  // Search beginning at end of array bc this should be closer to
  // msg_num in most expected use cases
  for (std::vector<ElkaBufferMsg>::reverse_iterator it =
       _buffer.rbegin();
       it != _buffer.rend(); it++) {
    if (it->_rmv_msg_num == msg_num) {
      get_elka_msg_id_attr(&buf_snd_id,
          NULL, NULL, NULL, NULL, it->_msg_id);
      if (!cmp_dev_id_t(buf_snd_id, msg_snd_id)) {
        LOG_INFO("erasing msg here");
        _buffer.erase(--(it.base()));
        break;
      }
    }
  }

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
  return ((msg_priority(p_type) < msg_priority(q_type)) &&
          p._push_msg_num < q._push_msg_num);
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

  _state = STATE_STOP;
}

elka::CommPort::~CommPort() {
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

uint8_t elka::CommPort::push_msg(elka_msg_s &elka_msg, bool tx) {
  elka::SerialBuffer *sb;
  struct elka_msg_id_s msg_id;

  get_elka_msg_id_attr(&msg_id, elka_msg.msg_id);

  // TODO what if device sending this message is deleted
  //      from routing table as message is being sent to you?
  if ((!broadcast_msg(msg_id.rcv_id) && !check_route(msg_id.rcv_id))
      || initial_msg(elka_msg.msg_id))
    return MSG_NULL;

  if (tx) {
    sb = _tx_buf;
    return sb->push_msg(elka_msg);
  } else  {
    // Specify msg_num and num_retries
    sb = _rx_buf;
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
  if ((!broadcast_msg(msg_id.rcv_id) && !check_route(msg_id.rcv_id))
      || initial_msg(elka_msg.msg_id))
    return MSG_NULL;

  if (tx) {
    sb = _tx_buf;
  } else  {
    // Specify msg_num and num_retries
    sb = _rx_buf;
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

  return sb->get_msg(elka_msg, elka_msg_ack, tx);
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

  return sb->remove_msg(elka_msg, elka_msg_ack, tx);
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

uint8_t elka::CommPort::parse_routing_msg(
    elka_msg_s &elka_msg,
    struct elka_msg_id_s &msg_id,
    elka_msg_ack_s &elka_ack,
    elka_msg_s &ret_routing_msg) {

  //FIXME debugging
  elka::CommPort::print_elka_route_msg(elka_msg);

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

    elka_ack.result = elka_msg_ack_s::ACK_UNSUPPORTED;

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

    elka_ack.result = elka_msg_ack_s::ACK_ACCEPTED;
    
    // Set and push response message if requested
    if (req_resp) {
      set_route_changed_msg(_id,
                            msg_id.snd_id,
                            false,
                            ret_routing_msg);
      _tx_buf->push_msg(ret_routing_msg);
    }
  }

  /*
  elka::CommPort::print_elka_route_msg(elka_msg);
  */

  elka::CommPort::print_routing_table(_routing_table);

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
  if (!check_route(dst))
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
      return (check_route(_id) &&
              check_route(dst) &&
              _routing_table[_id].check_prop(
                DEV_PROP_SPIN_MOTORS) &&
              _routing_table[dst].check_prop(
                DEV_PROP_HAS_MOTORS));
      break;
    case MSG_PORT_CTL:
      return (check_route(_id) &&
              check_route(dst) &&
              _routing_table[_id].check_prop(
                DEV_PROP_TRANSMISSION_CTL));
      break;
    case MSG_ELKA_CTL:
      return (check_route(_id) &&
              check_route(dst) &&
              _routing_table[_id].check_prop(
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
  
  LOG_INFO("----- ELKA route message");
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
      LOG_INFO("\tRouting table message");
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

#if defined(__ELKA_UBUNTU) 

/*
// Define device properties type
typedef uint8_t dev_prop_t;

// Define msg_id type
typedef uint64_t msg_id_t;
*/

#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

//TODO Define dev_id_t, dev_prop_t, msg_id_t.
//     They are returns or inputs to useful functions,
//     so it is necessary to have access to their conversions

//NOTE Important! Must define custom type casters in every
//     compilation unit of the Python extension module!
//     Otherwise, undefined behavior can ensue.
namespace pybind11 { namespace detail {
	template <> struct type_caster<dev_id_t> {
	public:
		/**
		 * This macro establishes the name 'dev_id_t' in
		 * function signatures
		 */
		PYBIND11_TYPE_CASTER(dev_id_t, _("dev_id_t"));

		/**
		 * Conversion part 1 (Python->C++): convert a PyObject into a dev_id_t 
		 * instance or return false upon failure. The second argument
		 * indicates whether implicit conversions should be applied.
		 */
		bool load(handle src, bool) {
			/* Extract PyObject from handle */
			PyObject *source = src.ptr();
			/* Try converting into a Python integer value */
			PyObject *tmp = PyNumber_Long(src.ptr());
			if (!tmp)
					return false;
			/* Now try to convert into a C++ int */
			value = (dev_id_t)PyInt_AsUnsignedLongMask(tmp);
			Py_DECREF(tmp);
			/* Ensure return code was OK (to avoid out-of-range errors etc) */
			return !(value == -1 && !PyErr_Occurred());
		}

		/**
		 * Conversion part 2 (C++ -> Python): convert an dev_id_t instance into
		 * a Python object. The second and third arguments are used to
		 * indicate the return value policy and parent object (for
		 * ``return_value_policy::reference_internal``) and are generally
		 * ignored by implicit casters.
		 */
		static handle cast(dev_id_t src, return_value_policy /* policy */, handle /* parent */) {
			return PyLong_FromUnsignedLong((unsigned long) src);
		}
	};

	template <> struct type_caster<dev_prop_t> {
	public:
		/**
		 * This macro establishes the name 'dev_prop_t' in
		 * function signatures
		 */
		PYBIND11_TYPE_CASTER(dev_prop_t, _("dev_prop_t"));

		bool load(handle src, bool convert) {
			PyObject *source = src.ptr();
			PyObject *tmp = PyNumber_Long(src.ptr());
			if (!tmp)
					return false;
			value = (dev_prop_t)PyInt_AsUnsignedLongMask(tmp);
			Py_DECREF(tmp);
			return !(value == -1 && !PyErr_Occurred());
		}

		static handle cast(dev_prop_t src, return_value_policy /* policy */, handle /* parent */) {
			return PyLong_FromUnsignedLong((unsigned long) src);
		}
	};

	template <> struct type_caster<msg_id_t> {
	public:
		/**
		 * This macro establishes the name 'msg_id_t' in
		 * function signatures
		 */
		PYBIND11_TYPE_CASTER(msg_id_t, _("msg_id_t"));

		bool load(handle src, bool convert) {
			/* Extract PyObject from handle */
			PyObject *source = src.ptr();
			PyObject *tmp = PyNumber_Long(src.ptr());
			if (!tmp)
					return false;
			value = (msg_id_t)PyInt_AsUnsignedLongLongMask(tmp);
			
			return !(value == -1 && !PyErr_Occurred());
		}

		static handle cast(msg_id_t src, return_value_policy /* policy */, handle /* parent */) {
			return PyLong_FromUnsignedLongLong((unsigned long long) src);
		}
	};

		/**
	template <> struct type_caster<elka_msg_s> {
	public:
		 * This macro establishes the name 'elka_msg_s' in
		 * function signatures and declares local variables
		 *	uint64 msg_id
		 *	uint16 msg_num
		 *	uint8 num_retries
		 *	uint8[256] data
		PYBIND11_TYPE_CASTER(elka_msg_s, _("elka_msg_s"));

		bool load(handle src, bool convert) {
			if (!src)
				return false;
			if (src.is_none()) {
				value = {};
				return true;
			}
	
			tuple args(src, true);
			if (args.check()) {
				if (len(args) < 4)
					return false;
				uint8_t len;
				value.msg_id = args[0].cast<unsigned long long>();
				value.msg_num = args[1].cast<unsigned int>();
				value.num_retries = args[2].cast<unsigned int>();

				get_elka_msg_id_attr(NULL,NULL,NULL,NULL, &len,
														 value.msg_id);

				//memcpy(value.data, args[3].cast<uint8_t*>(), len);
				return true;
			}

			return caster.load(src, convert);
		}

		static handle cast(const elka_msg_s &src,
											 return_value_policy policy,
											 handle parent) {
			return type_caster_base<elka_msg_s>::cast(&src, policy, parent);
		}

	private:
		type_caster_generic caster = typeid(elka_msg_s);

	};
		 */

}} // namespace pybind11::detail

typedef std::vector<dev_id_t> DevIdVector;
typedef std::vector<dev_prop_t> DevPropVector;
typedef std::map<dev_id_t, elka::DeviceRoute,dev_id_tCmp>
					RouteMap;

elka::DeviceRoute &route_map_getitem(RouteMap &m, dev_id_t k) {
	RouteMap::iterator it = m.find(k);
	if (it != m.end()) {
		return it->second;
	} else {
		throw py::index_error("No element %" PRDIT " in map");
	}
}

uint8_t *get_elka_msg_data(elka_msg_s &elka_msg) {
	return &elka_msg.data[0];
}

PYBIND11_MAKE_OPAQUE(DevIdVector);
PYBIND11_MAKE_OPAQUE(DevPropVector);
PYBIND11_MAKE_OPAQUE(RouteMap);

PYBIND11_MODULE(elka_comm__common, m) {
	m.def("cmp_dev_id_t", &cmp_dev_id_t);

	py::class_<DevIdVector>(m, "DevIdVector")
		.def(py::init<>())
		.def("__init__",
				 [](DevIdVector &v, const DevIdVector &c) {
						new (&v) DevIdVector(c);
				 }
		)
		.def("__iter__",
				 [](DevIdVector &v) {
						return py::make_iterator(v.begin(), v.end());
				 }, py::keep_alive<0,1>())
		.def("__len__", [](const DevIdVector &v)
											{return v.size(); })
		.def("begin", (DevIdVector::iterator (DevIdVector::*)
									 ()) &DevIdVector::begin)
		.def("end", (DevIdVector::iterator (DevIdVector::*)
									 ()) &DevIdVector::end)
		.def("insert", (DevIdVector::iterator (DevIdVector::*)
									 		(DevIdVector::iterator,
											 const dev_id_t &)) &DevIdVector::insert);

	py::class_<DevPropVector>(m, "DevPropVector")
		.def(py::init<>())
		.def("__init__",
				 [](DevPropVector &v, const DevPropVector &c) {
						new (&v) DevPropVector(c);
				 }
		)
		.def("__iter__",
				 [](DevPropVector &v) {
						return py::make_iterator(v.begin(), v.end());
				 }, py::keep_alive<0,1>())
		.def("__len__", [](const DevPropVector &v)
											{return v.size(); })
		.def("begin", (DevPropVector::iterator (DevPropVector::*)
									 ()) &DevPropVector::begin)
		.def("end", (DevPropVector::iterator (DevPropVector::*)
									 ()) &DevPropVector::end)
		.def("insert", (DevPropVector::iterator (DevPropVector::*)
									 		(DevPropVector::iterator,
											 const dev_prop_t &))
											&DevPropVector::insert);

	py::class_<RouteMap>(m, "RouteMap")
		.def(py::init<>())
		.def("__init__",
				 [](RouteMap &v, const RouteMap &c) {
						new (&v) RouteMap(c);
				 }
		)
		.def("__getitem__", &route_map_getitem,
				 py::is_operator(),
				 py::return_value_policy::reference)
		.def("__setitem__",
				 [](RouteMap &m,
					  dev_id_t k,
					  elka::DeviceRoute v) {
					 m[k] = v;
				 },
				 py::is_operator())
		.def("__iter__",
				 [](RouteMap &m) {
				 		return py::make_iterator(m.begin(), m.end());
				 }, py::keep_alive<0,1>());
			
	py::class_<dev_id_tCmp>(m, "dev_id_tCmp")
		.def("__call__",
				 &dev_id_tCmp::operator(),
				 py::is_operator());

	py::class_<elka::DeviceRoute>(m, "DeviceRoute")
		.def_readwrite("_route", &elka::DeviceRoute::_route)
		.def_readwrite("_props", &elka::DeviceRoute::_props)
		.def_readwrite("_heartbeat", &elka::DeviceRoute::_heartbeat)
		.def_readwrite("_new_dev", &elka::DeviceRoute::_new_dev)
		.def(py::init<>())
		.def("__init__",
				 [](elka::DeviceRoute &dr,
						DevIdVector *v,
						DevPropVector *p) {
						new (&dr) elka::DeviceRoute(v, p);
				 })
		.def("__init__",
				 [](elka::DeviceRoute &d1,
						elka::DeviceRoute *d2) {
						new (&d1) elka::DeviceRoute(d2);
				 })
		.def("add_prop", &elka::DeviceRoute::add_prop)
		.def("remove_prop", &elka::DeviceRoute::remove_prop)
		.def("check_prop", &elka::DeviceRoute::check_prop)
		.def("check_alive", &elka::DeviceRoute::check_alive)
		.def_static("route_cmp", &elka::DeviceRoute::route_cmp)
		.def_static("dev_props_cmp",
								&elka::DeviceRoute::dev_props_cmp)
		.def_static("print_dev_props",
								&elka::DeviceRoute::print_dev_props)
		.def_static("print_dev_route",
								&elka::DeviceRoute::print_dev_route);

	py::class_<elka::CommPort, elka::PyCommPort<>> (m, "CommPort")
		.def_readwrite("_id", &elka::CommPort::_id)
		.def_readwrite("_state", &elka::CommPort::_state)
		/* In case this method is ever implemented
		.def("push_msg",
			(uint8_t (elka::CommPort::*)(dev_id_t &, uint8_t,
																	 uint8_t, uint8_t *))
			&elka::CommPort::push_msg,
			"Push elka message by parameters")
		*/
		.def("push_msg",
				 (uint8_t (elka::CommPort::*)
									(elka_msg_s &, bool))
								  &elka::CommPort::push_msg,
				 "Push elka msg by elka_msg_s")
		.def("push_msg",
				 (uint8_t (elka::CommPort::*)
									(elka_msg_ack_s &, bool))
								  &elka::CommPort::push_msg,
				 "Push elka msg by elka_msg_ack_s")
		.def("get_msg", &elka::CommPort::get_msg)
		.def("remove_msg", &elka::CommPort::remove_msg)
		.def("pop_msg", &elka::CommPort::pop_msg)
		.def("parse_routing_msg",
				 &elka::CommPort::parse_routing_msg)
		.def("route_cmp", &elka::CommPort::route_cmp)
		.def("dev_props_cmp", &elka::CommPort::dev_props_cmp)
		.def("check_route_contains",
				 &elka::CommPort::check_route_contains)
		.def("check_route", &elka::CommPort::check_route)
		.def("change_route",
				 (void (elka::CommPort::*)
							 (dev_id_t &, elka::DeviceRoute *))
							 &elka::CommPort::change_route,
				 "Change route with device id and DeviceRoute")
		.def("change_route",
				 (void (elka::CommPort::*)
							 (dev_id_t &,
								std::vector<dev_id_t> *,
								std::vector<dev_prop_t> *))
							 &elka::CommPort::change_route,
				 "Change route with device id, device id vector, and\
device route vector")
		.def("get_next_dev", &elka::CommPort::get_next_dev)
		.def("set_dev_props_msg",
    		 &elka::CommPort::set_dev_props_msg)
		.def("set_route_changed_msg",
	  		 &elka::CommPort::set_route_changed_msg)
		.def("set_route_table_msg",
	  		 &elka::CommPort::set_route_table_msg)
		.def("set_route_changed_msg",
	  		 &elka::CommPort::set_route_changed_msg)
		.def("check_dev_compatible",
	  		 &elka::CommPort::check_dev_compatible)
		.def_static("print_elka_route_msg",
	  		 &elka::CommPort::print_elka_route_msg)
		.def_static("print_routing_table",
	  		 &elka::CommPort::print_routing_table);

	py::class_<elka_msg_s>(m, "elka_msg_s")
		.def_readwrite("msg_id", &elka_msg_s::msg_id)
		.def_readwrite("msg_num", &elka_msg_s::msg_num)
		.def_readwrite("num_retries", &elka_msg_s::num_retries)
		.def("get_data", &get_elka_msg_data,
				 py::return_value_policy::reference);

	py::class_<elka_msg_ack_s>(m, "elka_msg_ack_s")
		.def_readwrite("msg_id", &elka_msg_ack_s::msg_id)
		.def_readwrite("msg_num", &elka_msg_ack_s::msg_num)
		.def_readwrite("num_retries", &elka_msg_ack_s::num_retries)
		.def_readwrite("result", &elka_msg_ack_s::result);
}

#endif
