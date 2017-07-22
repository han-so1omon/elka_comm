#include <cstring>
#include <errno.h>

#include "elka_devices.h"

//-----------------Public Methods----------------------
elka::GroundPort::GroundPort(uint8_t port_num, uint8_t port_type,
    uint8_t buf_type, uint8_t size, char *dev_name) 
    : elka::CommPort(port_num, port_type, buf_type, size) {
  if (!(init() == ELKA_OK)) {
    LOG_ERR("Unable to initialize elka device %s",dev_name);
    errno = ECANCELED;
  } else {
    strcpy(_dev_name, dev_name);

    get_snd_params(&_snd_params, port_num, port_type,
      DEV_PROP_POSIX_SIDE);
  }
}

elka::GroundPort::~GroundPort() {
  if (!(deinit() == ELKA_OK)) {
    LOG_ERR("Unable to deinitialize elka device %s",
    _dev_name);
    errno = ECANCELED;
  };

  delete _tx_buf;
  delete _rx_buf;
}

int elka::GroundPort::init() {
  memset(&_elka_ack_snd, 0, sizeof(_elka_ack_snd));
  memset(&_elka_ack_rcv, 0, sizeof(_elka_ack_rcv));
  memset(&_elka_snd, 0, sizeof(_elka_snd));
  memset(&_elka_rcv, 0, sizeof(_elka_rcv));
  memset(&_elka_rcv_cmd, 0, sizeof(_elka_rcv_cmd));
  
  // Advertise attitude topic
  _elka_msg_pub = orb_advertise(
      ORB_ID(elka_msg), &_elka_snd);
  _elka_ack_pub = orb_advertise(
      ORB_ID(elka_msg_ack), &_elka_ack_snd);
  
  //orb_advert_t elka_msg_pub = orb_advertise(
  //    ORB_ID(elka_msg), NULL);
  //orb_advert_t elka_ack_pub = orb_advertise(
  //    ORB_ID(elka_msg_ack), NULL);

  // Add device properties
  _routing_table[_id].add_prop(DEV_PROP_POSIX_SIDE);
  _routing_table[_id].add_prop(DEV_PROP_PERFORM_LOCALIZATION);
  _routing_table[_id].add_prop(DEV_PROP_SENSE_LOCATION);
  _routing_table[_id].add_prop(DEV_PROP_SPIN_MOTORS);
  _routing_table[_id].add_prop(DEV_PROP_USE_CAMERA);
  _routing_table[_id].add_prop(DEV_PROP_HAS_CAMERA);

  start_port();

  return ELKA_OK;
}

// msg_num and num_retries are useful parameters only
// with MSG_ACK.
// In all other cases these parameters are handled upon
// message pushing/getting
uint8_t elka::GroundPort::add_msg(
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
    std::map<dev_id_t, DeviceRoute, dev_id_tCmp>::iterator
        dev_routes = _routing_table.begin();
    for (; dev_routes != _routing_table.end(); dev_routes++) {
      if ( check_dev_compatible(msg_type,
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

      // num_retries and msg_num not used if tx msg
      //elka_msg.num_retries = num_retries;
      //elka_msg.msg_num = msg_num;
      memcpy(elka_msg.data, data, len);

      push_msg(elka_msg, true);
    }
  }

  return msg_type;
}

// TODO eventually put this in common elka library
// TODO get rid of parameter, as _elka_snd should be set
uint8_t elka::GroundPort::send_msg(elka_msg_s &elka_msg) {
  uint8_t msg_type;
  get_elka_msg_id_attr(NULL, NULL, NULL, &msg_type, NULL,
      elka_msg.msg_id);

  orb_publish(ORB_ID(elka_msg),
              _elka_msg_pub,
              &elka_msg);

  // write to socket 
  /*  
  socket_write_elka_msg(
      _inet_proc.pid,
      elka_msg,
      CLIENT);

  socket_write_elka_msg(
      _inet_proc.pid,
      elka_msg,
      SERVER);
  */

  return msg_type;
}

uint8_t elka::GroundPort::send_msg(elka_msg_ack_s &elka_msg) {
  uint8_t msg_type;
  get_elka_msg_id_attr(NULL, NULL, NULL, &msg_type, NULL,
      elka_msg.msg_id);

  orb_publish(ORB_ID(elka_msg_ack),
              _elka_ack_pub,
              &elka_msg);

  // write to socket 
  /*  
  socket_write_elka_msg(
      _inet_proc.pid,
      elka_msg,
      CLIENT);

  socket_write_elka_msg(
      _inet_proc.pid,
      elka_msg,
      SERVER);
  */

  return msg_type;
}

// For now: Do nothing with elka_msg.msg_num field
uint8_t elka::GroundPort::parse_elka_msg(elka_msg_s &elka_msg) {
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
      initial_msg(elka_msg.msg_id)) {
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

uint8_t elka::GroundPort::parse_motor_cmd(elka_msg_s &elka_msg,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id) {
  uint8_t state = _state;

  // Get ELKA to STATE_RESUME state if possible
  switch (state) {
    case STATE_START:
      resume_port();
      break;
    case STATE_STOP:
      start_port();
      break;
    case STATE_PAUSE:
      resume_port();
      break;
    case STATE_RESUME:
      break;
    default:
      break;
  }

  // Check state again to see that it is STATE_RESUME
  state = _state;
  if (state == STATE_RESUME) {
    if (push_msg(elka_msg, false)) {
      return MSG_NULL;
    }
  } else {
    return MSG_FAILED;
  }

  return msg_id.type;
}

uint8_t elka::GroundPort::parse_port_ctl(elka_msg_s &elka_msg,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id) {

  elka_ack.msg_num = elka_msg.msg_num;
  
  get_elka_msg_id(&elka_ack.msg_id,
      msg_id.rcv_id, msg_id.snd_id,
      msg_id.snd_params,
      MSG_ACK, MSG_ACK_LENGTH);

  elka_ack.result = elka_msg_ack_s::ACK_FAILED;
  return MSG_FAILED; //TODO
}

uint8_t elka::GroundPort::parse_elka_ctl(elka_msg_s &elka_msg,
                        elka_msg_ack_s &elka_ack,
                        struct elka_msg_id_s &msg_id) {
  elka_ack.msg_num = elka_msg.msg_num;
  
  get_elka_msg_id(&elka_ack.msg_id,
      msg_id.rcv_id, msg_id.snd_id,
      msg_id.snd_params,
      MSG_ACK, MSG_ACK_LENGTH);

  uint8_t nxt_state = elka_msg.data[0];
  switch (nxt_state) {
    case STATE_START:
      if (!start_port()) {
        elka_ack.result = elka_msg_ack_s::ACK_FAILED;
        return MSG_FAILED;
      }
      break;
    case STATE_STOP:
      if (!stop_port()) {
        elka_ack.result = elka_msg_ack_s::ACK_FAILED;
        return MSG_FAILED;
      }
      break;
    case STATE_PAUSE:
      if (!pause_port()) {
        elka_ack.result = elka_msg_ack_s::ACK_FAILED;
        return MSG_FAILED;
      }
      break;
    case STATE_RESUME:
      if (!resume_port()) {
        elka_ack.result = elka_msg_ack_s::ACK_FAILED;
        return MSG_FAILED;
      }
      break;
    default:
      elka_ack.result = elka_msg_ack_s::ACK_UNSUPPORTED;
      return MSG_FAILED;
      break;
  }

  elka_ack.result = elka_msg_ack_s::ACK_ACCEPTED;

  return msg_id.type;
}

// Check ack for sent message
// Check ack with respect to port number from elka_ack.msg_id
uint8_t elka::GroundPort::check_ack(struct elka_msg_ack_s &elka_ack) {
  elka::SerialBuffer *sb;
  struct elka_msg_id_s msg_id;
  uint8_t ret;

  get_elka_msg_id_attr(&msg_id, elka_ack.msg_id);

  // Check that ack is meant for this device
  if (!cmp_dev_id_t(msg_id.rcv_id, _id)) {
    return elka_msg_ack_s::ACK_NULL;
  } else {
    sb = _tx_buf;
  }
 
  // First check if message number has recently been acked
  // If message has recently been acked then
  // return elka_msg_ack_s::ACK_NULL
  // Else then check if message exists in buffer
  //    If message exists, then return check_elka_ack(...)
  //        Then add message number to list of recently acked messages
  //        Then remove message from buffer
  //    Else return elka_msg_ack_s::ACK_NULL
  if ((ret = sb->check_recent_acks(elka_ack.msg_num)) !=
              elka_msg_ack_s::ACK_NULL) {
    ElkaBufferMsg *ebm;
    if ((ebm = sb->get_buffer_msg(
            elka_ack.msg_id, elka_ack.msg_num, false))) {
      if ( (ret = check_elka_ack(elka_ack,
                  ebm->_msg_id,
                  ebm->_rmv_msg_num,
                  ebm->_num_retries)) ==
           elka_msg_ack_s::ACK_FAILED) {
        LOG_WARN("Ack failed msg_id %" PRMIT " msg_num %d. %d retries",
            ebm->_msg_id, ebm->_rmv_msg_num,
            ebm->_num_retries);
      } else if (ret != elka_msg_ack_s::ACK_NULL) {
        // Message received and processed fine, so erase it
        LOG_INFO("erasing message");
        sb->erase_msg(elka_ack.msg_id, elka_ack.msg_num, false);
        sb->push_recent_acks(elka_ack.msg_num);
      }
    } else {
      ret = elka_msg_ack_s::ACK_NULL;
    }
  }

  return ret;
}

uint8_t elka::GroundPort::get_state() {
  return _state;
}

uint8_t elka::GroundPort::set_dev_state_msg(
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
  return state;
}

void elka::GroundPort::update_time() {
}

//-----------------Private Methods---------------------
void elka::GroundPort::wait_for_child(Child *child) {
	int pid, status;

	while ((pid = waitpid(-1, &status, 0)) != -1) {
		if (pid == child->pid) {
			child->pid = -1;
		}
	}

	// Check loop not necessary
	if (child->pid != -1) {
		LOG_ERR("Child %d died without being tracked", (int)child->pid);
	}
}

int elka::GroundPort::deinit() {
  _routing_table.clear();
  stop_port();
  return ELKA_OK;
}

bool elka::GroundPort::start_port() {
  _state = STATE_START;

  //FIXME determine client or server programattically
  // For client
  /*
  socket_proc_start(
      &_inet_proc,
      "192.168.1.1",
      CLIENT,
      _tx_buf,
      _rx_buf);
  
  // For server
  socket_proc_start(
      &_inet_proc,
      NULL,
      SERVER,
      _tx_buf,
      _rx_buf);
  */

  resume_port();

  return true;
}

bool elka::GroundPort::stop_port() {
  _state = STATE_STOP;
  
  //wait_for_child(&_inet_proc);

  return true;
}

bool elka::GroundPort::pause_port() {
  _state = STATE_PAUSE;
  return true;
}

bool elka::GroundPort::resume_port() {
  _state = STATE_RESUME;
  return true;
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

PYBIND11_MODULE(elka_comm__gnd_station, m) {

	py::class_<elka::GroundPort,
						 elka::PyGroundPort<>>(m, "GroundPort")
		.def_readwrite("_elka_snd", &elka::GroundPort::_elka_snd)
		.def_readwrite("_elka_rcv", &elka::GroundPort::_elka_rcv)
		.def_readwrite("_elka_rcv_cmd",
									 &elka::GroundPort::_elka_rcv_cmd)
		.def_readwrite("_elka_ack_snd",
									 &elka::GroundPort::_elka_ack_snd)
		.def_readwrite("_elka_ack_rcv",
									 &elka::GroundPort::_elka_ack_rcv)
		.def("__init__", 
				 [](elka::GroundPort &gp,
						uint8_t port_num, uint8_t port_type,
						uint8_t buf_type, uint8_t size, char *dev_name) {
						new (&gp) elka::PyGroundPort<>(
												port_num, port_type,
											  buf_type, size, dev_name);
				 })
		.def("add_msg", &elka::GroundPort::add_msg)
		.def("send_msg",
				 (uint8_t (elka::GroundPort::*)
									(elka_msg_s &))
									&elka::GroundPort::send_msg,
				 "Send elka message")
		.def("send_msg",
				 (uint8_t (elka::GroundPort::*)
									(elka_msg_ack_s &))
									&elka::GroundPort::send_msg,
				 "Send elka ack message")
		.def("parse_elka_msg", &elka::GroundPort::parse_elka_msg)
		.def("check_ack", &elka::GroundPort::check_ack)
		.def("get_state", &elka::GroundPort::get_state)
		.def("set_dev_state_msg",
				 &elka::GroundPort::set_dev_state_msg)
		.def("update_time",
				 &elka::GroundPort::update_time)
		.def("start_port",
				 &elka::GroundPort::start_port)
		.def("stop_port",
				 &elka::GroundPort::stop_port)
		.def("pause_port",
				 &elka::GroundPort::pause_port)
		.def("resume_port",
				 &elka::GroundPort::resume_port);
}

#endif
