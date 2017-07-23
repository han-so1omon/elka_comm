#if defined(__ELKA_UBUNTU) 

#include <gnd_station/pyelka_gnd_station.h>

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

  /* FIXME will we have to explicitly cast to (dev_prop_t) in c++?
	template <> struct type_caster<dev_prop_t> {
	public:
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

		static handle cast(dev_prop_t src, return_value_policy, handle) {
			return PyLong_FromUnsignedLong((unsigned long) src);
		}
	};
  */

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
    .def("get_instance", &elka::GroundPort::get_instance,
				 py::return_value_policy::reference)
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
		.def("parse_elka_msg",
          (uint8_t (elka::GroundPort::*)
                   (elka_msg_s &))
                &elka::GroundPort::parse_elka_msg)
		.def("parse_elka_msg",
          (uint8_t (elka::GroundPort::*)
                   (elka_msg_ack_s &))
                &elka::GroundPort::parse_elka_msg)
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
				 &elka::GroundPort::resume_port)
		.def("remote_ctl_port",
				 &elka::GroundPort::remote_ctl_port)
		.def("autopilot_ctl_port",
				 &elka::GroundPort::autopilot_ctl_port);

}

#endif
