#if defined(__ELKA_UBUNTU) 

#include <common/pyelka_common.h>

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

		bool load(handle src, bool convert) {
			PyObject *source = src.ptr();
			PyObject *tmp = PyNumber_Long(src.ptr());
			if (!tmp)
					return false;
			value = (dev_prop_t)PyInt_AsUnsignedLongMask(tmp);
			Py_DECREF(tmp);
			return !(value == -1 && !PyErr_Occurred());
		}

		static handle cast(dev_prop_t src, return_value_policy , handle ) {
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
		.def_readwrite("_hw_state", &elka::CommPort::_hw_state)
		.def_readwrite("_sw_state", &elka::CommPort::_sw_state)
		.def_readwrite("_prev_hw_state",
                   &elka::CommPort::_prev_hw_state)
		.def_readwrite("_prev_sw_state",
                   &elka::CommPort::_prev_sw_state)
		.def_readwrite("_spektrum_channel_kill",
                   &elka::CommPort::_spektrum_channel_kill)
		.def_readwrite("_spektrum_channel_switch",
                   &elka::CommPort::_spektrum_channel_switch)
		.def("__init__", 
				 [](elka::CommPort &cp,
						uint8_t port_n, uint8_t port_t,
						uint8_t buf_type, uint8_t size) {
						new (&cp) elka::PyCommPort<>(port_n, port_t,
																		 buf_type, size);
				 })
/*
  CommPort(uint8_t port_n, uint8_t port_t, uint8_t buf_type,
      uint8_t size);
  // virtual destructor so that derived class destructor is called.
  virtual ~CommPort(); 
*/

		/* In case this method is ever implemented
		.def("push_msg",
			(uint8_t (elka::CommPort::*)(dev_id_t &, uint8_t,
																	 uint8_t, uint8_t *))
			&elka::CommPort::push_msg,
			"Push elka message by parameters")
		*/
    .def("get_state", &elka::CommPort::get_state)
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
	  		 &elka::CommPort::print_routing_table)
    .def_static("print_elka_ctl_msg",
         &elka::CommPort::print_elka_ctl_msg)
    .def("print_elka_state",
        &elka::CommPort::print_elka_state);

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
