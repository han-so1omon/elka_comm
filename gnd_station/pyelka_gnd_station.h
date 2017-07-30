#pragma once

#include <common/elka.h>
#include <common/elka_comm.h>
#include <gnd_station/elka_devices.h>

// Python bindings for pybind11
#if defined(__ELKA_UBUNTU)

#include <pybind11/pybind11.h>
#include <common/pyelka_common.h>
 
namespace elka {

// Trampoline class for GroundPort. Defined to override virtual
// methods
template <class GroundPortBase = GroundPort> struct PyGroundPort : public PyCommPort<GroundPortBase> {

	// Inherit constructors
	using PyCommPort<GroundPortBase>::PyCommPort;

  uint8_t start_port() override {
    PYBIND11_OVERLOAD(uint8_t,
                      GroundPortBase, 
                      start_port, );
  }

  uint8_t stop_port() override {
    PYBIND11_OVERLOAD(uint8_t,
                      GroundPortBase, 
                      stop_port, );
  }

  uint8_t pause_port() override {
    PYBIND11_OVERLOAD(uint8_t,
                      GroundPortBase, 
                      pause_port, );
  }

  uint8_t resume_port() override {
    PYBIND11_OVERLOAD(uint8_t,
                      GroundPortBase, 
                      resume_port, );
  }

  uint8_t remote_ctl_port() override {
    PYBIND11_OVERLOAD(uint8_t,
                      GroundPortBase, 
                      remote_ctl_port, );
  }

  uint8_t autopilot_ctl_port() override {
    PYBIND11_OVERLOAD(uint8_t,
                      GroundPortBase, 
                      autopilot_ctl_port, );
  }

};

} // namespace elka

#endif
