#ifndef PYELKA_COMMON_H
#define PYELKA_COMMON_H

#include <common/elka.h>
#include <common/elka_comm.h>

// Python bindings for pybind11
#if defined(__ELKA_UBUNTU)

#include <pybind11/pybind11.h>

namespace py = pybind11;
 
namespace elka {

// Trampoline class for CommPort. Defined to override virtual
// methods
template <class CommPortBase = CommPort> struct PyCommPort : public CommPortBase {
  // Inherent constructors
  using CommPortBase::CommPortBase;

  uint8_t start_port() override {
    PYBIND11_OVERLOAD_PURE(uint8_t,
                           CommPortBase, 
                           start_port, );
  }

  uint8_t stop_port() override {
    PYBIND11_OVERLOAD_PURE(uint8_t,
                           CommPortBase, 
                           stop_port, );
  }

  uint8_t pause_port() override {
    PYBIND11_OVERLOAD_PURE(uint8_t,
                           CommPortBase, 
                           pause_port, );
  }

  uint8_t resume_port() override {
    PYBIND11_OVERLOAD_PURE(uint8_t,
                           CommPortBase, 
                           resume_port, );
  }

  uint8_t remote_ctl_port() override {
    PYBIND11_OVERLOAD_PURE(uint8_t,
                           CommPortBase, 
                           remote_ctl_port, );
  }

  uint8_t autopilot_ctl_port() override {
    PYBIND11_OVERLOAD_PURE(uint8_t,
                           CommPortBase, 
                           autopilot_ctl_port, );
  }

};

} // namespace elka

#endif // __ELKA_UBUNTU

#endif // PYELKA_COMMON_H
