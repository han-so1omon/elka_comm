#ifndef PYELKA_COMMON_H
#define PYELKA_COMMON_H

#include <common/elka.h>
#include <common/elka_comm.h>

// Python bindings for pybind11
#if defined(__ELKA_UBUNTU)
 
namespace elka {

// Trampoline class for CommPort. Defined to override virtual
// methods
template <class CommPortBase = CommPort> struct PyCommPort : public CommPortBase {
  // Inherent constructors
  using CommPortBase::CommPortBase;

  bool start_port() override {
    PYBIND11_OVERLOAD_PURE(bool,
                           CommPortBase, 
                           start_port, );
  }

  bool stop_port() override {
    PYBIND11_OVERLOAD_PURE(bool,
                           CommPortBase, 
                           stop_port, );
  }

  bool pause_port() override {
    PYBIND11_OVERLOAD_PURE(bool,
                           CommPortBase, 
                           pause_port, );
  }

  bool resume_port() override {
    PYBIND11_OVERLOAD_PURE(bool,
                           CommPortBase, 
                           resume_port, );
  }

  
};

} // namespace elka

#endif // __ELKA_UBUNTU

#endif // PYELKA_COMMON_H
