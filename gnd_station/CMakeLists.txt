if(${OS} STREQUAL "ubuntu")

pybind11_add_module(elka_comm__gnd_station
	MODULE
	pyelka_gnd_station.cpp
  elka_devices.cpp
  inet_comm.cpp
)

target_link_libraries(
  elka_comm__gnd_station
	PUBLIC
  elka_comm__common
	msg_gen
)

set_target_properties(elka_comm__gnd_station PROPERTIES
	LIBRARY_OUTPUT_DIRECTORY ${ELKA_BINARY_DIR}/python
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${GXX_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER__FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

add_dependencies(elka_comm__gnd_station
	elka_comm__common
	msg_gen
)

else()

message(
	FATAL_ERROR
	"OS ${OS} not supported with elka_comm ground station library"
)

endif()
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
