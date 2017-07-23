if(${OS} STREQUAL "ubuntu")

pybind11_add_module(elka_comm__gnd_station
	MODULE
	pyelka_gnd_station.cpp
  elka_devices.cpp
  inet_comm.cpp
)

target_link_libraries(
  elka_comm__gnd_station
  PRIVATE
  elka_comm__common
)

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
