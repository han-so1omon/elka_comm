if(${OS} STREQUAL "ubuntu")

pybind11_add_module(modules__elka_comm__gnd_station
	MODULE
  elka_devices.cpp
  inet_comm.cpp
)

target_link_libraries(
  modules__elka_comm__gnd_station
  PRIVATE
  modules__elka_comm__common
)

add_dependencies(modules__elka_comm__common msg_gen)

else()

message(
	FATAL_ERROR
	"OS ${OS} not supported with elka_comm ground station library"
)

endif()
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
