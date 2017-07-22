if(${OS} STREQUAL "posix" OR ${OS} STREQUAL "qurt")

px4_add_module(
	MODULE modules__elka__common
	COMPILE_FLAGS
	SRCS
		elka.cpp
    elka_comm.cpp
	DEPENDS
		platforms__common
)

elseif(${OS} STREQUAL "FreeRTOS")

add_library(elka_comm__common STATIC
	elka.cpp
	elka_comm.cpp
)

set(ELKA_LINK_LIBS
  "${ELKA_LINK_LIBS} elka_comm__common"
)

elseif(${OS} STREQUAL "ubuntu")

pybind11_add_module(elka_comm__common
	SHARED
	elka.cpp
	elka_comm.cpp
)

add_dependencies(elka_comm__common msg_gen)

else()

message(
	FATAL_ERROR
	"OS ${OS} not supported with elka_comm library"
)

endif()
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
