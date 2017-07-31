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
  ${ELKA_LINK_LIBS} elka_comm__common
	PARENT_SCOPE
)

elseif(${OS} STREQUAL "ubuntu")

pybind11_add_module(elka_comm__common
	SHARED
	pyelka_common.cpp
	elka.cpp
	elka_comm.cpp
)

target_link_libraries(
  elka_comm__common
	PUBLIC
	msg_gen
)

set_target_properties(elka_comm__common PROPERTIES
	LIBRARY_OUTPUT_DIRECTORY ${ELKA_BINARY_DIR}/python
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${GXX_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER__FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

add_dependencies(elka_comm__common msg_gen)

else()

message(
	FATAL_ERROR
	"OS ${OS} not supported with elka_comm library"
)

endif()
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
