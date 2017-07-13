add_library(elka_comm STATIC
	elka_devices.cpp
)

set(ELKA_LINK_LIBS
  "${ELKA_LINK_LIBS} elka_comm"
)


