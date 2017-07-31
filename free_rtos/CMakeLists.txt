add_library(elka_comm__free_rtos STATIC
	elka_devices.cpp
)

set(ELKA_LINK_LIBS
  ${ELKA_LINK_LIBS} elka_comm__free_rtos
  PARENT_SCOPE
)

