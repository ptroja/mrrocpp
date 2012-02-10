add_library(mp_generators
	../../generator/mp/mp_g_delta_tight_coop.cc
)

install(TARGETS mp_generators DESTINATION lib)
