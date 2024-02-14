nanobind_add_module(linefit MODULE linefit_ext.cpp)
target_link_libraries(linefit PRIVATE gm_lib)
install(TARGETS linefit LIBRARY DESTINATION .)