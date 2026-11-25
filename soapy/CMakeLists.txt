# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The HydraSDR Authors

# SoapySDR provides SOAPY_SDR_MODULE_UTIL, which builds the module with the
# right name, install location and ABI suffix. It links the in-tree hydrasdr
# library, so the module does not need a separately installed libhydrasdr.
SOAPY_SDR_MODULE_UTIL(
    TARGET hydraSupport
    SOURCES Registration.cpp Device.cpp
    LIBRARIES hydrasdr
)
target_include_directories(hydraSupport PRIVATE ${PROJECT_SOURCE_DIR}/include)
# The module uses std::clamp. Apple Clang defaults to an older standard.
target_compile_features(hydraSupport PRIVATE cxx_std_17)
# The installed module sits below the library directory, so it needs a runtime
# search path back to the installed libhydrasdr.
file(RELATIVE_PATH HYDRA_MODULE_TO_LIB
     "${CMAKE_INSTALL_FULL_LIBDIR}/SoapySDR/modules${SOAPY_SDR_ABI_VERSION}"
     "${CMAKE_INSTALL_FULL_LIBDIR}")
if(APPLE)
    set_target_properties(hydraSupport PROPERTIES INSTALL_RPATH "@loader_path/${HYDRA_MODULE_TO_LIB}")
elseif(UNIX)
    set_target_properties(hydraSupport PROPERTIES INSTALL_RPATH "$ORIGIN/${HYDRA_MODULE_TO_LIB}")
endif()

# Explicit hardware test of the module. Deliberately not registered with CTest:
# it opens a physical dongle.
if(BUILD_TESTING)
    add_executable(test_soapy_stream tests/test_soapy_stream.cpp)
    target_compile_features(test_soapy_stream PRIVATE cxx_std_17)
    target_include_directories(test_soapy_stream PRIVATE ${SoapySDR_INCLUDE_DIRS})
    target_link_libraries(test_soapy_stream PRIVATE ${SoapySDR_LIBRARIES})
    # Offline test of the native rate list, built against stub library functions.
    add_executable(test_soapy_rates tests/test_soapy_rates.cpp Device.cpp)
    target_compile_features(test_soapy_rates PRIVATE cxx_std_17)
    target_include_directories(test_soapy_rates PRIVATE ${PROJECT_SOURCE_DIR}/include
                                                        ${SoapySDR_INCLUDE_DIRS})
    target_link_libraries(test_soapy_rates PRIVATE ${SoapySDR_LIBRARIES})
    target_compile_definitions(test_soapy_rates PRIVATE SOAPY_HYDRA_TEST_HOOKS)
    add_test(NAME soapy_rates COMMAND test_soapy_rates)
endif()
