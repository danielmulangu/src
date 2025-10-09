add_compile_options(-Wno-error=sync-alignment)
add_library(ns3-ai
        model/memory-pool.cc
        model/train-var.h
        model/ns3-ai-rl.h
        model/ns3-ai-dl.h
)

target_include_directories(ns3-ai PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(ns3-ai PUBLIC
        ns3-core
        ns3-network
        ns3-internet
)

# Optional: Add sources and headers if you expand the module
set_target_properties(ns3-ai PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-zero-length-array")
set_source_files_properties(model/memory-pool.cc PROPERTIES COMPILE_FLAGS "-Wno-zero-length-array -Wno-deprecated-volatile")
include_directories(${CMAKE_SOURCE_DIR}/contrib/ns3-ai/model)
set(sources
        contrib/ns3-ai/model/memory-pool.cc
)
#
#set(source_files
#        model/memory-pool.cc
#)
#
#set(header_files
#        model/memory-pool.h
#        model/train-var.h
#        model/ns3-ai-rl.h
#        model/ns3-ai-dl.h
#)
#
#build_lib(
#        LIBNAME ns3-ai
#        SOURCE_FILES ${source_files}
#        HEADER_FILES ${header_files}
#        LIBRARIES_TO_LINK
#        ${libcore}
#)