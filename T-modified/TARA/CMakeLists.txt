# Create a custom library
add_library(
  my-lib
        lib/tara.cc
        lib/simlogs.cc
        lib/simconf.cc
)

# Link the custom library to the scratch executable
build_exec(
  EXECNAME sim
  SOURCE_FILES sim.cc
  LIBRARIES_TO_LINK my-lib
                    ${ns3-libs}
                    ${ns3-contrib-libs}

  EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/tara
)