include_directories(${CMAKE_SOURCE_DIR})



# Descend into the custom_distributions_loop_functions directory
add_subdirectory(custom_distributions_loop_functions)

# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_COMPILE_QTOPENGL)
  add_subdirectory(trajectory_loop_functions)
  add_subdirectory(id_loop_functions)

endif(ARGOS_COMPILE_QTOPENGL)


