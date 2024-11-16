
set(EXECUTABLE_TARGETS "")

add_executable(test main.cc)
list(APPEND EXECUTABLE_TARGETS test)

add_executable(test2 test.cc)
list(APPEND EXECUTABLE_TARGETS test2)


foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    # Set output directory for executables
    set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
                          RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_BINARY_DIR}")


      # target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE pangolin_Viewer)

      # Link Executable
      target_link_libraries(${EXECUTABLE_TARGET}
                              PUBLIC
                            #   ${PROJECT_NAME}
                              ${OpenCV_LIBS}
                              opencv_imgcodecs
                                opencv_videoio
                                opencv_core
                                opencv_features2d
                                opencv_calib3d
                                OpenMP::OpenMP_CXX
                                Threads::Threads
                            #   util_files
                            #   yaml-cpp
                            #   Eigen3::Eigen  
                            #   ${SUITESPARSE_LIBRARIES}                            
                        )
                        #   opencv_imgcodecs
                        #   opencv_videoio)

endforeach()