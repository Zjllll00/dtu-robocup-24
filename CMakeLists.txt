cmake_minimum_required(VERSION 3.8)
project(dtu_robocup_24 VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# =================================================================
#                  Raspberry PI Compilation arguments
# =================================================================
execute_process(COMMAND uname -m RESULT_VARIABLE IS_OK OUTPUT_VARIABLE CPU1)
string(STRIP ${CPU1} CPU)
# works for Raspberry 3 and 4
if (${CPU} MATCHES "armv7l" OR ${CPU} MATCHES "aarch64")
   message("# Is a RASPBERRY; CPU=${CPU} (Pi3=armv7l, pi4=aarch64)")
   #    set(EXTRA_CC_FLAGS " -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s -DRASPBERRY_PI -D${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O2 -g0 -DRASPBERRY_PI -I/home/local/git/CLI11/include")
   #set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2 -DRASPBERRY_PI")
else()
   message("# Not a RASPBERRY; CPU=${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic \
    -Wno-format-truncation -Wno-return-type \
    -std=c++20 ${EXTRA_CC_FLAGS}")
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")

macro(link_raspberry_libs node_name)
  if (${CPU} MATCHES "armv7l" OR ${CPU} MATCHES "aarch64")
    target_link_libraries(${node_name} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} readline gpiod rt)
  elseif(${CPU} MATCHES "x86_64")
    target_link_libraries(${node_name} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})
  else()
    target_link_libraries(${node_name} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} readline gpiod)
  endif()
endmacro()

# =================================================================
#                      Python Installation
# =================================================================
############################ Python ###############################
ament_python_install_package(${PROJECT_NAME})
file(GLOB py_scripts LIST_DIRECTORIES false RELATIVE "${CMALE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_LIST_DIR}/scripts/*" "${CMAKE_CURRENT_LIST_DIR}/scripts/**/*")
if (py_scripts)
  message("${py_scripts}")
  install(PROGRAMS
    ${py_scripts}
    DESTINATION lib/${PROJECT_NAME}
    )
endif()

######################### Launch Files ############################
install(DIRECTORY
        resources/launch
        resources/model
        DESTINATION share/${PROJECT_NAME})

ament_auto_package()