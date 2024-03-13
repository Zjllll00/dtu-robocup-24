cmake_minimum_required(VERSION 3.8)
project(dtu_robocup_24 VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

message("========================= CMake ===========================")

# =================================================================
#                  Raspberry PI Compilation arguments
# =================================================================
# Get CPU
execute_process(COMMAND uname -m RESULT_VARIABLE IS_OK OUTPUT_VARIABLE CPU1)
string(STRIP ${CPU1} CPU)

# Get system
set(OS_NAME "${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION}")
if (${CPU} MATCHES "armv7l")
  set(RPI "Raspberry Pi 3")
elseif(${CPU} MATCHES "aarch64")
  set(RPI "Raspberry Pi 4")
endif()

# Set extra compilation flags for raspberry pi
if (RPI)
  message("Compiling for ${RPI} - ${OS_NAME} [arch=${CPU}]")
  #set(EXTRA_CC_FLAGS " -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s -DRASPBERRY_PI -D${CPU}")
  set(EXTRA_CC_FLAGS "-D${CPU} -O2 -g0 -DRASPBERRY_PI -I/home/local/git/CLI11/include")
  #set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2 -DRASPBERRY_PI")
else()
   message("Compiling for ${OS_NAME} [cpu arch=${CPU}]")
   set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic \
    -Wno-format-truncation -Wno-return-type \
    -std=c++20 ${EXTRA_CC_FLAGS}")
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")

# =================================================================
#                     Wrapping compile function
# =================================================================
function(glob_sources list src_paths folder extension)
  set(FOLDER_A "${CMAKE_CURRENT_LIST_DIR}/${folder}/*${extension}")
  set(FOLDER_B "${CMAKE_CURRENT_LIST_DIR}/${folder}/**/*${extension}")

  # Grab source files
  file(GLOB src_files LIST_DIRECTORIES false RELATIVE "${CMALE_CURRENT_LIST_DIR}" "${FOLDER_A}" "${FOLDER_B}")
  set(${list} ${src_files} PARENT_SCOPE)
  set(${src_paths} "\n\t${FOLDER_A},\n\t${FOLDER_B}" PARENT_SCOPE)
endfunction()

macro(link_raspberry_libs node_name)
  if (${CPU} MATCHES "armv7l" OR ${CPU} MATCHES "aarch64")
    target_link_libraries(${node_name} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} readline gpiod rt)
  elseif(${CPU} MATCHES "x86_64") # If personal computer, do not link libs
    target_link_libraries(${node_name} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})
  else()
    target_link_libraries(${node_name} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} readline gpiod)
  endif()
endmacro()

macro(target_setup target)
  target_include_directories(${target} PRIVATE include)
  link_raspberry_libs(${target})
endmacro()

macro(make_library lib_name lib_folder)
  glob_sources(lib_src lib_paths "src/${lib_folder}" ".cpp")
  if (lib_src)
    message("Making C++ library ${lib_name} with sources at ${lib_paths}")
    ament_auto_add_library(${lib_name} ${lib_src})
    target_setup(${lib_name})
  else()
    message("No source found for C++ library ${lib_name} with sources at ${lib_paths}")
  endif()
endmacro(make_library)

macro(make_executable exe_name exe_folder)
  glob_sources(exe_src exe_paths "src/${exe_folder}" ".cpp")
  if (exe_src)
    message("Making C++ executable ${exe_name} with sources at ${exe_paths}")
    ament_auto_add_executable(${exe_name} ${exe_src})
    target_setup(${exe_name})
    if (TARGET common)
      target_link_libraries(${exe_name} common)
    endif()
  else() 
    message("No source found for C++ executable ${exe_name} with sources at ${exe_paths}")
  endif()
endmacro(make_executable)

macro(make_python_scripts scripts_folder)
  glob_sources(py_src py_paths ${scripts_folder} "")
  if (py_src)
    message("Adding Python scripts in ${py_paths}")

    # For debug, print found executable
    foreach(py_exe ${py_src})
      string(REGEX MATCH "([a-zA-Z0-9_.]*)$" py_name ${py_exe})
      message("\t-> Found script ${py_name}")
    endforeach()
    
    # Install them
    install(PROGRAMS
      ${py_src}
      DESTINATION lib/${PROJECT_NAME}
    )
  else()
    message("No Python script found in ${py_paths}")
  endif()
endmacro(make_python_scripts)

# =================================================================
#                      Targets definitions
# =================================================================
ament_python_install_package(${PROJECT_NAME})
make_python_scripts("scripts")

# =================================================================
#                      Other files & resources
# =================================================================
install(DIRECTORY
        resources/launch
        resources/model
        DESTINATION share/${PROJECT_NAME})

ament_auto_package()
message("========================== Make ==========================")