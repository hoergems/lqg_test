cmake_minimum_required (VERSION 2.6)

project (lqg_test)

if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE DEBUG)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11 -O3 -w")
set(CMAKE_CXX_FLAGS_RELEASE "-g -std=c++11 -O3 -w")
SET(BOOST_MIN_VERSION "1.54.0")
SET(PYTHON_MIN_VERSION "2.7.0")

include(FindPkgConfig)
include(GNUInstallDirs)

set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/include")

############################# LOAD PYTHON #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(PYTHON python-2.7)
    if(NOT PYTHON_FOUND)
       message(FATAL_ERROR "Python could not be found")
    endif()
    include_directories(${PYTHON_INCLUDE_DIRS})
    link_directories(${PYTHON_LIBRARY_DIRS})    
endif()


############################# LOAD BOOST #############################
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS python system thread timer)
if (NOT Boost_FOUND)
    set(Boost_INCLUDE_DIRS "${BOOST_INCLUDE_DIRS}")
    IF ( NOT Boost_INCLUDE_DIRS )
        message(FATAL_ERROR "Please point the environment variable BOOST_INCLUDE_DIRS to the include directory of Boost")
    ENDIF()
endif ()
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

############################# LOAD EIGEN #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(EIGEN eigen3)
    if(NOT EIGEN_FOUND)
       message(FATAL_ERROR "EIGEN could not be found")
    endif()
    include_directories(${EIGEN_INCLUDE_DIRS}) 
    message("-- Eigen include dirs: ${EIGEN_INCLUDE_DIRS}")
    message("-- Eigen library dirs: ${EIGEN_LIBRARY_DIRS}")   
endif()

############################# LOAD ROBOTS #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(ROBOTS robots)
    if(NOT ROBOTS_FOUND)
       message(FATAL_ERROR "ROBOTS could not be found")
    endif()
    include_directories(${ROBOTS_INCLUDE_DIRS})
    link_directories(${ROBOTS_LIBRARY_DIRS})     
endif()

############################# LOAD ROBOT_ENVIRONMENT #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(ROBOT_ENVIRONMENT robot_environment)
    if(NOT ROBOTS_FOUND)
       message(FATAL_ERROR "ROBOT_ENVIRONMENT could not be found")
    endif()
    include_directories(${ROBOT_ENVIRONMENT_INCLUDE_DIRS})
    link_directories(${ROBOT_ENVIRONMENT_LIBRARY_DIRS})     
endif()

############################# LOAD TCLAP #############################
find_package (TCLAP)
if (NOT TCLAP_FOUND)
message(${TCLAP_INCLUDE_DIRS})
include_directories(${TCLAP_INCLUDE_DIRS})
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/include/)

############################# LOAD PATH_PLANNER #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(PATH_PLANNER path_planner)
    if(NOT PATH_PLANNER_FOUND)
       message(FATAL_ERROR "PATH_PLANNER could not be found")
    endif()
    include_directories(${PATH_PLANNER_INCLUDE_DIRS})
    link_directories(${PATH_PLANNER_LIBRARY_DIRS})     
endif()

############################# LOAD FCL #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(FCL fcl)
    if(NOT FCL_FOUND)
       message(FATAL_ERROR "FCL could not be found")
    endif()
    include_directories(${FCL_INCLUDE_DIRS})
    link_directories(${FCL_LIBRARY_DIRS}) 
    message("-- FCL LIB DIRS ${FCL_LIBRARY_DIRS}")    
endif()

############################# LOAD KALMAN_FILTER #############################
if(PKG_CONFIG_FOUND)    
    pkg_check_modules(KALMAN_FILTER kalman_filter)
    if(NOT KALMAN_FILTER_FOUND)
       message(FATAL_ERROR "KALMAN_FILTER could not be found")
    endif()
    include_directories(${KALMAN_FILTER_INCLUDE_DIRS})
    link_directories(${KALMAN_FILTER_LIBRARY_DIRS})     
endif()

add_executable(lqg_test
               ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp               
               ${CMAKE_CURRENT_SOURCE_DIR}/src/option_parser.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/src/ini.c)
               
target_link_libraries (lqg_test                    
                       ${Boost_LIBRARIES}
                       ${PYTHON_LIBRARIES}
                       ${ROBOTS_LIBRARIES}
                       ${ROBOT_ENVIRONMENT_LIBRARIES}
                       ${FCL_LIBRARIES}
                       ${PATH_PLANNER_LIBRARIES}
                       ${KALMAN_FILTER_LIBRARIES}
                       ${TCLAP_LIBRARIES})


