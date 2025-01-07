# CMake config file for the brisk package
# It defines the following variables:
#   BRISK_VERSION   - the version of the package
#   BRISK_LIBRARIES - libraries to link against
set(BRISK_VERSION 2.0.7)

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was briskConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################
 
include(CMakeFindDependencyMacro)
find_dependency(OpenCV)

get_filename_component(DIRNAME "${CMAKE_CURRENT_LIST_FILE}" PATH)
include("${DIRNAME}/briskTargets.cmake")

set(BRISK_LIBRARIES ${BRISK_LIBRARIES} brisk::agast brisk::brisk)

check_required_components(brisk)
