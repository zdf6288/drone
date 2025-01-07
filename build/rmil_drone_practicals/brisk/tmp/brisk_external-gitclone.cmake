
if(NOT "/home/zhang/drone_workspace/build/rmil_drone_practicals/brisk/src/brisk_external-stamp/brisk_external-gitinfo.txt" IS_NEWER_THAN "/home/zhang/drone_workspace/build/rmil_drone_practicals/brisk/src/brisk_external-stamp/brisk_external-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/home/zhang/drone_workspace/build/rmil_drone_practicals/brisk/src/brisk_external-stamp/brisk_external-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E remove_directory "/home/zhang/drone_workspace/build/rmil_drone_practicals/brisk/src/brisk_external"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/zhang/drone_workspace/build/rmil_drone_practicals/brisk/src/brisk_external'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout "git@github.com:smartroboticslab/brisk.git" "brisk_external"
    WORKING_DIRECTORY "/home/zhang/drone_workspace/build/rmil_drone_practicals/brisk/src"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'git@github.com:smartroboticslab/brisk.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout origin/main --
  WORKING_DIRECTORY "/home/zhang/drone_workspace/build/rmil_drone_practicals/brisk/src/brisk_external"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'origin/main'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/home/zhang/drone_workspace/build/rmil_drone_practicals/brisk/src/brisk_external"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/zhang/drone_workspace/build/rmil_drone_practicals/brisk/src/brisk_external'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/home/zhang/drone_workspace/build/rmil_drone_practicals/brisk/src/brisk_external-stamp/brisk_external-gitinfo.txt"
    "/home/zhang/drone_workspace/build/rmil_drone_practicals/brisk/src/brisk_external-stamp/brisk_external-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/zhang/drone_workspace/build/rmil_drone_practicals/brisk/src/brisk_external-stamp/brisk_external-gitclone-lastrun.txt'")
endif()

