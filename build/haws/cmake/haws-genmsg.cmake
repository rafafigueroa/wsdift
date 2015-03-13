# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "haws: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ihaws:/home/rafa/wsdift/src/haws/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/indigo/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(haws_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Tags.msg" NAME_WE)
add_custom_target(_haws_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "haws" "/home/rafa/wsdift/src/haws/msg/Tags.msg" ""
)

get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Warning_Levels.msg" NAME_WE)
add_custom_target(_haws_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "haws" "/home/rafa/wsdift/src/haws/msg/Warning_Levels.msg" ""
)

get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Conflict.msg" NAME_WE)
add_custom_target(_haws_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "haws" "/home/rafa/wsdift/src/haws/msg/Conflict.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(haws
  "/home/rafa/wsdift/src/haws/msg/Tags.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/haws
)
_generate_msg_cpp(haws
  "/home/rafa/wsdift/src/haws/msg/Warning_Levels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/haws
)
_generate_msg_cpp(haws
  "/home/rafa/wsdift/src/haws/msg/Conflict.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/haws
)

### Generating Services

### Generating Module File
_generate_module_cpp(haws
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/haws
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(haws_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(haws_generate_messages haws_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Tags.msg" NAME_WE)
add_dependencies(haws_generate_messages_cpp _haws_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Warning_Levels.msg" NAME_WE)
add_dependencies(haws_generate_messages_cpp _haws_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Conflict.msg" NAME_WE)
add_dependencies(haws_generate_messages_cpp _haws_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(haws_gencpp)
add_dependencies(haws_gencpp haws_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS haws_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(haws
  "/home/rafa/wsdift/src/haws/msg/Tags.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/haws
)
_generate_msg_eus(haws
  "/home/rafa/wsdift/src/haws/msg/Warning_Levels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/haws
)
_generate_msg_eus(haws
  "/home/rafa/wsdift/src/haws/msg/Conflict.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/haws
)

### Generating Services

### Generating Module File
_generate_module_eus(haws
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/haws
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(haws_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(haws_generate_messages haws_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Tags.msg" NAME_WE)
add_dependencies(haws_generate_messages_eus _haws_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Warning_Levels.msg" NAME_WE)
add_dependencies(haws_generate_messages_eus _haws_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Conflict.msg" NAME_WE)
add_dependencies(haws_generate_messages_eus _haws_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(haws_geneus)
add_dependencies(haws_geneus haws_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS haws_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(haws
  "/home/rafa/wsdift/src/haws/msg/Tags.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/haws
)
_generate_msg_lisp(haws
  "/home/rafa/wsdift/src/haws/msg/Warning_Levels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/haws
)
_generate_msg_lisp(haws
  "/home/rafa/wsdift/src/haws/msg/Conflict.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/haws
)

### Generating Services

### Generating Module File
_generate_module_lisp(haws
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/haws
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(haws_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(haws_generate_messages haws_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Tags.msg" NAME_WE)
add_dependencies(haws_generate_messages_lisp _haws_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Warning_Levels.msg" NAME_WE)
add_dependencies(haws_generate_messages_lisp _haws_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Conflict.msg" NAME_WE)
add_dependencies(haws_generate_messages_lisp _haws_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(haws_genlisp)
add_dependencies(haws_genlisp haws_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS haws_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(haws
  "/home/rafa/wsdift/src/haws/msg/Tags.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/haws
)
_generate_msg_py(haws
  "/home/rafa/wsdift/src/haws/msg/Warning_Levels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/haws
)
_generate_msg_py(haws
  "/home/rafa/wsdift/src/haws/msg/Conflict.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/haws
)

### Generating Services

### Generating Module File
_generate_module_py(haws
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/haws
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(haws_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(haws_generate_messages haws_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Tags.msg" NAME_WE)
add_dependencies(haws_generate_messages_py _haws_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Warning_Levels.msg" NAME_WE)
add_dependencies(haws_generate_messages_py _haws_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rafa/wsdift/src/haws/msg/Conflict.msg" NAME_WE)
add_dependencies(haws_generate_messages_py _haws_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(haws_genpy)
add_dependencies(haws_genpy haws_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS haws_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/haws)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/haws
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(haws_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(haws_generate_messages_cpp geometry_msgs_generate_messages_cpp)
add_dependencies(haws_generate_messages_cpp nav_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/haws)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/haws
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(haws_generate_messages_eus std_msgs_generate_messages_eus)
add_dependencies(haws_generate_messages_eus geometry_msgs_generate_messages_eus)
add_dependencies(haws_generate_messages_eus nav_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/haws)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/haws
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(haws_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(haws_generate_messages_lisp geometry_msgs_generate_messages_lisp)
add_dependencies(haws_generate_messages_lisp nav_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/haws)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/haws\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/haws
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(haws_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(haws_generate_messages_py geometry_msgs_generate_messages_py)
add_dependencies(haws_generate_messages_py nav_msgs_generate_messages_py)
