# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "landmarks_msg: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ilandmarks_msg:/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(landmarks_msg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg" NAME_WE)
add_custom_target(_landmarks_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "landmarks_msg" "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg" ""
)

get_filename_component(_filename "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmarks_Msg.msg" NAME_WE)
add_custom_target(_landmarks_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "landmarks_msg" "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmarks_Msg.msg" "landmarks_msg/Landmark_Msg"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(landmarks_msg
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmarks_Msg.msg"
  "${MSG_I_FLAGS}"
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/landmarks_msg
)
_generate_msg_cpp(landmarks_msg
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/landmarks_msg
)

### Generating Services

### Generating Module File
_generate_module_cpp(landmarks_msg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/landmarks_msg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(landmarks_msg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(landmarks_msg_generate_messages landmarks_msg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg" NAME_WE)
add_dependencies(landmarks_msg_generate_messages_cpp _landmarks_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmarks_Msg.msg" NAME_WE)
add_dependencies(landmarks_msg_generate_messages_cpp _landmarks_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(landmarks_msg_gencpp)
add_dependencies(landmarks_msg_gencpp landmarks_msg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS landmarks_msg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(landmarks_msg
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmarks_Msg.msg"
  "${MSG_I_FLAGS}"
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/landmarks_msg
)
_generate_msg_eus(landmarks_msg
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/landmarks_msg
)

### Generating Services

### Generating Module File
_generate_module_eus(landmarks_msg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/landmarks_msg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(landmarks_msg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(landmarks_msg_generate_messages landmarks_msg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg" NAME_WE)
add_dependencies(landmarks_msg_generate_messages_eus _landmarks_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmarks_Msg.msg" NAME_WE)
add_dependencies(landmarks_msg_generate_messages_eus _landmarks_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(landmarks_msg_geneus)
add_dependencies(landmarks_msg_geneus landmarks_msg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS landmarks_msg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(landmarks_msg
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmarks_Msg.msg"
  "${MSG_I_FLAGS}"
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/landmarks_msg
)
_generate_msg_lisp(landmarks_msg
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/landmarks_msg
)

### Generating Services

### Generating Module File
_generate_module_lisp(landmarks_msg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/landmarks_msg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(landmarks_msg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(landmarks_msg_generate_messages landmarks_msg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg" NAME_WE)
add_dependencies(landmarks_msg_generate_messages_lisp _landmarks_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmarks_Msg.msg" NAME_WE)
add_dependencies(landmarks_msg_generate_messages_lisp _landmarks_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(landmarks_msg_genlisp)
add_dependencies(landmarks_msg_genlisp landmarks_msg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS landmarks_msg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(landmarks_msg
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmarks_Msg.msg"
  "${MSG_I_FLAGS}"
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/landmarks_msg
)
_generate_msg_nodejs(landmarks_msg
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/landmarks_msg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(landmarks_msg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/landmarks_msg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(landmarks_msg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(landmarks_msg_generate_messages landmarks_msg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg" NAME_WE)
add_dependencies(landmarks_msg_generate_messages_nodejs _landmarks_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmarks_Msg.msg" NAME_WE)
add_dependencies(landmarks_msg_generate_messages_nodejs _landmarks_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(landmarks_msg_gennodejs)
add_dependencies(landmarks_msg_gennodejs landmarks_msg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS landmarks_msg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(landmarks_msg
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmarks_Msg.msg"
  "${MSG_I_FLAGS}"
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/landmarks_msg
)
_generate_msg_py(landmarks_msg
  "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/landmarks_msg
)

### Generating Services

### Generating Module File
_generate_module_py(landmarks_msg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/landmarks_msg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(landmarks_msg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(landmarks_msg_generate_messages landmarks_msg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmark_Msg.msg" NAME_WE)
add_dependencies(landmarks_msg_generate_messages_py _landmarks_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/src/SLAM/landmark_msg/msg/Landmarks_Msg.msg" NAME_WE)
add_dependencies(landmarks_msg_generate_messages_py _landmarks_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(landmarks_msg_genpy)
add_dependencies(landmarks_msg_genpy landmarks_msg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS landmarks_msg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/landmarks_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/landmarks_msg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(landmarks_msg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/landmarks_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/landmarks_msg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(landmarks_msg_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/landmarks_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/landmarks_msg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(landmarks_msg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/landmarks_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/landmarks_msg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(landmarks_msg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/landmarks_msg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/landmarks_msg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/landmarks_msg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(landmarks_msg_generate_messages_py std_msgs_generate_messages_py)
endif()
