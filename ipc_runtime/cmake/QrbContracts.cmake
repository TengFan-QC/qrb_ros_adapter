# Builds the C++ for the qrb_contracts schemas as qrb_ros_adapter::contracts_proto.
#
# The schemas come from the installed qrb_contracts package. Following the
# protobuf convention, they are looked up as qrb/contract/... in its include
# directory; set QRB_CONTRACTS_PROTO_DIR for a package that does not install
# them there. Generated code stays in the build directory and is installed with
# this package, so its consumers do not need qrb_contracts. The descriptor
# headers of qrb_contracts are HCM-specific and are used by the HCM IPC Adapter
# package.

include_guard(GLOBAL)

find_package(qrb_contracts REQUIRED CONFIG)
get_target_property(_qrb_contracts_include_dirs qrb_contracts::qrb_contracts
  INTERFACE_INCLUDE_DIRECTORIES)
find_path(QRB_CONTRACTS_PROTO_DIR qrb/contract/geometry/v1/pose.proto
  HINTS ${_qrb_contracts_include_dirs}
  DOC "protoc --proto_path for the qrb_contracts schemas (qrb/contract/...)")
if(NOT QRB_CONTRACTS_PROTO_DIR)
  message(FATAL_ERROR
    "qrb_contracts schemas (qrb/contract/...) not found in "
    "${_qrb_contracts_include_dirs}. Set QRB_CONTRACTS_PROTO_DIR to the "
    "directory that contains them.")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/QrbProtobuf.cmake")

set(_qrb_contracts_proto_root "${QRB_CONTRACTS_PROTO_DIR}")
set(_qrb_contracts_gen_root "${CMAKE_CURRENT_BINARY_DIR}/qrb_contracts_generated")
file(MAKE_DIRECTORY "${_qrb_contracts_gen_root}")

file(GLOB_RECURSE _qrb_contracts_proto_files CONFIGURE_DEPENDS
  RELATIVE "${_qrb_contracts_proto_root}"
  "${_qrb_contracts_proto_root}/qrb/contract/*.proto")

set(_qrb_contracts_proto_sources "")
set(_qrb_contracts_proto_headers "")
foreach(_proto IN LISTS _qrb_contracts_proto_files)
  get_filename_component(_proto_dir "${_proto}" DIRECTORY)
  get_filename_component(_proto_name "${_proto}" NAME_WE)
  set(_proto_source "${_qrb_contracts_gen_root}/${_proto_dir}/${_proto_name}.pb.cc")
  set(_proto_header "${_qrb_contracts_gen_root}/${_proto_dir}/${_proto_name}.pb.h")
  add_custom_command(
    OUTPUT "${_proto_source}" "${_proto_header}"
    COMMAND ${QRB_PROTOC}
            --proto_path=${_qrb_contracts_proto_root}
            --cpp_out=${_qrb_contracts_gen_root}
            "${_qrb_contracts_proto_root}/${_proto}"
    DEPENDS "${_qrb_contracts_proto_root}/${_proto}"
    COMMENT "protoc ${_proto}"
    VERBATIM)
  list(APPEND _qrb_contracts_proto_sources "${_proto_source}")
  list(APPEND _qrb_contracts_proto_headers "${_proto_header}")
endforeach()

# Shared so that the HCM and ROS libraries loaded into one process register
# each Contract message in the protobuf descriptor pool only once.
add_library(qrb_contracts_proto SHARED
  ${_qrb_contracts_proto_sources}
  ${_qrb_contracts_proto_headers})
target_include_directories(qrb_contracts_proto SYSTEM PUBLIC
  "$<BUILD_INTERFACE:${_qrb_contracts_gen_root}>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(qrb_contracts_proto PUBLIC protobuf::libprotobuf)
target_compile_features(qrb_contracts_proto PUBLIC cxx_std_17)
set_target_properties(qrb_contracts_proto PROPERTIES
  OUTPUT_NAME qrb_ros_adapter_contracts
  EXPORT_NAME contracts_proto
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR})

add_library(qrb_ros_adapter::contracts_proto ALIAS qrb_contracts_proto)

install(DIRECTORY "${_qrb_contracts_gen_root}/" DESTINATION include
  FILES_MATCHING PATTERN "*.pb.h")
