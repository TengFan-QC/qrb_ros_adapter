# Finds Protobuf and sets QRB_PROTOC to the protoc used for code generation.
#
# protobuf's own CMake package is preferred: from protobuf 22 on, its targets
# carry the abseil link interface that FindProtobuf (module mode) leaves out.
# When cross-compiling, pass Protobuf_PROTOC_EXECUTABLE so that a host protoc
# is used instead of the one in the target sysroot.

find_package(Protobuf CONFIG QUIET)
if(NOT Protobuf_FOUND)
  find_package(Protobuf REQUIRED)
endif()

if(Protobuf_PROTOC_EXECUTABLE)
  set(QRB_PROTOC "${Protobuf_PROTOC_EXECUTABLE}")
elseif(TARGET protobuf::protoc)
  set(QRB_PROTOC protobuf::protoc)
else()
  find_program(QRB_PROTOC_PROGRAM protoc)
  if(NOT QRB_PROTOC_PROGRAM)
    message(FATAL_ERROR "protoc not found; set Protobuf_PROTOC_EXECUTABLE")
  endif()
  set(QRB_PROTOC "${QRB_PROTOC_PROGRAM}")
endif()
