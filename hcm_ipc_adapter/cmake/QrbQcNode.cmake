# Locates QCNode and defines QCNode::QCNode.
#
# The QCNode headers and library are looked up in the system prefixes, which in
# a Yocto build is the recipe sysroot populated by the qcnode recipe.
#
# Installed with qrb_hcm_ipc_adapter, whose package config includes it: the IPC
# Adapter Node is built into libQCNode.so.

if(NOT TARGET QCNode::QCNode)
  find_path(QRB_QCNODE_INCLUDE_DIR QC/Node/Ifs/QCNodeIfs.hpp)
  find_library(QRB_QCNODE_LIBRARY QCNode)
  if(NOT QRB_QCNODE_INCLUDE_DIR OR NOT QRB_QCNODE_LIBRARY)
    message(FATAL_ERROR
      "QCNode not found. Add qcnode to the build dependencies, or point "
      "CMAKE_PREFIX_PATH at a QCNode installation.")
  endif()
  add_library(QCNode::QCNode SHARED IMPORTED)
  set_target_properties(QCNode::QCNode PROPERTIES
    IMPORTED_LOCATION "${QRB_QCNODE_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${QRB_QCNODE_INCLUDE_DIR}")
endif()

set(QRB_QCNODE_TARGET_SOC "8797" CACHE STRING "QCNode target SoC (QC_TARGET_SOC)")
