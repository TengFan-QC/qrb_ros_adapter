# -----------------------------------------------------------------------------
# The HCM side of the end-to-end test, added by hcm_ipc_adapter in the qcnode
# build:
#   hcm_probe_node.*       HcmStreamProbeNode, a QCNode registered with
#                          QCNodeFactory as QC_NODE_TYPE_CUSTOM_2
#   SampleStreamProbe.cpp  its sample, `qcrun QCNodeSampleApp -t StreamProbe`,
#                          run in a pipeline with SampleIpcAdapter
# Both are built into libQCNodeSample.so, which gets the headers of the Adapter,
# its runtime and qrb_contracts through QCNode. QCNodeSample is defined later in
# the qcnode build (tests/sample), so the calls are deferred.
# -----------------------------------------------------------------------------
set(_probe_dir ${CMAKE_CURRENT_SOURCE_DIR})
cmake_language(EVAL CODE "
  cmake_language(DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL
    target_sources QCNodeSample PRIVATE
      ${_probe_dir}/hcm_probe_node.cpp
      ${_probe_dir}/SampleStreamProbe.cpp)
  cmake_language(DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL
    target_include_directories QCNodeSample PRIVATE
      ${_probe_dir}
      ${_probe_dir}/../src
      ${_probe_dir}/../../hcm_ipc_adapter/sample)")
