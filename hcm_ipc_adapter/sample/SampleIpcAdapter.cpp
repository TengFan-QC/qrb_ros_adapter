// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file SampleIpcAdapter.cpp
 * @brief Sample for the IPC Adapter QCNode, runnable with QCNodeSampleApp (-t IpcAdapter).
 *
 * The sample creates the IPC Adapter node through QCNodeFactory with one Flow, then every
 * 1/fps second from a worker thread it calls ProcessFrameDescriptor() with a frame descriptor
 * holding the Contract descriptor. For HCM to ROS (direction 0) it sends a fixed XYZ point
 * cloud or a pose moving along x; for ROS to HCM (direction 1) it fetches the latest ROS
 * message into the descriptor. The ROS Bridge (ros_ipc_bridge) must be running.
 *
 * Attributes:
 *   contract_type : (optional) qrb.contract.sensor.v1.PointCloud2 (default) or
 *                   qrb.contract.geometry.v1.PoseWithCovarianceStamped
 *   direction     : (optional) 0: HCM to ROS (default), 1: ROS to HCM
 *   ros_topic     : (optional) default /qrb_ros_adapter/sample/points or .../pose
 *   reliability   : (optional) reliable or best_effort, default the ROS default
 *   durability    : (optional) volatile or transient_local, default the ROS default
 *   depth         : (optional) 1..32, default the ROS default
 *   fps           : (optional) frames per second, 0 for no pause, default 30
 *   payload_bytes : (optional) point cloud payload sent to ROS, default 1048576
 *   buffer_bytes  : (optional) point cloud buffer received from ROS, default 33554432
 */

#include "QC/Node/Ifs/QCNodeFactory.hpp"
#include "QC/Node/NodeFrameDescriptor.hpp"
#include "QC/sample/BufferManager.hpp"
#include "QC/sample/SampleIF.hpp"
#include "QRB/Memory/Geometry/PoseDescriptor.hpp"
#include "QRB/Memory/Sensor/PointCloudDescriptor.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_node.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <thread>

namespace QC
{
namespace sample
{

using namespace QC::Node;

class SampleIpcAdapter : public SampleIF
{
public:
    SampleIpcAdapter() = default;
    ~SampleIpcAdapter() = default;

    QCStatus_e Init( std::string name, SampleConfig_t &config ) override;
    QCStatus_e Start() override;
    QCStatus_e Stop() override;
    QCStatus_e Deinit() override;

private:
    QCStatus_e SetupDescriptor();
    QCBufferDescriptorBase_t &NextFrame();
    bool GetMonitor( qrb::adapter::HcmIpcAdapterMonitoringData &monitor );
    void ThreadMain();

private:
    std::unique_ptr<QCNodeIfs> m_node;
    BufferManager *m_pBufMgr = nullptr;
    BufferDescriptor_t m_buffer;
    QRB::Memory::PointCloudDescriptor_t m_cloud;
    QRB::Memory::PoseDescriptor_t m_pose;
    bool m_isPointCloud = true;
    bool m_isHcmToRos = true;
    uint32_t m_fps = 30;
    uint32_t m_payloadBytes = 1048576;
    uint32_t m_bufferBytes = 33554432;

    std::thread m_thread;
    std::atomic<bool> m_stop{ false };
    uint64_t m_frames = 0;
    uint64_t m_errorCount = 0;

    static constexpr const char *kPointCloudContract = "qrb.contract.sensor.v1.PointCloud2";
    static constexpr const char *kPoseContract =
            "qrb.contract.geometry.v1.PoseWithCovarianceStamped";
    // The Adapter reads its descriptor from buffer 0 unless globalBufferIdMap says otherwise.
    static constexpr uint32_t kAdapterBufferId = 0;
    static constexpr uint32_t kPointStep = 3 * sizeof( float );
    static constexpr uint64_t kMaxReportedErrors = 10;
};

QCStatus_e SampleIpcAdapter::Init( std::string name, SampleConfig_t &config )
{
    QCStatus_e ret = SampleIF::Init( name, QC_NODE_TYPE_IPC_ADAPTER );

    std::string contract = Get( config, "contract_type", kPointCloudContract );
    uint32_t direction = Get( config, "direction", (uint32_t) 0 );
    if ( QC_STATUS_OK == ret )
    {
        if ( kPoseContract == contract )
        {
            m_isPointCloud = false;
        }
        else if ( kPointCloudContract != contract )
        {
            QC_ERROR( "unsupported contract_type: %s", contract.c_str() );
            ret = QC_STATUS_BAD_ARGUMENTS;
        }
        else
        {
            m_isPointCloud = true;
        }
        if ( direction > 1 )
        {
            QC_ERROR( "direction must be 0 (HCM to ROS) or 1 (ROS to HCM): %u", direction );
            ret = QC_STATUS_BAD_ARGUMENTS;
        }
        m_isHcmToRos = ( 0 == direction );
        m_fps = Get( config, "fps", (uint32_t) 30 );
        m_payloadBytes = Get( config, "payload_bytes", (uint32_t) 1048576 );
        m_bufferBytes = Get( config, "buffer_bytes", (uint32_t) 33554432 );
    }

    if ( QC_STATUS_OK == ret )
    {
        ret = SetupDescriptor();
    }

    if ( QC_STATUS_OK == ret )
    {
        ret = QCNodeFactory::CreateNode( QC_NODE_TYPE_IPC_ADAPTER, m_node );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "failed to create the IPC Adapter node: %d", ret );
        }
    }

    if ( QC_STATUS_OK == ret )
    {
        std::string defaultTopic =
                m_isPointCloud ? "/qrb_ros_adapter/sample/points" : "/qrb_ros_adapter/sample/pose";
        DataTree dt;
        dt.Set<std::string>( "static.name", name );
        dt.Set<uint32_t>( "static.id", m_nodeId.id );
        dt.Set<std::string>( "static.contract_type", contract );
        dt.Set<uint32_t>( "static.direction", direction );
        dt.Set<std::string>( "static.ros_topic", Get( config, "ros_topic", defaultTopic ) );
        std::string reliability = Get( config, "reliability", "" );
        std::string durability = Get( config, "durability", "" );
        uint32_t depth = Get( config, "depth", (uint32_t) 0 );
        if ( !reliability.empty() )
        {
            dt.Set<std::string>( "static.qos_profile.reliability", reliability );
        }
        if ( !durability.empty() )
        {
            dt.Set<std::string>( "static.qos_profile.durability", durability );
        }
        if ( 0 != depth )
        {
            dt.Set<uint32_t>( "static.qos_profile.depth", depth );
        }

        QCNodeInit_t nodeInit;
        nodeInit.config = dt.Dump();
        ret = m_node->Initialize( nodeInit );
        if ( QC_STATUS_OK != ret )
        {
            // NO_RESOURCE: the ROS Bridge is not running.
            QC_ERROR( "failed to initialize the IPC Adapter node with %s: %d",
                      nodeInit.config.c_str(), ret );
        }
    }

    return ret;
}

QCStatus_e SampleIpcAdapter::SetupDescriptor()
{
    QCStatus_e ret = QC_STATUS_OK;

    if ( false == m_isPointCloud )
    {
        // The descriptor carries the pose itself, it does not need a DMA buffer.
        (void) m_pose.frameId.Assign( "map" );
        return ret;
    }

    uint32_t size = m_isHcmToRos ? m_payloadBytes : m_bufferBytes;
    if ( m_isHcmToRos && ( size < kPointStep ) )
    {
        QC_ERROR( "payload_bytes must hold at least one point: %u", size );
        return QC_STATUS_BAD_ARGUMENTS;
    }

    m_pBufMgr = BufferManager::Get( m_nodeId, m_logger.GetLevel() );
    if ( nullptr == m_pBufMgr )
    {
        QC_ERROR( "failed to get the buffer manager" );
        return QC_STATUS_NOMEM;
    }

    ret = m_pBufMgr->Allocate( BufferProps_t( size ), m_buffer );
    if ( QC_STATUS_OK != ret )
    {
        QC_ERROR( "failed to allocate a %u bytes DMA buffer: %d", size, ret );
        return ret;
    }
    m_cloud = m_buffer;

    if ( m_isHcmToRos )
    {
        // An unordered XYZ cloud: point i is at ( i, i, i ) millimeters.
        m_cloud.width = size / kPointStep;
        m_cloud.height = 1;
        m_cloud.pointStep = kPointStep;
        m_cloud.rowStep = m_cloud.width * kPointStep;
        m_cloud.isDense = true;
        const char *names[3] = { "x", "y", "z" };
        for ( uint32_t i = 0; i < 3; i++ )
        {
            m_cloud.fields[i] = QRB::Memory::PointField_t( names[i], i * (uint32_t) sizeof( float ),
                                                           QRB::Memory::POINT_FIELD_TYPE_FLOAT32 );
        }
        m_cloud.numFields = 3;
        m_cloud.validSize = m_cloud.rowStep;
        (void) m_cloud.frameId.Assign( "map" );
        for ( uint32_t i = 0; i < m_cloud.width; i++ )
        {
            float xyz[3] = { i * 0.001f, i * 0.001f, i * 0.001f };
            (void) memcpy( m_cloud.GetPointPtr( i ), xyz, sizeof( xyz ) );
        }
    }

    return ret;
}

QCStatus_e SampleIpcAdapter::Start()
{
    QCStatus_e ret = m_node->Start();

    if ( QC_STATUS_OK == ret )
    {
        m_stop = false;
        m_thread = std::thread( &SampleIpcAdapter::ThreadMain, this );
    }
    else
    {
        QC_ERROR( "failed to start the IPC Adapter node: %d", ret );
    }

    return ret;
}

QCBufferDescriptorBase_t &SampleIpcAdapter::NextFrame()
{
    uint64_t nowNs = (uint64_t) std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::steady_clock::now().time_since_epoch() )
                             .count();
    if ( m_isPointCloud )
    {
        m_cloud.timestampNs = nowNs;
        return m_cloud;
    }

    // Move along the x axis of "map" by 1 cm per frame.
    m_pose.timestampNs = nowNs;
    m_pose.poseWithCovariance.pose.position.x = (double) m_frames * 0.01;
    return m_pose;
}

bool SampleIpcAdapter::GetMonitor( qrb::adapter::HcmIpcAdapterMonitoringData &monitor )
{
    uint32_t size = sizeof( monitor );
    return QC_STATUS_OK == m_node->GetMonitoringIfs().Place( &monitor, size );
}

void SampleIpcAdapter::ThreadMain()
{
    const auto period = std::chrono::nanoseconds( ( 0 == m_fps ) ? 0 : 1000000000ULL / m_fps );
    auto next = std::chrono::steady_clock::now();
    auto report = next + std::chrono::seconds( 1 );
    uint64_t noData = 0;
    qrb::adapter::HcmIpcAdapterMonitoringData monitor{};

    if ( false == m_isHcmToRos )
    {
        (void) GetMonitor( monitor );
        noData = monitor.noDataCount;
    }

    while ( false == m_stop )
    {
        NodeFrameDescriptor frameDesc( 1 );
        (void) frameDesc.SetBuffer( kAdapterBufferId, NextFrame() );

        QCStatus_e ret = m_node->ProcessFrameDescriptor( frameDesc );
        if ( QC_STATUS_OK != ret )
        {
            m_errorCount++;
            if ( m_errorCount <= kMaxReportedErrors )
            {
                QC_ERROR( "ProcessFrameDescriptor failed: %d", ret );
            }
        }
        else if ( m_isHcmToRos )
        {
            m_frames++;
        }
        else if ( GetMonitor( monitor ) && ( monitor.noDataCount == noData ) )
        {
            // The call fetched a ROS message rather than finding none.
            m_frames++;
        }
        else
        {
            noData = monitor.noDataCount;
        }

        auto now = std::chrono::steady_clock::now();
        if ( now >= report )
        {
            printf( "%s %s %llu frames, %llu errors\n", m_name.c_str(),
                    m_isHcmToRos ? "sent" : "received", (unsigned long long) m_frames,
                    (unsigned long long) m_errorCount );
            report += std::chrono::seconds( 1 );
        }

        next += period;
        if ( next > now )
        {
            std::this_thread::sleep_until( next );
        }
        else
        {
            next = now;
        }
    }
}

QCStatus_e SampleIpcAdapter::Stop()
{
    m_stop = true;
    if ( m_thread.joinable() )
    {
        m_thread.join();
    }

    QCStatus_e ret = m_node->Stop();
    if ( QC_STATUS_OK == ret )
    {
        qrb::adapter::HcmIpcAdapterMonitoringData monitor{};
        if ( GetMonitor( monitor ) )
        {
            printf( "%s %s %llu frames, %llu errors; adapter: frameCount=%llu "
                    "errorCount=%llu noDataCount=%llu averageLatencyNs=%llu "
                    "maxLatencyNs=%llu\n",
                    m_name.c_str(), m_isHcmToRos ? "sent" : "received",
                    (unsigned long long) m_frames, (unsigned long long) m_errorCount,
                    (unsigned long long) monitor.frameCount,
                    (unsigned long long) monitor.errorCount,
                    (unsigned long long) monitor.noDataCount,
                    (unsigned long long) monitor.averageLatencyNs,
                    (unsigned long long) monitor.maxLatencyNs );
        }
        if ( 0 != m_errorCount )
        {
            ret = QC_STATUS_FAIL;
        }
    }
    else
    {
        QC_ERROR( "failed to stop the IPC Adapter node: %d", ret );
    }

    return ret;
}

QCStatus_e SampleIpcAdapter::Deinit()
{
    QCStatus_e ret = QC_STATUS_OK;

    if ( nullptr != m_node )
    {
        ret = m_node->DeInitialize();
        m_node.reset();
    }

    if ( nullptr != m_pBufMgr )
    {
        if ( nullptr != m_buffer.pBuf )
        {
            (void) m_pBufMgr->Free( m_buffer );
        }
        BufferManager::Put( m_pBufMgr );
        m_pBufMgr = nullptr;
    }

    return ret;
}

REGISTER_SAMPLE( IpcAdapter, SampleIpcAdapter );

}   // namespace sample
}   // namespace QC
