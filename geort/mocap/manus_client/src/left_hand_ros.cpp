// SDKMinimalClient_left_ros.cpp
// Left-hand Manus client with ROS2 publishers (mirrors the right-hand behavior)

// Copyright (c) Meta Platforms
// License as per project root

#include "SDKMinimalClient.hpp"
#include "ManusSDKTypes.h"

#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <memory>
#include <mutex>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// ---------------- Orientation helpers ----------------
struct Quaternion { float w, x, y, z; };
struct Vector3 { float x, y, z; };

// Z-Y-X intrinsic (yaw-pitch-roll) from quaternion; output in radians
static Vector3 QuaternionToEuler(const Quaternion& q) {
    Vector3 e{};
    const float sinr_cosp = 2.f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.f - 2.f * (q.x * q.x + q.y * q.y);
    e.x = std::atan2(sinr_cosp, cosr_cosp); // roll

    const float sinp = 2.f * (q.w * q.y - q.z * q.x);
    if (std::fabs(sinp) >= 1.f)
        e.y = std::copysign(M_PI / 2.f, sinp); // pitch, clamp
    else
        e.y = std::asin(sinp);

    const float siny_cosp = 2.f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.f - 2.f * (q.y * q.y + q.z * q.z);
    e.z = std::atan2(siny_cosp, cosy_cosp); // yaw
    return e;
}

SDKMinimalClient* SDKMinimalClient::s_Instance = nullptr;

int main(int argc, char * argv[])
{
    // Init ROS first (so publishers exist before data starts flowing)
    rclcpp::init(argc, argv);

    std::cout << "Starting LEFT-hand minimal client with ROS!\n";
    SDKMinimalClient t_Client;
    t_Client.Initialize();
    std::cout << "Left-hand minimal client initialized.\n";

    t_Client.Run();

    std::cout << "Left-hand minimal client finished. Shutting down.\n";
    t_Client.ShutDown();

    // Clean ROS
    rclcpp::shutdown();
    return 0;
}

SDKMinimalClient::SDKMinimalClient()
{
    s_Instance = this;
}

SDKMinimalClient::~SDKMinimalClient()
{
    s_Instance = nullptr;
}

ClientReturnCode SDKMinimalClient::Initialize()
{
    if (!PlatformSpecificInitialization())
        return ClientReturnCode::ClientReturnCode_FailedPlatformSpecificInitialization;

    const ClientReturnCode t_IntializeResult = InitializeSDK();
    if (t_IntializeResult != ClientReturnCode::ClientReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;

    return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKMinimalClient::InitializeSDK()
{
    const SDKReturnCode t_InitializeResult = CoreSdk_Initialize(SessionType::SessionType_CoreSDK);
    if (t_InitializeResult != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;

    const ClientReturnCode t_CallBackResults = RegisterAllCallbacks();
    if (t_CallBackResults != ::ClientReturnCode::ClientReturnCode_Success)
        return t_CallBackResults;

    CoordinateSystemVUH t_VUH;
    CoordinateSystemVUH_Init(&t_VUH);
    t_VUH.handedness = Side::Side_Left;                 // <<< LEFT
    t_VUH.up         = AxisPolarity::AxisPolarity_PositiveY;
    t_VUH.view       = AxisView::AxisView_ZFromViewer;
    t_VUH.unitScale  = 1.0f;

    const SDKReturnCode t_CoordinateResult = CoreSdk_InitializeCoordinateSystemWithVUH(t_VUH, false);
    if (t_CoordinateResult != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;

    return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKMinimalClient::ShutDown()
{
    const SDKReturnCode t_Result = CoreSdk_ShutDown();
    if (t_Result != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToShutDownSDK;

    if (!PlatformSpecificShutdown())
        return ClientReturnCode::ClientReturnCode_FailedPlatformSpecificShutdown;

    return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKMinimalClient::RegisterAllCallbacks()
{
    const SDKReturnCode t_RegisterSkeletonCallbackResult =
        CoreSdk_RegisterCallbackForSkeletonStream(*OnSkeletonStreamCallback);
    if (t_RegisterSkeletonCallbackResult != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;

    return ClientReturnCode::ClientReturnCode_Success;
}

void SDKMinimalClient::Run()
{
    // --- ROS publishers (left-hand prefixed) ---
    // auto node = std::make_shared<rclcpp::Node>("manus_left_node");
    // auto x_pub   = node->create_publisher<std_msgs::msg::Float32MultiArray>("lh_x_manus_rotations", 10);
    // auto y_pub   = node->create_publisher<std_msgs::msg::Float32MultiArray>("lh_y_manus_rotations", 10);
    // auto z_pub   = node->create_publisher<std_msgs::msg::Float32MultiArray>("lh_z_manus_rotations", 10);
    // auto pos_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("lh_manus_positions", 10);
    // auto quat_pub= node->create_publisher<std_msgs::msg::Float32MultiArray>("lh_manus_quats", 10);

	auto node = std::make_shared<rclcpp::Node>("manus_left_node");
    auto x_pub   = node->create_publisher<std_msgs::msg::Float32MultiArray>("x_manus_rotations", 10);
    auto y_pub   = node->create_publisher<std_msgs::msg::Float32MultiArray>("y_manus_rotations", 10);
    auto z_pub   = node->create_publisher<std_msgs::msg::Float32MultiArray>("z_manus_rotations", 10);
    auto pos_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("manus_positions", 10);
    auto quat_pub= node->create_publisher<std_msgs::msg::Float32MultiArray>("manus_quats", 10);

    std::vector<float> x_rot, y_rot, z_rot, positions, quats;
    std_msgs::msg::Float32MultiArray x_msg, y_msg, z_msg, pos_msg, quat_msg;

    std::cout << "LEFT client: looking for Manus Core host...\n";
    while (Connect() != ClientReturnCode::ClientReturnCode_Success) {
        std::cout << "LEFT client: could not connect. Retrying in 1s.\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    std::cout << "LEFT client connected. Setting up skeleton.\n";
    LoadTestSkeleton();

    Quaternion q{};

    while (m_Running)
    {
        // swap newest skeleton
        m_SkeletonMutex.lock();
        if (m_NextSkeleton != nullptr) {
            if (m_Skeleton != nullptr) delete m_Skeleton;
            m_Skeleton = m_NextSkeleton;
            m_NextSkeleton = nullptr;
        }
        m_SkeletonMutex.unlock();

        if (m_Skeleton != nullptr && !m_Skeleton->skeletons.empty())
        {
            std::cout << "[LH] frame: " << m_FrameCounter << "\n";

            for (const auto& skel : m_Skeleton->skeletons)
            {
                // clear per-frame buffers
                x_rot.clear(); y_rot.clear(); z_rot.clear();
                positions.clear(); quats.clear();

                for (int i = 0; i < skel.info.nodesCount; ++i)
                {
                    const SkeletonNode node = skel.nodes[i];

                    // Positions (XYZ)
                    positions.push_back(node.transform.position.x);
                    positions.push_back(node.transform.position.y);
                    positions.push_back(node.transform.position.z);

                    // Quaternions (XYZW)
                    q.w = node.transform.rotation.w;
                    q.x = node.transform.rotation.x;
                    q.y = node.transform.rotation.y;
                    q.z = node.transform.rotation.z;

                    quats.push_back(q.x);
                    quats.push_back(q.y);
                    quats.push_back(q.z);
                    quats.push_back(q.w);

                    // Euler (roll, pitch, yaw)
                    Vector3 e = QuaternionToEuler(q);
                    x_rot.push_back(e.x);
                    y_rot.push_back(e.y);
                    z_rot.push_back(e.z);
                }

                // Publish
                x_msg.data = x_rot;   x_pub->publish(x_msg);
                y_msg.data = y_rot;   y_pub->publish(y_msg);
                z_msg.data = z_rot;   z_pub->publish(z_msg);
                pos_msg.data = positions; pos_pub->publish(pos_msg);
                quat_msg.data = quats;    quat_pub->publish(quat_msg);
            }
            ++m_FrameCounter;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
        // (no need to spin_some here because we only publish; node handles timers if any)
    }
}

ClientReturnCode SDKMinimalClient::Connect()
{
    SDKReturnCode t_StartResult = CoreSdk_LookForHosts(1, false);
    if (t_StartResult != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToFindHosts;

    uint32_t t_NumberOfHostsFound = 0;
    SDKReturnCode t_NumberResult = CoreSdk_GetNumberOfAvailableHostsFound(&t_NumberOfHostsFound);
    if (t_NumberResult != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToFindHosts;

    if (t_NumberOfHostsFound == 0)
        return ClientReturnCode::ClientReturnCode_FailedToFindHosts;

    std::unique_ptr<ManusHost[]> t_AvailableHosts(new ManusHost[t_NumberOfHostsFound]);
    SDKReturnCode t_HostsResult = CoreSdk_GetAvailableHostsFound(t_AvailableHosts.get(), t_NumberOfHostsFound);
    if (t_HostsResult != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToFindHosts;

    SDKReturnCode t_ConnectResult = CoreSdk_ConnectToHost(t_AvailableHosts[0]);
    if (t_ConnectResult == SDKReturnCode::SDKReturnCode_NotConnected)
        return ClientReturnCode::ClientReturnCode_FailedToConnect;

    return ClientReturnCode::ClientReturnCode_Success;
}

void SDKMinimalClient::LoadTestSkeleton()
{
    uint32_t t_SklIndex = 0;

    SkeletonSetupInfo t_SKL;
    SkeletonSetupInfo_Init(&t_SKL);
    t_SKL.type = SkeletonType::SkeletonType_Hand;
    t_SKL.settings.scaleToTarget = true;
    t_SKL.settings.targetType = SkeletonTargetType::SkeletonTargetType_UserIndexData;
    t_SKL.settings.skeletonTargetUserIndexData.userIndex = 0; // same user index as right-hand is typical
    CopyString(t_SKL.name, sizeof(t_SKL.name), std::string("LeftHand"));

    if (CoreSdk_CreateSkeletonSetup(t_SKL, &t_SklIndex) != SDKReturnCode::SDKReturnCode_Success)
        return;

    if (!SetupHandNodes(t_SklIndex)) return;
    if (!SetupHandChains(t_SklIndex)) return;

    uint32_t t_ID = 0;
    if (CoreSdk_LoadSkeleton(t_SklIndex, &t_ID) != SDKReturnCode::SDKReturnCode_Success)
        return;
}

NodeSetup SDKMinimalClient::CreateNodeSetup(uint32_t p_Id, uint32_t p_ParentId,
                                            float p_PosX, float p_PosY, float p_PosZ,
                                            std::string p_Name)
{
    NodeSetup t_Node;
    NodeSetup_Init(&t_Node);
    t_Node.id = p_Id;
    CopyString(t_Node.name, sizeof(t_Node.name), p_Name);
    t_Node.type = NodeType::NodeType_Joint;
    t_Node.parentID = p_ParentId;
    t_Node.settings.usedSettings = NodeSettingsFlag::NodeSettingsFlag_None;

    t_Node.transform.position.x = p_PosX;
    t_Node.transform.position.y = p_PosY;
    t_Node.transform.position.z = p_PosZ;
    return t_Node;
}

ManusVec3 SDKMinimalClient::CreateManusVec3(float p_X, float p_Y, float p_Z)
{
    ManusVec3 t_Vec{p_X, p_Y, p_Z};
    return t_Vec;
}

bool SDKMinimalClient::SetupHandNodes(uint32_t p_SklIndex)
{
    const uint32_t t_NumFingers = 5;
    const uint32_t t_NumJoints  = 4;

    // Use the LEFT file’s seed positions (fine to keep)
    ManusVec3 t_Fingers[t_NumFingers * t_NumJoints] = {
        CreateManusVec3(0.024950f, 0.000000f, 0.025320f),
        CreateManusVec3(0.000000f, 0.000000f, 0.032742f),
        CreateManusVec3(0.000000f, 0.000000f, 0.028739f),
        CreateManusVec3(0.000000f, 0.000000f, 0.028739f),

        CreateManusVec3(0.011181f, 0.000000f, 0.052904f),
        CreateManusVec3(0.000000f, 0.000000f, 0.038257f),
        CreateManusVec3(0.000000f, 0.000000f, 0.020884f),
        CreateManusVec3(0.000000f, 0.000000f, 0.018759f),

        CreateManusVec3(0.000000f, 0.000000f, 0.051287f),
        CreateManusVec3(0.000000f, 0.000000f, 0.041861f),
        CreateManusVec3(0.000000f, 0.000000f, 0.024766f),
        CreateManusVec3(0.000000f, 0.000000f, 0.019683f),

        CreateManusVec3(-0.011274f, 0.000000f, 0.049802f),
        CreateManusVec3(0.000000f, 0.000000f, 0.039736f),
        CreateManusVec3(0.000000f, 0.000000f, 0.023564f),
        CreateManusVec3(0.000000f, 0.000000f, 0.019868f),

        CreateManusVec3(-0.020145f, 0.000000f, 0.047309f),
        CreateManusVec3(0.000000f, 0.000000f, 0.033175f),
        CreateManusVec3(0.000000f, 0.000000f, 0.018020f),
        CreateManusVec3(0.000000f, 0.000000f, 0.019129f),
    };

    if (CoreSdk_AddNodeToSkeletonSetup(p_SklIndex, CreateNodeSetup(0, 0, 0, 0, 0, "Hand")) !=
        SDKReturnCode::SDKReturnCode_Success)
        return false;

    int t_FingerId = 0;
    for (uint32_t i = 0; i < t_NumFingers; ++i)
    {
        uint32_t t_ParentID = 0;
        for (uint32_t j = 0; j < t_NumJoints; ++j)
        {
            if (CoreSdk_AddNodeToSkeletonSetup(
                    p_SklIndex,
                    CreateNodeSetup(1 + t_FingerId + j, t_ParentID,
                                    t_Fingers[i * 4 + j].x, t_Fingers[i * 4 + j].y, t_Fingers[i * 4 + j].z,
                                    "fingerdigit")) != SDKReturnCode::SDKReturnCode_Success)
            {
                printf("Failed to Add Node To Skeleton Setup.\n");
                return false;
            }
            t_ParentID = 1 + t_FingerId + j;
        }
        t_FingerId += t_NumJoints;
    }
    return true;
}

bool SDKMinimalClient::SetupHandChains(uint32_t p_SklIndex)
{
    {
        ChainSettings t_ChainSettings;
        ChainSettings_Init(&t_ChainSettings);
        t_ChainSettings.usedSettings = ChainType::ChainType_Hand;
        t_ChainSettings.hand.handMotion = HandMotion::HandMotion_IMU;
        t_ChainSettings.hand.fingerChainIdsUsed = 5;
        t_ChainSettings.hand.fingerChainIds[0] = 1;
        t_ChainSettings.hand.fingerChainIds[1] = 2;
        t_ChainSettings.hand.fingerChainIds[2] = 3;
        t_ChainSettings.hand.fingerChainIds[3] = 4;
        t_ChainSettings.hand.fingerChainIds[4] = 5;

        ChainSetup t_Chain;
        ChainSetup_Init(&t_Chain);
        t_Chain.id = 0;
        t_Chain.type = ChainType::ChainType_Hand;
        t_Chain.dataType = ChainType::ChainType_Hand;
        t_Chain.side = Side::Side_Left;   // <<< LEFT
        t_Chain.dataIndex = 0;
        t_Chain.nodeIdCount = 1;
        t_Chain.nodeIds[0] = 0;
        t_Chain.settings = t_ChainSettings;

        if (CoreSdk_AddChainToSkeletonSetup(p_SklIndex, t_Chain) != SDKReturnCode::SDKReturnCode_Success)
            return false;
    }

    const ChainType t_FingerTypes[5] = {
        ChainType::ChainType_FingerThumb,
        ChainType::ChainType_FingerIndex,
        ChainType::ChainType_FingerMiddle,
        ChainType::ChainType_FingerRing,
        ChainType::ChainType_FingerPinky
    };

    for (int i = 0; i < 5; ++i)
    {
        ChainSettings t_ChainSettings;
        ChainSettings_Init(&t_ChainSettings);
        t_ChainSettings.usedSettings = t_FingerTypes[i];
        t_ChainSettings.finger.handChainId = 0;
        t_ChainSettings.finger.metacarpalBoneId = -1;
        t_ChainSettings.finger.useLeafAtEnd = false;

        ChainSetup t_Chain;
        ChainSetup_Init(&t_Chain);
        t_Chain.id = i + 1;
        t_Chain.type = t_FingerTypes[i];
        t_Chain.dataType = t_FingerTypes[i];
        t_Chain.side = Side::Side_Left;   // <<< LEFT
        t_Chain.dataIndex = 0;

        t_Chain.nodeIdCount = 4;
        if (i == 0) {
            t_Chain.nodeIds[0] = 1;
            t_Chain.nodeIds[1] = 2;
            t_Chain.nodeIds[2] = 3;
            t_Chain.nodeIds[3] = 4;
        } else {
            t_Chain.nodeIds[0] = (i * 4) + 1;
            t_Chain.nodeIds[1] = (i * 4) + 2;
            t_Chain.nodeIds[2] = (i * 4) + 3;
            t_Chain.nodeIds[3] = (i * 4) + 4;
        }
        t_Chain.settings = t_ChainSettings;

        if (CoreSdk_AddChainToSkeletonSetup(p_SklIndex, t_Chain) != SDKReturnCode::SDKReturnCode_Success)
            return false;
    }
    return true;
}

void SDKMinimalClient::OnSkeletonStreamCallback(const SkeletonStreamInfo* const p_SkeletonStreamInfo)
{
    if (!s_Instance) return;

    ClientSkeletonCollection* t_NxtClientSkeleton = new ClientSkeletonCollection();
    t_NxtClientSkeleton->skeletons.resize(p_SkeletonStreamInfo->skeletonsCount);

    for (uint32_t i = 0; i < p_SkeletonStreamInfo->skeletonsCount; ++i)
    {
        CoreSdk_GetSkeletonInfo(i, &t_NxtClientSkeleton->skeletons[i].info);
        t_NxtClientSkeleton->skeletons[i].nodes =
            new SkeletonNode[t_NxtClientSkeleton->skeletons[i].info.nodesCount];
        CoreSdk_GetSkeletonData(i,
                                t_NxtClientSkeleton->skeletons[i].nodes,
                                t_NxtClientSkeleton->skeletons[i].info.nodesCount);
    }

    s_Instance->m_SkeletonMutex.lock();
    if (s_Instance->m_NextSkeleton != nullptr) delete s_Instance->m_NextSkeleton;
    s_Instance->m_NextSkeleton = t_NxtClientSkeleton;
    s_Instance->m_SkeletonMutex.unlock();
}