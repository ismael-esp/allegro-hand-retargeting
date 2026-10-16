// SDKMinimalClient_left_ros.cpp
// Left-hand Manus client with ROS2 publishers (mirrors the right-hand behavior)

#include "SDKMinimalClient.hpp"
#include "ManusSDKTypes.h"

#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

struct Quaternion { float w, x, y, z; };
struct Vector3 { float x, y, z; };

// Z-Y-X intrinsic (yaw-pitch-roll) from quaternion; output in radians
static Vector3 QuaternionToEuler(const Quaternion& q) {
    Vector3 e{};
    const float sinr_cosp = 2.f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.f - 2.f * (q.x * q.x + q.y * q.y);
    e.x = std::atan2(sinr_cosp, cosr_cosp); // roll

    const float sinp = 2.f * (q.w * q.y - q.z * q.x);
    if (std::fabs(sinp) >= 1.f) e.y = std::copysign(M_PI / 2.f, sinp);
    else e.y = std::asin(sinp);

    const float siny_cosp = 2.f * (q.w * q.z + q.x * q.y);
    const float cosy_cosp = 1.f - 2.f * (q.y * q.y + q.z * q.z);
    e.z = std::atan2(siny_cosp, cosy_cosp);
    return e;
}

SDKMinimalClient* SDKMinimalClient::s_Instance = nullptr;

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    std::cout << "Starting LEFT-hand minimal client with ROS!\n";
    SDKMinimalClient t_Client;
    t_Client.Initialize();
    std::cout << "Left-hand minimal client initialized.\n";

    t_Client.Run();

    std::cout << "Left-hand minimal client finished. Shutting down.\n";
    t_Client.ShutDown();

    rclcpp::shutdown();
    return 0;
}

SDKMinimalClient::SDKMinimalClient() { s_Instance = this; }
SDKMinimalClient::~SDKMinimalClient() { s_Instance = nullptr; }

ClientReturnCode SDKMinimalClient::Initialize() {
    if (!PlatformSpecificInitialization())
        return ClientReturnCode::ClientReturnCode_FailedPlatformSpecificInitialization;

    const ClientReturnCode r = InitializeSDK();
    if (r != ClientReturnCode::ClientReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;
    return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKMinimalClient::InitializeSDK() {
    if (CoreSdk_Initialize(SessionType::SessionType_CoreSDK) != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;

    if (RegisterAllCallbacks() != ClientReturnCode::ClientReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;

    CoordinateSystemVUH vuh;
    CoordinateSystemVUH_Init(&vuh);
    vuh.handedness = Side::Side_Left; // LEFT
    vuh.up         = AxisPolarity::AxisPolarity_PositiveY;
    vuh.view       = AxisView::AxisView_ZFromViewer;
    vuh.unitScale  = 1.0f;

    if (CoreSdk_InitializeCoordinateSystemWithVUH(vuh, false) != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;

    return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKMinimalClient::ShutDown() {
    if (CoreSdk_ShutDown() != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToShutDownSDK;
    if (!PlatformSpecificShutdown())
        return ClientReturnCode::ClientReturnCode_FailedPlatformSpecificShutdown;
    return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode SDKMinimalClient::RegisterAllCallbacks() {
    if (CoreSdk_RegisterCallbackForSkeletonStream(*OnSkeletonStreamCallback) != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToInitialize;
    return ClientReturnCode::ClientReturnCode_Success;
}

void SDKMinimalClient::Run() {

    auto node = std::make_shared<rclcpp::Node>("manus_left_node");
    auto x_pub   = node->create_publisher<std_msgs::msg::Float32MultiArray>("lh_x_manus_rotations", 10);
    auto y_pub   = node->create_publisher<std_msgs::msg::Float32MultiArray>("lh_y_manus_rotations", 10);
    auto z_pub   = node->create_publisher<std_msgs::msg::Float32MultiArray>("lh_z_manus_rotations", 10);
    auto pos_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("lh_manus_positions", 10);
    auto quat_pub= node->create_publisher<std_msgs::msg::Float32MultiArray>("lh_manus_quats", 10);

    // auto node    = std::make_shared<rclcpp::Node>("manus_left_node");
    // auto x_pub   = node->create_publisher<std_msgs::msg::Float32MultiArray>("x_manus_rotations", 10);
    // auto y_pub   = node->create_publisher<std_msgs::msg::Float32MultiArray>("y_manus_rotations", 10);
    // auto z_pub   = node->create_publisher<std_msgs::msg::Float32MultiArray>("z_manus_rotations", 10);
    // auto pos_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("manus_positions", 10);
    // auto quat_pub= node->create_publisher<std_msgs::msg::Float32MultiArray>("manus_quats", 10);

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
    while (m_Running) {
        m_SkeletonMutex.lock();
        if (m_NextSkeleton != nullptr) {
            if (m_Skeleton != nullptr) delete m_Skeleton;
            m_Skeleton = m_NextSkeleton; m_NextSkeleton = nullptr;
        }
        m_SkeletonMutex.unlock();

        if (m_Skeleton != nullptr && !m_Skeleton->skeletons.empty()) {
            std::cout << "[LH] frame: " << m_FrameCounter << "\n";
            for (const auto& skel : m_Skeleton->skeletons) {
                x_rot.clear(); y_rot.clear(); z_rot.clear(); positions.clear(); quats.clear();
                for (int i = 0; i < skel.info.nodesCount; ++i) {
                    const SkeletonNode node = skel.nodes[i];
                    positions.push_back(node.transform.position.x);
                    positions.push_back(node.transform.position.y);
                    positions.push_back(node.transform.position.z);

                    q.w = node.transform.rotation.w;
                    q.x = node.transform.rotation.x;
                    q.y = node.transform.rotation.y;
                    q.z = node.transform.rotation.z;

                    quats.push_back(q.x); quats.push_back(q.y); quats.push_back(q.z); quats.push_back(q.w);

                    Vector3 e = QuaternionToEuler(q);
                    x_rot.push_back(e.x); y_rot.push_back(e.y); z_rot.push_back(e.z);
                }
                x_msg.data = x_rot; y_msg.data = y_rot; z_msg.data = z_rot;
                pos_msg.data = positions; quat_msg.data = quats;
                x_pub->publish(x_msg); y_pub->publish(y_msg); z_pub->publish(z_msg);
                pos_pub->publish(pos_msg); quat_pub->publish(quat_msg);
            }
            ++m_FrameCounter;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}

ClientReturnCode SDKMinimalClient::Connect() {
    if (CoreSdk_LookForHosts(1, false) != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToFindHosts;

    uint32_t nHosts = 0;
    if (CoreSdk_GetNumberOfAvailableHostsFound(&nHosts) != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
    if (nHosts == 0) return ClientReturnCode::ClientReturnCode_FailedToFindHosts;

    std::unique_ptr<ManusHost[]> hosts(new ManusHost[nHosts]);
    if (CoreSdk_GetAvailableHostsFound(hosts.get(), nHosts) != SDKReturnCode::SDKReturnCode_Success)
        return ClientReturnCode::ClientReturnCode_FailedToFindHosts;

    if (CoreSdk_ConnectToHost(hosts[0]) == SDKReturnCode::SDKReturnCode_NotConnected)
        return ClientReturnCode::ClientReturnCode_FailedToConnect;

    return ClientReturnCode::ClientReturnCode_Success;
}

void SDKMinimalClient::LoadTestSkeleton() {
    uint32_t skl = 0;
    SkeletonSetupInfo s; SkeletonSetupInfo_Init(&s);
    s.type = SkeletonType::SkeletonType_Hand;
    s.settings.scaleToTarget = true;
    s.settings.targetType = SkeletonTargetType::SkeletonTargetType_UserIndexData;
    s.settings.skeletonTargetUserIndexData.userIndex = 0;
    CopyString(s.name, sizeof(s.name), std::string("LeftHand"));
    if (CoreSdk_CreateSkeletonSetup(s, &skl) != SDKReturnCode::SDKReturnCode_Success) return;
    if (!SetupHandNodes(skl)) return;
    if (!SetupHandChains(skl)) return;
    uint32_t id = 0;
    if (CoreSdk_LoadSkeleton(skl, &id) != SDKReturnCode::SDKReturnCode_Success) return;
}

NodeSetup SDKMinimalClient::CreateNodeSetup(uint32_t id, uint32_t parent,
                                            float px, float py, float pz,
                                            std::string name) {
    NodeSetup node; NodeSetup_Init(&node);
    node.id = id; CopyString(node.name, sizeof(node.name), name);
    node.type = NodeType::NodeType_Joint;
    node.parentID = parent;
    node.settings.usedSettings = NodeSettingsFlag::NodeSettingsFlag_None;
    node.transform.position.x = px; node.transform.position.y = py; node.transform.position.z = pz;
    return node;
}

ManusVec3 SDKMinimalClient::CreateManusVec3(float x, float y, float z) { ManusVec3 v{x,y,z}; return v; }

bool SDKMinimalClient::SetupHandNodes(uint32_t skl) {
    const uint32_t NF=5, NJ=4;
    ManusVec3 f[NF*NJ] = {
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

    if (CoreSdk_AddNodeToSkeletonSetup(skl, CreateNodeSetup(0,0,0,0,0,"Hand")) != SDKReturnCode::SDKReturnCode_Success)
        return false;

    int fid=0;
    for (uint32_t i=0;i<NF;i++){
        uint32_t parent=0;
        for (uint32_t j=0;j<NJ;j++){
            if (CoreSdk_AddNodeToSkeletonSetup(
                    skl, CreateNodeSetup(1+fid+j, parent, f[i*4+j].x, f[i*4+j].y, f[i*4+j].z, "fingerdigit"))
                != SDKReturnCode::SDKReturnCode_Success) return false;
            parent = 1+fid+j;
        }
        fid += NJ;
    }
    return true;
}

bool SDKMinimalClient::SetupHandChains(uint32_t skl) {
    {
        ChainSettings cs; ChainSettings_Init(&cs);
        cs.usedSettings = ChainType::ChainType_Hand;
        cs.hand.handMotion = HandMotion::HandMotion_IMU;
        cs.hand.fingerChainIdsUsed = 5;
        cs.hand.fingerChainIds[0]=1; cs.hand.fingerChainIds[1]=2; cs.hand.fingerChainIds[2]=3;
        cs.hand.fingerChainIds[3]=4; cs.hand.fingerChainIds[4]=5;

        ChainSetup c; ChainSetup_Init(&c);
        c.id=0; c.type=ChainType::ChainType_Hand; c.dataType=ChainType::ChainType_Hand;
        c.side=Side::Side_Left; c.dataIndex=0; c.nodeIdCount=1; c.nodeIds[0]=0; c.settings=cs;

        if (CoreSdk_AddChainToSkeletonSetup(skl, c) != SDKReturnCode::SDKReturnCode_Success) return false;
    }

    const ChainType FT[5] = {
        ChainType::ChainType_FingerThumb, ChainType::ChainType_FingerIndex,
        ChainType::ChainType_FingerMiddle, ChainType::ChainType_FingerRing,
        ChainType::ChainType_FingerPinky};

    for (int i=0;i<5;i++){
        ChainSettings cs; ChainSettings_Init(&cs);
        cs.usedSettings = FT[i];
        cs.finger.handChainId = 0; cs.finger.metacarpalBoneId = -1; cs.finger.useLeafAtEnd = false;

        ChainSetup c; ChainSetup_Init(&c);
        c.id=i+1; c.type=FT[i]; c.dataType=FT[i]; c.side=Side::Side_Left; c.dataIndex=0;
        c.nodeIdCount=4;
        if (i==0){ c.nodeIds[0]=1; c.nodeIds[1]=2; c.nodeIds[2]=3; c.nodeIds[3]=4; }
        else     { c.nodeIds[0]=(i*4)+1; c.nodeIds[1]=(i*4)+2; c.nodeIds[2]=(i*4)+3; c.nodeIds[3]=(i*4)+4; }
        c.settings=cs;
        if (CoreSdk_AddChainToSkeletonSetup(skl, c) != SDKReturnCode::SDKReturnCode_Success) return false;
    }
    return true;
}

void SDKMinimalClient::OnSkeletonStreamCallback(const SkeletonStreamInfo* const info) {
    if (!s_Instance) return;
    ClientSkeletonCollection* nxt = new ClientSkeletonCollection();
    nxt->skeletons.resize(info->skeletonsCount);
    for (uint32_t i=0;i<info->skeletonsCount;i++){
        CoreSdk_GetSkeletonInfo(i, &nxt->skeletons[i].info);
        nxt->skeletons[i].nodes = new SkeletonNode[nxt->skeletons[i].info.nodesCount];
        CoreSdk_GetSkeletonData(i, nxt->skeletons[i].nodes, nxt->skeletons[i].info.nodesCount);
    }
    s_Instance->m_SkeletonMutex.lock();
    if (s_Instance->m_NextSkeleton != nullptr) delete s_Instance->m_NextSkeleton;
    s_Instance->m_NextSkeleton = nxt;
    s_Instance->m_SkeletonMutex.unlock();
}
