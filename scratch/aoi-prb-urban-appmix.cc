// Scenario: Single-cell NR with the same urban deployment/control logic as
// aoi-prb-urban-onoff.cc, but with semantically-meaningful traffic classes:
// light FTP-style file/object fetch, moderate cloud gaming, and heavy
// NGMN video streaming.
// codex resume 019c7023-f3e2-7e00-acea-5d1b1b73577d

#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/buildings-channel-condition-model.h"
#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/ideal-beamforming-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#if __has_include("ns3/opengym-module.h")
#define HAVE_OPENGYM
#include "ns3/opengym-module.h"
#endif
#include "ns3/bwp-manager-gnb.h"
#include "ns3/bwp-manager-ue.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-epc-tft.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-gnb-mac.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-mac-scheduler-ofdma-aequitas.h"
#include "ns3/nr-mac-scheduler-ofdma-baseline.h"
#include "ns3/nr-mac-scheduler-ofdma-pf.h"
#include "ns3/nr-mac-scheduler-ofdma-rr.h"
#include "ns3/nr-mac-scheduler-ns3.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/ping-helper.h"
#include "ns3/traffic-generator-helper.h"
#include "ns3/traffic-generator-ngmn-ftp-multi.h"
#include "ns3/traffic-generator-ngmn-gaming.h"
#include "ns3/traffic-generator-ngmn-video.h"
#include "ns3/seq-ts-size-header.h"
#include "ns3/tag.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AoiPrbUrbanOnOff");

enum TrafficType : uint8_t
{
    LIGHT_HTTP = 0,
    MODERATE_GAMING = 1,
    HEAVY_VIDEO = 2,
    TRAFFIC_TYPES = 3
};

static constexpr uint8_t G_DPP_ACTION_COUNT = 6;

class TxTimeTag : public Tag
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::TxTimeTag").SetParent<Tag>().AddConstructor<TxTimeTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    uint32_t GetSerializedSize() const override
    {
        return sizeof(uint64_t);
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU64(m_txTimeNs);
    }

    void Deserialize(TagBuffer i) override
    {
        m_txTimeNs = i.ReadU64();
    }

    void Print(std::ostream& os) const override
    {
        os << m_txTimeNs;
    }

    void SetTxTime(Time t)
    {
        m_txTimeNs = static_cast<uint64_t>(t.GetNanoSeconds());
    }

    Time GetTxTime() const
    {
        return NanoSeconds(m_txTimeNs);
    }

  private:
    uint64_t m_txTimeNs{0};
};

class AoiIdentityTag : public Tag
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::AoiIdentityTag").SetParent<Tag>().AddConstructor<AoiIdentityTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    uint32_t GetSerializedSize() const override
    {
        return sizeof(uint64_t) + sizeof(uint32_t) + sizeof(uint8_t) + sizeof(uint64_t);
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU64(m_packetId);
        i.WriteU32(m_ueIdx);
        i.WriteU8(m_trafficType);
        i.WriteU64(m_txTimeNs);
    }

    void Deserialize(TagBuffer i) override
    {
        m_packetId = i.ReadU64();
        m_ueIdx = i.ReadU32();
        m_trafficType = i.ReadU8();
        m_txTimeNs = i.ReadU64();
    }

    void Print(std::ostream& os) const override
    {
        os << "pkt=" << m_packetId << " ue=" << m_ueIdx << " class="
           << static_cast<uint32_t>(m_trafficType) << " txNs=" << m_txTimeNs;
    }

    void SetPacketId(uint64_t packetId)
    {
        m_packetId = packetId;
    }

    uint64_t GetPacketId() const
    {
        return m_packetId;
    }

    void SetUeIdx(uint32_t ueIdx)
    {
        m_ueIdx = ueIdx;
    }

    uint32_t GetUeIdx() const
    {
        return m_ueIdx;
    }

    void SetTrafficType(uint8_t trafficType)
    {
        m_trafficType = trafficType;
    }

    uint8_t GetTrafficType() const
    {
        return m_trafficType;
    }

    void SetTxTime(Time t)
    {
        m_txTimeNs = static_cast<uint64_t>(t.GetNanoSeconds());
    }

    Time GetTxTime() const
    {
        return NanoSeconds(m_txTimeNs);
    }

  private:
    uint64_t m_packetId{0};
    uint32_t m_ueIdx{0};
    uint8_t m_trafficType{0};
    uint64_t m_txTimeNs{0};
};

struct FlowStats
{
    uint64_t rxBytes{0};
    uint64_t rxPkts{0};
    double aoiSumMs{0.0};
    uint64_t aoiSamples{0};
    double lastRxTime{0.0};
};

struct UeStats
{
    std::array<FlowStats, TRAFFIC_TYPES> flow{};
};

struct PrbStats
{
    uint64_t prbTotal{0};
    uint64_t tbBytes{0};
    uint64_t tbCount{0};
    uint64_t tbError{0};
    double mcsSum{0.0};
    uint64_t mcsCount{0};
    double tblerSum{0.0};
    uint64_t tblerCount{0};
    double sinrSumDb{0.0};
    uint64_t sinrSamples{0};
};

enum RadioEnvState : uint8_t
{
    ENV_LOS = 0,
    ENV_NLOS = 1,
    ENV_STATES = 2
};

struct EnvKpiStats
{
    uint64_t rxBytes{0};
    uint64_t rxPkts{0};
    double aoiSumMs{0.0};
    uint64_t aoiSamples{0};
    double dwellTimeS{0.0};
    uint64_t dwellSamples{0};
};

struct PerUeTrafficProfile
{
    double burstRateMbps{100.0};
    uint32_t burstPktSize{1200};
    bool burstRandomize{true};
    double burstOnMs{180.0};
    double burstOffMs{120.0};
    double burstOnMinMs{140.0};
    double burstOnMaxMs{260.0};
    double burstOffMinMs{80.0};
    double burstOffMaxMs{220.0};
    double backgroundRateKbps{500.0};
    uint32_t backgroundPktSize{400};
    std::string trafficClass{"legacy"};
};

struct AppStateSegment
{
    double startS{0.0};
    double endS{0.0};
    int32_t state{0}; // 0=ftp_only, 1=video_only, 2=ftp_video
};

static std::vector<UeStats> g_ueStats;
static std::vector<PrbStats> g_prbStats;
static std::vector<std::array<EnvKpiStats, ENV_STATES>> g_envStatsByUe;
static Ptr<BuildingsChannelConditionModel> g_losConditionModel;
static Ptr<MobilityModel> g_gnbMobility;
static std::vector<Ptr<MobilityModel>> g_ueMobilityByIdx;
static std::vector<int8_t> g_lastLosStateByUe;
static std::vector<double> g_lastLosStateSampleTimeByUe;
static std::vector<double> g_policyBaseStartSByUe;
static std::vector<int32_t> g_policyPhaseOffsetByUe;
static double g_policySegmentDurationS = 0.6;
static std::vector<std::vector<AppStateSegment>> g_appStateTimelineByUe;
static std::vector<double> g_packetLevelAoiSamplesMs;
static std::array<std::vector<double>, TRAFFIC_TYPES> g_packetLevelAoiSamplesByTypeMs;

static Ptr<BwpManagerGnb> g_gnbBwpMgr;
static std::unordered_map<uint16_t, Ptr<BwpManagerUe>> g_ueMgrByRnti;
static uint8_t g_lowBwpId = 0;
static uint8_t g_highBwpId = 1;
static uint8_t g_initialBwpId = g_lowBwpId;
static uint8_t g_mcsLowThreshold = 8;
static uint8_t g_mcsHighThreshold = 12;
static uint32_t g_mcsSwitchCount = 5;
static double g_minSwitchIntervalMs = 0.0;
static bool g_enableMcsSwitch = false;
static bool g_enableHarqReTx = true;

struct McsSwitchState
{
    uint32_t lowCount{0};
    uint32_t highCount{0};
    double lastSwitchTime{0.0};
};

static std::unordered_map<uint16_t, McsSwitchState> g_mcsStateByRnti;
static std::unordered_map<uint16_t, uint32_t> g_ueIdxByRnti;
static std::vector<uint16_t> g_rntiByUeIdx;
static std::vector<uint32_t> g_totalPrbByBwp;

// Control loop state
static bool g_enableOpenGym = false;
static uint32_t g_openGymPort = 5555;
static bool g_enableRlBwpControl = true;
static bool g_enableRlMcsControl = false;
static bool g_rlDebug = false;
static double g_rewardLambdaSwitch = 0.01;
static double g_envStepTime = 0.02; // 20 ms
static bool g_enableLosNlosStats = true;
static double g_losSamplePeriodS = 0.02;
static double g_simTime = 10.0;
static double g_appStartTime = 0.1;
static double g_switchDelayMsCfg = 3.0;
static uint32_t g_switchDelaySymbols = 14;
static uint8_t g_switchDelayNumerology = 0;
static bool g_useSymbolicSwitchDelay = false;
static double g_queueNormBytes = 200000.0;
static uint8_t g_initialMcs = 10;
static bool g_enablePerUeMcsControl = false;
static uint32_t g_rlcMaxTxBufferBytes = 512 * 1024;    // 256 KB
static int32_t g_staticMcsOffset = 0;
static double g_prbDemandC0 = 5.0;
static double g_prbDemandC1 = 2.0;

static std::vector<std::array<uint64_t, TRAFFIC_TYPES>> g_txBytesByType;
static std::vector<std::array<uint64_t, TRAFFIC_TYPES>> g_txPacketsByType;
static std::vector<std::array<uint64_t, TRAFFIC_TYPES>> g_rxPacketsByType;
static std::vector<std::array<uint64_t, TRAFFIC_TYPES>> g_rxMissingTagPacketsByType;
static std::vector<std::array<double, TRAFFIC_TYPES>> g_lastDeliveredAoiMs;
static std::vector<std::array<double, TRAFFIC_TYPES>> g_lastRxTimeByType;
static std::vector<double> g_lastNodeDeliveredAoiMsByUe;
static std::vector<double> g_lastNodeRxTimeByUe;
static std::vector<double> g_lastCqiByUe;
static std::vector<double> g_lastSinrDbByUe;
static std::vector<double> g_lastMcsByUe;
static std::vector<double> g_lastTblerByUe;
static std::vector<double> g_lastRequestedMcsOffsetByUe;
static std::vector<double> g_lastBwpSwitchTimeSByUe;
static std::vector<double> g_rlcDlQueueBytesByUe;
static std::vector<std::unordered_map<uint8_t, double>> g_rlcDlQueueBytesByUeAndLcid;
static bool g_rlcTxBufferTraceConnected = false;
static bool g_rlcAoiLifecycleTraceConnected = false;
static std::vector<double> g_prevRewardQueueBytesByUe;
static std::vector<uint8_t> g_currentBwpByUe;
static std::vector<double> g_switchCooldownUntilSByUe;
static std::vector<uint64_t> g_stepAssignedPrbByUe;
static std::vector<uint64_t> g_stepTbBytesByUe;
static std::vector<uint64_t> g_stepTbCountByUe;
static std::vector<uint64_t> g_stepTbErrorByUe;
static std::vector<uint64_t> g_rxPrbCumByUe;
static std::vector<uint64_t> g_rxTbBytesCumByUe;
static std::vector<uint64_t> g_rxTbCountCumByUe;
static std::vector<uint64_t> g_rxTbErrorCumByUe;
static std::vector<uint64_t> g_rxTbErrorBytesCumByUe;
static std::vector<uint64_t> g_schedCountCumByUe;
static std::vector<uint64_t> g_harqFlushCountByUe;
static std::vector<uint64_t> g_bwpSwitchExecCountByUe;
static uint64_t g_bwpSwitchExec0To1Count = 0;
static uint64_t g_bwpSwitchExec1To0Count = 0;
static std::vector<uint64_t> g_outstandingAoiBytesByUe;
static std::vector<uint64_t> g_lastDeliveryTimeNsByUe;
static std::vector<uint64_t> g_lastStepAssignedPrbByUe;
static std::vector<double> g_lastStepSpectralEfficiencyByUe;
static std::vector<double> g_lastBwpStepSpectralEfficiency;
static std::vector<double> g_lastBwpAvgMcs;
static std::vector<uint8_t> g_targetMcsByUe;
static std::array<double, 2> g_bwpOccupancyUeTimeS = {0.0, 0.0};
static double g_bwpOccupancySamplePeriodS = 0.001; // 1 ms sampling

static std::vector<Ptr<NrMacSchedulerNs3>> g_dlSchedulers;
static std::vector<Ptr<NrGnbMac>> g_gnbMacs;
static std::vector<Ptr<NrMacSchedulerOfdmaAequitas>> g_aequitasSchedulers;
static std::vector<Ptr<NrMacSchedulerOfdmaBaseline>> g_baselineSchedulers;

struct PendingBwpSwitch
{
    EventId m_deadlineEvent;
    uint8_t m_targetBwp{0};
};

static std::unordered_map<uint16_t, PendingBwpSwitch> g_pendingBwpSwitchByRnti;
static void TriggerDppPolicyEpoch();

static uint32_t g_lastIntervalSwitchCount = 0;
static uint32_t g_nextIntervalSwitchCount = 0;
static double g_lastMeanAoiMs = 0.0;
static double g_lastQueueOverflowRatio = 0.0;
static double g_lastMeanPrbUtility = 0.0;
static double g_lastMeanRewardAoiTerm = 0.0;
static double g_lastMeanRewardThrTerm = 0.0;
static double g_lastMeanRewardSeTerm = 0.0;
static double g_lastMeanRewardSwitchPenalty = 0.0;
static double g_lastMeanRewardTotal = 0.0;
static uint32_t g_lastRequestedBwp0Count = 0;
static uint32_t g_lastRequestedBwp1Count = 0;
static uint32_t g_nextRequestedBwp0Count = 0;
static uint32_t g_nextRequestedBwp1Count = 0;
static uint32_t g_lastSwitchRejectCooldownCount = 0;
static uint32_t g_lastSwitchRejectNoRntiCount = 0;
static uint32_t g_lastSwitchRejectSameTargetCount = 0;
static uint32_t g_lastSwitchRejectMgrBusyCount = 0;
static uint32_t g_lastSwitchRejectMgrMissingCount = 0;
static uint32_t g_lastSwitchRejectUeMgrMissingCount = 0;
static uint32_t g_nextSwitchRejectCooldownCount = 0;
static uint32_t g_nextSwitchRejectNoRntiCount = 0;
static uint32_t g_nextSwitchRejectSameTargetCount = 0;
static uint32_t g_nextSwitchRejectMgrBusyCount = 0;
static uint32_t g_nextSwitchRejectMgrMissingCount = 0;
static uint32_t g_nextSwitchRejectUeMgrMissingCount = 0;
static std::vector<double> g_prevRewardAoiMsByUe;
static std::vector<double> g_prevRewardThrMbpsByUe;
static std::vector<uint64_t> g_prevRewardTxBytesByUe;
static std::vector<uint64_t> g_prevRewardRxBytesByUe;
static std::vector<std::array<uint64_t, TRAFFIC_TYPES>> g_prevRewardTxBytesByType;
static std::vector<double> g_lastRewardPrevQueueBytesByUe;
static std::vector<double> g_lastRewardPrevAoiMsByUe;
static std::vector<uint64_t> g_lastRewardArrivedBytesByUe;
static std::vector<uint64_t> g_lastRewardDeliveredBytesByUe;
static uint64_t g_nextAoiPacketId = 1;

struct OutstandingAoiPacketInfo
{
    uint32_t ueIdx{0};
    uint8_t trafficType{0};
    uint64_t enqueueTimeNs{0};
    uint64_t txTimeNs{0};
    uint32_t packetBytes{0};
};

static std::unordered_map<uint64_t, OutstandingAoiPacketInfo> g_outstandingAoiPacketById;
static std::vector<std::array<std::multimap<uint64_t, uint64_t>, TRAFFIC_TYPES>> g_outstandingAoiByUeType;
static std::vector<std::multimap<uint64_t, uint64_t>> g_outstandingAoiByUe;
static std::vector<uint32_t> g_lastActualSwitchByUe;
static std::vector<uint32_t> g_nextActualSwitchByUe;

struct IntervalMetricAccumulator
{
    double meanThrMbpsSum{0.0};
    double meanAoiMsSum{0.0};
    double meanPrbUtilitySum{0.0};
    double queueOverflowRatioSum{0.0};
    double switchCountSum{0.0};
    double rewardAoiTermSum{0.0};
    double rewardThrTermSum{0.0};
    double rewardSeTermSum{0.0};
    double rewardSwitchPenaltySum{0.0};
    double rewardTotalSum{0.0};
    uint64_t samples{0};
};

static IntervalMetricAccumulator g_intervalMetrics;
static std::string g_metricsTraceFile = "";
static std::unique_ptr<std::ofstream> g_metricsTraceStream;
static std::string g_aoiTraceFile = "";
static std::unique_ptr<std::ofstream> g_aoiTraceStream;
static int32_t g_aoiTraceUe = 0;
static std::string g_aoiStateTraceFile = "";
static std::unique_ptr<std::ofstream> g_aoiStateTraceStream;
static int32_t g_aoiStateTraceUe = 0;
static std::string g_queueTraceFile = "";
static std::unique_ptr<std::ofstream> g_queueTraceStream;
static int32_t g_queueTraceUe = 0;
static std::string g_appStateTraceFile = "";
static std::unique_ptr<std::ofstream> g_appStateTraceStream;
static int32_t g_appStateTraceUe = 0;
static std::string g_burstStateTraceFile = "";
static std::unique_ptr<std::ofstream> g_burstStateTraceStream;
static std::string g_causeTraceFile = "";
static std::unique_ptr<std::ofstream> g_causeTraceStream;
static double g_causeSamplePeriodS = 0.005;
static std::vector<uint64_t> g_causePrevRxPrbByUe;
static std::vector<uint64_t> g_causePrevRxTbBytesByUe;
static std::vector<uint64_t> g_causePrevRxTbCountByUe;
static std::vector<uint64_t> g_causePrevRxTbErrorByUe;
static std::vector<uint64_t> g_causePrevSchedCountByUe;
static std::string g_sinrTraceFile = "";
static std::unique_ptr<std::ofstream> g_sinrTraceStream;
static double g_sinrSamplePeriodS = 0.01;
static std::string g_layoutTraceFile = "";
static std::string g_trajectoryTraceFile = "";
static double g_trajectorySamplePeriodS = 0.1;
static std::unique_ptr<std::ofstream> g_trajectoryTraceStream;
static bool g_dppScoreDebug = false;
static std::string g_dppScoreTraceFile = "";
static std::unique_ptr<std::ofstream> g_dppScoreTraceStream;
static int32_t g_dppScoreTraceUe = -1;
static bool g_debugDpp = false;
static std::string g_dppPosteriorTraceFile = "";
static std::unique_ptr<std::ofstream> g_dppPosteriorTraceStream;
static std::string g_dppTrafficStateTraceFile = "";
static std::unique_ptr<std::ofstream> g_dppTrafficStateTraceStream;

static std::string g_schedulerPolicy = "rr"; // rr|pf|aequitas|age_optimal|tps|dgs

// Baseline policy configuration
static std::string g_bwpBaseline = "none"; // none|dt|aequitas|queue|dpp
static std::string g_mcsBaseline = "cqi"; // cqi|aams
static double g_aequitasDeadlineMs = 100.0;
static bool g_aequitasEnableMcsSelection = true;
static uint32_t g_policySwitchCooldownSteps = 2;
static std::vector<uint32_t> g_policyCooldownStepsByUe;

// Queue-Threshold BWP baseline parameters (Device_Power_Saving paper)
static uint32_t g_bwpQueueThreshold = 1000; // threshold in bytes

// AAMS MCS baseline parameters
static double g_aamsTargetBler = 0.10; // 10% target BLER
static uint8_t g_aamsMcsOffset = 2;

// Age-Optimal scheduling parameters
static double g_ageOptimalAoiThresholdMs = 20.0;
static double g_ageOptimalGammaPenalty = 20.0;
static double g_tpsDeadlineMs = 100.0;
static double g_dgsDelayTargetMs = 100.0;
static bool g_rlDrqnProfile = false;
static double g_dqnDelayTargetMs = 80.0;
static double g_dqnThrTargetMbps = 2.0;
static double g_dqnLatencyUeRatio = 0.5;
static double g_dqnAlpha = 0.7; // delay weight for latency-sensitive UEs
static double g_dqnBeta = 0.3;  // throughput weight for latency-sensitive UEs
static int32_t g_appmixStateOverride = -1;

// DT-like predictive baseline parameters
static double g_dtDelayTargetMs = 80.0;
static double g_dtQueueHigh = 0.70;
static double g_dtQueueLow = 0.20;
static double g_dtPredictGainAoi = 0.8;
static double g_dtPredictGainQueue = 0.8;
static double g_dtEmaAlpha = 0.25;
static std::vector<double> g_dtEmaAoiMsByUe;
static std::vector<double> g_dtEmaQueueNormByUe;

// DPP baseline parameters and trigger-epoch state.
static double g_dppV = 1;
static double g_dppLambdaSwitch = 0.5;
static double g_dppLambdaBler = 0.5;
static double g_dppEpochMinIntervalS = 0.010; // 10 ms
static double g_dppLastEpochS = -1.0;
static uint32_t g_dppSwitchCountAccum = 0;
static EventId g_dppEpochEvent;
static double g_dppMuPriorMeanBytes = 1.0;
static double g_dppMuPriorPrecision = 1.0;
static double g_dppMuObsPrecision = 2.0;
static double g_dppErrPriorMeanBytes = 0.0;
static double g_dppErrPriorPrecision = 1.0;
static double g_dppErrObsPrecision = 2.0;
static double g_dppPosteriorDiscount = 0.9;
static double g_dppExploreRandomDurationS = 1.0; // random exploration only during early warmup
static Ptr<UniformRandomVariable> g_dppExploreRv;
static std::vector<std::array<double, G_DPP_ACTION_COUNT>> g_dppMuPostMeanByUe;
static std::vector<std::array<double, G_DPP_ACTION_COUNT>> g_dppMuPostPrecisionByUe;
static std::vector<std::array<double, G_DPP_ACTION_COUNT>> g_dppErrPostMeanByUe;
static std::vector<std::array<double, G_DPP_ACTION_COUNT>> g_dppErrPostPrecisionByUe;
static std::vector<int32_t> g_dppLastActionByUe;
static std::vector<bool> g_dppHasLastActionByUe;
static std::vector<double> g_dppLastDecisionTimeSByUe;
static std::vector<double> g_dppLastEpochDeltaSByUe;
static std::vector<uint64_t> g_dppLastTbBytesCumByUe;
static std::vector<uint64_t> g_dppLastTbErrorBytesCumByUe;

static bool
HasPendingBwpSwitch(uint16_t rnti)
{
    auto it = g_pendingBwpSwitchByRnti.find(rnti);
    return it != g_pendingBwpSwitchByRnti.end() && it->second.m_deadlineEvent.IsPending();
}

static bool
GetPendingBwpSwitchByUe(uint32_t ueIdx, uint8_t* targetBwp)
{
    if (ueIdx >= g_rntiByUeIdx.size())
    {
        return false;
    }
    const uint16_t rnti = g_rntiByUeIdx[ueIdx];
    if (rnti == 0)
    {
        return false;
    }
    auto it = g_pendingBwpSwitchByRnti.find(rnti);
    if (it == g_pendingBwpSwitchByRnti.end() || !it->second.m_deadlineEvent.IsPending())
    {
        return false;
    }
    if (targetBwp)
    {
        *targetBwp = it->second.m_targetBwp;
    }
    return true;
}

static void
FlushDlHarqStateForUe(uint16_t rnti, uint8_t bwpId)
{
    if (bwpId >= g_dlSchedulers.size() || bwpId >= g_gnbMacs.size())
    {
        return;
    }

    Ptr<NrMacSchedulerNs3> sched = g_dlSchedulers[bwpId];
    if (sched)
    {
        sched->FlushDlHarqProcesses(rnti);
    }
    Ptr<NrGnbMac> gnbMac = g_gnbMacs[bwpId];
    if (gnbMac)
    {
        gnbMac->FlushDlHarqBuffers(rnti);
    }
    auto idxIt = g_ueIdxByRnti.find(rnti);
    if (idxIt != g_ueIdxByRnti.end() && idxIt->second < g_harqFlushCountByUe.size())
    {
        g_harqFlushCountByUe[idxIt->second]++;
    }
}

static void
ExecutePendingBwpSwitch(uint16_t rnti, uint8_t targetBwp)
{
    auto ueIt = g_ueMgrByRnti.find(rnti);
    if (!g_gnbBwpMgr || ueIt == g_ueMgrByRnti.end())
    {
        g_pendingBwpSwitchByRnti.erase(rnti);
        return;
    }

    uint8_t currentBwp = g_initialBwpId;
    uint8_t forced = g_gnbBwpMgr->GetForcedUeBwp(rnti);
    if (forced != std::numeric_limits<uint8_t>::max())
    {
        currentBwp = forced;
    }
    if (currentBwp == g_lowBwpId && targetBwp == g_highBwpId)
    {
        g_bwpSwitchExec0To1Count++;
    }
    else if (currentBwp == g_highBwpId && targetBwp == g_lowBwpId)
    {
        g_bwpSwitchExec1To0Count++;
    }
    FlushDlHarqStateForUe(rnti, currentBwp);

    g_gnbBwpMgr->ForceUeBwp(rnti, targetBwp);
    ueIt->second->ForceActiveBwp(targetBwp);
    g_mcsStateByRnti[rnti].lastSwitchTime = Simulator::Now().GetSeconds();
    auto idxIt = g_ueIdxByRnti.find(rnti);
    if (idxIt != g_ueIdxByRnti.end() && idxIt->second < g_currentBwpByUe.size())
    {
        uint32_t ueIdx = idxIt->second;
        g_currentBwpByUe[ueIdx] = targetBwp;
        if (ueIdx < g_bwpSwitchExecCountByUe.size())
        {
            g_bwpSwitchExecCountByUe[ueIdx]++;
        }
    }
    g_pendingBwpSwitchByRnti.erase(rnti);
}

static bool
SwitchUeBwp(uint16_t rnti, uint8_t targetBwp)
{
    if (!g_gnbBwpMgr)
    {
        g_nextSwitchRejectMgrMissingCount++;
        return false;
    }
    if (g_gnbBwpMgr->IsSwitching(rnti) || HasPendingBwpSwitch(rnti))
    {
        g_nextSwitchRejectMgrBusyCount++;
        return false;
    }
    auto ueIt = g_ueMgrByRnti.find(rnti);
    if (ueIt == g_ueMgrByRnti.end())
    {
        g_nextSwitchRejectUeMgrMissingCount++;
        return false;
    }

    PendingBwpSwitch pending;
    pending.m_targetBwp = targetBwp;
    pending.m_deadlineEvent =
        Simulator::Schedule(MilliSeconds(g_switchDelayMsCfg), &ExecutePendingBwpSwitch, rnti, targetBwp);
    g_pendingBwpSwitchByRnti[rnti] = pending;

    auto idxIt = g_ueIdxByRnti.find(rnti);
    if (idxIt != g_ueIdxByRnti.end() && idxIt->second < g_switchCooldownUntilSByUe.size())
    {
        uint32_t ueIdx = idxIt->second;
        g_switchCooldownUntilSByUe[ueIdx] =
            Simulator::Now().GetSeconds() + (g_switchDelayMsCfg / 1000.0);
    }
    return true;
}

static void
DlSchedulingTrace(NrSchedulingCallbackInfo info)
{
    if (info.m_rnti == 0 || info.m_mcs == std::numeric_limits<uint8_t>::max())
    {
        return;
    }
    auto idxIt = g_ueIdxByRnti.find(info.m_rnti);
    if (idxIt != g_ueIdxByRnti.end() && idxIt->second < g_lastMcsByUe.size())
    {
        const uint32_t ueIdx = idxIt->second;
        g_lastMcsByUe[ueIdx] = static_cast<double>(info.m_mcs);
        if (ueIdx < g_schedCountCumByUe.size())
        {
            g_schedCountCumByUe[ueIdx]++;
        }
    }
    TriggerDppPolicyEpoch();
    if (!g_enableMcsSwitch)
    {
        return;
    }
    auto& st = g_mcsStateByRnti[info.m_rnti];
    double now = Simulator::Now().GetSeconds();
    if ((now - st.lastSwitchTime) * 1000.0 < g_minSwitchIntervalMs)
    {
        return;
    }

    if (info.m_mcs <= g_mcsLowThreshold)
    {
        st.lowCount++;
        st.highCount = 0;
    }
    else if (info.m_mcs >= g_mcsHighThreshold)
    {
        st.highCount++;
        st.lowCount = 0;
    }
    else
    {
        st.lowCount = 0;
        st.highCount = 0;
    }

    uint8_t currentBwp = g_initialBwpId;
    if (g_gnbBwpMgr)
    {
        uint8_t forced = g_gnbBwpMgr->GetForcedUeBwp(info.m_rnti);
        if (forced != std::numeric_limits<uint8_t>::max())
        {
            currentBwp = forced;
        }
    }

    if (st.lowCount >= g_mcsSwitchCount && currentBwp != g_lowBwpId)
    {
        SwitchUeBwp(info.m_rnti, g_lowBwpId);
        st.lowCount = 0;
    }
    else if (st.highCount >= g_mcsSwitchCount && currentBwp != g_highBwpId)
    {
        SwitchUeBwp(info.m_rnti, g_highBwpId);
        st.highCount = 0;
    }
}

static void
RegisterUeManager(Ptr<NrUeNetDevice> ueDev, uint32_t ueIdx)
{
    Ptr<NrUePhy> uePhy = ueDev->GetPhy(0);
    uint16_t rnti = uePhy->GetRnti();
    if (rnti == 0)
    {
        Simulator::Schedule(MilliSeconds(1), &RegisterUeManager, ueDev, ueIdx);
        return;
    }
    g_ueMgrByRnti[rnti] = ueDev->GetBwpManager();
    g_ueIdxByRnti[rnti] = ueIdx;
    if (ueIdx < g_rntiByUeIdx.size())
    {
        g_rntiByUeIdx[ueIdx] = rnti;
    }
    if (g_gnbBwpMgr)
    {
        g_gnbBwpMgr->ForceUeBwp(rnti, g_initialBwpId);
    }
    ueDev->GetBwpManager()->ForceActiveBwp(g_initialBwpId);
    if (ueIdx < g_currentBwpByUe.size())
    {
        g_currentBwpByUe[ueIdx] = g_initialBwpId;
    }
    if (g_enablePerUeMcsControl && ueIdx < g_targetMcsByUe.size())
    {
        for (auto const& sched : g_dlSchedulers)
        {
            if (sched)
            {
                sched->SetDlMcsOverrideForRnti(rnti, g_targetMcsByUe[ueIdx]);
            }
        }
    }
}

static void
RecordAppRx(uint32_t ueIdx, uint8_t type, Ptr<const Packet> p, double aoiMs)
{
    if (ueIdx >= g_ueStats.size() || type >= TRAFFIC_TYPES)
    {
        return;
    }
    auto& st = g_ueStats[ueIdx].flow[type];
    st.rxBytes += p->GetSize();
    st.rxPkts++;
    g_rxPacketsByType[ueIdx][type]++;
    st.aoiSumMs += aoiMs;
    st.aoiSamples++;
    if (std::isfinite(aoiMs) && aoiMs >= 0.0)
    {
        g_packetLevelAoiSamplesMs.push_back(aoiMs);
        if (type < TRAFFIC_TYPES)
        {
            g_packetLevelAoiSamplesByTypeMs[type].push_back(aoiMs);
        }
    }
    st.lastRxTime = Simulator::Now().GetSeconds();
    g_lastDeliveredAoiMs[ueIdx][type] = aoiMs;
    g_lastRxTimeByType[ueIdx][type] = st.lastRxTime;
    if (ueIdx < g_lastNodeDeliveredAoiMsByUe.size())
    {
        g_lastNodeDeliveredAoiMsByUe[ueIdx] = aoiMs;
    }
    if (ueIdx < g_lastNodeRxTimeByUe.size())
    {
        g_lastNodeRxTimeByUe[ueIdx] = st.lastRxTime;
    }

    if (g_aoiTraceStream && g_aoiTraceStream->is_open() &&
        ueIdx == static_cast<uint32_t>(std::max<int32_t>(0, g_aoiTraceUe)))
    {
        (*g_aoiTraceStream) << std::fixed << std::setprecision(6) << Simulator::Now().GetSeconds()
                            << "," << ueIdx << "," << static_cast<uint32_t>(type) << ","
                            << p->GetSize() << "," << aoiMs << "\n";
        g_aoiTraceStream->flush();
    }
}

static void
TagPacketTx(Ptr<const Packet> p)
{
    auto* mutablePacket = const_cast<Packet*>(PeekPointer(p));
    if (!mutablePacket)
    {
        return;
    }
    TxTimeTag tag;
    tag.SetTxTime(Simulator::Now());
    mutablePacket->ReplacePacketTag(tag);
    mutablePacket->AddByteTag(tag, 0, mutablePacket->GetSize());
}

static void
TagAoiIdentity(Ptr<const Packet> p, uint32_t ueIdx, uint8_t type)
{
    auto* mutablePacket = const_cast<Packet*>(PeekPointer(p));
    if (!mutablePacket)
    {
        return;
    }
    AoiIdentityTag tag;
    tag.SetPacketId(g_nextAoiPacketId++);
    tag.SetUeIdx(ueIdx);
    tag.SetTrafficType(type);
    tag.SetTxTime(Simulator::Now());
    mutablePacket->ReplacePacketTag(tag);
    mutablePacket->AddByteTag(tag, 0, mutablePacket->GetSize());
}

static bool
ExtractAoiIdentity(Ptr<const Packet> p,
                   uint64_t* packetId,
                   uint32_t* ueIdx,
                   uint8_t* type,
                   Time* txTime)
{
    if (!p)
    {
        return false;
    }
    AoiIdentityTag tag;
    if (!p->PeekPacketTag(tag) && !p->FindFirstMatchingByteTag(tag))
    {
        return false;
    }
    if (packetId)
    {
        *packetId = tag.GetPacketId();
    }
    if (ueIdx)
    {
        *ueIdx = tag.GetUeIdx();
    }
    if (type)
    {
        *type = tag.GetTrafficType();
    }
    if (txTime)
    {
        *txTime = tag.GetTxTime();
    }
    return true;
}

static void
RegisterOutstandingAoiPacket(uint64_t packetId,
                             uint32_t ueIdx,
                             uint8_t type,
                             Time enqueueTime,
                             Time txTime,
                             uint32_t packetBytes)
{
    if (type >= TRAFFIC_TYPES || ueIdx >= g_outstandingAoiByUe.size() ||
        ueIdx >= g_outstandingAoiByUeType.size())
    {
        return;
    }
    if (g_outstandingAoiPacketById.find(packetId) != g_outstandingAoiPacketById.end())
    {
        return;
    }
    OutstandingAoiPacketInfo info;
    info.ueIdx = ueIdx;
    info.trafficType = type;
    info.enqueueTimeNs = static_cast<uint64_t>(enqueueTime.GetNanoSeconds());
    info.txTimeNs = static_cast<uint64_t>(txTime.GetNanoSeconds());
    info.packetBytes = packetBytes;
    g_outstandingAoiPacketById.emplace(packetId, info);
    g_outstandingAoiByUeType[ueIdx][type].emplace(info.enqueueTimeNs, packetId);
    g_outstandingAoiByUe[ueIdx].emplace(info.enqueueTimeNs, packetId);
    if (ueIdx < g_outstandingAoiBytesByUe.size())
    {
        g_outstandingAoiBytesByUe[ueIdx] += static_cast<uint64_t>(packetBytes);
    }
}

static bool
UnregisterOutstandingAoiPacket(uint64_t packetId, OutstandingAoiPacketInfo* infoOut = nullptr)
{
    auto it = g_outstandingAoiPacketById.find(packetId);
    if (it == g_outstandingAoiPacketById.end())
    {
        return false;
    }
    OutstandingAoiPacketInfo info = it->second;
    if (info.ueIdx < g_outstandingAoiByUeType.size() && info.trafficType < TRAFFIC_TYPES)
    {
        auto& byType = g_outstandingAoiByUeType[info.ueIdx][info.trafficType];
        auto range = byType.equal_range(info.enqueueTimeNs);
        for (auto rit = range.first; rit != range.second; ++rit)
        {
            if (rit->second == packetId)
            {
                byType.erase(rit);
                break;
            }
        }
    }
    if (info.ueIdx < g_outstandingAoiByUe.size())
    {
        auto& byUe = g_outstandingAoiByUe[info.ueIdx];
        auto range = byUe.equal_range(info.enqueueTimeNs);
        for (auto rit = range.first; rit != range.second; ++rit)
        {
            if (rit->second == packetId)
            {
                byUe.erase(rit);
                break;
            }
        }
    }
    if (info.ueIdx < g_outstandingAoiBytesByUe.size())
    {
        uint64_t bytes = static_cast<uint64_t>(info.packetBytes);
        if (g_outstandingAoiBytesByUe[info.ueIdx] >= bytes)
        {
            g_outstandingAoiBytesByUe[info.ueIdx] -= bytes;
        }
        else
        {
            g_outstandingAoiBytesByUe[info.ueIdx] = 0u;
        }
    }
    g_outstandingAoiPacketById.erase(it);
    if (infoOut)
    {
        *infoOut = info;
    }
    return true;
}

static void
OnRlcEnqueueAoiTrace(Ptr<const Packet> p)
{
    uint64_t packetId = 0;
    uint32_t ueIdx = 0;
    uint8_t type = 0;
    Time txTime = Seconds(0);
    if (!ExtractAoiIdentity(p, &packetId, &ueIdx, &type, &txTime))
    {
        return;
    }
    RegisterOutstandingAoiPacket(
        packetId,
        ueIdx,
        type,
        Simulator::Now(),
        txTime,
        p ? static_cast<uint32_t>(p->GetSize()) : 0u);
}

static void
OnRlcFinalDropAoiTrace(Ptr<const Packet> p)
{
    uint64_t packetId = 0;
    if (!ExtractAoiIdentity(p, &packetId, nullptr, nullptr, nullptr))
    {
        return;
    }
    UnregisterOutstandingAoiPacket(packetId);
}

static void
OnDlHarqFinalDropAoiTrace(Ptr<const Packet> p)
{
    OnRlcFinalDropAoiTrace(p);
}

static void
OnAppTx(uint32_t ueIdx, uint8_t type, Ptr<const Packet> p)
{
    if (ueIdx >= g_txBytesByType.size() || type >= TRAFFIC_TYPES)
    {
        return;
    }
    TagPacketTx(p);
    TagAoiIdentity(p, ueIdx, type);
    g_txBytesByType[ueIdx][type] += p->GetSize();
    g_txPacketsByType[ueIdx][type]++;
}

static int8_t
QueryUeLosState(uint32_t ueIdx)
{
    if (!g_enableLosNlosStats || !g_losConditionModel || !g_gnbMobility ||
        ueIdx >= g_ueMobilityByIdx.size() || !g_ueMobilityByIdx[ueIdx])
    {
        return -1;
    }
    Ptr<ChannelCondition> cond = g_losConditionModel->GetChannelCondition(g_gnbMobility, g_ueMobilityByIdx[ueIdx]);
    if (!cond)
    {
        return -1;
    }
    return cond->IsLos() ? 1 : 0;
}

static void
AccumulateLosNlosDwellUntil(uint32_t ueIdx, double nowS)
{
    if (ueIdx >= g_lastLosStateByUe.size() || ueIdx >= g_lastLosStateSampleTimeByUe.size() ||
        ueIdx >= g_envStatsByUe.size())
    {
        return;
    }
    const int8_t prevState = g_lastLosStateByUe[ueIdx];
    const double prevSampleS = g_lastLosStateSampleTimeByUe[ueIdx];
    if (prevState >= 0 && prevSampleS >= 0.0)
    {
        const double dt = nowS - prevSampleS;
        if (dt > 0.0)
        {
            const uint8_t env = prevState ? ENV_LOS : ENV_NLOS;
            g_envStatsByUe[ueIdx][env].dwellTimeS += dt;
        }
    }
    g_lastLosStateSampleTimeByUe[ueIdx] = nowS;
}

static void
SampleLosNlosState()
{
    if (!g_enableLosNlosStats)
    {
        return;
    }
    const double nowS = Simulator::Now().GetSeconds();
    for (uint32_t ueIdx = 0; ueIdx < g_envStatsByUe.size(); ++ueIdx)
    {
        AccumulateLosNlosDwellUntil(ueIdx, nowS);
        const int8_t state = QueryUeLosState(ueIdx);
        g_lastLosStateByUe[ueIdx] = state;
        if (state >= 0)
        {
            const uint8_t env = state ? ENV_LOS : ENV_NLOS;
            g_envStatsByUe[ueIdx][env].dwellSamples++;
        }
    }

    const double nextS = nowS + g_losSamplePeriodS;
    if (nextS < g_simTime - 1e-9)
    {
        Simulator::Schedule(Seconds(g_losSamplePeriodS), &SampleLosNlosState);
    }
}

static void
FinalizeLosNlosDwellStats(double endTimeS)
{
    if (!g_enableLosNlosStats)
    {
        return;
    }
    for (uint32_t ueIdx = 0; ueIdx < g_envStatsByUe.size(); ++ueIdx)
    {
        AccumulateLosNlosDwellUntil(ueIdx, endTimeS);
    }
}

static void
RecordEnvRxKpi(uint32_t ueIdx, Ptr<const Packet> p, double aoiMs)
{
    if (!g_enableLosNlosStats || ueIdx >= g_envStatsByUe.size())
    {
        return;
    }
    const int8_t state = QueryUeLosState(ueIdx);
    if (state < 0)
    {
        return;
    }
    const uint8_t env = state ? ENV_LOS : ENV_NLOS;
    auto& st = g_envStatsByUe[ueIdx][env];
    if (p)
    {
        st.rxBytes += p->GetSize();
    }
    st.rxPkts++;
    if (std::isfinite(aoiMs) && aoiMs >= 0.0)
    {
        st.aoiSumMs += aoiMs;
        st.aoiSamples++;
    }
}

static void
WriteLayoutTrace(const std::vector<Box>& buildingBoxes)
{
    if (g_layoutTraceFile.empty())
    {
        return;
    }
    std::ofstream out(g_layoutTraceFile, std::ios::out);
    if (!out.is_open())
    {
        NS_LOG_UNCOND("Failed to open layoutTraceFile: " << g_layoutTraceFile);
        return;
    }
    out << "entity,id,x,y,z,xMin,xMax,yMin,yMax,zMin,zMax\n";
    if (g_gnbMobility)
    {
        const Vector p = g_gnbMobility->GetPosition();
        out << "gnb,0," << p.x << "," << p.y << "," << p.z << ",,,,,,\n";
    }
    for (uint32_t ueIdx = 0; ueIdx < g_ueMobilityByIdx.size(); ++ueIdx)
    {
        Ptr<MobilityModel> mm = g_ueMobilityByIdx[ueIdx];
        if (!mm)
        {
            continue;
        }
        const Vector p = mm->GetPosition();
        out << "ue," << ueIdx << "," << p.x << "," << p.y << "," << p.z << ",,,,,,\n";
    }
    for (uint32_t i = 0; i < buildingBoxes.size(); ++i)
    {
        const Box& b = buildingBoxes[i];
        out << "building," << i << ",,,,"
            << b.xMin << "," << b.xMax << "," << b.yMin << "," << b.yMax << "," << b.zMin << ","
            << b.zMax << "\n";
    }
    out.flush();
    NS_LOG_UNCOND("Saved layoutTraceFile: " << g_layoutTraceFile);
}

static void
SampleTrajectoryTrace()
{
    if (!g_trajectoryTraceStream || !g_trajectoryTraceStream->is_open())
    {
        return;
    }
    const double nowS = Simulator::Now().GetSeconds();
    Vector gnbPos(0.0, 0.0, 0.0);
    if (g_gnbMobility)
    {
        gnbPos = g_gnbMobility->GetPosition();
    }
    for (uint32_t ueIdx = 0; ueIdx < g_ueMobilityByIdx.size(); ++ueIdx)
    {
        Ptr<MobilityModel> mm = g_ueMobilityByIdx[ueIdx];
        if (!mm)
        {
            continue;
        }
        const Vector p = mm->GetPosition();
        const double dx = p.x - gnbPos.x;
        const double dy = p.y - gnbPos.y;
        const double dz = p.z - gnbPos.z;
        const double distM = std::sqrt(dx * dx + dy * dy + dz * dz);
        const int8_t losState = QueryUeLosState(ueIdx);
        (*g_trajectoryTraceStream) << nowS << "," << ueIdx << "," << p.x << "," << p.y << "," << p.z
                                   << "," << distM << "," << static_cast<int32_t>(losState) << "\n";
    }
    const double nextS = nowS + g_trajectorySamplePeriodS;
    if (nextS < g_simTime + 1e-9)
    {
        Simulator::Schedule(Seconds(g_trajectorySamplePeriodS), &SampleTrajectoryTrace);
    }
}

static void
SampleSinrTrace()
{
    if (!g_sinrTraceStream || !g_sinrTraceStream->is_open())
    {
        return;
    }
    const double nowS = Simulator::Now().GetSeconds();
    for (uint32_t ueIdx = 0; ueIdx < g_lastSinrDbByUe.size(); ++ueIdx)
    {
        const double sinrDb = g_lastSinrDbByUe[ueIdx];
        const int8_t losState = QueryUeLosState(ueIdx);
        const uint32_t bwpId =
            (ueIdx < g_currentBwpByUe.size()) ? static_cast<uint32_t>(g_currentBwpByUe[ueIdx]) : 0u;
        (*g_sinrTraceStream) << nowS << "," << ueIdx << "," << sinrDb << "," << static_cast<int32_t>(losState)
                             << "," << bwpId << "\n";
    }
    const double nextS = nowS + g_sinrSamplePeriodS;
    if (nextS < g_simTime + 1e-9)
    {
        Simulator::Schedule(Seconds(g_sinrSamplePeriodS), &SampleSinrTrace);
    }
}

static void
SampleCauseTrace()
{
    if (!g_causeTraceStream || !g_causeTraceStream->is_open())
    {
        return;
    }
    const double nowS = Simulator::Now().GetSeconds();
    for (uint32_t ueIdx = 0; ueIdx < g_currentBwpByUe.size(); ++ueIdx)
    {
        const uint64_t prbCum = (ueIdx < g_rxPrbCumByUe.size()) ? g_rxPrbCumByUe[ueIdx] : 0u;
        const uint64_t tbBytesCum = (ueIdx < g_rxTbBytesCumByUe.size()) ? g_rxTbBytesCumByUe[ueIdx] : 0u;
        const uint64_t tbCountCum = (ueIdx < g_rxTbCountCumByUe.size()) ? g_rxTbCountCumByUe[ueIdx] : 0u;
        const uint64_t tbErrorCum = (ueIdx < g_rxTbErrorCumByUe.size()) ? g_rxTbErrorCumByUe[ueIdx] : 0u;
        const uint64_t schedCum = (ueIdx < g_schedCountCumByUe.size()) ? g_schedCountCumByUe[ueIdx] : 0u;

        const uint64_t dPrb = (ueIdx < g_causePrevRxPrbByUe.size()) ? (prbCum - g_causePrevRxPrbByUe[ueIdx]) : 0u;
        const uint64_t dTbBytes =
            (ueIdx < g_causePrevRxTbBytesByUe.size()) ? (tbBytesCum - g_causePrevRxTbBytesByUe[ueIdx]) : 0u;
        const uint64_t dTbCount =
            (ueIdx < g_causePrevRxTbCountByUe.size()) ? (tbCountCum - g_causePrevRxTbCountByUe[ueIdx]) : 0u;
        const uint64_t dTbError =
            (ueIdx < g_causePrevRxTbErrorByUe.size()) ? (tbErrorCum - g_causePrevRxTbErrorByUe[ueIdx]) : 0u;
        const uint64_t dSched =
            (ueIdx < g_causePrevSchedCountByUe.size()) ? (schedCum - g_causePrevSchedCountByUe[ueIdx]) : 0u;

        if (ueIdx < g_causePrevRxPrbByUe.size())
        {
            g_causePrevRxPrbByUe[ueIdx] = prbCum;
            g_causePrevRxTbBytesByUe[ueIdx] = tbBytesCum;
            g_causePrevRxTbCountByUe[ueIdx] = tbCountCum;
            g_causePrevRxTbErrorByUe[ueIdx] = tbErrorCum;
            g_causePrevSchedCountByUe[ueIdx] = schedCum;
        }

        uint8_t pendingTarget = 0;
        const bool hasPending = GetPendingBwpSwitchByUe(ueIdx, &pendingTarget);
        const double cooldownUntil =
            (ueIdx < g_switchCooldownUntilSByUe.size()) ? g_switchCooldownUntilSByUe[ueIdx] : 0.0;
        const double cooldownRemainMs = std::max(0.0, (cooldownUntil - nowS) * 1000.0);
        const double queueBytes = (ueIdx < g_rlcDlQueueBytesByUe.size()) ? g_rlcDlQueueBytesByUe[ueIdx] : 0.0;
        const int8_t losState = QueryUeLosState(ueIdx);
        const double sinrDb = (ueIdx < g_lastSinrDbByUe.size()) ? g_lastSinrDbByUe[ueIdx] : -100.0;
        const double cqi = (ueIdx < g_lastCqiByUe.size()) ? g_lastCqiByUe[ueIdx] : 0.0;
        const double mcs = (ueIdx < g_lastMcsByUe.size()) ? g_lastMcsByUe[ueIdx] : 0.0;
        const double tbler = (ueIdx < g_lastTblerByUe.size()) ? g_lastTblerByUe[ueIdx] : -1.0;
        const uint64_t harqFlush = (ueIdx < g_harqFlushCountByUe.size()) ? g_harqFlushCountByUe[ueIdx] : 0u;
        const uint64_t switchExec =
            (ueIdx < g_bwpSwitchExecCountByUe.size()) ? g_bwpSwitchExecCountByUe[ueIdx] : 0u;

        const uint32_t currentBwp =
            (ueIdx < g_currentBwpByUe.size()) ? static_cast<uint32_t>(g_currentBwpByUe[ueIdx]) : 0u;
        (*g_causeTraceStream) << nowS << "," << ueIdx << "," << currentBwp
                              << "," << static_cast<uint32_t>(hasPending ? 1u : 0u) << ","
                              << static_cast<uint32_t>(hasPending ? pendingTarget : currentBwp) << ","
                              << cooldownRemainMs << "," << queueBytes << "," << sinrDb << "," << cqi << "," << mcs
                              << "," << tbler << "," << static_cast<int32_t>(losState) << "," << dSched << ","
                              << dPrb << "," << dTbBytes << "," << dTbCount << "," << dTbError << "," << harqFlush
                              << "," << switchExec << "\n";
    }

    const double nextS = nowS + g_causeSamplePeriodS;
    if (nextS < g_simTime + 1e-9)
    {
        Simulator::Schedule(Seconds(g_causeSamplePeriodS), &SampleCauseTrace);
    }
}

static void
SampleBwpOccupancy()
{
    const double nowS = Simulator::Now().GetSeconds();
    if (g_simTime <= 0.0 || g_currentBwpByUe.empty())
    {
        return;
    }
    const double dt = std::max(0.0, g_bwpOccupancySamplePeriodS);
    if (dt <= 0.0)
    {
        return;
    }

    uint32_t cnt0 = 0;
    uint32_t cnt1 = 0;
    for (uint8_t bwp : g_currentBwpByUe)
    {
        if (bwp == g_lowBwpId)
        {
            cnt0++;
        }
        else if (bwp == g_highBwpId)
        {
            cnt1++;
        }
    }
    g_bwpOccupancyUeTimeS[0] += static_cast<double>(cnt0) * dt;
    g_bwpOccupancyUeTimeS[1] += static_cast<double>(cnt1) * dt;

    const double nextS = nowS + dt;
    if (nextS < g_simTime + 1e-9)
    {
        Simulator::Schedule(Seconds(dt), &SampleBwpOccupancy);
    }
}

static void
OnPacketSinkRx(uint32_t ueIdx, uint8_t type, Ptr<const Packet> p, const Address&)
{
    uint64_t packetId = 0;
    uint32_t taggedUeIdx = ueIdx;
    uint8_t taggedType = type;
    Time txTime = Seconds(0);
    bool haveIdentity = ExtractAoiIdentity(p, &packetId, &taggedUeIdx, &taggedType, &txTime);
    if (!haveIdentity)
    {
        TxTimeTag txTag;
        if (!p->PeekPacketTag(txTag) && !p->FindFirstMatchingByteTag(txTag))
        {
            if (ueIdx < g_rxMissingTagPacketsByType.size() && type < TRAFFIC_TYPES)
            {
                g_rxMissingTagPacketsByType[ueIdx][type]++;
            }
            // Fallback path for traffic sources that do not propagate custom tags
            // (e.g., some OnOff configurations). Keep throughput/AoI accounting alive.
            const double lastRxS =
                (ueIdx < g_lastRxTimeByType.size() && type < TRAFFIC_TYPES) ? g_lastRxTimeByType[ueIdx][type] : 0.0;
            const double nowS = Simulator::Now().GetSeconds();
            double aoiMs = (lastRxS > 0.0) ? std::max(0.0, (nowS - lastRxS) * 1000.0) : g_envStepTime * 1000.0;
            if (ueIdx < g_lastDeliveryTimeNsByUe.size())
            {
                g_lastDeliveryTimeNsByUe[ueIdx] = static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
            }
            RecordEnvRxKpi(ueIdx, p, aoiMs);
            RecordAppRx(ueIdx, type, p, aoiMs);
            return;
        }
        double aoiMs = (Simulator::Now() - txTag.GetTxTime()).GetMilliSeconds();
        if (ueIdx < g_lastDeliveryTimeNsByUe.size())
        {
            g_lastDeliveryTimeNsByUe[ueIdx] = static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
        }
        RecordEnvRxKpi(ueIdx, p, aoiMs);
        RecordAppRx(ueIdx, type, p, aoiMs);
        return;
    }
    OutstandingAoiPacketInfo info;
    double aoiMs = 0.0;
    if (UnregisterOutstandingAoiPacket(packetId, &info))
    {
        aoiMs = (Simulator::Now() - NanoSeconds(info.enqueueTimeNs)).GetMilliSeconds();
        taggedUeIdx = info.ueIdx;
        taggedType = info.trafficType;
    }
    else
    {
        aoiMs = (Simulator::Now() - txTime).GetMilliSeconds();
    }
    if (taggedUeIdx < g_lastDeliveryTimeNsByUe.size())
    {
        g_lastDeliveryTimeNsByUe[taggedUeIdx] = static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
    }
    RecordEnvRxKpi(taggedUeIdx, p, aoiMs);
    RecordAppRx(taggedUeIdx, taggedType, p, aoiMs);
}

static void
ConnectAoiPacketSinkTrace(Ptr<Application> app, uint32_t ueIdx, uint8_t type)
{
    if (!app)
    {
        if (Simulator::Now().GetSeconds() + 0.05 < g_simTime)
        {
            Simulator::Schedule(MilliSeconds(50), &ConnectAoiPacketSinkTrace, app, ueIdx, type);
        }
        return;
    }

    Ptr<PacketSink> sink = DynamicCast<PacketSink>(app);
    if (!sink)
    {
        if (Simulator::Now().GetSeconds() + 0.05 < g_simTime)
        {
            Simulator::Schedule(MilliSeconds(50), &ConnectAoiPacketSinkTrace, app, ueIdx, type);
        }
        return;
    }

    sink->TraceConnectWithoutContext("Rx", MakeBoundCallback(&OnPacketSinkRx, ueIdx, type));
}

[[maybe_unused]] static void
BurstOnOffStateTrace(uint32_t ueIdx, bool oldState, bool newState)
{
    (void)oldState;
    if (!g_burstStateTraceStream || !g_burstStateTraceStream->is_open())
    {
        return;
    }
    if (ueIdx != static_cast<uint32_t>(std::max<int32_t>(0, g_aoiTraceUe)))
    {
        return;
    }
    (*g_burstStateTraceStream) << std::fixed << std::setprecision(6)
                               << Simulator::Now().GetSeconds() << "," << ueIdx << ","
                               << static_cast<uint32_t>(newState) << "\n";
}

static void
OnRxPacketTraceUe(uint32_t ueIdx, RxPacketTraceParams params)
{
    if (ueIdx >= g_prbStats.size())
    {
        return;
    }
    g_prbStats[ueIdx].prbTotal += params.m_rbAssignedNum;
    if (!params.m_corrupt)
    {
        g_prbStats[ueIdx].tbBytes += params.m_tbSize;
    }
    g_prbStats[ueIdx].tbCount++;
    if (params.m_corrupt)
    {
        g_prbStats[ueIdx].tbError++;
    }
    if (params.m_mcs != std::numeric_limits<uint8_t>::max())
    {
        g_prbStats[ueIdx].mcsSum += params.m_mcs;
        g_prbStats[ueIdx].mcsCount++;
    }
    if (params.m_tbler >= 0.0)
    {
        g_prbStats[ueIdx].tblerSum += params.m_tbler;
        g_prbStats[ueIdx].tblerCount++;
        if (ueIdx < g_lastTblerByUe.size())
        {
            g_lastTblerByUe[ueIdx] = params.m_tbler;
        }
    }
    if (ueIdx < g_rxPrbCumByUe.size())
    {
        g_rxPrbCumByUe[ueIdx] += params.m_rbAssignedNum;
        g_rxTbCountCumByUe[ueIdx]++;
        if (!params.m_corrupt)
        {
            g_rxTbBytesCumByUe[ueIdx] += params.m_tbSize;
        }
        if (params.m_corrupt)
        {
            g_rxTbErrorCumByUe[ueIdx]++;
            g_rxTbErrorBytesCumByUe[ueIdx] += params.m_tbSize;
        }
    }
    if (ueIdx < g_stepAssignedPrbByUe.size())
    {
        g_stepAssignedPrbByUe[ueIdx] += params.m_rbAssignedNum;
        if (!params.m_corrupt)
        {
            g_stepTbBytesByUe[ueIdx] += params.m_tbSize;
        }
        g_stepTbCountByUe[ueIdx]++;
        if (params.m_corrupt)
        {
            g_stepTbErrorByUe[ueIdx]++;
        }
        g_lastCqiByUe[ueIdx] = static_cast<double>(params.m_cqi);
        if (params.m_sinr > 0.0)
        {
            double sinrDb = 10.0 * std::log10(params.m_sinr);
            g_prbStats[ueIdx].sinrSumDb += sinrDb;
            g_prbStats[ueIdx].sinrSamples++;
            g_lastSinrDbByUe[ueIdx] = sinrDb;
        }
        if (params.m_mcs != std::numeric_limits<uint8_t>::max())
        {
            g_lastMcsByUe[ueIdx] = static_cast<double>(params.m_mcs);
        }
        if (params.m_bwpId != std::numeric_limits<uint16_t>::max())
        {
            g_currentBwpByUe[ueIdx] = static_cast<uint8_t>(params.m_bwpId);
        }
    }
}

static double
Clamp01(double v)
{
    if (!std::isfinite(v))
    {
        return 0.0;
    }
    return std::max(0.0, std::min(1.0, v));
}

static double
ComputeQuantileLinear(const std::vector<double>& sortedValues, double q)
{
    if (sortedValues.empty())
    {
        return 0.0;
    }
    if (sortedValues.size() == 1)
    {
        return sortedValues.front();
    }
    const double qClamped = std::max(0.0, std::min(1.0, q));
    const double pos = qClamped * static_cast<double>(sortedValues.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = static_cast<size_t>(std::ceil(pos));
    if (lo == hi)
    {
        return sortedValues[lo];
    }
    const double frac = pos - static_cast<double>(lo);
    return sortedValues[lo] * (1.0 - frac) + sortedValues[hi] * frac;
}

struct PacketAoiStats
{
    double meanMs{0.0};
    double p25Ms{0.0};
    double p50Ms{0.0};
    double p75Ms{0.0};
    double minMs{0.0};
    double maxMs{0.0};
};

static PacketAoiStats
ComputePacketAoiStats(const std::vector<double>& samplesMs)
{
    PacketAoiStats out;
    if (samplesMs.empty())
    {
        return out;
    }
    std::vector<double> sorted = samplesMs;
    std::sort(sorted.begin(), sorted.end());
    out.meanMs =
        std::accumulate(sorted.begin(), sorted.end(), 0.0) / static_cast<double>(sorted.size());
    out.p25Ms = ComputeQuantileLinear(sorted, 0.25);
    out.p50Ms = ComputeQuantileLinear(sorted, 0.50);
    out.p75Ms = ComputeQuantileLinear(sorted, 0.75);
    out.minMs = sorted.front();
    out.maxMs = sorted.back();
    return out;
}

static int32_t
EstimateBaseMcsFromCqi(double cqi)
{
    // CQI 1..15 to MCS 0..27 (non-cumulative baseline reference).
    if (!std::isfinite(cqi) || cqi <= 0.0)
    {
        return 0;
    }
    const double cqiClamped = std::max(1.0, std::min(15.0, cqi));
    const double mcs = (cqiClamped - 1.0) * (27.0 / 14.0);
    return std::max(0, std::min(27, static_cast<int32_t>(std::round(mcs))));
}

static double
ComputeOFDMASymbolDurationMs(uint8_t numerology)
{
    // 15 kHz * 2^mu SCS with 14 OFDM symbols/slot.
    const double slotDurationMs = 1.0 / std::pow(2.0, static_cast<double>(numerology));
    return slotDurationMs / 14.0;
}

static uint8_t
GetCurrentUeBwp(uint32_t ueIdx)
{
    if (ueIdx >= g_currentBwpByUe.size())
    {
        return g_initialBwpId;
    }
    return g_currentBwpByUe[ueIdx];
}

static double
ComputeDlRlcQueueBytes(uint32_t ueIdx)
{
    if (ueIdx >= g_rlcDlQueueBytesByUe.size())
    {
        return 0.0;
    }
    return std::max(0.0, g_rlcDlQueueBytesByUe[ueIdx]);
}

static double
ComputeDppBacklogRatio(uint32_t ueIdx)
{
    const double queueBytes = ComputeDlRlcQueueBytes(ueIdx);
    uint32_t activeLcids = 1u;
    if (ueIdx < g_rlcDlQueueBytesByUeAndLcid.size())
    {
        activeLcids = std::max<uint32_t>(1u, static_cast<uint32_t>(g_rlcDlQueueBytesByUeAndLcid[ueIdx].size()));
    }
    const double denom = static_cast<double>(activeLcids) *
                         static_cast<double>(std::max<uint32_t>(1u, g_rlcMaxTxBufferBytes));
    return Clamp01(queueBytes / std::max(1.0, denom));
}

static void
RlcTxBufferSizeTrace(uint16_t rnti, uint8_t lcid, uint32_t bytes)
{
    auto it = g_ueIdxByRnti.find(rnti);
    if (it == g_ueIdxByRnti.end())
    {
        return;
    }
    uint32_t ueIdx = it->second;
    if (ueIdx >= g_rlcDlQueueBytesByUe.size())
    {
        return;
    }

    if (ueIdx >= g_rlcDlQueueBytesByUeAndLcid.size())
    {
        return;
    }

    // Aggregate exact per-LCID RLC Tx buffer sizes across LCIDs for each UE.
    g_rlcDlQueueBytesByUeAndLcid[ueIdx][lcid] = static_cast<double>(bytes);

    double totalBytes = 0.0;
    for (const auto& [_, qBytes] : g_rlcDlQueueBytesByUeAndLcid[ueIdx])
    {
        totalBytes += std::max(0.0, qBytes);
    }
    g_rlcDlQueueBytesByUe[ueIdx] = totalBytes;
    TriggerDppPolicyEpoch();
}

static void
ConnectRlcTxBufferSizeTraces()
{
    if (g_rlcTxBufferTraceConnected)
    {
        return;
    }

    bool ok = Config::ConnectWithoutContextFailSafe(
        "/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/NrGnbRrc/UeMap/*/DataRadioBearerMap/*/"
        "NrRlc/$ns3::NrRlcUm/TxBufferSize",
        MakeCallback(&RlcTxBufferSizeTrace));
    if (!ok)
    {
        if (Simulator::Now().GetSeconds() + 0.05 < g_simTime)
        {
            Simulator::Schedule(MilliSeconds(50), &ConnectRlcTxBufferSizeTraces);
        }
        return;
    }
    g_rlcTxBufferTraceConnected = true;
}

static void
ConnectRlcAoiLifecycleTraces()
{
    if (g_rlcAoiLifecycleTraceConnected)
    {
        return;
    }

    bool okEnqueue = false;
    bool okDrop = false;
    okEnqueue = okEnqueue ||
                Config::ConnectWithoutContextFailSafe(
                    "/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/NrGnbRrc/UeMap/*/DataRadioBearerMap/*/"
                    "NrRlc/$ns3::NrRlcUm/TxEnqueue",
                    MakeCallback(&OnRlcEnqueueAoiTrace));
    okEnqueue = okEnqueue ||
                Config::ConnectWithoutContextFailSafe(
                    "/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/NrGnbRrc/UeMap/*/DataRadioBearerMap/*/"
                    "NrRlc/$ns3::NrRlcAm/TxEnqueue",
                    MakeCallback(&OnRlcEnqueueAoiTrace));
    okDrop = okDrop ||
             Config::ConnectWithoutContextFailSafe(
                 "/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/NrGnbRrc/UeMap/*/DataRadioBearerMap/*/"
                 "NrRlc/$ns3::NrRlcUm/TxDrop",
                 MakeCallback(&OnRlcFinalDropAoiTrace));
    okDrop = okDrop ||
             Config::ConnectWithoutContextFailSafe(
                 "/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/NrGnbRrc/UeMap/*/DataRadioBearerMap/*/"
                 "NrRlc/$ns3::NrRlcAm/TxDrop",
                 MakeCallback(&OnRlcFinalDropAoiTrace));

    if (!(okEnqueue && okDrop))
    {
        if (Simulator::Now().GetSeconds() + 0.05 < g_simTime)
        {
            Simulator::Schedule(MilliSeconds(50), &ConnectRlcAoiLifecycleTraces);
        }
        return;
    }
    g_rlcAoiLifecycleTraceConnected = true;
}

static double
ComputeCurrentAoiMs(uint32_t ueIdx, uint8_t type)
{
    if (ueIdx >= g_outstandingAoiByUeType.size() || type >= TRAFFIC_TYPES)
    {
        return 0.0;
    }
    const auto& outstanding = g_outstandingAoiByUeType[ueIdx][type];
    if (outstanding.empty())
    {
        return 0.0;
    }
    return std::max(
        0.0,
        static_cast<double>((Simulator::Now() - NanoSeconds(outstanding.begin()->first)).GetMilliSeconds()));
}

[[maybe_unused]] static double
ComputeUeHolAoiMs(uint32_t ueIdx)
{
    if (ueIdx >= g_outstandingAoiByUe.size())
    {
        return 0.0;
    }
    const auto& outstanding = g_outstandingAoiByUe[ueIdx];
    if (outstanding.empty())
    {
        return 0.0;
    }
    return std::max(
        0.0,
        static_cast<double>((Simulator::Now() - NanoSeconds(outstanding.begin()->first)).GetMilliSeconds()));
}

static double
ComputeUeMeanAoiMs(uint32_t ueIdx)
{
    if (ueIdx >= g_ueStats.size())
    {
        return std::max(1.0, g_envStepTime * 1000.0);
    }

    const auto& http = g_ueStats[ueIdx].flow[LIGHT_HTTP];
    const auto& gaming = g_ueStats[ueIdx].flow[MODERATE_GAMING];
    const auto& video = g_ueStats[ueIdx].flow[HEAVY_VIDEO];

    const uint64_t samples = http.aoiSamples + gaming.aoiSamples + video.aoiSamples;
    if (samples > 0)
    {
        const double sumMs = http.aoiSumMs + gaming.aoiSumMs + video.aoiSumMs;
        return std::max(0.0, sumMs / static_cast<double>(samples));
    }

    // Fallback before first Rx sample: receiver-side age since last delivery timestamp.
    if (ueIdx < g_lastNodeRxTimeByUe.size())
    {
        const double lastRxS = g_lastNodeRxTimeByUe[ueIdx];
        const double nowS = Simulator::Now().GetSeconds();
        if (lastRxS > 0.0 && nowS >= lastRxS)
        {
            return std::max(0.0, (nowS - lastRxS) * 1000.0);
        }
    }

    if (ueIdx < g_lastNodeDeliveredAoiMsByUe.size() && g_lastNodeDeliveredAoiMsByUe[ueIdx] > 0.0)
    {
        return g_lastNodeDeliveredAoiMsByUe[ueIdx];
    }

    // Before first delivery, expose at least one step horizon (avoid persistent 0 KPI).
    return std::max(1.0, g_envStepTime * 1000.0);
}

static double
ComputeUeKpiAoiMs(uint32_t ueIdx)
{
    // Unified KPI AoI: packet-level AoI.
    return ComputeUeMeanAoiMs(ueIdx);
}

static double
ComputeExpectedReceiverAoiMs(uint32_t ueIdx, double nowMs)
{
    if (ueIdx >= g_lastRxTimeByType.size() || ueIdx >= g_lastDeliveredAoiMs.size())
    {
        return 0.0;
    }

    double expectedAoi = 0.0;
    for (uint8_t t = 0; t < TRAFFIC_TYPES; ++t)
    {
        const double lastRxMs = g_lastRxTimeByType[ueIdx][t] * 1000.0;
        if (lastRxMs > 0.0)
        {
            expectedAoi = std::max(expectedAoi, std::max(0.0, nowMs - lastRxMs));
        }
        else
        {
            expectedAoi = std::max(expectedAoi, g_lastDeliveredAoiMs[ueIdx][t]);
        }
    }
    return expectedAoi;
}

static void
RecordAoiStateTrace()
{
    if (!g_aoiStateTraceStream || !g_aoiStateTraceStream->is_open())
    {
        return;
    }

    uint32_t ueIdx = static_cast<uint32_t>(std::max<int32_t>(0, g_aoiStateTraceUe));
    if (ueIdx >= g_ueStats.size())
    {
        return;
    }

    (*g_aoiStateTraceStream) << std::fixed << std::setprecision(6) << Simulator::Now().GetSeconds()
                             << "," << ueIdx << "," << static_cast<uint32_t>(GetCurrentUeBwp(ueIdx)) << ","
                             << ComputeDlRlcQueueBytes(ueIdx) << ","
                             << ComputeUeMeanAoiMs(ueIdx) << ","
                             << ComputeCurrentAoiMs(ueIdx, LIGHT_HTTP) << ","
                             << ComputeCurrentAoiMs(ueIdx, MODERATE_GAMING) << ","
                             << ComputeCurrentAoiMs(ueIdx, HEAVY_VIDEO) << "\n";
    g_aoiStateTraceStream->flush();
}

static void
RecordQueueTrace()
{
    if (!g_queueTraceStream || !g_queueTraceStream->is_open())
    {
        return;
    }

    uint32_t ueIdx = static_cast<uint32_t>(std::max<int32_t>(0, g_queueTraceUe));
    if (ueIdx >= g_ueStats.size())
    {
        return;
    }

    (*g_queueTraceStream) << std::fixed << std::setprecision(6) << Simulator::Now().GetSeconds()
                          << "," << ueIdx << "," << static_cast<uint32_t>(GetCurrentUeBwp(ueIdx))
                          << "," << ComputeDlRlcQueueBytes(ueIdx) << "\n";
    g_queueTraceStream->flush();
}

static void
ScheduleStateTraces()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        RecordAoiStateTrace();
        RecordQueueTrace();
        return;
    }

    RecordAoiStateTrace();
    RecordQueueTrace();
    Simulator::Schedule(Seconds(g_envStepTime), &ScheduleStateTraces);
}

static double
ComputeUePrbUtility(uint32_t ueIdx)
{
    if (ueIdx >= g_ueStats.size())
    {
        return 0.0;
    }
    double queueBytes = ComputeDlRlcQueueBytes(ueIdx);
    uint8_t bwp = GetCurrentUeBwp(ueIdx);
    uint32_t totalPrb = 1;
    if (bwp < g_totalPrbByBwp.size())
    {
        totalPrb = std::max<uint32_t>(1u, g_totalPrbByBwp[bwp]);
    }
    double mcs = (ueIdx < g_lastMcsByUe.size()) ? g_lastMcsByUe[ueIdx] : static_cast<double>(g_initialMcs);
    double bytesPerPrb = std::max(1.0, g_prbDemandC0 + g_prbDemandC1 * mcs);
    double demandedPrb = queueBytes / std::max(1.0, bytesPerPrb);
    return demandedPrb / static_cast<double>(totalPrb);
}

static double
ComputeUeThroughputMbps(uint32_t ueIdx)
{
    if (ueIdx >= g_ueStats.size())
    {
        return 0.0;
    }
    uint64_t rxBytes = 0;
    for (uint8_t type = 0; type < TRAFFIC_TYPES; ++type)
    {
        rxBytes += g_ueStats[ueIdx].flow[type].rxBytes;
    }
    double durationS = std::max(1e-6, Simulator::Now().GetSeconds() - g_appStartTime);
    return static_cast<double>(rxBytes) * 8.0 / durationS / 1e6;
}

static double
ComputeUeStepSpectralEfficiency(uint32_t ueIdx)
{
    if (ueIdx >= g_stepAssignedPrbByUe.size() || ueIdx >= g_stepTbBytesByUe.size())
    {
        return 0.0;
    }
    uint64_t assignedPrb = g_stepAssignedPrbByUe[ueIdx];
    if (assignedPrb == 0)
    {
        return 0.0;
    }
    double deliveredBits = static_cast<double>(g_stepTbBytesByUe[ueIdx]) * 8.0;
    return deliveredBits / static_cast<double>(assignedPrb);
}

static void
RecordIntervalMetrics(uint32_t switchCount)
{
    if (Simulator::Now().GetSeconds() < g_appStartTime)
    {
        return;
    }
    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    if (numUes == 0)
    {
        return;
    }

    double meanThrMbps = 0.0;
    double meanAoiMs = 0.0;
    double meanPrbUtility = 0.0;
    double queueOverflowCnt = 0.0;
    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        if (ueIdx < g_lastStepAssignedPrbByUe.size())
        {
            g_lastStepAssignedPrbByUe[ueIdx] = (ueIdx < g_stepAssignedPrbByUe.size()) ? g_stepAssignedPrbByUe[ueIdx] : 0u;
        }
        if (ueIdx < g_lastStepSpectralEfficiencyByUe.size())
        {
            g_lastStepSpectralEfficiencyByUe[ueIdx] = ComputeUeStepSpectralEfficiency(ueIdx);
        }
        meanThrMbps += ComputeUeThroughputMbps(ueIdx);
        meanAoiMs += ComputeUeKpiAoiMs(ueIdx);
        meanPrbUtility += ComputeUePrbUtility(ueIdx);
        if (ComputeDlRlcQueueBytes(ueIdx) > g_queueNormBytes)
        {
            queueOverflowCnt += 1.0;
        }
    }
    std::fill(g_lastBwpStepSpectralEfficiency.begin(), g_lastBwpStepSpectralEfficiency.end(), 0.0);
    std::fill(g_lastBwpAvgMcs.begin(), g_lastBwpAvgMcs.end(), 0.0);
    std::vector<double> bwpAssignedPrb(g_totalPrbByBwp.size(), 0.0);
    std::vector<double> bwpDeliveredBits(g_totalPrbByBwp.size(), 0.0);
    std::vector<double> bwpMcsSum(g_totalPrbByBwp.size(), 0.0);
    std::vector<uint32_t> bwpMcsCount(g_totalPrbByBwp.size(), 0u);
    for (uint32_t ueIdx = 0; ueIdx < g_lastStepAssignedPrbByUe.size() && ueIdx < g_lastStepSpectralEfficiencyByUe.size(); ++ueIdx)
    {
        uint8_t bwp = GetCurrentUeBwp(ueIdx);
        if (bwp >= g_totalPrbByBwp.size())
        {
            continue;
        }
        bwpAssignedPrb[bwp] += static_cast<double>(g_lastStepAssignedPrbByUe[ueIdx]);
        bwpDeliveredBits[bwp] += static_cast<double>((ueIdx < g_stepTbBytesByUe.size()) ? g_stepTbBytesByUe[ueIdx] : 0u) * 8.0;
        if (ueIdx < g_lastMcsByUe.size())
        {
            bwpMcsSum[bwp] += g_lastMcsByUe[ueIdx];
            bwpMcsCount[bwp]++;
        }
    }
    for (uint32_t bwp = 0; bwp < g_lastBwpStepSpectralEfficiency.size(); ++bwp)
    {
        if (bwpAssignedPrb[bwp] > 0.0)
        {
            g_lastBwpStepSpectralEfficiency[bwp] = bwpDeliveredBits[bwp] / bwpAssignedPrb[bwp];
        }
        if (bwpMcsCount[bwp] > 0)
        {
            g_lastBwpAvgMcs[bwp] = bwpMcsSum[bwp] / static_cast<double>(bwpMcsCount[bwp]);
        }
    }
    meanThrMbps /= numUes;
    meanAoiMs /= numUes;
    meanPrbUtility /= numUes;
    double queueOverflowRatio = queueOverflowCnt / numUes;

    g_lastMeanAoiMs = meanAoiMs;
    g_lastQueueOverflowRatio = queueOverflowRatio;
    g_lastMeanPrbUtility = meanPrbUtility;

    g_intervalMetrics.meanThrMbpsSum += meanThrMbps;
    g_intervalMetrics.meanAoiMsSum += meanAoiMs;
    g_intervalMetrics.meanPrbUtilitySum += meanPrbUtility;
    g_intervalMetrics.queueOverflowRatioSum += queueOverflowRatio;
    g_intervalMetrics.switchCountSum += switchCount;
    g_intervalMetrics.rewardAoiTermSum += g_lastMeanRewardAoiTerm;
    g_intervalMetrics.rewardThrTermSum += g_lastMeanRewardThrTerm;
    g_intervalMetrics.rewardSeTermSum += g_lastMeanRewardSeTerm;
    g_intervalMetrics.rewardSwitchPenaltySum += g_lastMeanRewardSwitchPenalty;
    g_intervalMetrics.rewardTotalSum += g_lastMeanRewardTotal;
    g_intervalMetrics.samples++;

    if (g_metricsTraceStream && g_metricsTraceStream->is_open())
    {
        (*g_metricsTraceStream) << std::fixed << std::setprecision(6)
                                << Simulator::Now().GetSeconds() << "," << meanThrMbps << ","
                                << meanAoiMs << "," << meanPrbUtility << ","
                                << queueOverflowRatio << "," << switchCount << ","
                                << g_lastMeanRewardAoiTerm << "," << g_lastMeanRewardThrTerm
                                << "," << g_lastMeanRewardSeTerm << ","
                                << g_lastMeanRewardSwitchPenalty << "," << g_lastMeanRewardTotal
                                << "\n";
        g_metricsTraceStream->flush();
    }
}

static void
ApplyPerUeMcsOverridesToSchedulers()
{
    for (auto const& sched : g_dlSchedulers)
    {
        if (!sched)
        {
            continue;
        }
        for (uint32_t ueIdx = 0; ueIdx < g_targetMcsByUe.size() && ueIdx < g_rntiByUeIdx.size();
             ++ueIdx)
        {
            uint16_t rnti = g_rntiByUeIdx[ueIdx];
            if (rnti == 0)
            {
                continue;
            }
            sched->SetDlMcsOverrideForRnti(rnti, g_targetMcsByUe[ueIdx]);
        }
    }
}

static void
ApplyAequitasInputsToSchedulers()
{
    for (auto const& sched : g_aequitasSchedulers)
    {
        if (!sched)
        {
            continue;
        }
        for (uint32_t ueIdx = 0; ueIdx < g_rntiByUeIdx.size(); ++ueIdx)
        {
            uint16_t rnti = g_rntiByUeIdx[ueIdx];
            if (rnti == 0)
            {
                continue;
            }
            sched->SetUeAoiState(rnti, ComputeUeMeanAoiMs(ueIdx), g_aequitasDeadlineMs);
        }
    }
}

static bool
ShouldDriveAequitasScheduler()
{
    return g_schedulerPolicy == "aequitas" || g_bwpBaseline == "aequitas";
}

static std::string
ToLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}

static std::string
ToUpper(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
    return s;
}

static bool
CanSwitchByCooldown(uint32_t ueIdx)
{
    if (ueIdx >= g_policyCooldownStepsByUe.size())
    {
        return false;
    }
    return g_policyCooldownStepsByUe[ueIdx] == 0;
}

static bool
TrySwitchUeToBwp(uint32_t ueIdx, uint8_t targetBwp)
{
    if (ueIdx >= g_rntiByUeIdx.size() || ueIdx >= g_currentBwpByUe.size())
    {
        return false;
    }
    if (!CanSwitchByCooldown(ueIdx))
    {
        g_nextSwitchRejectCooldownCount++;
        return false;
    }
    uint16_t rnti = g_rntiByUeIdx[ueIdx];
    if (rnti == 0)
    {
        g_nextSwitchRejectNoRntiCount++;
        return false;
    }
    uint8_t current = GetCurrentUeBwp(ueIdx);
    if (current == targetBwp)
    {
        g_nextSwitchRejectSameTargetCount++;
        return false;
    }
    if (!SwitchUeBwp(rnti, targetBwp))
    {
        return false;
    }
    g_policyCooldownStepsByUe[ueIdx] = g_policySwitchCooldownSteps;
    return true;
}

static uint32_t
RunDtPolicyStep()
{
    uint32_t switchCount = 0;
    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        double queueNorm = Clamp01(ComputeDlRlcQueueBytes(ueIdx) / std::max(1.0, g_queueNormBytes));
        double aoiMs = ComputeUeMeanAoiMs(ueIdx);

        double emaQ = g_dtEmaQueueNormByUe[ueIdx];
        double emaA = g_dtEmaAoiMsByUe[ueIdx];
        g_dtEmaQueueNormByUe[ueIdx] = (1.0 - g_dtEmaAlpha) * emaQ + g_dtEmaAlpha * queueNorm;
        g_dtEmaAoiMsByUe[ueIdx] = (1.0 - g_dtEmaAlpha) * emaA + g_dtEmaAlpha * aoiMs;

        double predQ = queueNorm + g_dtPredictGainQueue * (queueNorm - g_dtEmaQueueNormByUe[ueIdx]);
        double predA = aoiMs + g_dtPredictGainAoi * (aoiMs - g_dtEmaAoiMsByUe[ueIdx]);
        predQ = std::max(0.0, predQ);
        predA = std::max(0.0, predA);

        bool urgent = (predQ >= g_dtQueueHigh) || (predA >= g_dtDelayTargetMs);
        bool relaxed = (predQ <= g_dtQueueLow) && (predA <= 0.7 * g_dtDelayTargetMs);

        uint8_t current = GetCurrentUeBwp(ueIdx);
        if (urgent && current != g_highBwpId)
        {
            if (TrySwitchUeToBwp(ueIdx, g_highBwpId))
            {
                switchCount++;
            }
        }
        else if (relaxed && current != g_lowBwpId)
        {
            if (TrySwitchUeToBwp(ueIdx, g_lowBwpId))
            {
                switchCount++;
            }
        }
    }
    return switchCount;
}

static uint32_t
RunQueueThresholdPolicyStep()
{
    uint32_t switchCount = 0;
    for (uint32_t ueIdx = 0; ueIdx < g_rlcDlQueueBytesByUe.size(); ++ueIdx)
    {
        if (g_policyCooldownStepsByUe[ueIdx] > 0)
        {
            continue;
        }

        uint32_t queueSize = static_cast<uint32_t>(g_rlcDlQueueBytesByUe[ueIdx]);
        uint8_t currentBwp = g_currentBwpByUe[ueIdx];
        uint8_t targetBwp = currentBwp;

        if (queueSize > g_bwpQueueThreshold && currentBwp == g_lowBwpId)
        {
            targetBwp = g_highBwpId;
        }
        else if (queueSize <= g_bwpQueueThreshold && currentBwp == g_highBwpId)
        {
            targetBwp = g_lowBwpId;
        }

        if (targetBwp != currentBwp)
        {
            uint16_t rnti = g_rntiByUeIdx[ueIdx];
            if (SwitchUeBwp(rnti, targetBwp))
            {
                switchCount++;
                g_policyCooldownStepsByUe[ueIdx] = g_policySwitchCooldownSteps;
            }
        }
    }
    return switchCount;
}

enum DppAction : uint8_t
{
    DPP_BWP0_CQI = 0,
    DPP_BWP0_CQI_P1 = 1,
    DPP_BWP0_CQI_P2 = 2,
    DPP_BWP1_CQI = 3,
    DPP_BWP1_CQI_M1 = 4,
    DPP_BWP1_CQI_M2 = 5,
    DPP_ACTIONS = G_DPP_ACTION_COUNT
};

static const char*
GetAppStateLabel(int state)
{
    switch (state)
    {
    case 0:
        return "ftp_only";
    case 1:
        return "video_only";
    case 2:
        return "ftp_video";
    default:
        return "unknown";
    }
}

static void
DecodeDppAction(uint8_t action, uint8_t* targetBwp, int32_t* mcsDelta)
{
    uint8_t bwp = g_lowBwpId;
    int32_t delta = 0;
    switch (action)
    {
    case DPP_BWP0_CQI:
        bwp = g_lowBwpId;
        delta = 0;
        break;
    case DPP_BWP0_CQI_P1:
        bwp = g_lowBwpId;
        delta = 1;
        break;
    case DPP_BWP0_CQI_P2:
        bwp = g_lowBwpId;
        delta = 2;
        break;
    case DPP_BWP1_CQI:
        bwp = g_highBwpId;
        delta = 0;
        break;
    case DPP_BWP1_CQI_M1:
        bwp = g_highBwpId;
        delta = -1;
        break;
    case DPP_BWP1_CQI_M2:
        bwp = g_highBwpId;
        delta = -2;
        break;
    default:
        bwp = g_lowBwpId;
        delta = 0;
        break;
    }
    if (targetBwp)
    {
        *targetBwp = bwp;
    }
    if (mcsDelta)
    {
        *mcsDelta = delta;
    }
}

static void
UpdateDppPosteriorForUe(uint32_t ueIdx)
{
    if (ueIdx >= g_dppHasLastActionByUe.size() || ueIdx >= g_dppLastActionByUe.size())
    {
        return;
    }
    const uint64_t tbBytesCum = (ueIdx < g_rxTbBytesCumByUe.size()) ? g_rxTbBytesCumByUe[ueIdx] : 0u;
    const uint64_t tbErrBytesCum =
        (ueIdx < g_rxTbErrorBytesCumByUe.size()) ? g_rxTbErrorBytesCumByUe[ueIdx] : 0u;
    const double nowS = Simulator::Now().GetSeconds();
    const double prevTimeS =
        (ueIdx < g_dppLastDecisionTimeSByUe.size()) ? g_dppLastDecisionTimeSByUe[ueIdx] : nowS;
    const double deltaS = std::max(1e-6, nowS - prevTimeS);

    if (g_dppHasLastActionByUe[ueIdx])
    {
        const int32_t lastAction = g_dppLastActionByUe[ueIdx];
        if (lastAction >= 0 && lastAction < static_cast<int32_t>(DPP_ACTIONS))
        {
            const uint64_t prevTbBytes =
                (ueIdx < g_dppLastTbBytesCumByUe.size()) ? g_dppLastTbBytesCumByUe[ueIdx] : 0u;
            const uint64_t prevTbErrBytes = (ueIdx < g_dppLastTbErrorBytesCumByUe.size())
                                                ? g_dppLastTbErrorBytesCumByUe[ueIdx]
                                                : 0u;
            const uint64_t dTbBytes = (tbBytesCum >= prevTbBytes) ? (tbBytesCum - prevTbBytes) : 0u;
            const uint64_t dTbErrBytes =
                (tbErrBytesCum >= prevTbErrBytes) ? (tbErrBytesCum - prevTbErrBytes) : 0u;
            const double obsSuccessBytes = static_cast<double>(dTbBytes);
            const double obsErrBytes = static_cast<double>(dTbErrBytes);

            auto& muMean = g_dppMuPostMeanByUe[ueIdx][lastAction];
            auto& muPrec = g_dppMuPostPrecisionByUe[ueIdx][lastAction];
            const double muMeanBefore = muMean;
            const double muPrecBefore = muPrec;
            const double obsPrec = std::max(1e-9, g_dppMuObsPrecision);
            const double rho = Clamp01(g_dppPosteriorDiscount);
            // Discounted Bayesian update (forgetting factor):
            // shrink posterior toward prior before assimilating current observation.
            muMean = rho * muMean + (1.0 - rho) * std::max(0.0, g_dppMuPriorMeanBytes);
            muPrec = rho * muPrec + (1.0 - rho) * std::max(1e-9, g_dppMuPriorPrecision);
            const double newPrec = muPrec + obsPrec;
            muMean = (muPrec * muMean + obsPrec * obsSuccessBytes) / std::max(1e-9, newPrec);
            muPrec = newPrec;

            auto& errMean = g_dppErrPostMeanByUe[ueIdx][lastAction];
            auto& errPrec = g_dppErrPostPrecisionByUe[ueIdx][lastAction];
            const double errMeanBefore = errMean;
            const double errPrecBefore = errPrec;
            const double errObsPrec = std::max(1e-9, g_dppErrObsPrecision);
            errMean = rho * errMean + (1.0 - rho) * std::max(0.0, g_dppErrPriorMeanBytes);
            errPrec = rho * errPrec + (1.0 - rho) * std::max(1e-9, g_dppErrPriorPrecision);
            const double newErrPrec = errPrec + errObsPrec;
            errMean = (errPrec * errMean + errObsPrec * obsErrBytes) / std::max(1e-9, newErrPrec);
            errPrec = newErrPrec;

            if (g_dppPosteriorTraceStream && g_dppPosteriorTraceStream->is_open())
            {
                (*g_dppPosteriorTraceStream) << std::fixed << std::setprecision(6)
                                             << nowS << "," << ueIdx << "," << lastAction << ","
                                             << deltaS << "," << obsSuccessBytes << "," << obsErrBytes << ","
                                             << muMeanBefore << "," << muPrecBefore << ","
                                             << muMean << "," << muPrec << ","
                                             << errMeanBefore << "," << errPrecBefore << ","
                                             << errMean << "," << errPrec << "," << rho << "\n";
            }
            if (g_debugDpp)
            {
                NS_LOG_UNCOND("[dpp-post] t=" << std::fixed << std::setprecision(6) << nowS
                                              << " ue=" << ueIdx
                                              << " lastAction=" << lastAction
                                              << " dS=" << deltaS
                                              << " obsSuccBytes=" << obsSuccessBytes
                                              << " obsErrBytes=" << obsErrBytes
                                              << " mu(" << muMeanBefore << "->" << muMean << ")"
                                              << " muPrec(" << muPrecBefore << "->" << muPrec << ")"
                                              << " err(" << errMeanBefore << "->" << errMean << ")"
                                              << " errPrec(" << errPrecBefore << "->" << errPrec << ")"
                                              << " rho=" << rho);
            }
        }
    }

    if (ueIdx < g_dppLastTbBytesCumByUe.size())
    {
        g_dppLastTbBytesCumByUe[ueIdx] = tbBytesCum;
    }
    if (ueIdx < g_dppLastTbErrorBytesCumByUe.size())
    {
        g_dppLastTbErrorBytesCumByUe[ueIdx] = tbErrBytesCum;
    }
    if (ueIdx < g_dppLastDecisionTimeSByUe.size())
    {
        g_dppLastDecisionTimeSByUe[ueIdx] = nowS;
    }
    if (ueIdx < g_dppLastEpochDeltaSByUe.size())
    {
        g_dppLastEpochDeltaSByUe[ueIdx] = deltaS;
    }
}

static double
EstimateExpectedGoodputBayesianDummy(uint32_t ueIdx, uint8_t targetBwp, uint8_t targetMcs)
{
    // Pure Bayesian posterior mean of expected success TB bytes mu_i(t,a) per epoch.
    uint8_t action = DPP_BWP0_CQI;
    if (targetBwp == g_lowBwpId && targetMcs >= 2)
    {
        const int32_t baseMcs = (ueIdx < g_lastCqiByUe.size()) ? EstimateBaseMcsFromCqi(g_lastCqiByUe[ueIdx])
                                                               : static_cast<int32_t>(g_initialMcs);
        const int32_t delta = static_cast<int32_t>(targetMcs) - baseMcs;
        if (delta >= 2)
        {
            action = DPP_BWP0_CQI_P2;
        }
        else if (delta >= 1)
        {
            action = DPP_BWP0_CQI_P1;
        }
        else
        {
            action = DPP_BWP0_CQI;
        }
    }
    else if (targetBwp == g_highBwpId)
    {
        const int32_t baseMcs = (ueIdx < g_lastCqiByUe.size()) ? EstimateBaseMcsFromCqi(g_lastCqiByUe[ueIdx])
                                                               : static_cast<int32_t>(g_initialMcs);
        const int32_t delta = static_cast<int32_t>(targetMcs) - baseMcs;
        if (delta <= -2)
        {
            action = DPP_BWP1_CQI_M2;
        }
        else if (delta <= -1)
        {
            action = DPP_BWP1_CQI_M1;
        }
        else
        {
            action = DPP_BWP1_CQI;
        }
    }
    if (ueIdx < g_dppMuPostMeanByUe.size())
    {
        return std::max(0.0, g_dppMuPostMeanByUe[ueIdx][action]);
    }
    return std::max(0.0, g_dppMuPriorMeanBytes);
}

static double
EstimateExpectedErrorBytesBayesianDummy(uint32_t ueIdx, uint8_t targetBwp, uint8_t targetMcs)
{
    // Pure Bayesian posterior mean of expected error TB bytes e_i(t,a) per epoch.
    uint8_t action = DPP_BWP0_CQI;
    if (targetBwp == g_lowBwpId)
    {
        const int32_t baseMcs = (ueIdx < g_lastCqiByUe.size()) ? EstimateBaseMcsFromCqi(g_lastCqiByUe[ueIdx])
                                                               : static_cast<int32_t>(g_initialMcs);
        const int32_t delta = static_cast<int32_t>(targetMcs) - baseMcs;
        action = (delta >= 2) ? DPP_BWP0_CQI_P2 : ((delta >= 1) ? DPP_BWP0_CQI_P1 : DPP_BWP0_CQI);
    }
    else if (targetBwp == g_highBwpId)
    {
        const int32_t baseMcs = (ueIdx < g_lastCqiByUe.size()) ? EstimateBaseMcsFromCqi(g_lastCqiByUe[ueIdx])
                                                               : static_cast<int32_t>(g_initialMcs);
        const int32_t delta = static_cast<int32_t>(targetMcs) - baseMcs;
        action = (delta <= -2) ? DPP_BWP1_CQI_M2 : ((delta <= -1) ? DPP_BWP1_CQI_M1 : DPP_BWP1_CQI);
    }
    if (ueIdx < g_dppErrPostMeanByUe.size())
    {
        return std::max(0.0, g_dppErrPostMeanByUe[ueIdx][action]);
    }
    return std::max(0.0, g_dppErrPriorMeanBytes);
}

static uint32_t
RunDppPolicyEpochStep()
{
    uint32_t switchCount = 0;
    const uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        UpdateDppPosteriorForUe(ueIdx);
        const double backlogBytes = std::max(0.0, ComputeDlRlcQueueBytes(ueIdx));
        const double backlog = ComputeDppBacklogRatio(ueIdx);
        const double epochDeltaS =
            (ueIdx < g_dppLastEpochDeltaSByUe.size()) ? std::max(1e-6, g_dppLastEpochDeltaSByUe[ueIdx])
                                                      : std::max(1e-6, g_envStepTime);
        const uint8_t currentBwp = GetCurrentUeBwp(ueIdx);
        const int32_t cqiMcs = (ueIdx < g_lastCqiByUe.size()) ? EstimateBaseMcsFromCqi(g_lastCqiByUe[ueIdx])
                                                              : static_cast<int32_t>(g_initialMcs);
        const int32_t currentMcs =
            (ueIdx < g_targetMcsByUe.size()) ? static_cast<int32_t>(g_targetMcsByUe[ueIdx])
                                             : std::max(0, std::min(27, cqiMcs));
        const uint8_t currentMcsClamped = static_cast<uint8_t>(std::max(0, std::min(27, currentMcs)));
        const double muCurrent = EstimateExpectedGoodputBayesianDummy(ueIdx, currentBwp, currentMcsClamped);
        const double gHatCurrent = std::max(0.0, muCurrent) / (g_dppEpochMinIntervalS * 1000);
        // C_sw = (switch_delay / epoch=1ms) * mu(a_curr,t): opportunity cost of staying on current action.
        const double switchCost = std::max(0.0, g_switchDelayMsCfg) * gHatCurrent;

        double bestScore = -std::numeric_limits<double>::infinity();
        uint8_t bestAction = DPP_BWP0_CQI;
        uint8_t bestTargetBwp = currentBwp;
        uint8_t bestTargetMcs = static_cast<uint8_t>(std::max(0, std::min(27, cqiMcs)));
        int32_t bestDelta = 0;
        std::array<uint8_t, DPP_ACTIONS> actionTargetBwp{};
        std::array<uint8_t, DPP_ACTIONS> actionTargetMcs{};
        std::array<int32_t, DPP_ACTIONS> actionMcsDelta{};
        std::array<double, DPP_ACTIONS> actionMu{};
        std::array<double, DPP_ACTIONS> actionE{};
        std::array<double, DPP_ACTIONS> actionGoodputHat{};
        std::array<double, DPP_ACTIONS> actionQueueWeightedGoodput{};
        std::array<double, DPP_ACTIONS> actionPenaltySwitch{};
        std::array<double, DPP_ACTIONS> actionPenaltyBler{};
        std::array<double, DPP_ACTIONS> actionPenaltyTotal{};
        std::array<double, DPP_ACTIONS> actionScore{};

        for (uint8_t a = 0; a < DPP_ACTIONS; ++a)
        {
            uint8_t targetBwp = currentBwp;
            int32_t mcsDelta = 0;
            DecodeDppAction(a, &targetBwp, &mcsDelta);
            const int32_t targetMcs = std::max(0, std::min(27, cqiMcs + mcsDelta));
            const double mu = EstimateExpectedGoodputBayesianDummy(ueIdx, targetBwp, static_cast<uint8_t>(targetMcs));
            const double e =
                EstimateExpectedErrorBytesBayesianDummy(ueIdx, targetBwp, static_cast<uint8_t>(targetMcs));
            const double switchIndicator = (targetBwp != currentBwp) ? 1.0 : 0.0;
            const double goodputHat = std::max(0.0, mu);
            // score(a) = Q_i(t)*mu_hat_i(a,t) - V*(lambda_e*e_hat_i(a,t) + lambda_sw*C_sw*I_sw(a))
            const double queueWeightedGoodput = backlog * goodputHat;
            const double penaltySwitch =
                g_dppV * g_dppLambdaSwitch * switchCost * switchIndicator;
            const double penaltyBler = g_dppV * g_dppLambdaBler * std::max(0.0, e);
            const double penaltyTotal = penaltySwitch + penaltyBler;
            const double score = queueWeightedGoodput - penaltyTotal;
            actionTargetBwp[a] = targetBwp;
            actionTargetMcs[a] = static_cast<uint8_t>(targetMcs);
            actionMcsDelta[a] = mcsDelta;
            actionMu[a] = mu;
            actionE[a] = e;
            actionGoodputHat[a] = goodputHat;
            actionQueueWeightedGoodput[a] = queueWeightedGoodput;
            actionPenaltySwitch[a] = penaltySwitch;
            actionPenaltyBler[a] = penaltyBler;
            actionPenaltyTotal[a] = penaltyTotal;
            actionScore[a] = score;
            if (score > bestScore)
            {
                bestScore = score;
                bestAction = a;
                bestTargetBwp = targetBwp;
                bestTargetMcs = static_cast<uint8_t>(targetMcs);
                bestDelta = mcsDelta;
            }
        }
        const bool inRandomExploreWindow =
            (g_dppExploreRandomDurationS > 0.0) &&
            (Simulator::Now().GetSeconds() < g_dppExploreRandomDurationS);
        if (inRandomExploreWindow)
        {
            if (!g_dppExploreRv)
            {
                g_dppExploreRv = CreateObject<UniformRandomVariable>();
            }
            const uint8_t sampledAction =
                static_cast<uint8_t>(g_dppExploreRv->GetInteger(0, static_cast<uint32_t>(DPP_ACTIONS - 1)));
            bestAction = sampledAction;
            bestTargetBwp = actionTargetBwp[sampledAction];
            bestTargetMcs = actionTargetMcs[sampledAction];
            bestDelta = actionMcsDelta[sampledAction];
            bestScore = actionScore[sampledAction];
            if (g_debugDpp)
            {
                NS_LOG_UNCOND("[dpp-explore] t=" << std::fixed << std::setprecision(6)
                                                 << Simulator::Now().GetSeconds()
                                                 << " ue=" << ueIdx
                                                 << " sampledAction=" << static_cast<uint32_t>(sampledAction)
                                                 << " targetBwp=" << static_cast<uint32_t>(bestTargetBwp)
                                                 << " targetMcs=" << static_cast<uint32_t>(bestTargetMcs)
                                                 << " score=" << bestScore);
            }
        }
        const bool traceThisUe = (g_dppScoreTraceUe < 0) || (ueIdx == static_cast<uint32_t>(g_dppScoreTraceUe));
        if (traceThisUe && (g_dppScoreDebug || (g_dppScoreTraceStream && g_dppScoreTraceStream->is_open())))
        {
            for (uint8_t a = 0; a < DPP_ACTIONS; ++a)
            {
                const int selected = (a == bestAction) ? 1 : 0;
                if (g_dppScoreTraceStream && g_dppScoreTraceStream->is_open())
                {
                    (*g_dppScoreTraceStream) << std::fixed << std::setprecision(6)
                                             << Simulator::Now().GetSeconds() << ","
                                             << ueIdx << "," << static_cast<uint32_t>(a) << ","
                                             << selected << "," << static_cast<uint32_t>(actionTargetBwp[a]) << ","
                                             << static_cast<uint32_t>(actionTargetMcs[a]) << ","
                                             << actionMcsDelta[a] << "," << backlogBytes << ","
                                             << backlog << "," << epochDeltaS << ","
                                             << actionMu[a] << "," << actionE[a] << ","
                                             << gHatCurrent << "," << switchCost << ","
                                             << ((actionTargetBwp[a] != currentBwp) ? 1 : 0) << ","
                                             << actionGoodputHat[a] << ","
                                             << actionQueueWeightedGoodput[a] << ","
                                             << actionPenaltySwitch[a] << ","
                                             << actionPenaltyBler[a] << "," << actionPenaltyTotal[a] << ","
                                             << actionScore[a] << "\n";
                }
            }
            if (g_dppScoreTraceStream && g_dppScoreTraceStream->is_open())
            {
                g_dppScoreTraceStream->flush();
            }
            if (g_debugDpp)
            {
                for (uint8_t a = 0; a < DPP_ACTIONS; ++a)
                {
                    NS_LOG_UNCOND("[dpp-score] t=" << std::fixed << std::setprecision(6)
                                                   << Simulator::Now().GetSeconds()
                                                   << " ue=" << ueIdx
                                                   << " action=" << static_cast<uint32_t>(a)
                                                   << " sel=" << ((a == bestAction) ? 1 : 0)
                                                   << " targetBwp="
                                                   << static_cast<uint32_t>(actionTargetBwp[a])
                                                   << " targetMcs="
                                                   << static_cast<uint32_t>(actionTargetMcs[a])
                                                   << " mcsDelta=" << actionMcsDelta[a]
                                                   << " backlogBytes=" << backlogBytes
                                                   << " backlogRatio=" << backlog
                                                   << " mu=" << actionMu[a]
                                                   << " e=" << actionE[a]
                                                   << " muCurrent=" << gHatCurrent
                                                   << " switchCost=" << switchCost
                                                   << " switchInd="
                                                   << ((actionTargetBwp[a] != currentBwp) ? 1 : 0)
                                                   << " qWeighted=" << actionQueueWeightedGoodput[a]
                                                   << " penSw=" << actionPenaltySwitch[a]
                                                   << " penE=" << actionPenaltyBler[a]
                                                   << " score=" << actionScore[a]);
                }
            }
            if (g_dppScoreDebug)
            {
                NS_LOG_UNCOND("[dpp-score] t=" << Simulator::Now().GetSeconds()
                                               << " ue=" << ueIdx
                                               << " bestAction=" << static_cast<uint32_t>(bestAction)
                                               << " backlogBytes=" << backlogBytes
                                               << " backlogRatio=" << backlog
                                               << " epochDeltaS=" << epochDeltaS
                                               << " gHatCurrent=" << gHatCurrent
                                               << " switchCost=" << switchCost
                                               << " mu=" << actionMu[bestAction]
                                               << " e=" << actionE[bestAction]
                                               << " gHat=" << actionGoodputHat[bestAction]
                                               << " qWeightedGoodput="
                                               << actionQueueWeightedGoodput[bestAction]
                                               << " penaltySwitch=" << actionPenaltySwitch[bestAction]
                                               << " penaltyBler=" << actionPenaltyBler[bestAction]
                                               << " score=" << actionScore[bestAction]);
            }
        }

        if (ueIdx < g_dppLastActionByUe.size())
        {
            g_dppLastActionByUe[ueIdx] = static_cast<int32_t>(bestAction);
        }
        if (ueIdx < g_dppHasLastActionByUe.size())
        {
            g_dppHasLastActionByUe[ueIdx] = true;
        }
        if (ueIdx < g_targetMcsByUe.size())
        {
            g_targetMcsByUe[ueIdx] = bestTargetMcs;
        }
        if (ueIdx < g_lastRequestedMcsOffsetByUe.size())
        {
            g_lastRequestedMcsOffsetByUe[ueIdx] = static_cast<double>(bestDelta);
        }
        if (bestTargetBwp != currentBwp && TrySwitchUeToBwp(ueIdx, bestTargetBwp))
        {
            switchCount++;
        }
    }
    ApplyPerUeMcsOverridesToSchedulers();
    return switchCount;
}

static void
RunDppPolicyEpoch()
{
    if (g_bwpBaseline != "dpp" || Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    g_dppLastEpochS = Simulator::Now().GetSeconds();
    g_dppSwitchCountAccum += RunDppPolicyEpochStep();
    const double nextDelayS = std::max(1.0e-6, g_dppEpochMinIntervalS);
    if (Simulator::Now().GetSeconds() + nextDelayS < g_simTime)
    {
        g_dppEpochEvent = Simulator::Schedule(Seconds(nextDelayS), &RunDppPolicyEpoch);
    }
}

static void
TriggerDppPolicyEpoch()
{
    if (g_bwpBaseline != "dpp" || Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    if (g_dppEpochEvent.IsPending())
    {
        return;
    }
    const double nowS = Simulator::Now().GetSeconds();
    const double elapsedS = (g_dppLastEpochS < 0.0) ? 1.0e9 : (nowS - g_dppLastEpochS);
    const double delayS = (g_dppLastEpochS < 0.0) ? 0.0 : std::max(0.0, g_dppEpochMinIntervalS - elapsedS);
    g_dppEpochEvent = Simulator::Schedule(Seconds(delayS), &RunDppPolicyEpoch);
}

static void
RunAamsMcsPolicyStep()
{
    for (uint32_t ueIdx = 0; ueIdx < g_ueStats.size(); ++ueIdx)
    {
        if (ueIdx < g_stepTbCountByUe.size() && g_stepTbCountByUe[ueIdx] > 0)
        {
            double bler = static_cast<double>(g_stepTbErrorByUe[ueIdx]) /
                          static_cast<double>(g_stepTbCountByUe[ueIdx]);

            if (bler > g_aamsTargetBler)
            {
                if (g_targetMcsByUe[ueIdx] >= g_aamsMcsOffset)
                {
                    g_targetMcsByUe[ueIdx] -= g_aamsMcsOffset;
                }
                else
                {
                    g_targetMcsByUe[ueIdx] = 0;
                }
            }
            else if (bler < (g_aamsTargetBler * 0.1))
            {
                if (g_targetMcsByUe[ueIdx] <= 28 - g_aamsMcsOffset)
                {
                    g_targetMcsByUe[ueIdx] += g_aamsMcsOffset;
                }
                else
                {
                    g_targetMcsByUe[ueIdx] = 28;
                }
            }
        }
    }
    ApplyPerUeMcsOverridesToSchedulers();
}

static void
RunStaticMcsOffsetPolicyStep()
{
    for (uint32_t ueIdx = 0; ueIdx < g_ueStats.size(); ++ueIdx)
    {
        // Apply offset against CQI-derived baseline MCS (non-cumulative).
        const int32_t baseMcs = (ueIdx < g_lastCqiByUe.size())
                                    ? EstimateBaseMcsFromCqi(g_lastCqiByUe[ueIdx])
                                    : static_cast<int32_t>(g_initialMcs);
        int32_t targetMcs = std::max(0, std::min(27, baseMcs + g_staticMcsOffset));
        g_targetMcsByUe[ueIdx] = static_cast<uint8_t>(targetMcs);
    }
    ApplyPerUeMcsOverridesToSchedulers();
}

static void
UpdateBaselineSchedulerWeights()
{
    if (g_schedulerPolicy != "age_optimal" &&
        g_schedulerPolicy != "tps" && g_schedulerPolicy != "dgs")
    {
        return;
    }

    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    double nowMs = Simulator::Now().GetMilliSeconds();

    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        uint16_t rnti = g_rntiByUeIdx[ueIdx];
        if (rnti == 0)
        {
            continue;
        }

        const uint32_t qBytes = static_cast<uint32_t>(ComputeDlRlcQueueBytes(ueIdx));
        const double aoiMs = ComputeUeMeanAoiMs(ueIdx);
        const double receiverAoiMs = ComputeExpectedReceiverAoiMs(ueIdx, nowMs);
        const double mcs = (ueIdx < g_lastMcsByUe.size()) ? g_lastMcsByUe[ueIdx] : 0.0;
        const double tbErrorRate =
            (ueIdx < g_stepTbCountByUe.size() && g_stepTbCountByUe[ueIdx] > 0)
                ? static_cast<double>(g_stepTbErrorByUe[ueIdx]) /
                      static_cast<double>(g_stepTbCountByUe[ueIdx])
                : 0.0;


        for (auto& sched : g_baselineSchedulers)
        {
            if (!sched)
            {
                continue;
            }

            if (g_schedulerPolicy == "age_optimal" && receiverAoiMs < g_ageOptimalAoiThresholdMs)
            {
                sched->SetUeReceiverAoiMs(rnti, 0.0);
            }
            else
            {
                sched->SetUeReceiverAoiMs(rnti, receiverAoiMs);
            }

            sched->SetUeAoiMs(rnti, aoiMs);
            sched->SetUeHolDelayMs(rnti, aoiMs);
            sched->SetUeQueueSize(rnti, qBytes);
            sched->SetUeDlMcs(rnti, mcs);
            sched->SetUeTbErrorRate(rnti, tbErrorRate);
            sched->SetUeDeadlineMs(rnti,
                                   (g_schedulerPolicy == "dgs")
                                       ? g_dgsDelayTargetMs
                                       : ((g_schedulerPolicy == "tps") ? g_tpsDeadlineMs
                                                                       : g_aequitasDeadlineMs));
            sched->SetStepDurationMs(g_envStepTime * 1000.0);
            sched->SetAgeOptimalGamma(g_ageOptimalGammaPenalty);
        }
    }
}

static void
ScheduleBaselinePolicyStep()
{
    if (Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }

    for (uint32_t ueIdx = 0; ueIdx < g_policyCooldownStepsByUe.size(); ++ueIdx)
    {
        if (g_policyCooldownStepsByUe[ueIdx] > 0)
        {
            g_policyCooldownStepsByUe[ueIdx]--;
        }
    }

    uint32_t switchCount = 0;
    if (g_bwpBaseline == "dt")
    {
        switchCount = RunDtPolicyStep();
    }
    else if (g_bwpBaseline == "queue")
    {
        switchCount = RunQueueThresholdPolicyStep();
    }
    else if (g_bwpBaseline == "dpp")
    {
        switchCount = g_dppSwitchCountAccum;
        g_dppSwitchCountAccum = 0;
    }
    else if (ShouldDriveAequitasScheduler())
    {
        ApplyAequitasInputsToSchedulers();
    }

    if (g_bwpBaseline != "dpp")
    {
        if (g_mcsBaseline == "aams")
        {
            RunAamsMcsPolicyStep();
        }
        else if (g_staticMcsOffset != 0)
        {
            RunStaticMcsOffsetPolicyStep();
        }
    }

    UpdateBaselineSchedulerWeights();

    RecordIntervalMetrics(switchCount);
    g_lastIntervalSwitchCount = g_nextIntervalSwitchCount;
    g_nextIntervalSwitchCount = switchCount;
    std::fill(g_stepAssignedPrbByUe.begin(), g_stepAssignedPrbByUe.end(), 0);
    std::fill(g_stepTbBytesByUe.begin(), g_stepTbBytesByUe.end(), 0);
    std::fill(g_stepTbCountByUe.begin(), g_stepTbCountByUe.end(), 0);
    std::fill(g_stepTbErrorByUe.begin(), g_stepTbErrorByUe.end(), 0);
    std::fill(g_stepTbErrorByUe.begin(), g_stepTbErrorByUe.end(), 0);
    Simulator::Schedule(Seconds(g_envStepTime), &ScheduleBaselinePolicyStep);
}

static uint64_t
ComputeUeTxBytes(uint32_t ueIdx)
{
    if (ueIdx >= g_txBytesByType.size())
    {
        return 0u;
    }
    uint64_t total = 0u;
    for (uint8_t t = 0; t < TRAFFIC_TYPES; ++t)
    {
        total += g_txBytesByType[ueIdx][t];
    }
    return total;
}

static uint64_t
ComputeUeStepArrivedBytesByType(uint32_t ueIdx, uint8_t type)
{
    if (ueIdx >= g_txBytesByType.size() || ueIdx >= g_prevRewardTxBytesByType.size() || type >= TRAFFIC_TYPES)
    {
        return 0u;
    }
    const uint64_t curr = g_txBytesByType[ueIdx][type];
    const uint64_t prev = g_prevRewardTxBytesByType[ueIdx][type];
    return (curr >= prev) ? (curr - prev) : 0u;
}

static uint64_t
ComputeUeStepArrivedBytes(uint32_t ueIdx)
{
    uint64_t total = 0u;
    for (uint8_t t = 0; t < TRAFFIC_TYPES; ++t)
    {
        total += ComputeUeStepArrivedBytesByType(ueIdx, t);
    }
    return total;
}

static double
ComputeUeStepBler(uint32_t ueIdx)
{
    if (ueIdx >= g_stepTbCountByUe.size() || ueIdx >= g_stepTbErrorByUe.size())
    {
        return 0.0;
    }
    const uint64_t tbCount = g_stepTbCountByUe[ueIdx];
    if (tbCount == 0)
    {
        return 0.0;
    }
    return Clamp01(static_cast<double>(g_stepTbErrorByUe[ueIdx]) / static_cast<double>(tbCount));
}

static int32_t
ComputeCurrentAppStateCode(uint32_t ueIdx)
{
    if (ueIdx >= g_policyBaseStartSByUe.size() || ueIdx >= g_policyPhaseOffsetByUe.size())
    {
        return -1;
    }
    if (ueIdx < g_appStateTimelineByUe.size() && !g_appStateTimelineByUe[ueIdx].empty())
    {
        const double nowS = Simulator::Now().GetSeconds();
        const auto& timeline = g_appStateTimelineByUe[ueIdx];
        for (const auto& seg : timeline)
        {
            if (nowS >= seg.startS && nowS < seg.endS)
            {
                return seg.state;
            }
        }
        if (nowS < timeline.front().startS)
        {
            return timeline.front().state;
        }
        return timeline.back().state;
    }
    const double nowS = Simulator::Now().GetSeconds();
    const double baseS = g_policyBaseStartSByUe[ueIdx];
    const int32_t phase = g_policyPhaseOffsetByUe[ueIdx];
    const double segS = std::max(1e-6, g_policySegmentDurationS);
    const int32_t segIdx = (nowS > baseS) ? static_cast<int32_t>(std::floor((nowS - baseS) / segS)) : 0;
    if (g_appmixStateOverride >= 0)
    {
        return g_appmixStateOverride; // 0=ftp_only, 1=video_only, 2=ftp_video
    }
    return (phase + segIdx) % 3;
}

static std::array<double, 4>
ComputeDrqnLiteBwpMetrics()
{
    std::array<double, 4> metrics = {0.0, 0.0, 0.0, 0.0}; // active0, bler0, active1, bler1
    std::array<double, 2> active = {0.0, 0.0};
    std::array<double, 2> blerSum = {0.0, 0.0};
    std::array<double, 2> blerCount = {0.0, 0.0};
    const double numUes = std::max<uint32_t>(1u, static_cast<uint32_t>(g_ueStats.size()));
    for (uint32_t ueIdx = 0; ueIdx < g_ueStats.size(); ++ueIdx)
    {
        const uint8_t bwp = GetCurrentUeBwp(ueIdx);
        if (bwp > 1)
        {
            continue;
        }
        active[bwp] += 1.0;
        const double tbCount =
            (ueIdx < g_stepTbCountByUe.size()) ? static_cast<double>(g_stepTbCountByUe[ueIdx]) : 0.0;
        if (tbCount > 0.0)
        {
            blerSum[bwp] += ComputeUeStepBler(ueIdx);
            blerCount[bwp] += 1.0;
        }
    }
    metrics[0] = Clamp01(active[0] / numUes);
    metrics[2] = Clamp01(active[1] / numUes);
    metrics[1] = (blerCount[0] > 0.0) ? Clamp01(blerSum[0] / blerCount[0]) : 0.0;
    metrics[3] = (blerCount[1] > 0.0) ? Clamp01(blerSum[1] / blerCount[1]) : 0.0;
    return metrics;
}

static std::vector<double>
BuildDrqnAppmixObservation(uint32_t ueIdx, const std::array<double, 4>& bwpMetrics)
{
    const double blerNorm = Clamp01(ComputeUeStepBler(ueIdx));
    const double queueNorm = Clamp01(ComputeDlRlcQueueBytes(ueIdx) / std::max(1.0, g_queueNormBytes));
    const double channelNorm = (ueIdx < g_lastCqiByUe.size()) ? Clamp01(g_lastCqiByUe[ueIdx] / 15.0) : 0.0;
    const double delayNorm = Clamp01(ComputeUeMeanAoiMs(ueIdx) / std::max(1.0, g_dqnDelayTargetMs));
    const double deliveredBytes =
        (ueIdx < g_stepTbBytesByUe.size()) ? static_cast<double>(g_stepTbBytesByUe[ueIdx]) : 0.0;
    const uint64_t arrivedBytes = ComputeUeStepArrivedBytes(ueIdx);
    const double txSizeNorm = Clamp01(deliveredBytes / std::max(1.0, g_queueNormBytes));
    const double incomingNorm = Clamp01(static_cast<double>(arrivedBytes) / std::max(1.0, g_queueNormBytes));
    double dropRate = 0.0;
    if (arrivedBytes > 0)
    {
        dropRate = Clamp01(std::max(0.0, static_cast<double>(arrivedBytes) - deliveredBytes) /
                           static_cast<double>(arrivedBytes));
    }
    const uint8_t currentBwp = GetCurrentUeBwp(ueIdx);
    const double bwp0 = (currentBwp == g_lowBwpId) ? 1.0 : 0.0;
    const double bwp1 = (currentBwp == g_highBwpId) ? 1.0 : 0.0;

    const uint64_t arrivedFtp = ComputeUeStepArrivedBytesByType(ueIdx, LIGHT_HTTP);
    const uint64_t arrivedVideo = ComputeUeStepArrivedBytesByType(ueIdx, HEAVY_VIDEO);
    const double totalArrived = std::max(1.0, static_cast<double>(arrivedBytes));
    const double ftpShare = Clamp01(static_cast<double>(arrivedFtp) / totalArrived);
    const double videoShare = Clamp01(static_cast<double>(arrivedVideo) / totalArrived);
    const int32_t stateCode = ComputeCurrentAppStateCode(ueIdx);
    const bool burstHeavy = (stateCode == 1 || stateCode == 2);
    const double burstPhase = burstHeavy ? 1.0 : 0.0;

    return {blerNorm,
            dropRate,
            queueNorm,
            channelNorm,
            delayNorm,
            txSizeNorm,
            incomingNorm,
            bwp0,
            bwp1,
            bwpMetrics[0],
            bwpMetrics[1],
            bwpMetrics[2],
            bwpMetrics[3],
            ftpShare,
            videoShare,
            burstPhase};
}

#ifdef HAVE_OPENGYM
static Ptr<OpenGymSpace>
MyGetObservationSpace()
{
    const uint32_t featuresPerUe = g_rlDrqnProfile ? 16u : 8u;
    const uint32_t obsDim = featuresPerUe * static_cast<uint32_t>(g_ueStats.size());
    std::vector<uint32_t> shape = {obsDim};
    if (g_rlDrqnProfile)
    {
        return CreateObject<OpenGymBoxSpace>(-1.0e6f, 1.0e6f, shape, TypeNameGet<float>());
    }
    return CreateObject<OpenGymBoxSpace>(-2.0f, 2.0f, shape, TypeNameGet<float>());
}

static Ptr<OpenGymSpace>
MyGetActionSpace()
{
    const uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    if (numUes <= 1)
    {
        return CreateObject<OpenGymDiscreteSpace>(
            g_rlDrqnProfile ? 2u : (g_enableRlMcsControl ? 10u : 2u));
    }
    std::vector<uint32_t> shape = {numUes};
    return CreateObject<OpenGymBoxSpace>(
        0.0f,
        g_rlDrqnProfile ? 1.0f : (g_enableRlMcsControl ? 9.0f : 1.0f),
        shape,
        TypeNameGet<uint32_t>());
}

static bool
MyGetGameOver()
{
    return Simulator::Now().GetSeconds() >= g_simTime;
}

static Ptr<OpenGymDataContainer>
MyGetObservation()
{
    const uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    const uint32_t featuresPerUe = g_rlDrqnProfile ? 16u : 8u;
    std::vector<uint32_t> shape = {featuresPerUe * numUes};
    Ptr<OpenGymBoxContainer<float>> box = CreateObject<OpenGymBoxContainer<float>>(shape);
    const auto bwpMetrics = ComputeDrqnLiteBwpMetrics();

    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        if (g_rlDrqnProfile)
        {
            const auto obs = BuildDrqnAppmixObservation(ueIdx, bwpMetrics);
            for (double v : obs)
            {
                box->AddValue(static_cast<float>(v));
            }
            continue;
        }
        // Backward-compatible minimal profile.
        box->AddValue(static_cast<float>(GetCurrentUeBwp(ueIdx) == g_highBwpId ? 1.0 : -1.0));
        box->AddValue(static_cast<float>(ComputeDlRlcQueueBytes(ueIdx)));
        box->AddValue(static_cast<float>(ComputeUeMeanAoiMs(ueIdx)));
        box->AddValue(static_cast<float>(ComputeUeThroughputMbps(ueIdx)));
        box->AddValue(static_cast<float>((ueIdx < g_lastCqiByUe.size()) ? g_lastCqiByUe[ueIdx] : 0.0));
        box->AddValue(static_cast<float>(ComputeUeStepBler(ueIdx)));
        box->AddValue(static_cast<float>(ComputeUeStepSpectralEfficiency(ueIdx)));
        box->AddValue(static_cast<float>(ComputeUePrbUtility(ueIdx)));
    }
    return box;
}

static float
MyGetReward()
{
    const uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    if (numUes == 0)
    {
        return 0.0f;
    }

    double rewardSum = 0.0;
    double aoiTermSum = 0.0;
    double thrTermSum = 0.0;
    double switchPenaltySum = 0.0;
    double meanAoi = 0.0;
    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        const double aoiMs = ComputeUeMeanAoiMs(ueIdx);
        meanAoi += aoiMs;
        const double delayNorm = Clamp01(aoiMs / std::max(1.0, g_dqnDelayTargetMs));
        const double deliveredBytes =
            (ueIdx < g_stepTbBytesByUe.size()) ? static_cast<double>(g_stepTbBytesByUe[ueIdx]) : 0.0;
        const double goodputMbps = deliveredBytes * 8.0 / std::max(1e-6, g_envStepTime) / 1e6;
        const double thrNorm = Clamp01(goodputMbps / std::max(1e-6, g_dqnThrTargetMbps));
        const uint32_t latencyUes = static_cast<uint32_t>(std::round(g_dqnLatencyUeRatio * numUes));
        const bool isLatencySensitive = (ueIdx < latencyUes);
        const double lambda = isLatencySensitive ? g_dqnAlpha : (1.0 - g_dqnAlpha);
        const double mu = isLatencySensitive ? g_dqnBeta : (1.0 - g_dqnBeta);
        const double aoiTerm = -(lambda * delayNorm);
        const double thrTerm = -(mu * (1.0 - thrNorm));
        const double switchPenalty =
            (ueIdx < g_lastActualSwitchByUe.size() && g_lastActualSwitchByUe[ueIdx] > 0)
                ? g_rewardLambdaSwitch
                : 0.0;
        const double reward = aoiTerm + thrTerm - switchPenalty;

        rewardSum += reward;
        aoiTermSum += aoiTerm;
        thrTermSum += thrTerm;
        switchPenaltySum += -switchPenalty;

        if (ueIdx < g_lastRewardPrevAoiMsByUe.size())
        {
            g_lastRewardPrevAoiMsByUe[ueIdx] = (ueIdx < g_prevRewardAoiMsByUe.size()) ? g_prevRewardAoiMsByUe[ueIdx] : aoiMs;
        }
        if (ueIdx < g_prevRewardAoiMsByUe.size())
        {
            g_prevRewardAoiMsByUe[ueIdx] = aoiMs;
        }
        if (ueIdx < g_lastRewardPrevQueueBytesByUe.size())
        {
            g_lastRewardPrevQueueBytesByUe[ueIdx] = ComputeDlRlcQueueBytes(ueIdx);
        }
        if (ueIdx < g_lastRewardArrivedBytesByUe.size())
        {
            g_lastRewardArrivedBytesByUe[ueIdx] = ComputeUeStepArrivedBytes(ueIdx);
        }
        if (ueIdx < g_lastRewardDeliveredBytesByUe.size())
        {
            g_lastRewardDeliveredBytesByUe[ueIdx] = (ueIdx < g_stepTbBytesByUe.size()) ? g_stepTbBytesByUe[ueIdx] : 0u;
        }
        if (ueIdx < g_prevRewardTxBytesByUe.size())
        {
            g_prevRewardTxBytesByUe[ueIdx] = ComputeUeTxBytes(ueIdx);
        }
        if (ueIdx < g_prevRewardTxBytesByType.size() && ueIdx < g_txBytesByType.size())
        {
            g_prevRewardTxBytesByType[ueIdx] = g_txBytesByType[ueIdx];
        }
    }
    g_lastMeanAoiMs = meanAoi / static_cast<double>(numUes);
    g_lastMeanRewardAoiTerm = aoiTermSum / static_cast<double>(numUes);
    g_lastMeanRewardThrTerm = thrTermSum / static_cast<double>(numUes);
    g_lastMeanRewardSeTerm = 0.0;
    g_lastMeanRewardSwitchPenalty = switchPenaltySum / static_cast<double>(numUes);
    g_lastMeanRewardTotal = rewardSum / static_cast<double>(numUes);
    return static_cast<float>(g_lastMeanRewardTotal);
}

static std::string
MyGetExtraInfo()
{
    const uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    double meanThrMbps = 0.0;
    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        meanThrMbps += ComputeUeThroughputMbps(ueIdx);
    }
    if (numUes > 0)
    {
        meanThrMbps /= static_cast<double>(numUes);
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "mean_aoi_ms=" << g_lastMeanAoiMs << "|"
        << "queue_overflow_ratio=" << g_lastQueueOverflowRatio << "|"
        << "mean_prb_utility=" << g_lastMeanPrbUtility << "|"
        << "switch_count=" << g_lastIntervalSwitchCount << "|"
        << "requested_bwp0_count=" << g_lastRequestedBwp0Count << "|"
        << "requested_bwp1_count=" << g_lastRequestedBwp1Count << "|"
        << "mean_thr_mbps=" << meanThrMbps << "|"
        << "reward_aoi_penalty=" << g_lastMeanRewardAoiTerm << "|"
        << "reward_goodput_term=" << g_lastMeanRewardThrTerm << "|"
        << "reward_aux_term=0|"
        << "reward_se_term=" << g_lastMeanRewardSeTerm << "|"
        << "reward_switch_penalty=" << g_lastMeanRewardSwitchPenalty << "|"
        << "reward_total=" << g_lastMeanRewardTotal << "|"
        << "bwp0_se="
        << ((g_lowBwpId < g_lastBwpStepSpectralEfficiency.size()) ? g_lastBwpStepSpectralEfficiency[g_lowBwpId] : 0.0)
        << "|bwp1_se="
        << ((g_highBwpId < g_lastBwpStepSpectralEfficiency.size()) ? g_lastBwpStepSpectralEfficiency[g_highBwpId] : 0.0)
        << "|bwp0_total_prb=" << ((g_lowBwpId < g_totalPrbByBwp.size()) ? g_totalPrbByBwp[g_lowBwpId] : 0u)
        << "|bwp1_total_prb=" << ((g_highBwpId < g_totalPrbByBwp.size()) ? g_totalPrbByBwp[g_highBwpId] : 0u);

    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        const double goodputMbps =
            ((ueIdx < g_lastRewardDeliveredBytesByUe.size()) ? static_cast<double>(g_lastRewardDeliveredBytesByUe[ueIdx]) : 0.0) *
            8.0 / std::max(1e-6, g_envStepTime) / 1.0e6;
        oss << "|ue" << ueIdx << "_thr_mbps=" << ComputeUeThroughputMbps(ueIdx)
            << "|ue" << ueIdx << "_goodput_mbps=" << goodputMbps
            << "|ue" << ueIdx << "_aoi_ms=" << ComputeUeMeanAoiMs(ueIdx)
            << "|ue" << ueIdx << "_prev_aoi_ms="
            << ((ueIdx < g_lastRewardPrevAoiMsByUe.size()) ? g_lastRewardPrevAoiMsByUe[ueIdx] : 0.0)
            << "|ue" << ueIdx << "_bler=" << ComputeUeStepBler(ueIdx)
            << "|ue" << ueIdx << "_hol_age_ms=" << ComputeUeMeanAoiMs(ueIdx)
            << "|ue" << ueIdx << "_se=" << ComputeUeStepSpectralEfficiency(ueIdx)
            << "|ue" << ueIdx << "_assigned_prb="
            << ((ueIdx < g_lastStepAssignedPrbByUe.size()) ? static_cast<double>(g_lastStepAssignedPrbByUe[ueIdx]) : 0.0)
            << "|ue" << ueIdx << "_arrived_bytes="
            << ((ueIdx < g_lastRewardArrivedBytesByUe.size()) ? static_cast<double>(g_lastRewardArrivedBytesByUe[ueIdx]) : 0.0)
            << "|ue" << ueIdx << "_delivered_bytes="
            << ((ueIdx < g_lastRewardDeliveredBytesByUe.size()) ? static_cast<double>(g_lastRewardDeliveredBytesByUe[ueIdx]) : 0.0)
            << "|ue" << ueIdx << "_queue_bytes=" << ComputeDlRlcQueueBytes(ueIdx)
            << "|ue" << ueIdx << "_prev_queue_bytes="
            << ((ueIdx < g_lastRewardPrevQueueBytesByUe.size()) ? g_lastRewardPrevQueueBytesByUe[ueIdx] : 0.0)
            << "|ue" << ueIdx << "_actual_switch="
            << ((ueIdx < g_lastActualSwitchByUe.size()) ? static_cast<double>(g_lastActualSwitchByUe[ueIdx]) : 0.0)
            << "|ue" << ueIdx << "_current_bwp=" << static_cast<double>(GetCurrentUeBwp(ueIdx));
    }
    return oss.str();
}

static bool
MyExecuteActions(Ptr<OpenGymDataContainer> action)
{
    const uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    std::vector<uint32_t> codes(numUes, g_enableRlMcsControl ? 2u : 0u);

    if (numUes <= 1)
    {
        Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(action);
        if (discrete)
        {
            codes[0] = discrete->GetValue();
        }
    }
    else
    {
        if (auto boxU = DynamicCast<OpenGymBoxContainer<uint32_t>>(action))
        {
            for (uint32_t i = 0; i < numUes; ++i)
            {
                codes[i] = boxU->GetValue(i);
            }
        }
        else if (auto boxF = DynamicCast<OpenGymBoxContainer<float>>(action))
        {
            for (uint32_t i = 0; i < numUes; ++i)
            {
                codes[i] = static_cast<uint32_t>(std::lround(boxF->GetValue(i)));
            }
        }
    }

    uint32_t switchCount = 0;
    uint32_t requestedBwp0Count = 0;
    uint32_t requestedBwp1Count = 0;
    g_nextSwitchRejectCooldownCount = 0;
    g_nextSwitchRejectNoRntiCount = 0;
    g_nextSwitchRejectSameTargetCount = 0;
    g_nextSwitchRejectMgrBusyCount = 0;
    g_nextSwitchRejectMgrMissingCount = 0;
    g_nextSwitchRejectUeMgrMissingCount = 0;
    std::fill(g_nextActualSwitchByUe.begin(), g_nextActualSwitchByUe.end(), 0u);
    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        const uint32_t code = std::min<uint32_t>(9u, codes[ueIdx]);
        uint8_t bwpTarget = 0;
        uint8_t deltaIdx = 2;
        if (g_enableRlMcsControl)
        {
            bwpTarget = static_cast<uint8_t>(code / 5u);
            deltaIdx = static_cast<uint8_t>(code % 5u);
        }
        else
        {
            // Accept both raw {0,1} and encoded {0..9} inputs.
            bwpTarget = (code <= 1u) ? static_cast<uint8_t>(code)
                                     : static_cast<uint8_t>(std::min<uint32_t>(1u, code / 5u));
        }

        int8_t delta = 0;
        switch (deltaIdx)
        {
        case 0: delta = -2; break;
        case 1: delta = -1; break;
        case 3: delta = +1; break;
        case 4: delta = +2; break;
        default: delta = 0; break;
        }
        if (ueIdx < g_lastRequestedMcsOffsetByUe.size())
        {
            g_lastRequestedMcsOffsetByUe[ueIdx] = static_cast<double>(delta);
        }

        if (bwpTarget == 0)
        {
            requestedBwp0Count++;
        }
        else
        {
            requestedBwp1Count++;
        }

        if (g_enableRlMcsControl && ueIdx < g_targetMcsByUe.size())
        {
            const int baseMcs =
                (ueIdx < g_lastCqiByUe.size()) ? EstimateBaseMcsFromCqi(g_lastCqiByUe[ueIdx]) : static_cast<int>(g_initialMcs);
            int targetMcs = std::max(0, std::min(27, baseMcs + static_cast<int>(delta)));
            g_targetMcsByUe[ueIdx] = static_cast<uint8_t>(targetMcs);
        }

        if (g_enableRlBwpControl && ueIdx < g_rntiByUeIdx.size() && g_rntiByUeIdx[ueIdx] != 0)
        {
            const uint8_t current = GetCurrentUeBwp(ueIdx);
            const uint8_t target = (bwpTarget == 0) ? g_lowBwpId : g_highBwpId;
            if (current != target && TrySwitchUeToBwp(ueIdx, target))
            {
                switchCount++;
                if (ueIdx < g_nextActualSwitchByUe.size())
                {
                    g_nextActualSwitchByUe[ueIdx] = 1;
                }
                if (ueIdx < g_lastBwpSwitchTimeSByUe.size())
                {
                    g_lastBwpSwitchTimeSByUe[ueIdx] = Simulator::Now().GetSeconds();
                }
            }
        }
    }
    if (g_enableRlMcsControl && !g_targetMcsByUe.empty())
    {
        ApplyPerUeMcsOverridesToSchedulers();
    }
    g_nextRequestedBwp0Count = requestedBwp0Count;
    g_nextRequestedBwp1Count = requestedBwp1Count;
    g_nextIntervalSwitchCount = switchCount;
    return true;
}

static void
ScheduleNextStateRead(Ptr<OpenGymInterface> openGym)
{
    if (!openGym || Simulator::Now().GetSeconds() >= g_simTime)
    {
        return;
    }
    for (uint32_t ueIdx = 0; ueIdx < g_policyCooldownStepsByUe.size(); ++ueIdx)
    {
        if (g_policyCooldownStepsByUe[ueIdx] > 0)
        {
            g_policyCooldownStepsByUe[ueIdx]--;
        }
    }

    g_lastIntervalSwitchCount = g_nextIntervalSwitchCount;
    g_lastRequestedBwp0Count = g_nextRequestedBwp0Count;
    g_lastRequestedBwp1Count = g_nextRequestedBwp1Count;
    g_lastSwitchRejectCooldownCount = g_nextSwitchRejectCooldownCount;
    g_lastSwitchRejectNoRntiCount = g_nextSwitchRejectNoRntiCount;
    g_lastSwitchRejectSameTargetCount = g_nextSwitchRejectSameTargetCount;
    g_lastSwitchRejectMgrBusyCount = g_nextSwitchRejectMgrBusyCount;
    g_lastSwitchRejectMgrMissingCount = g_nextSwitchRejectMgrMissingCount;
    g_lastSwitchRejectUeMgrMissingCount = g_nextSwitchRejectUeMgrMissingCount;
    g_lastActualSwitchByUe = g_nextActualSwitchByUe;

    RecordIntervalMetrics(g_lastIntervalSwitchCount);
    g_nextIntervalSwitchCount = 0;
    g_nextRequestedBwp0Count = 0;
    g_nextRequestedBwp1Count = 0;
    g_nextSwitchRejectCooldownCount = 0;
    g_nextSwitchRejectNoRntiCount = 0;
    g_nextSwitchRejectSameTargetCount = 0;
    g_nextSwitchRejectMgrBusyCount = 0;
    g_nextSwitchRejectMgrMissingCount = 0;
    g_nextSwitchRejectUeMgrMissingCount = 0;
    std::fill(g_nextActualSwitchByUe.begin(), g_nextActualSwitchByUe.end(), 0u);

    openGym->NotifyCurrentState();
    std::fill(g_stepAssignedPrbByUe.begin(), g_stepAssignedPrbByUe.end(), 0);
    std::fill(g_stepTbBytesByUe.begin(), g_stepTbBytesByUe.end(), 0);
    std::fill(g_stepTbCountByUe.begin(), g_stepTbCountByUe.end(), 0);
    std::fill(g_stepTbErrorByUe.begin(), g_stepTbErrorByUe.end(), 0);
    Simulator::Schedule(Seconds(g_envStepTime), &ScheduleNextStateRead, openGym);
}
#endif

int
main(int argc, char* argv[])
{
    uint32_t numUes = 20;
    double simTime = g_simTime;
    double appStart = g_appStartTime;
    // double lowFreqHz = 3.5e9;
    double lowFreqHz = 700e6;
    double highFreqHz = 6e9;
    double lowBandwidthHz = 10e6;
    double highBandwidthHz = 60e6;
    double gnbTxPowerDbm = 10.0;
    double ueDistance = 20.0;
    double ueRadius = 100.0;
    double uePatrolRadius = 140.0;
    double ueSpacing = 3.0;
    uint32_t startJitterMs = 50;
    double ueSpeed = 2.0;
    bool enableMobility = true;
    double switchDelayMs = g_switchDelayMsCfg;
    uint32_t randomSeed = 1;
    uint32_t randomRun = 1;

    // Bursty traffic
    double burstRateMbps = 8.0;
    uint32_t burstPktSize = 1200;
    double burstOnMs = 180.0;
    double burstOffMs = 120.0;
    bool burstRandomize = true;
    double burstOnMinMs = 140.0;
    double burstOnMaxMs = 260.0;
    double burstOffMinMs = 80.0;
    double burstOffMaxMs = 220.0;

    // Background traffic
    double backgroundRateKbps = 500.0;
    uint32_t backgroundPktSize = 500;
    std::string trafficModel = "legacy"; // legacy|mixed
    double appLoadScale = 2.5;
    double mixedLightRatio = 0.4;
    double mixedModerateRatio = 0.4;
    double mixedHeavyRatio = 0.2;
    double segmentDurationS = 0.6; // legacy fixed duration fallback
    double segmentDurationMinMs = 500.0;
    double segmentDurationMaxMs = 2000.0;
    int32_t appmixStateOverride = g_appmixStateOverride;

    // Channel dynamics
    bool enableShadowing = true;
    double channelUpdateMs = 1000.0;
    uint32_t extraBuildings = 3;
    std::string summaryFile = "";
    std::string metricsTraceFile = "";
    std::string aoiTraceFile = "";
    int32_t aoiTraceUe = 0;
    std::string aoiStateTraceFile = "";
    int32_t aoiStateTraceUe = 0;
    std::string queueTraceFile = "";
    int32_t queueTraceUe = 0;
    std::string appStateTraceFile = "";
    int32_t appStateTraceUe = 0;
    std::string burstStateTraceFile = "";
    std::string causeTraceFile = "";
    double causeSamplePeriodS = 0.005;
    std::string sinrTraceFile = "";
    double sinrSamplePeriodS = 0.01;
    std::string layoutTraceFile = "";
    std::string trajectoryTraceFile = "";
    double trajectorySamplePeriodS = 0.1;
    bool debugDpp = false;

    std::string errorModel = "ns3::NrEesmCcT2";

    CommandLine cmd(__FILE__);
    cmd.AddValue("numUes", "Number of UEs", numUes);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("appStart", "Application start time (s)", appStart);
    cmd.AddValue("lowFreqHz", "Low-band carrier frequency (Hz)", lowFreqHz);
    cmd.AddValue("highFreqHz", "High-band carrier frequency (Hz)", highFreqHz);
    cmd.AddValue("lowBandwidthHz", "Low-band bandwidth (Hz)", lowBandwidthHz);
    cmd.AddValue("highBandwidthHz", "High-band bandwidth (Hz)", highBandwidthHz);
    cmd.AddValue("gnbTxPowerDbm", "gNB Tx power (dBm)", gnbTxPowerDbm);
    cmd.AddValue("ueDistance", "Base UE distance from gNB (m)", ueDistance);
    cmd.AddValue("ueRadius", "Radius of the circle for UE placement (m)", ueRadius);
    cmd.AddValue("uePatrolRadius",
                 "Radius of the UE mobility patrol area around gNB when mobility is enabled (m)",
                 uePatrolRadius);
    cmd.AddValue("ueSpacing", "UE spacing along x-axis (m)", ueSpacing);
    cmd.AddValue("startJitterMs", "App start jitter (ms)", startJitterMs);
    cmd.AddValue("trafficModel", "Traffic model: legacy|mixed", trafficModel);
    cmd.AddValue("appLoadScale",
                 "Overall semantic app traffic load scale (>0). Scales FTP file size, "
                 "gaming packet size, and video packets/packet size.",
                 appLoadScale);
    cmd.AddValue("mixedLightRatio", "UE ratio for light mixed-traffic class", mixedLightRatio);
    cmd.AddValue("mixedModerateRatio", "UE ratio for moderate mixed-traffic class", mixedModerateRatio);
    cmd.AddValue("mixedHeavyRatio", "UE ratio for heavy mixed-traffic class", mixedHeavyRatio);
    cmd.AddValue("ueSpeed", "UE speed along x-axis (m/s)", ueSpeed);
    cmd.AddValue("enableMobility", "Enable UE mobility", enableMobility);
    cmd.AddValue("randomSeed", "Global ns-3 RNG seed (fixed for reproducibility)", randomSeed);
    cmd.AddValue("randomRun", "Global ns-3 RNG run number", randomRun);
    cmd.AddValue("initialBwpId", "Initial BWP id (0=low,1=high)", g_initialBwpId);
    cmd.AddValue("mcsLowThreshold", "Switch to low BWP if MCS <= threshold", g_mcsLowThreshold);
    cmd.AddValue("mcsHighThreshold", "Switch to high BWP if MCS >= threshold", g_mcsHighThreshold);
    cmd.AddValue("mcsSwitchCount", "Consecutive MCS samples required to switch", g_mcsSwitchCount);
    cmd.AddValue("minSwitchIntervalMs", "Minimum time between switches (ms)", g_minSwitchIntervalMs);
    cmd.AddValue("enableMcsSwitch", "Enable MCS-based BWP switching", g_enableMcsSwitch);
    cmd.AddValue("enableHarqReTx", "Enable HARQ retransmissions in the NR scheduler", g_enableHarqReTx);
    cmd.AddValue("switchDelayMs", "BWP switching delay (ms)", switchDelayMs);
    cmd.AddValue("useSymbolicSwitchDelay",
                 "Derive BWP switch delay from OFDMA symbol count and numerology",
                 g_useSymbolicSwitchDelay);
    cmd.AddValue("switchDelaySymbols", "BWP switch delay in OFDMA symbols", g_switchDelaySymbols);
    cmd.AddValue("switchDelayNumerology",
                 "Numerology (mu) used to convert switchDelaySymbols to ms",
                 g_switchDelayNumerology);
    cmd.AddValue("envStepTime", "Control step time (s)", g_envStepTime);
    cmd.AddValue("enableOpenGym", "Enable OpenGym interface (ns3-gym)", g_enableOpenGym);
    cmd.AddValue("openGymPort", "OpenGym TCP port", g_openGymPort);
    cmd.AddValue("enableRlBwpControl", "Use RL actions for BWP switching", g_enableRlBwpControl);
    cmd.AddValue("enableRlMcsControl", "Use RL actions for MCS delta control", g_enableRlMcsControl);
    cmd.AddValue("rlDebug", "Enable verbose RL action/switch debug logs", g_rlDebug);
    cmd.AddValue("rewardLambdaSwitch", "Reward penalty weight for actual BWP switches in RL profile",
                 g_rewardLambdaSwitch);
    cmd.AddValue("rlDrqnProfile", "Use DRQN state/action/reward profile for OpenGym RL", g_rlDrqnProfile);
    cmd.AddValue("dqnDelayTargetMs", "DRQN delay normalization target (ms)", g_dqnDelayTargetMs);
    cmd.AddValue("dqnThrTargetMbps", "DRQN throughput normalization target (Mbps)", g_dqnThrTargetMbps);
    cmd.AddValue("dqnLatencyUeRatio", "Fraction of UEs treated as latency-sensitive [0,1]", g_dqnLatencyUeRatio);
    cmd.AddValue("dqnAlpha", "Delay weight for latency-sensitive UEs", g_dqnAlpha);
    cmd.AddValue("dqnBeta", "Throughput weight for latency-sensitive UEs", g_dqnBeta);
    cmd.AddValue("enableLosNlosStats",
                 "Enable UE LOS/NLOS dwell and environment-conditioned KPI statistics",
                 g_enableLosNlosStats);
    cmd.AddValue("losSamplePeriodS",
                 "LOS/NLOS dwell sampling period in seconds",
                 g_losSamplePeriodS);
    cmd.AddValue("initialMcs", "Initial fixed DL MCS", g_initialMcs);
    cmd.AddValue("staticMcsOffset", "Static MCS offset to apply to CQI-based MCS", g_staticMcsOffset);
    cmd.AddValue("prbDemandC0", "Linear PRB-demand proxy intercept in bytes/PRB", g_prbDemandC0);
    cmd.AddValue("prbDemandC1", "Linear PRB-demand proxy slope in bytes/PRB per MCS", g_prbDemandC1);
    cmd.AddValue("queueNormBytes", "Queue normalization/overflow threshold in bytes", g_queueNormBytes);
    cmd.AddValue("rlcMaxTxBufferBytes",
                 "Per-RLC-entity MaxTxBufferSize used for RLC config and DPP backlog ratio normalization",
                 g_rlcMaxTxBufferBytes);
    cmd.AddValue("bwpBaseline",
                 "Built-in baseline policy: none|dt|aequitas|queue|dpp",
                 g_bwpBaseline);
    cmd.AddValue("bwpQueueThreshold",
                 "Queue-threshold baseline switch threshold in bytes.",
                 g_bwpQueueThreshold);
    cmd.AddValue("dppV", "DPP V coefficient", g_dppV);
    cmd.AddValue("dppLambdaSwitch", "DPP switch penalty lambda", g_dppLambdaSwitch);
    cmd.AddValue("dppLambdaBler", "DPP expected error-bytes penalty lambda", g_dppLambdaBler);
    cmd.AddValue("dppEpochMinIntervalS",
                 "DPP epoch interval in seconds (default 0.01 for 10ms control).",
                 g_dppEpochMinIntervalS);
    cmd.AddValue("dppMuPriorMeanMbps",
                 "DPP Bayesian prior mean of expected successful TB bytes per epoch.",
                 g_dppMuPriorMeanBytes);
    cmd.AddValue("dppMuPriorPrecision",
                 "DPP Bayesian prior precision for expected goodput",
                 g_dppMuPriorPrecision);
    cmd.AddValue("dppMuObsPrecision",
                 "DPP Bayesian observation precision for expected goodput",
                 g_dppMuObsPrecision);
    cmd.AddValue("dppPosteriorDiscount",
                 "DPP posterior forgetting factor rho in [0,1] (default 0.9).",
                 g_dppPosteriorDiscount);
    cmd.AddValue("dppExploreRandomDurationS",
                 "Initial random-action exploration window for DPP in seconds (then pure score policy).",
                 g_dppExploreRandomDurationS);
    cmd.AddValue("dppScoreDebug",
                 "Enable verbose DPP raw-score tracing to log and optional CSV file",
                 g_dppScoreDebug);
    cmd.AddValue("dppScoreTraceFile",
                 "Write per-UE per-action DPP raw-score terms CSV to this file path.",
                 g_dppScoreTraceFile);
    cmd.AddValue("dppScoreTraceUe",
                 "UE index filter for DPP raw-score tracing (-1 means all UEs).",
                 g_dppScoreTraceUe);
    cmd.AddValue("debug-dpp",
                 "Enable DPP debug bundle (posterior, estimator/score terms, traffic state timeline).",
                 debugDpp);
    cmd.AddValue("schedulerPolicy",
                 "NR MAC scheduler policy: rr|pf|aequitas|age_optimal|tps|dgs",
                 g_schedulerPolicy);
    cmd.AddValue("aequitasDeadlineMs",
                 "AoI deadline (ms) supplied to the Aequitas scheduler baseline.",
                 g_aequitasDeadlineMs);
    cmd.AddValue("aequitasEnableMcsSelection",
                 "Enable Aequitas-specific MCS selection; if false, use AoI-aware ordering only.",
                 g_aequitasEnableMcsSelection);
    cmd.AddValue("ageOptimalGammaPenalty",
                 "Lagrange penalty gamma used by Age-Optimal scheduler metric.",
                 g_ageOptimalGammaPenalty);
    cmd.AddValue("tpsDeadlineMs",
                 "Deadline target (ms) used by TPS surrogate.",
                 g_tpsDeadlineMs);
    cmd.AddValue("dgsDelayTargetMs",
                 "Target delay (ms) used as tau in DGS slack metric.",
                 g_dgsDelayTargetMs);
    cmd.AddValue("policySwitchCooldownSteps",
                 "Minimum control steps between two BWP switches of the same UE",
                 g_policySwitchCooldownSteps);
    cmd.AddValue("dtDelayTargetMs", "DT baseline delay target (ms)", g_dtDelayTargetMs);
    cmd.AddValue("dtQueueHigh", "DT baseline high queue threshold [0,1]", g_dtQueueHigh);
    cmd.AddValue("dtQueueLow", "DT baseline low queue threshold [0,1]", g_dtQueueLow);
    cmd.AddValue("dtPredictGainAoi", "DT baseline AoI trend prediction gain", g_dtPredictGainAoi);
    cmd.AddValue("dtPredictGainQueue",
                 "DT baseline queue trend prediction gain",
                 g_dtPredictGainQueue);
    cmd.AddValue("dtEmaAlpha", "DT baseline EMA alpha", g_dtEmaAlpha);
    cmd.AddValue("segmentDurationS",
                 "Legacy fixed appmix segment duration (s), used when min=max or as fallback.",
                 segmentDurationS);
    cmd.AddValue("segmentDurationMinMs",
                 "Minimum randomized appmix traffic-state segment duration (ms).",
                 segmentDurationMinMs);
    cmd.AddValue("segmentDurationMaxMs",
                 "Maximum randomized appmix traffic-state segment duration (ms).",
                 segmentDurationMaxMs);
    cmd.AddValue("appmixStateOverride",
                 "Override appmix state for all segments: -1=disabled, 0=ftp_only, 1=video_only, 2=ftp_video.",
                 appmixStateOverride);
    cmd.AddValue("burstRateMbps", "Bursty traffic rate (Mbps) during ON", burstRateMbps);
    cmd.AddValue("burstPktSize", "Bursty packet size (bytes)", burstPktSize);
    cmd.AddValue("burstOnMs", "Bursty ON duration (ms)", burstOnMs);
    cmd.AddValue("burstOffMs", "Bursty OFF duration (ms)", burstOffMs);
    cmd.AddValue("burstRandomize", "Use randomized burst On/Off durations", burstRandomize);
    cmd.AddValue("burstOnMinMs", "Random burst ON minimum duration (ms)", burstOnMinMs);
    cmd.AddValue("burstOnMaxMs", "Random burst ON maximum duration (ms)", burstOnMaxMs);
    cmd.AddValue("burstOffMinMs", "Random burst OFF minimum duration (ms)", burstOffMinMs);
    cmd.AddValue("burstOffMaxMs", "Random burst OFF maximum duration (ms)", burstOffMaxMs);
    cmd.AddValue("backgroundRateKbps", "Background traffic rate (Kbps)", backgroundRateKbps);
    cmd.AddValue("backgroundPktSize", "Background packet size (bytes)", backgroundPktSize);
    cmd.AddValue("enableShadowing", "Enable shadowing in pathloss", enableShadowing);
    cmd.AddValue("channelUpdateMs", "3GPP channel model update period (ms)", channelUpdateMs);
    cmd.AddValue("extraBuildings",
                 "Number of additional obstruction layers (0=off, 1=medium, 2=dense, 3=urban-core, 4=urban-mega).",
                 extraBuildings);
    cmd.AddValue("summaryFile", "Append run summary to this file path", summaryFile);
    cmd.AddValue("metricsTraceFile", "Write interval metrics CSV to this file path", metricsTraceFile);
    cmd.AddValue("aoiTraceFile",
                 "Write per-packet AoI trace CSV for the selected UE to this file path.",
                 aoiTraceFile);
    cmd.AddValue("aoiTraceUe", "UE index for per-packet AoI tracing.", aoiTraceUe);
    cmd.AddValue("aoiStateTraceFile",
                 "Write fixed-step AoI state trace CSV for the selected UE to this file path.",
                 aoiStateTraceFile);
    cmd.AddValue("aoiStateTraceUe", "UE index for fixed-step AoI state tracing.", aoiStateTraceUe);
    cmd.AddValue("queueTraceFile",
                 "Write fixed-step total RLC queue trace CSV for the selected UE to this file path.",
                 queueTraceFile);
    cmd.AddValue("queueTraceUe", "UE index for fixed-step total RLC queue tracing.", queueTraceUe);
    cmd.AddValue("appStateTraceFile",
                 "Write app traffic-state interval trace CSV for the selected UE to this file path.",
                 appStateTraceFile);
    cmd.AddValue("appStateTraceUe", "UE index for app traffic-state interval tracing.", appStateTraceUe);
    cmd.AddValue("burstStateTraceFile",
                 "Write burst source-side On/Off state trace CSV for the selected UE to this file path.",
                 burstStateTraceFile);
    cmd.AddValue("causeTraceFile",
                 "Write 5ms-cadence PHY/MAC/RRC observable cause trace CSV for all UEs.",
                 causeTraceFile);
    cmd.AddValue("causeSamplePeriodS",
                 "Cause trace sampling period in seconds.",
                 causeSamplePeriodS);
    cmd.AddValue("sinrTraceFile",
                 "Write fixed-step SINR trace CSV for all UEs to this file path.",
                 sinrTraceFile);
    cmd.AddValue("sinrSamplePeriodS",
                 "SINR trace sampling period in seconds.",
                 sinrSamplePeriodS);
    cmd.AddValue("layoutTraceFile",
                 "Write one-shot layout CSV (gNB, UE initial positions, building boxes) to this file path.",
                 layoutTraceFile);
    cmd.AddValue("trajectoryTraceFile",
                 "Write UE trajectory CSV sampled in time (with LOS/NLOS state) to this file path.",
                 trajectoryTraceFile);
    cmd.AddValue("trajectorySamplePeriodS",
                 "UE trajectory trace sampling period in seconds.",
                 trajectorySamplePeriodS);
    cmd.Parse(argc, argv);
    RngSeedManager::SetSeed(randomSeed);
    RngSeedManager::SetRun(randomRun);

    g_simTime = simTime;
    g_appStartTime = appStart;
    g_switchDelayMsCfg = switchDelayMs;
    if (g_useSymbolicSwitchDelay)
    {
        const double symbolMs = ComputeOFDMASymbolDurationMs(g_switchDelayNumerology);
        g_switchDelayMsCfg = static_cast<double>(g_switchDelaySymbols) * symbolMs;
    }
    if (g_minSwitchIntervalMs <= 0.0)
    {
        g_minSwitchIntervalMs = std::max(1.0, 10.0 * g_switchDelayMsCfg);
    }
    g_bwpBaseline = ToLower(g_bwpBaseline);
    g_schedulerPolicy = ToLower(g_schedulerPolicy);
    g_mcsBaseline = ToLower(g_mcsBaseline);
    trafficModel = ToLower(trafficModel);
    if (g_bwpBaseline != "none" && g_bwpBaseline != "dt" &&
        g_bwpBaseline != "aequitas" && g_bwpBaseline != "queue" &&
        g_bwpBaseline != "dpp")
    {
        NS_ABORT_MSG("Invalid bwpBaseline. Supported values: none|dt|aequitas|queue|dpp");
    }
    if (g_schedulerPolicy != "rr" && g_schedulerPolicy != "pf" && g_schedulerPolicy != "aequitas" &&
        g_schedulerPolicy != "age_optimal" && g_schedulerPolicy != "tps" &&
        g_schedulerPolicy != "dgs")
    {
        NS_ABORT_MSG("Invalid schedulerPolicy. Supported values: rr|pf|aequitas|age_optimal|tps|dgs");
    }
    if (g_mcsBaseline != "cqi" && g_mcsBaseline != "aams")
    {
        NS_ABORT_MSG("Invalid mcsBaseline. Supported values: cqi|aams");
    }
    if (g_bwpBaseline != "none" && g_enableOpenGym)
    {
        NS_LOG_UNCOND("bwpBaseline is set; forcing enableOpenGym=false for standalone baseline mode.");
        g_enableOpenGym = false;
    }
    if (g_enableOpenGym && g_enableMcsSwitch)
    {
        NS_LOG_UNCOND("OpenGym enabled: disabling threshold-based MCS switching to avoid policy conflicts.");
        g_enableMcsSwitch = false;
    }
    g_aequitasDeadlineMs = std::max(1.0, g_aequitasDeadlineMs);
    g_ageOptimalGammaPenalty = std::max(0.0, g_ageOptimalGammaPenalty);
    g_tpsDeadlineMs = std::max(1.0, g_tpsDeadlineMs);
    g_dgsDelayTargetMs = std::max(1.0, g_dgsDelayTargetMs);
    g_dppV = std::max(0.0, g_dppV);
    g_dppLambdaSwitch = std::max(0.0, g_dppLambdaSwitch);
    g_dppLambdaBler = std::max(0.0, g_dppLambdaBler);
    g_dppEpochMinIntervalS = std::max(1.0e-6, g_dppEpochMinIntervalS);
    g_dppMuPriorMeanBytes = std::max(0.0, g_dppMuPriorMeanBytes);
    g_dppMuPriorPrecision = std::max(1e-9, g_dppMuPriorPrecision);
    g_dppMuObsPrecision = std::max(1e-9, g_dppMuObsPrecision);
    g_dppErrPriorMeanBytes = std::max(0.0, g_dppErrPriorMeanBytes);
    g_dppErrPriorPrecision = std::max(1e-9, g_dppErrPriorPrecision);
    g_dppErrObsPrecision = std::max(1e-9, g_dppErrObsPrecision);
    g_dppPosteriorDiscount = Clamp01(g_dppPosteriorDiscount);
    g_dppExploreRandomDurationS = std::max(0.0, g_dppExploreRandomDurationS);
    g_debugDpp = debugDpp;
    if (g_debugDpp)
    {
        g_dppScoreDebug = true;
        g_dppScoreTraceUe = -1;
        if (g_dppScoreTraceFile.empty())
        {
            g_dppScoreTraceFile = "scratch/rl_bwp/runs/debug_dpp_score.csv";
        }
        if (g_dppPosteriorTraceFile.empty())
        {
            g_dppPosteriorTraceFile = "scratch/rl_bwp/runs/debug_dpp_posterior.csv";
        }
        if (g_dppTrafficStateTraceFile.empty())
        {
            g_dppTrafficStateTraceFile = "scratch/rl_bwp/runs/debug_dpp_traffic_state.csv";
        }
    }
    g_bwpQueueThreshold = std::max<uint32_t>(1u, g_bwpQueueThreshold);
    g_rlcMaxTxBufferBytes = std::max<uint32_t>(1u, g_rlcMaxTxBufferBytes);
    g_enablePerUeMcsControl =
        (g_mcsBaseline == "aams" || g_staticMcsOffset != 0 || g_bwpBaseline == "dpp" ||
         (g_enableOpenGym && g_enableRlMcsControl));
    if (trafficModel != "legacy" && trafficModel != "mixed")
    {
        NS_ABORT_MSG("Invalid trafficModel. Supported values: legacy|mixed");
    }
    appLoadScale = std::max(0.05, appLoadScale);
    mixedLightRatio = std::max(0.0, mixedLightRatio);
    mixedModerateRatio = std::max(0.0, mixedModerateRatio);
    mixedHeavyRatio = std::max(0.0, mixedHeavyRatio);
    double mixedRatioSum = mixedLightRatio + mixedModerateRatio + mixedHeavyRatio;
    if (mixedRatioSum <= 0.0)
    {
        mixedLightRatio = 0.4;
        mixedModerateRatio = 0.4;
        mixedHeavyRatio = 0.2;
        mixedRatioSum = 1.0;
    }
    if (appmixStateOverride < -1 || appmixStateOverride > 2)
    {
        NS_ABORT_MSG("Invalid appmixStateOverride. Supported values: -1|0|1|2");
    }
    g_appmixStateOverride = appmixStateOverride;
    g_dqnDelayTargetMs = std::max(1.0, g_dqnDelayTargetMs);
    g_dqnThrTargetMbps = std::max(1e-6, g_dqnThrTargetMbps);
    g_dqnLatencyUeRatio = Clamp01(g_dqnLatencyUeRatio);
    g_dqnAlpha = Clamp01(g_dqnAlpha);
    g_dqnBeta = Clamp01(g_dqnBeta);
    segmentDurationS = std::max(0.05, segmentDurationS);
    segmentDurationMinMs = std::max(50.0, segmentDurationMinMs);
    segmentDurationMaxMs = std::max(segmentDurationMinMs, segmentDurationMaxMs);
    uePatrolRadius = std::max(10.0, uePatrolRadius);
    g_losSamplePeriodS = std::max(0.005, g_losSamplePeriodS);
    g_dtQueueLow = std::max(0.0, std::min(1.0, g_dtQueueLow));
    g_dtQueueHigh = std::max(0.0, std::min(1.0, g_dtQueueHigh));
    if (g_dtQueueLow > g_dtQueueHigh)
    {
        std::swap(g_dtQueueLow, g_dtQueueHigh);
    }
    g_ueStats.assign(numUes, UeStats{});
    g_prbStats.assign(numUes, PrbStats{});
    g_envStatsByUe.assign(numUes, std::array<EnvKpiStats, ENV_STATES>{});
    g_ueMobilityByIdx.assign(numUes, nullptr);
    g_lastLosStateByUe.assign(numUes, -1);
    g_lastLosStateSampleTimeByUe.assign(numUes, -1.0);
    g_policyBaseStartSByUe.assign(numUes, appStart);
    g_policyPhaseOffsetByUe.assign(numUes, 0);
    g_policySegmentDurationS = segmentDurationS;
    g_appStateTimelineByUe.assign(numUes, std::vector<AppStateSegment>{});
    g_packetLevelAoiSamplesMs.clear();
    for (auto& v : g_packetLevelAoiSamplesByTypeMs)
    {
        v.clear();
    }
    g_losConditionModel = g_enableLosNlosStats ? CreateObject<BuildingsChannelConditionModel>() : nullptr;
    g_gnbMobility = nullptr;
    g_rntiByUeIdx.assign(numUes, 0);
    g_txBytesByType.assign(numUes, std::array<uint64_t, TRAFFIC_TYPES>{});
    g_txPacketsByType.assign(numUes, std::array<uint64_t, TRAFFIC_TYPES>{});
    g_rxPacketsByType.assign(numUes, std::array<uint64_t, TRAFFIC_TYPES>{});
    g_rxMissingTagPacketsByType.assign(numUes, std::array<uint64_t, TRAFFIC_TYPES>{});
    g_lastDeliveredAoiMs.assign(numUes, std::array<double, TRAFFIC_TYPES>{});
    g_lastRxTimeByType.assign(numUes, std::array<double, TRAFFIC_TYPES>{});
    g_lastNodeDeliveredAoiMsByUe.assign(numUes, 0.0);
    g_lastNodeRxTimeByUe.assign(numUes, 0.0);
    g_lastCqiByUe.assign(numUes, 0.0);
    g_lastSinrDbByUe.assign(numUes, 0.0);
    g_lastMcsByUe.assign(numUes, static_cast<double>(g_initialMcs));
    g_lastTblerByUe.assign(numUes, -1.0);
    g_lastRequestedMcsOffsetByUe.assign(numUes, 0.0);
    g_lastBwpSwitchTimeSByUe.assign(numUes, 0.0);
    g_rlcDlQueueBytesByUe.assign(numUes, 0.0);
    g_rlcDlQueueBytesByUeAndLcid.assign(numUes, {});
    g_outstandingAoiByUeType.assign(numUes, {});
    g_outstandingAoiByUe.assign(numUes, {});
    g_outstandingAoiPacketById.clear();
    g_outstandingAoiBytesByUe.assign(numUes, 0u);
    g_lastDeliveryTimeNsByUe.assign(numUes, static_cast<uint64_t>(Simulator::Now().GetNanoSeconds()));
    g_nextAoiPacketId = 1;
    g_prevRewardQueueBytesByUe.assign(numUes, 0.0);
    g_currentBwpByUe.assign(numUes, g_initialBwpId);
    g_switchCooldownUntilSByUe.assign(numUes, 0.0);
    g_stepAssignedPrbByUe.assign(numUes, 0);
    g_stepTbBytesByUe.assign(numUes, 0);
    g_stepTbCountByUe.assign(numUes, 0);
    g_stepTbErrorByUe.assign(numUes, 0);
    g_rxPrbCumByUe.assign(numUes, 0);
    g_rxTbBytesCumByUe.assign(numUes, 0);
    g_rxTbCountCumByUe.assign(numUes, 0);
    g_rxTbErrorCumByUe.assign(numUes, 0);
    g_rxTbErrorBytesCumByUe.assign(numUes, 0);
    g_schedCountCumByUe.assign(numUes, 0);
    g_harqFlushCountByUe.assign(numUes, 0);
    g_bwpSwitchExecCountByUe.assign(numUes, 0);
    g_bwpSwitchExec0To1Count = 0;
    g_bwpSwitchExec1To0Count = 0;
    g_causePrevRxPrbByUe.assign(numUes, 0);
    g_causePrevRxTbBytesByUe.assign(numUes, 0);
    g_causePrevRxTbCountByUe.assign(numUes, 0);
    g_causePrevRxTbErrorByUe.assign(numUes, 0);
    g_causePrevSchedCountByUe.assign(numUes, 0);
    g_lastStepAssignedPrbByUe.assign(numUes, 0);
    g_lastStepSpectralEfficiencyByUe.assign(numUes, 0.0);
    g_lastBwpStepSpectralEfficiency.assign(2, 0.0);
    g_lastBwpAvgMcs.assign(2, 0.0);
    g_prevRewardAoiMsByUe.assign(numUes, 0.0);
    g_prevRewardThrMbpsByUe.assign(numUes, 0.0);
    g_prevRewardTxBytesByUe.assign(numUes, 0u);
    g_prevRewardRxBytesByUe.assign(numUes, 0u);
    g_prevRewardTxBytesByType.assign(numUes, std::array<uint64_t, TRAFFIC_TYPES>{});
    g_lastRewardPrevQueueBytesByUe.assign(numUes, 0.0);
    g_lastRewardPrevAoiMsByUe.assign(numUes, 0.0);
    g_lastRewardArrivedBytesByUe.assign(numUes, 0u);
    g_lastRewardDeliveredBytesByUe.assign(numUes, 0u);
    g_lastActualSwitchByUe.assign(numUes, 0);
    g_nextActualSwitchByUe.assign(numUes, 0);
    g_targetMcsByUe.assign(numUes, g_initialMcs);
    g_policyCooldownStepsByUe.assign(numUes, 0);
    g_dtEmaAoiMsByUe.assign(numUes, 0.0);
    g_dtEmaQueueNormByUe.assign(numUes, 0.0);
    std::array<double, G_DPP_ACTION_COUNT> muMeanInit{};
    std::array<double, G_DPP_ACTION_COUNT> muPrecInit{};
    std::array<double, G_DPP_ACTION_COUNT> errMeanInit{};
    std::array<double, G_DPP_ACTION_COUNT> errPrecInit{};
    muMeanInit.fill(std::max(0.0, g_dppMuPriorMeanBytes));
    muPrecInit.fill(std::max(1e-9, g_dppMuPriorPrecision));
    errMeanInit.fill(std::max(0.0, g_dppErrPriorMeanBytes));
    errPrecInit.fill(std::max(1e-9, g_dppErrPriorPrecision));
    g_dppMuPostMeanByUe.assign(numUes, muMeanInit);
    g_dppMuPostPrecisionByUe.assign(numUes, muPrecInit);
    g_dppErrPostMeanByUe.assign(numUes, errMeanInit);
    g_dppErrPostPrecisionByUe.assign(numUes, errPrecInit);
    g_dppLastActionByUe.assign(numUes, -1);
    g_dppHasLastActionByUe.assign(numUes, false);
    g_dppLastDecisionTimeSByUe.assign(numUes, Simulator::Now().GetSeconds());
    g_dppLastEpochDeltaSByUe.assign(numUes, std::max(1e-6, g_dppEpochMinIntervalS));
    g_dppLastTbBytesCumByUe.assign(numUes, 0u);
    g_dppLastTbErrorBytesCumByUe.assign(numUes, 0u);
    g_ueIdxByRnti.clear();
    g_dlSchedulers.clear();
    g_gnbMacs.clear();
    g_pendingBwpSwitchByRnti.clear();
    g_totalPrbByBwp.clear();
    g_rlcTxBufferTraceConnected = false;
    g_rlcAoiLifecycleTraceConnected = false;
    g_lastIntervalSwitchCount = 0;
    g_nextIntervalSwitchCount = 0;
    g_lastRequestedBwp0Count = 0;
    g_lastRequestedBwp1Count = 0;
    g_nextRequestedBwp0Count = 0;
    g_nextRequestedBwp1Count = 0;
    g_lastSwitchRejectCooldownCount = 0;
    g_lastSwitchRejectNoRntiCount = 0;
    g_lastSwitchRejectSameTargetCount = 0;
    g_lastSwitchRejectMgrBusyCount = 0;
    g_lastSwitchRejectMgrMissingCount = 0;
    g_lastSwitchRejectUeMgrMissingCount = 0;
    g_nextSwitchRejectCooldownCount = 0;
    g_nextSwitchRejectNoRntiCount = 0;
    g_nextSwitchRejectSameTargetCount = 0;
    g_nextSwitchRejectMgrBusyCount = 0;
    g_nextSwitchRejectMgrMissingCount = 0;
    g_nextSwitchRejectUeMgrMissingCount = 0;
    g_intervalMetrics = IntervalMetricAccumulator{};
    g_bwpOccupancyUeTimeS = {0.0, 0.0};
    g_dppLastEpochS = -1.0;
    g_dppSwitchCountAccum = 0;
    g_dppEpochEvent = EventId();
    g_dppExploreRv = nullptr;
    g_metricsTraceFile = metricsTraceFile;
    g_aoiTraceFile = aoiTraceFile;
    g_aoiTraceUe = aoiTraceUe;
    g_aoiStateTraceFile = aoiStateTraceFile;
    g_aoiStateTraceUe = aoiStateTraceUe;
    g_queueTraceFile = queueTraceFile;
    g_queueTraceUe = queueTraceUe;
    g_appStateTraceFile = appStateTraceFile;
    g_appStateTraceUe = appStateTraceUe;
    g_burstStateTraceFile = burstStateTraceFile;
    g_causeTraceFile = causeTraceFile;
    g_causeSamplePeriodS = std::max(0.001, causeSamplePeriodS);
    g_sinrTraceFile = sinrTraceFile;
    g_sinrSamplePeriodS = std::max(0.001, sinrSamplePeriodS);
    g_layoutTraceFile = layoutTraceFile;
    g_trajectoryTraceFile = trajectoryTraceFile;
    g_trajectorySamplePeriodS = std::max(0.005, trajectorySamplePeriodS);
    g_metricsTraceStream.reset();
    g_aoiTraceStream.reset();
    g_aoiStateTraceStream.reset();
    g_queueTraceStream.reset();
    g_appStateTraceStream.reset();
    g_burstStateTraceStream.reset();
    g_causeTraceStream.reset();
    g_sinrTraceStream.reset();
    g_trajectoryTraceStream.reset();
    g_dppScoreTraceStream.reset();
    g_dppPosteriorTraceStream.reset();
    g_dppTrafficStateTraceStream.reset();
    if (!g_metricsTraceFile.empty())
    {
        g_metricsTraceStream = std::make_unique<std::ofstream>(g_metricsTraceFile, std::ios::out);
        if (g_metricsTraceStream->is_open())
        {
            (*g_metricsTraceStream) << "time_s,mean_thr_mbps,mean_aoi_ms,"
                                    << "mean_prb_utility,queue_overflow_ratio,switch_count,"
                                    << "requested_bwp0_count,requested_bwp1_count,"
                                    << "reward_aoi_penalty,reward_aux_term,reward_se_term,"
                                    << "reward_switch_penalty,"
                                    << "reward_total\n";
        }
        else
        {
            NS_LOG_UNCOND("Failed to open metricsTraceFile: " << g_metricsTraceFile);
            g_metricsTraceStream.reset();
        }
    }
    if (!g_aoiTraceFile.empty())
    {
        g_aoiTraceStream = std::make_unique<std::ofstream>(g_aoiTraceFile, std::ios::out);
        if (g_aoiTraceStream->is_open())
        {
            (*g_aoiTraceStream) << "time_s,ue_idx,traffic_type,pkt_bytes,aoi_ms\n";
            g_aoiTraceStream->flush();
        }
        else
        {
            NS_LOG_UNCOND("Failed to open aoiTraceFile: " << g_aoiTraceFile);
            g_aoiTraceStream.reset();
        }
    }
    if (!g_aoiStateTraceFile.empty())
    {
        g_aoiStateTraceStream = std::make_unique<std::ofstream>(g_aoiStateTraceFile, std::ios::out);
        if (g_aoiStateTraceStream->is_open())
        {
            (*g_aoiStateTraceStream)
                << "time_s,ue_idx,current_bwp,queue_bytes,aoi_node_ms,aoi_ftp_ms,aoi_gaming_ms,aoi_video_ms\n";
            g_aoiStateTraceStream->flush();
        }
        else
        {
            NS_LOG_UNCOND("Failed to open aoiStateTraceFile: " << g_aoiStateTraceFile);
            g_aoiStateTraceStream.reset();
        }
    }
    if (!g_queueTraceFile.empty())
    {
        g_queueTraceStream = std::make_unique<std::ofstream>(g_queueTraceFile, std::ios::out);
        if (g_queueTraceStream->is_open())
        {
            (*g_queueTraceStream) << "time_s,ue_idx,current_bwp,total_rlc_queue_bytes\n";
            g_queueTraceStream->flush();
        }
        else
        {
            NS_LOG_UNCOND("Failed to open queueTraceFile: " << g_queueTraceFile);
            g_queueTraceStream.reset();
        }
    }
    if (!g_appStateTraceFile.empty())
    {
        g_appStateTraceStream = std::make_unique<std::ofstream>(g_appStateTraceFile, std::ios::out);
        if (g_appStateTraceStream->is_open())
        {
            (*g_appStateTraceStream)
                << "ue_idx,state_code,state_label,start_s,end_s,ftp_on,video_on\n";
            g_appStateTraceStream->flush();
        }
        else
        {
            NS_LOG_UNCOND("Failed to open appStateTraceFile: " << g_appStateTraceFile);
            g_appStateTraceStream.reset();
        }
    }
    if (!g_burstStateTraceFile.empty())
    {
        g_burstStateTraceStream =
            std::make_unique<std::ofstream>(g_burstStateTraceFile, std::ios::out);
        if (g_burstStateTraceStream->is_open())
        {
            (*g_burstStateTraceStream) << "time_s,ue_idx,state\n";
        }
        else
        {
            NS_LOG_UNCOND("Failed to open burstStateTraceFile: " << g_burstStateTraceFile);
            g_burstStateTraceStream.reset();
        }
    }
    if (!g_causeTraceFile.empty())
    {
        g_causeTraceStream =
            std::make_unique<std::ofstream>(g_causeTraceFile, std::ios::out);
        if (g_causeTraceStream->is_open())
        {
            (*g_causeTraceStream)
                << "time_s,ue_idx,current_bwp,pending_switch,pending_target_bwp,cooldown_remaining_ms,"
                << "rlc_queue_bytes,sinr_db,cqi,mcs,tbler,los_state,delta_sched_count,delta_prb,"
                << "delta_tb_bytes,delta_tb_count,delta_tb_error,harq_flush_count,switch_exec_count\n";
            g_causeTraceStream->flush();
        }
        else
        {
            NS_LOG_UNCOND("Failed to open causeTraceFile: " << g_causeTraceFile);
            g_causeTraceStream.reset();
        }
    }
    if (!g_sinrTraceFile.empty())
    {
        g_sinrTraceStream =
            std::make_unique<std::ofstream>(g_sinrTraceFile, std::ios::out);
        if (g_sinrTraceStream->is_open())
        {
            (*g_sinrTraceStream) << "time_s,ue_idx,sinr_db,los_state,current_bwp\n";
            g_sinrTraceStream->flush();
        }
        else
        {
            NS_LOG_UNCOND("Failed to open sinrTraceFile: " << g_sinrTraceFile);
            g_sinrTraceStream.reset();
        }
    }
    if (!g_trajectoryTraceFile.empty())
    {
        g_trajectoryTraceStream =
            std::make_unique<std::ofstream>(g_trajectoryTraceFile, std::ios::out);
        if (g_trajectoryTraceStream->is_open())
        {
            (*g_trajectoryTraceStream) << "time_s,ue_idx,x,y,z,dist_to_gnb_m,los_state\n";
            g_trajectoryTraceStream->flush();
        }
        else
        {
            NS_LOG_UNCOND("Failed to open trajectoryTraceFile: " << g_trajectoryTraceFile);
            g_trajectoryTraceStream.reset();
        }
    }
    if (!g_dppScoreTraceFile.empty())
    {
        g_dppScoreTraceStream =
            std::make_unique<std::ofstream>(g_dppScoreTraceFile, std::ios::out);
        if (g_dppScoreTraceStream->is_open())
        {
            (*g_dppScoreTraceStream)
                << "time_s,ue_idx,action_id,selected,target_bwp,target_mcs,mcs_delta,"
                << "backlog_bytes,backlog_ratio,epoch_delta_s,mu_post_mean_bytes,e_post_mean_bytes,"
                << "mu_current_bytes,switch_cost_bytes,switch_indicator,"
                << "g_hat_bytes,q_weighted_g_hat,penalty_switch,penalty_bler,penalty_total,score\n";
            g_dppScoreTraceStream->flush();
        }
        else
        {
            NS_LOG_UNCOND("Failed to open dppScoreTraceFile: " << g_dppScoreTraceFile);
            g_dppScoreTraceStream.reset();
        }
    }
    if (!g_dppPosteriorTraceFile.empty())
    {
        g_dppPosteriorTraceStream =
            std::make_unique<std::ofstream>(g_dppPosteriorTraceFile, std::ios::out);
        if (g_dppPosteriorTraceStream->is_open())
        {
            (*g_dppPosteriorTraceStream)
                << "time_s,ue_idx,last_action,delta_s,obs_success_bytes,obs_error_bytes,"
                << "mu_mean_before,mu_prec_before,mu_mean_after,mu_prec_after,"
                << "err_mean_before,err_prec_before,err_mean_after,err_prec_after,rho\n";
            g_dppPosteriorTraceStream->flush();
        }
        else
        {
            NS_LOG_UNCOND("Failed to open dppPosteriorTraceFile: " << g_dppPosteriorTraceFile);
            g_dppPosteriorTraceStream.reset();
        }
    }
    if (!g_dppTrafficStateTraceFile.empty())
    {
        g_dppTrafficStateTraceStream =
            std::make_unique<std::ofstream>(g_dppTrafficStateTraceFile, std::ios::out);
        if (g_dppTrafficStateTraceStream->is_open())
        {
            (*g_dppTrafficStateTraceStream)
                << "ue_idx,state_code,state_label,start_s,end_s,ftp_on,video_on\n";
            g_dppTrafficStateTraceStream->flush();
        }
        else
        {
            NS_LOG_UNCOND("Failed to open dppTrafficStateTraceFile: " << g_dppTrafficStateTraceFile);
            g_dppTrafficStateTraceStream.reset();
        }
    }

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    const double gnbX = 0.0;
    const double gnbY = 0.0;
    gnbPos->Add(Vector(gnbX, gnbY, 10.0));
    gnbMobility.SetPositionAllocator(gnbPos);
    gnbMobility.Install(gnbNodes);
    g_gnbMobility = gnbNodes.Get(0)->GetObject<MobilityModel>();

    MobilityHelper ueMobility;
    if (enableMobility)
    {
        std::ostringstream speedExpr;
        speedExpr << "ns3::ConstantRandomVariable[Constant=" << std::max(0.2, ueSpeed) << "]";
        ueMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                    "Bounds",
                                    RectangleValue(Rectangle(gnbX - uePatrolRadius,
                                                             gnbX + uePatrolRadius,
                                                             gnbY - uePatrolRadius,
                                                             gnbY + uePatrolRadius)),
                                    "Speed",
                                    StringValue(speedExpr.str()),
                                    "Distance",
                                    DoubleValue(std::max(1.0, ueSpeed * 2.0)));
    }
    else
    {
        ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    }

    Ptr<UniformDiscPositionAllocator> uePos = CreateObject<UniformDiscPositionAllocator>();
    uePos->SetX(gnbX);
    uePos->SetY(gnbY);
    uePos->SetRho(enableMobility ? std::min(ueRadius, uePatrolRadius * 0.9) : ueRadius);
    ueMobility.SetPositionAllocator(uePos);
    ueMobility.Install(ueNodes);
    for (uint32_t i = 0; i < numUes; ++i)
    {
        g_ueMobilityByIdx[i] = ueNodes.Get(i)->GetObject<MobilityModel>();
    }

    // Build a moderately dense default urban core.
    // Additional layers can be enabled via --extraBuildings to further amplify blockage diversity.
    std::vector<Box> placedBuildingBoxes;
    uint32_t skippedOverlappingBuildings = 0;
    auto overlaps = [](const Box& a, const Box& b) {
        const bool xOverlap = std::max(a.xMin, b.xMin) < std::min(a.xMax, b.xMax);
        const bool yOverlap = std::max(a.yMin, b.yMin) < std::min(a.yMax, b.yMax);
        const bool zOverlap = std::max(a.zMin, b.zMin) < std::min(a.zMax, b.zMax);
        return xOverlap && yOverlap && zOverlap;
    };
    auto addBuilding = [&](double x1,
                           double x2,
                           double y1,
                           double y2,
                           double z2,
                           uint32_t floors,
                           Building::ExtWallsType_t wall = Building::ConcreteWithWindows) {
        Box candidate(x1, x2, y1, y2, 0.0, z2);
        for (const auto& existing : placedBuildingBoxes)
        {
            if (overlaps(candidate, existing))
            {
                skippedOverlappingBuildings++;
                return;
            }
        }
        Ptr<Building> b = CreateObject<Building>();
        b->SetBoundaries(candidate);
        b->SetBuildingType(Building::Residential);
        b->SetExtWallsType(wall);
        b->SetNFloors(floors);
        b->SetNRoomsX(1);
        b->SetNRoomsY(1);
        placedBuildingBoxes.push_back(candidate);
    };

    addBuilding(-18.0, -6.0, -7.0, 9.0, 18.0, 4);
    addBuilding(12.0, 24.0, 10.0, 22.0, 18.0, 3);
    addBuilding(14.0, 30.0, -22.0, -10.0, 16.0, 3);
    addBuilding(-4.0, 6.0, 16.0, 30.0, 20.0, 4);
    addBuilding(-2.0, 8.0, -30.0, -16.0, 18.0, 3);
    // Always add a denser urban core around gNB so UE patrol trajectories alternate LOS/NLOS.
    addBuilding(-8.0, -2.0, -13.0, -9.0, 22.0, 5);
    addBuilding(14.0, 20.0, -6.0, 4.0, 22.0, 5);
    addBuilding(-10.0, -4.0, -20.0, -14.0, 20.0, 4);
    addBuilding(4.0, 10.0, 8.0, 14.0, 20.0, 4);
    addBuilding(-22.0, -16.0, 12.0, 18.0, 20.0, 4);
    addBuilding(16.0, 22.0, -30.0, -24.0, 20.0, 4);
    addBuilding(-32.0,
                -22.0,
                -6.0,
                8.0,
                28.0,
                7,
                Building::ConcreteWithoutWindows);

    if (extraBuildings >= 1)
    {
        addBuilding(-12.0, -4.0, 18.0, 24.0, 16.0, 3);
        addBuilding(-2.0, 6.0, -16.0, -10.0, 15.0, 3);
        addBuilding(2.0, 10.0, -6.0, 2.0, 14.0, 2);
        addBuilding(22.0, 30.0, 2.0, 8.0, 18.0, 3);
    }
    if (extraBuildings >= 2)
    {
        addBuilding(-6.0, 2.0, 8.0, 14.0, 16.0, 3);
        addBuilding(-30.0, -22.0, -14.0, -8.0, 16.0, 3);
        addBuilding(-20.0, -12.0, -18.0, -12.0, 18.0, 4);
        addBuilding(22.0, 28.0, -9.0, -3.0, 18.0, 4);
    }
    if (extraBuildings >= 3)
    {
        // Urban-core dense blockers near gNB/UE cloud center to bias high-frequency NLOS penalty.
        addBuilding(-1.5, 1.5, 3.0, 7.0, 24.0, 6);
        addBuilding(-1.5, 1.5, -7.0, -3.0, 24.0, 6);
        addBuilding(6.0, 9.0, 3.0, 7.0, 22.0, 5);
        addBuilding(10.0, 13.0, -7.0, -3.0, 22.0, 5);
        addBuilding(-9.0, -6.0, 10.0, 14.0, 20.0, 5);
        addBuilding(-15.0, -12.0, -24.0, -20.0, 20.0, 5);
        addBuilding(24.0, 27.0, -1.5, 1.5, 20.0, 5);
        addBuilding(-30.0, -26.0, 10.0, 14.0, 18.0, 4);
    }
    if (extraBuildings >= 4)
    {
        // Urban-mega ring: many tall peripheral blocks to increase NLOS prevalence while
        // preserving some LOS windows near the street canyons.
        addBuilding(-42.0, -36.0, -30.0, -22.0, 26.0, 8);
        addBuilding(-42.0, -36.0, -18.0, -10.0, 24.0, 7);
        addBuilding(-42.0, -36.0, -6.0, 2.0, 24.0, 7);
        addBuilding(-42.0, -36.0, 6.0, 14.0, 24.0, 7);
        addBuilding(-42.0, -36.0, 18.0, 26.0, 26.0, 8);

        addBuilding(36.0, 42.0, -30.0, -22.0, 26.0, 8);
        addBuilding(36.0, 42.0, -18.0, -10.0, 24.0, 7);
        addBuilding(36.0, 42.0, -6.0, 2.0, 24.0, 7);
        addBuilding(36.0, 42.0, 6.0, 14.0, 24.0, 7);
        addBuilding(36.0, 42.0, 18.0, 26.0, 26.0, 8);

        addBuilding(-18.0, -10.0, 34.0, 40.0, 22.0, 6);
        addBuilding(-4.0, 4.0, 34.0, 40.0, 22.0, 6);
        addBuilding(10.0, 18.0, 34.0, 40.0, 22.0, 6);

        addBuilding(-18.0, -10.0, -40.0, -34.0, 22.0, 6);
        addBuilding(-4.0, 4.0, -40.0, -34.0, 22.0, 6);
        addBuilding(10.0, 18.0, -40.0, -34.0, 22.0, 6);
    }
    if (skippedOverlappingBuildings > 0)
    {
        NS_LOG_UNCOND("Skipped overlapping buildings: " << skippedOverlappingBuildings);
    }

    WriteLayoutTrace(placedBuildingBoxes);

    BuildingsHelper::Install(gnbNodes);
    BuildingsHelper::Install(ueNodes);

    if (g_trajectoryTraceStream && g_trajectoryTraceStream->is_open())
    {
        Simulator::Schedule(Seconds(0.0), &SampleTrajectoryTrace);
    }
    if (g_sinrTraceStream && g_sinrTraceStream->is_open())
    {
        Simulator::Schedule(Seconds(0.0), &SampleSinrTrace);
    }
    if (g_causeTraceStream && g_causeTraceStream->is_open())
    {
        Simulator::Schedule(Seconds(0.0), &SampleCauseTrace);
    }
    Simulator::Schedule(Seconds(0.0), &SampleBwpOccupancy);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    beamformingHelper->SetAttribute("BeamformingPeriodicity", TimeValue(MilliSeconds(channelUpdateMs)));
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Buildings", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(enableShadowing));
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(channelUpdateMs)));
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(g_rlcMaxTxBufferBytes));
    Config::SetDefault("ns3::NrRlcAm::MaxTxBufferSize", UintegerValue(g_rlcMaxTxBufferBytes));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf lowConf(lowFreqHz, lowBandwidthHz, 1);
    CcBwpCreator::SimpleOperationBandConf highConf(highFreqHz, highBandwidthHz, 1);
    OperationBandInfo bandLow = ccBwpCreator.CreateOperationBandContiguousCc(lowConf);
    OperationBandInfo bandHigh = ccBwpCreator.CreateOperationBandContiguousCc(highConf);
    channelHelper->AssignChannelsToBands({bandLow, bandHigh});
    auto allBwps = CcBwpCreator::GetAllBwps({bandLow, bandHigh});

    if (g_schedulerPolicy == "aequitas" || g_bwpBaseline == "aequitas")
    {
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaAequitas::GetTypeId());
    }
    else if (g_schedulerPolicy == "age_optimal" ||
             g_schedulerPolicy == "tps" || g_schedulerPolicy == "dgs")
    {
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaBaseline::GetTypeId());
        nrHelper->SetSchedulerAttribute("Mode", StringValue(ToUpper(g_schedulerPolicy)));
    }
    else if (g_schedulerPolicy == "pf")
    {
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaPF::GetTypeId());
    }
    else
    {
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaRR::GetTypeId());
    }
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(g_enableHarqReTx));
    if (g_schedulerPolicy == "aequitas" || g_bwpBaseline == "aequitas")
    {
        nrHelper->SetSchedulerAttribute("EnableMcsSelection",
                                        BooleanValue(g_aequitasEnableMcsSelection));
    }
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(g_enablePerUeMcsControl));
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(false));
    if (g_enablePerUeMcsControl)
    {
        nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(g_initialMcs));
    }

    Config::SetDefault("ns3::NrAmc::ErrorModelType",
                       TypeIdValue(TypeId::LookupByName(errorModel)));
    nrHelper->SetDlErrorModel(errorModel);
    nrHelper->SetUlErrorModel(errorModel);
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(gnbTxPowerDbm));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    InternetStackHelper internet;
    internet.Install(ueNodes);

    auto [remoteHost, remoteHostAddr] = epcHelper->SetupRemoteHost("100Gb/s", 2500, MilliSeconds(1));
    (void)remoteHostAddr;

    Ipv4InterfaceContainer ueIfaces = epcHelper->AssignUeIpv4Address(ueDevs);
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    Ptr<NrGnbNetDevice> gnbNetDev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(0));
    if (gnbNetDev)
    {
        g_gnbBwpMgr = gnbNetDev->GetBwpManager();
        if (g_gnbBwpMgr)
        {
            // The scenario-level deferred switch timer owns the full switch-delay budget.
            g_gnbBwpMgr->SetAttributeSwitchingDelay(MilliSeconds(0));
        }
        for (uint8_t bwpId = 0; bwpId < gnbNetDev->GetCcMapSize(); ++bwpId)
        {
            Ptr<NrGnbMac> gnbMac = gnbNetDev->GetMac(bwpId);
            g_gnbMacs.push_back(gnbMac);
            if (gnbMac)
            {
                gnbMac->TraceConnectWithoutContext("DlScheduling",
                                                   MakeCallback(&DlSchedulingTrace));
                gnbMac->TraceConnectWithoutContext("DlHarqFinalDrop",
                                                   MakeCallback(&OnDlHarqFinalDropAoiTrace));
            }
            Ptr<NrMacSchedulerNs3> sched =
                DynamicCast<NrMacSchedulerNs3>(gnbNetDev->GetScheduler(bwpId));
            g_dlSchedulers.push_back(sched);
            g_aequitasSchedulers.push_back(DynamicCast<NrMacSchedulerOfdmaAequitas>(sched));
            g_baselineSchedulers.push_back(DynamicCast<NrMacSchedulerOfdmaBaseline>(sched));
            if (g_enablePerUeMcsControl && sched)
            {
                sched->SetAttribute("FixedMcsDl", BooleanValue(true));
                sched->SetAttribute("StartingMcsDl", UintegerValue(g_initialMcs));
            }
            Ptr<NrGnbPhy> gnbPhy = gnbNetDev->GetPhy(bwpId);
            uint32_t totalPrb = gnbPhy ? gnbPhy->GetRbNum() : 1u;
            g_totalPrbByBwp.push_back(totalPrb);
        }
    }

    Simulator::Schedule(MilliSeconds(300), &ConnectRlcTxBufferSizeTraces);
    Simulator::Schedule(MilliSeconds(300), &ConnectRlcAoiLifecycleTraces);

    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(i));
        if (ueDev)
        {
            RegisterUeManager(ueDev, i);
        }
    }

    // Attach UE PHY Rx trace for PRB/TB stats
    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(i));
        for (uint8_t bwpId = 0; bwpId < ueDev->GetCcMapSize(); ++bwpId)
        {
            Ptr<NrSpectrumPhy> ueSpectrumPhy = ueDev->GetPhy(bwpId)->GetSpectrumPhy();
            ueSpectrumPhy->TraceConnectWithoutContext("RxPacketTraceUe",
                                                      MakeBoundCallback(&OnRxPacketTraceUe, i));
        }
    }

    // Application setup: each UE alternates over time between FTP, VIDEO, and BOTH.
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;
    ApplicationContainer pingApps;
    Ptr<UniformRandomVariable> startJitter = CreateObject<UniformRandomVariable>();
    std::vector<PerUeTrafficProfile> ueTrafficProfiles(numUes);
    std::mt19937 trafficRng(12345);
    for (uint32_t i = 0; i < numUes; ++i)
    {
        ueTrafficProfiles[i].trafficClass = "dynamic_ftp_video";
    }

    const std::string tgProtocol = "ns3::UdpSocketFactory";
    if (trafficModel == "legacy")
    {
        NS_LOG_UNCOND("trafficModel=legacy is treated the same as mixed (dynamic FTP/VIDEO cycling).");
    }
    const uint32_t ftpPacketSize = 1000;
    const double ftpReadingTimeMeanS = 0.1;
    const double ftpFileSizeMu = 12 + std::log(appLoadScale);
    const double ftpFileSizeSigma = 0.3;
    const uint32_t ftpMaxFileSizeBytes =
        static_cast<uint32_t>(std::max(100000.0, 1500000.0 * appLoadScale));
    const uint32_t heavyVideoPacketsPerFrame =
        static_cast<uint32_t>(std::max(1.0, std::round(64.0 * appLoadScale)));
    const Time heavyVideoInterframe = MilliSeconds(30);
    const double heavyVideoPacketSizeScale = 1000.0 * appLoadScale;
    const double heavyVideoPacketSizeShape = 1.0;
    const double heavyVideoPacketSizeBound = std::max(200.0, 1600.0 * appLoadScale);
    const double heavyVideoPacketTimeScale = 2.5;
    const double heavyVideoPacketTimeShape = 1.0;
    const double heavyVideoPacketTimeBound = 6.0;
    std::uniform_int_distribution<int> phaseDist(0, 2);

    for (uint32_t i = 0; i < numUes; ++i)
    {
        const double jitter = startJitter->GetInteger(0, startJitterMs) / 1000.0;
        Time stopTime = Seconds(simTime);
        const double baseStartS = appStart + jitter;
        const int phase = phaseDist(trafficRng);
        if (i < g_policyBaseStartSByUe.size())
        {
            g_policyBaseStartSByUe[i] = baseStartS;
        }
        if (i < g_policyPhaseOffsetByUe.size())
        {
            g_policyPhaseOffsetByUe[i] = phase;
        }
        const uint16_t ftpPort = static_cast<uint16_t>(4000 + i);
        PacketSinkHelper ftpSinkHelper(tgProtocol, InetSocketAddress(Ipv4Address::GetAny(), ftpPort));
        ApplicationContainer ftpSinkApps = ftpSinkHelper.Install(ueNodes.Get(i));
        serverApps.Add(ftpSinkApps);
        ConnectAoiPacketSinkTrace(ftpSinkApps.Get(0), i, LIGHT_HTTP);

        const uint16_t videoPort = static_cast<uint16_t>(6000 + i);
        PacketSinkHelper videoSinkHelper(tgProtocol, InetSocketAddress(Ipv4Address::GetAny(), videoPort));
        ApplicationContainer videoSinkApps = videoSinkHelper.Install(ueNodes.Get(i));
        serverApps.Add(videoSinkApps);
        ConnectAoiPacketSinkTrace(videoSinkApps.Get(0), i, HEAVY_VIDEO);

        {
            Ptr<NrEpcTft> ftpTft = Create<NrEpcTft>();
            NrEpcTft::PacketFilter pf;
            pf.localPortStart = ftpPort;
            pf.localPortEnd = ftpPort;
            ftpTft->Add(pf);
            NrEpsBearer bearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
            nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(i), bearer, ftpTft);
        }
        {
            Ptr<NrEpcTft> videoTft = Create<NrEpcTft>();
            NrEpcTft::PacketFilter pf;
            pf.localPortStart = videoPort;
            pf.localPortEnd = videoPort;
            videoTft->Add(pf);
            NrEpsBearer bearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
            nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(i), bearer, videoTft);
        }

        PingHelper ping(ueIfaces.GetAddress(i));
        ApplicationContainer uePingApps = ping.Install(remoteHost);
        pingApps.Add(uePingApps);

        ftpSinkApps.Start(Seconds(appStart * 0.5));
        ftpSinkApps.Stop(stopTime);
        videoSinkApps.Start(Seconds(appStart * 0.5));
        videoSinkApps.Stop(stopTime);
        uePingApps.Start(Seconds(appStart * 0.25));
        uePingApps.Stop(Seconds(appStart * 0.45));

        std::uniform_real_distribution<double> segDurDist(segmentDurationMinMs / 1000.0,
                                                          segmentDurationMaxMs / 1000.0);
        double segStartS = baseStartS;
        int segIdx = 0;
        while (segStartS < simTime)
        {
            const double segDurS = std::max(1e-3, segDurDist(trafficRng));
            const double segStopS = std::min(simTime, segStartS + segDurS);
            if (segStopS <= segStartS + 1e-6)
            {
                break;
            }
            {
                int state = (phase + segIdx) % 3;
                if (appmixStateOverride >= 0)
                {
                    state = appmixStateOverride;
                }
                if (i < g_appStateTimelineByUe.size())
                {
                    g_appStateTimelineByUe[i].push_back({segStartS, segStopS, state});
                }
                const bool enableFtp = (state == 0 || state == 2);
                const bool enableVideo = (state == 1 || state == 2);
                if (g_appStateTraceStream && g_appStateTraceStream->is_open() &&
                    i == static_cast<uint32_t>(std::max<int32_t>(0, g_appStateTraceUe)))
                {
                    const std::string stateLabel = GetAppStateLabel(state);
                    (*g_appStateTraceStream) << i << "," << state << "," << stateLabel << ","
                                             << std::fixed << std::setprecision(6) << segStartS << ","
                                             << segStopS << "," << (enableFtp ? 1 : 0) << ","
                                             << (enableVideo ? 1 : 0) << "\n";
                    g_appStateTraceStream->flush();
                }
                if (g_dppTrafficStateTraceStream && g_dppTrafficStateTraceStream->is_open())
                {
                    (*g_dppTrafficStateTraceStream) << i << "," << state << "," << GetAppStateLabel(state)
                                                   << "," << std::fixed << std::setprecision(6)
                                                   << segStartS << "," << segStopS << ","
                                                   << (enableFtp ? 1 : 0) << ","
                                                   << (enableVideo ? 1 : 0) << "\n";
                }
                if (g_debugDpp)
                {
                    NS_LOG_UNCOND("[dpp-traffic] ue=" << i
                                                      << " state=" << state
                                                      << " label=" << GetAppStateLabel(state)
                                                      << " start=" << std::fixed << std::setprecision(6)
                                                      << segStartS
                                                      << " end=" << segStopS
                                                      << " ftp=" << (enableFtp ? 1 : 0)
                                                      << " video=" << (enableVideo ? 1 : 0));
                }

                if (enableFtp)
                {
                    TrafficGeneratorHelper ftpHelper(tgProtocol,
                                                     Address(),
                                                     TrafficGeneratorNgmnFtpMulti::GetTypeId());
                    ftpHelper.SetAttribute("Remote",
                                           AddressValue(InetSocketAddress(ueIfaces.GetAddress(i), ftpPort)));
                    ftpHelper.SetAttribute("PacketSize", UintegerValue(ftpPacketSize));
                    ftpHelper.SetAttribute("ReadingTimeMean", DoubleValue(ftpReadingTimeMeanS));
                    ftpHelper.SetAttribute("FileSizeMu", DoubleValue(ftpFileSizeMu));
                    ftpHelper.SetAttribute("FileSizeSigma", DoubleValue(ftpFileSizeSigma));
                    ftpHelper.SetAttribute("MaxFileSize", UintegerValue(ftpMaxFileSizeBytes));
                    ApplicationContainer ftpApps = ftpHelper.Install(remoteHost);
                    clientApps.Add(ftpApps);
                    Ptr<TrafficGenerator> ftp = DynamicCast<TrafficGenerator>(ftpApps.Get(0));
                    if (ftp)
                    {
                        ftp->TraceConnectWithoutContext("Tx", MakeBoundCallback(&OnAppTx, i, LIGHT_HTTP));
                    }
                    ftpApps.Start(Seconds(segStartS));
                    ftpApps.Stop(Seconds(segStopS));
                }

                if (enableVideo)
                {
                    TrafficGeneratorHelper videoHelper(tgProtocol,
                                                       Address(),
                                                       TrafficGeneratorNgmnVideo::GetTypeId());
                    videoHelper.SetAttribute("Remote",
                                             AddressValue(InetSocketAddress(ueIfaces.GetAddress(i), videoPort)));
                    videoHelper.SetAttribute("NumberOfPacketsInFrame",
                                             UintegerValue(heavyVideoPacketsPerFrame));
                    videoHelper.SetAttribute("InterframeIntervalTime", TimeValue(heavyVideoInterframe));
                    videoHelper.SetAttribute("PacketSizeScale", DoubleValue(heavyVideoPacketSizeScale));
                    videoHelper.SetAttribute("PacketSizeShape", DoubleValue(heavyVideoPacketSizeShape));
                    videoHelper.SetAttribute("PacketSizeBound", DoubleValue(heavyVideoPacketSizeBound));
                    videoHelper.SetAttribute("PacketTimeScale", DoubleValue(heavyVideoPacketTimeScale));
                    videoHelper.SetAttribute("PacketTimeShape", DoubleValue(heavyVideoPacketTimeShape));
                    videoHelper.SetAttribute("PacketTimeBound", DoubleValue(heavyVideoPacketTimeBound));
                    ApplicationContainer videoApps = videoHelper.Install(remoteHost);
                    clientApps.Add(videoApps);
                    Ptr<TrafficGenerator> video = DynamicCast<TrafficGenerator>(videoApps.Get(0));
                    if (video)
                    {
                        video->TraceConnectWithoutContext("Tx", MakeBoundCallback(&OnAppTx, i, HEAVY_VIDEO));
                    }
                    videoApps.Start(Seconds(segStartS));
                    videoApps.Stop(Seconds(segStopS));
                }
            }
            segStartS = segStopS;
            ++segIdx;
        }
    }

    if (ShouldDriveAequitasScheduler())
    {
        ApplyAequitasInputsToSchedulers();
    }

#ifdef HAVE_OPENGYM
    Ptr<OpenGymInterface> openGym;
    if (g_enableOpenGym)
    {
        openGym = CreateObject<OpenGymInterface>(g_openGymPort);
        openGym->SetGetObservationSpaceCb(MakeCallback(&MyGetObservationSpace));
        openGym->SetGetActionSpaceCb(MakeCallback(&MyGetActionSpace));
        openGym->SetGetGameOverCb(MakeCallback(&MyGetGameOver));
        openGym->SetGetObservationCb(MakeCallback(&MyGetObservation));
        openGym->SetGetRewardCb(MakeCallback(&MyGetReward));
        openGym->SetGetExtraInfoCb(MakeCallback(&MyGetExtraInfo));
        openGym->SetExecuteActionsCb(MakeCallback(&MyExecuteActions));
    }
#else
    if (g_enableOpenGym)
    {
        NS_ABORT_MSG("OpenGym requested but ns3/opengym-module.h is unavailable in this build.");
    }
#endif

    if (g_enableOpenGym)
    {
#ifdef HAVE_OPENGYM
        Simulator::Schedule(Seconds(g_envStepTime), &ScheduleNextStateRead, openGym);
#endif
    }
    else
    {
        Simulator::Schedule(Seconds(g_envStepTime), &ScheduleBaselinePolicyStep);
    }
    if (g_bwpBaseline == "dpp")
    {
        TriggerDppPolicyEpoch();
    }
    if ((g_aoiStateTraceStream && g_aoiStateTraceStream->is_open()) ||
        (g_queueTraceStream && g_queueTraceStream->is_open()))
    {
        Simulator::Schedule(Seconds(g_envStepTime), &ScheduleStateTraces);
    }
    if (g_enableLosNlosStats && appStart < simTime)
    {
        Simulator::Schedule(Seconds(appStart), &SampleLosNlosState);
    }

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    FinalizeLosNlosDwellStats(simTime);

    double duration = simTime - appStart;
    if (duration <= 0.0)
    {
        duration = simTime;
    }

    uint64_t totalPrbAll = 0;
    uint64_t totalBytesAll = 0;
    uint64_t totalHttpBytesAll = 0;
    uint64_t totalGamingBytesAll = 0;
    uint64_t totalVideoBytesAll = 0;
    uint64_t totalTbCountAll = 0;
    uint64_t totalTbErrorAll = 0;
    double totalMcsSumAll = 0.0;
    uint64_t totalMcsCountAll = 0;
    double totalTblerSumAll = 0.0;
    uint64_t totalTblerCountAll = 0;
    double totalLosTimeAll = 0.0;
    double totalNlosTimeAll = 0.0;
    uint64_t totalLosBytesAll = 0;
    uint64_t totalNlosBytesAll = 0;
    double totalLosAoiSumMsAll = 0.0;
    double totalNlosAoiSumMsAll = 0.0;
    uint64_t totalLosAoiSamplesAll = 0;
    uint64_t totalNlosAoiSamplesAll = 0;

    NS_LOG_UNCOND("\n=== Per-UE AoI/Throughput/PRB Summary ===");
    for (uint32_t i = 0; i < numUes; ++i)
    {
        const auto& http = g_ueStats[i].flow[LIGHT_HTTP];
        const auto& gaming = g_ueStats[i].flow[MODERATE_GAMING];
        const auto& video = g_ueStats[i].flow[HEAVY_VIDEO];
        uint64_t totalBytes = http.rxBytes + gaming.rxBytes + video.rxBytes;
        double thrMbps = (totalBytes * 8.0) / duration / 1e6;
        const double goodputFtpUeMbps = (static_cast<double>(http.rxBytes) * 8.0) / duration / 1e6;
        const double goodputGamingUeMbps = (static_cast<double>(gaming.rxBytes) * 8.0) / duration / 1e6;
        const double goodputVideoUeMbps = (static_cast<double>(video.rxBytes) * 8.0) / duration / 1e6;
        double avgAoiFtpMs = (http.aoiSamples > 0) ? (http.aoiSumMs / http.aoiSamples) : 0.0;
        double avgAoiGamingMs =
            (gaming.aoiSamples > 0) ? (gaming.aoiSumMs / gaming.aoiSamples) : 0.0;
        double avgAoiVideoMs = (video.aoiSamples > 0) ? (video.aoiSumMs / video.aoiSamples) : 0.0;

        const auto& prb = g_prbStats[i];
        const auto& envLos = g_envStatsByUe[i][ENV_LOS];
        const auto& envNlos = g_envStatsByUe[i][ENV_NLOS];
        const double losTimeS = envLos.dwellTimeS;
        const double nlosTimeS = envNlos.dwellTimeS;
        const double envTimeS = losTimeS + nlosTimeS;
        const double losRatio = (envTimeS > 0.0) ? (losTimeS / envTimeS) : 0.0;
        const double nlosRatio = (envTimeS > 0.0) ? (nlosTimeS / envTimeS) : 0.0;
        const double thrLosMbps =
            (losTimeS > 0.0) ? (static_cast<double>(envLos.rxBytes) * 8.0 / losTimeS / 1e6) : 0.0;
        const double thrNlosMbps =
            (nlosTimeS > 0.0) ? (static_cast<double>(envNlos.rxBytes) * 8.0 / nlosTimeS / 1e6) : 0.0;
        const double avgAoiLosMs =
            (envLos.aoiSamples > 0) ? (envLos.aoiSumMs / envLos.aoiSamples) : 0.0;
        const double avgAoiNlosMs =
            (envNlos.aoiSamples > 0) ? (envNlos.aoiSumMs / envNlos.aoiSamples) : 0.0;
        double bytesPerPrb =
            (prb.prbTotal > 0) ? (static_cast<double>(totalBytes) / prb.prbTotal) : 0.0;
        double avgMcs = (prb.mcsCount > 0) ? (prb.mcsSum / prb.mcsCount) : 0.0;
        double bler = (prb.tbCount > 0) ? (static_cast<double>(prb.tbError) / prb.tbCount) : 0.0;
        double avgTbler =
            (prb.tblerCount > 0) ? (prb.tblerSum / prb.tblerCount) : 0.0;

        NS_LOG_UNCOND(std::fixed << std::setprecision(3)
                                 << "UE" << i
                                 << " class=" << ueTrafficProfiles[i].trafficClass
                                 << " thr=" << thrMbps << " Mbps"
                                 << " goodput(FTP/GAMING/VIDEO)="
                                 << goodputFtpUeMbps << "/" << goodputGamingUeMbps << "/"
                                 << goodputVideoUeMbps << " Mbps"
                                 << " AoI(node)=" << ComputeUeKpiAoiMs(i) << " ms"
                                 << " packetAoI(FTP/GAMING/VIDEO)="
                                 << avgAoiFtpMs << "/" << avgAoiGamingMs << "/" << avgAoiVideoMs
                                 << " ms"
                                 << " avgMcs=" << avgMcs
                                 << " bler=" << bler
                                 << " avgTbler=" << avgTbler
                                 << " PRB(total)=" << prb.prbTotal
                                 << " bytes/PRB=" << bytesPerPrb
                                 << " LOS_ratio=" << losRatio
                                 << " NLOS_ratio=" << nlosRatio
                                 << " thr(LOS)=" << thrLosMbps << " Mbps"
                                 << " thr(NLOS)=" << thrNlosMbps << " Mbps"
                                 << " AoI(LOS)=" << avgAoiLosMs << " ms"
                                 << " AoI(NLOS)=" << avgAoiNlosMs << " ms");

        totalPrbAll += prb.prbTotal;
        totalBytesAll += totalBytes;
        totalHttpBytesAll += http.rxBytes;
        totalGamingBytesAll += gaming.rxBytes;
        totalVideoBytesAll += video.rxBytes;
        totalTbCountAll += prb.tbCount;
        totalTbErrorAll += prb.tbError;
        totalMcsSumAll += prb.mcsSum;
        totalMcsCountAll += prb.mcsCount;
        totalTblerSumAll += prb.tblerSum;
        totalTblerCountAll += prb.tblerCount;
        totalLosTimeAll += losTimeS;
        totalNlosTimeAll += nlosTimeS;
        totalLosBytesAll += envLos.rxBytes;
        totalNlosBytesAll += envNlos.rxBytes;
        totalLosAoiSumMsAll += envLos.aoiSumMs;
        totalNlosAoiSumMsAll += envNlos.aoiSumMs;
        totalLosAoiSamplesAll += envLos.aoiSamples;
        totalNlosAoiSamplesAll += envNlos.aoiSamples;
    }

    double bytesPerPrbAll =
        (totalPrbAll > 0) ? (static_cast<double>(totalBytesAll) / totalPrbAll) : 0.0;
    double runMeanThrMbps = 0.0;
    double runMeanAoiMs = 0.0;
    double runQueueOverflowRatio = 0.0;
    double runMeanSwitchCount = 0.0;
    double runTotalSwitchCount = 0.0;
    if (g_intervalMetrics.samples > 0)
    {
        double denom = static_cast<double>(g_intervalMetrics.samples);
        runMeanThrMbps = g_intervalMetrics.meanThrMbpsSum / denom;
        runMeanAoiMs = g_intervalMetrics.meanAoiMsSum / denom;
        runQueueOverflowRatio = g_intervalMetrics.queueOverflowRatioSum / denom;
        runMeanSwitchCount = g_intervalMetrics.switchCountSum / denom;
    }
    runTotalSwitchCount = g_intervalMetrics.switchCountSum;
    double avgMcsAll = (totalMcsCountAll > 0) ? (totalMcsSumAll / totalMcsCountAll) : 0.0;
    double blerAll =
        (totalTbCountAll > 0) ? (static_cast<double>(totalTbErrorAll) / totalTbCountAll) : 0.0;
    double avgTblerAll =
        (totalTblerCountAll > 0) ? (totalTblerSumAll / totalTblerCountAll) : 0.0;
    const double totalEnvTimeAll = totalLosTimeAll + totalNlosTimeAll;
    const double losRatioAll = (totalEnvTimeAll > 0.0) ? (totalLosTimeAll / totalEnvTimeAll) : 0.0;
    const double nlosRatioAll =
        (totalEnvTimeAll > 0.0) ? (totalNlosTimeAll / totalEnvTimeAll) : 0.0;
    const double thrLosAllMbps =
        (totalLosTimeAll > 0.0) ? (static_cast<double>(totalLosBytesAll) * 8.0 / totalLosTimeAll / 1e6) : 0.0;
    const double thrNlosAllMbps = (totalNlosTimeAll > 0.0)
                                      ? (static_cast<double>(totalNlosBytesAll) * 8.0 / totalNlosTimeAll / 1e6)
                                      : 0.0;
    const double avgAoiLosAllMs =
        (totalLosAoiSamplesAll > 0) ? (totalLosAoiSumMsAll / totalLosAoiSamplesAll) : 0.0;
    const double avgAoiNlosAllMs =
        (totalNlosAoiSamplesAll > 0) ? (totalNlosAoiSumMsAll / totalNlosAoiSamplesAll) : 0.0;
    const double dppServiceRateGoodputMbps =
        (duration > 0.0) ? (static_cast<double>(totalBytesAll) * 8.0 / duration / 1e6) : 0.0;
    const double goodputFtpMbps =
        (duration > 0.0) ? (static_cast<double>(totalHttpBytesAll) * 8.0 / duration / 1e6) : 0.0;
    const double goodputGamingMbps =
        (duration > 0.0) ? (static_cast<double>(totalGamingBytesAll) * 8.0 / duration / 1e6) : 0.0;
    const double goodputVideoMbps =
        (duration > 0.0) ? (static_cast<double>(totalVideoBytesAll) * 8.0 / duration / 1e6) : 0.0;

    const PacketAoiStats packetAoiAll = ComputePacketAoiStats(g_packetLevelAoiSamplesMs);
    const PacketAoiStats packetAoiFtp = ComputePacketAoiStats(g_packetLevelAoiSamplesByTypeMs[LIGHT_HTTP]);
    const PacketAoiStats packetAoiGaming =
        ComputePacketAoiStats(g_packetLevelAoiSamplesByTypeMs[MODERATE_GAMING]);
    const PacketAoiStats packetAoiVideo =
        ComputePacketAoiStats(g_packetLevelAoiSamplesByTypeMs[HEAVY_VIDEO]);
    uint64_t actualBwpSwitchExecTotal = 0u;
    for (uint64_t v : g_bwpSwitchExecCountByUe)
    {
        actualBwpSwitchExecTotal += v;
    }
    const double actualBwpSwitchExecMeanPerUe =
        (numUes > 0) ? (static_cast<double>(actualBwpSwitchExecTotal) / static_cast<double>(numUes))
                     : 0.0;
    const double totalUeTimeS = static_cast<double>(numUes) * std::max(1e-9, g_simTime);
    const double bwp0OccupancyRatio = Clamp01(g_bwpOccupancyUeTimeS[0] / totalUeTimeS);
    const double bwp1OccupancyRatio = Clamp01(g_bwpOccupancyUeTimeS[1] / totalUeTimeS);

    NS_LOG_UNCOND("\n=== Aggregate PRB (bytes-share estimate) ===");
    NS_LOG_UNCOND(std::fixed << std::setprecision(3)
                             << "runMeanThr=" << runMeanThrMbps << " Mbps "
                             << "runMeanAoI=" << runMeanAoiMs << " ms "
                             << "runQueueOverflowRatio=" << runQueueOverflowRatio << " "
                             << "runMeanSwitchCount=" << runMeanSwitchCount << " "
                             << "runTotalSwitchCount=" << runTotalSwitchCount << " "
                             << "actualBwpSwitchExecTotal=" << actualBwpSwitchExecTotal << " "
                             << "actualBwpSwitchExec0To1=" << g_bwpSwitchExec0To1Count << " "
                             << "actualBwpSwitchExec1To0=" << g_bwpSwitchExec1To0Count << " "
                             << "actualBwpSwitchExecMeanPerUe=" << actualBwpSwitchExecMeanPerUe
                             << " "
                             << "avgMcs=" << avgMcsAll
                             << " bler=" << blerAll
                             << " avgTbler=" << avgTblerAll
                             << " PRB(total)=" << totalPrbAll
                             << " bytes/PRB=" << bytesPerPrbAll);
    NS_LOG_UNCOND(std::fixed << std::setprecision(3)
                             << "LOS_ratio=" << losRatioAll
                             << " NLOS_ratio=" << nlosRatioAll
                             << " thr(LOS)=" << thrLosAllMbps << " Mbps"
                             << " thr(NLOS)=" << thrNlosAllMbps << " Mbps"
                             << " AoI(LOS)=" << avgAoiLosAllMs << " ms"
                             << " AoI(NLOS)=" << avgAoiNlosAllMs << " ms"
                             << " serviceRateGoodput=" << dppServiceRateGoodputMbps << " Mbps"
                             << " bwpOccupancyRatio(BWP0/BWP1)=" << bwp0OccupancyRatio << "/"
                             << bwp1OccupancyRatio
                             << " packetAoI(mean/p25/p50/p75/min/max)="
                             << packetAoiAll.meanMs << "/" << packetAoiAll.p25Ms << "/"
                             << packetAoiAll.p50Ms << "/" << packetAoiAll.p75Ms << "/"
                             << packetAoiAll.minMs << "/" << packetAoiAll.maxMs << " ms");
    NS_LOG_UNCOND(std::fixed << std::setprecision(3)
                             << "goodputByType(FTP/GAMING/VIDEO)=" << goodputFtpMbps << "/"
                             << goodputGamingMbps << "/" << goodputVideoMbps << " Mbps"
                             << " packetAoIByTypeMean(FTP/GAMING/VIDEO)=" << packetAoiFtp.meanMs
                             << "/" << packetAoiGaming.meanMs << "/" << packetAoiVideo.meanMs
                             << " ms");

    if (!summaryFile.empty())
    {
        std::ofstream out(summaryFile, std::ios::app);
        if (out.is_open())
        {
            out << std::fixed << std::setprecision(6);
            out << "simTime=" << simTime << ",appStart=" << appStart << ",numUes=" << numUes
                << ",bwpBaseline=" << g_bwpBaseline << ",trafficModel=" << trafficModel
                << ",burstRateMbps=" << burstRateMbps
                << ",backgroundRateKbps=" << backgroundRateKbps
                << ",duration=" << duration << ",avgMcs=" << avgMcsAll << ",bler=" << blerAll
                << ",avgTbler=" << avgTblerAll << ",runMeanThrMbps=" << runMeanThrMbps
                << ",runMeanAoiMs=" << runMeanAoiMs
                << ",runQueueOverflowRatio=" << runQueueOverflowRatio
                << ",runMeanSwitchCount=" << runMeanSwitchCount
                << ",runTotalSwitchCount=" << runTotalSwitchCount
                << ",actualBwpSwitchExecTotal=" << actualBwpSwitchExecTotal
                << ",actualBwpSwitchExec0To1=" << g_bwpSwitchExec0To1Count
                << ",actualBwpSwitchExec1To0=" << g_bwpSwitchExec1To0Count
                << ",actualBwpSwitchExecMeanPerUe=" << actualBwpSwitchExecMeanPerUe
                << ",prbTotal=" << totalPrbAll
                << ",bytesPerPrb=" << bytesPerPrbAll
                << ",losRatio=" << losRatioAll << ",nlosRatio=" << nlosRatioAll
                << ",thrLosMbps=" << thrLosAllMbps << ",thrNlosMbps=" << thrNlosAllMbps
                << ",aoiLosMs=" << avgAoiLosAllMs << ",aoiNlosMs=" << avgAoiNlosAllMs
                << ",dppServiceRateGoodputMbps=" << dppServiceRateGoodputMbps
                << ",bwp0OccupancyRatio=" << bwp0OccupancyRatio
                << ",bwp1OccupancyRatio=" << bwp1OccupancyRatio
                << ",bwp0OccupancyUeTimeS=" << g_bwpOccupancyUeTimeS[0]
                << ",bwp1OccupancyUeTimeS=" << g_bwpOccupancyUeTimeS[1]
                << ",goodputFtpMbps=" << goodputFtpMbps
                << ",goodputGamingMbps=" << goodputGamingMbps
                << ",goodputVideoMbps=" << goodputVideoMbps
                << ",packetAoiMeanMs=" << packetAoiAll.meanMs
                << ",packetAoiP25Ms=" << packetAoiAll.p25Ms
                << ",packetAoiP50Ms=" << packetAoiAll.p50Ms
                << ",packetAoiP75Ms=" << packetAoiAll.p75Ms
                << ",packetAoiMinMs=" << packetAoiAll.minMs
                << ",packetAoiMaxMs=" << packetAoiAll.maxMs
                << ",packetAoiFtpMeanMs=" << packetAoiFtp.meanMs
                << ",packetAoiFtpP25Ms=" << packetAoiFtp.p25Ms
                << ",packetAoiFtpP50Ms=" << packetAoiFtp.p50Ms
                << ",packetAoiFtpP75Ms=" << packetAoiFtp.p75Ms
                << ",packetAoiFtpMinMs=" << packetAoiFtp.minMs
                << ",packetAoiFtpMaxMs=" << packetAoiFtp.maxMs
                << ",packetAoiGamingMeanMs=" << packetAoiGaming.meanMs
                << ",packetAoiGamingP25Ms=" << packetAoiGaming.p25Ms
                << ",packetAoiGamingP50Ms=" << packetAoiGaming.p50Ms
                << ",packetAoiGamingP75Ms=" << packetAoiGaming.p75Ms
                << ",packetAoiGamingMinMs=" << packetAoiGaming.minMs
                << ",packetAoiGamingMaxMs=" << packetAoiGaming.maxMs
                << ",packetAoiVideoMeanMs=" << packetAoiVideo.meanMs
                << ",packetAoiVideoP25Ms=" << packetAoiVideo.p25Ms
                << ",packetAoiVideoP50Ms=" << packetAoiVideo.p50Ms
                << ",packetAoiVideoP75Ms=" << packetAoiVideo.p75Ms
                << ",packetAoiVideoMinMs=" << packetAoiVideo.minMs
                << ",packetAoiVideoMaxMs=" << packetAoiVideo.maxMs
                << "\n";
            for (uint32_t i = 0; i < numUes; ++i)
            {
                const auto& http = g_ueStats[i].flow[LIGHT_HTTP];
                const auto& gaming = g_ueStats[i].flow[MODERATE_GAMING];
                const auto& video = g_ueStats[i].flow[HEAVY_VIDEO];
                uint64_t totalBytes = http.rxBytes + gaming.rxBytes + video.rxBytes;
                double thrMbps = (totalBytes * 8.0) / duration / 1e6;
                const double goodputFtpUeMbps =
                    (static_cast<double>(http.rxBytes) * 8.0) / duration / 1e6;
                const double goodputGamingUeMbps =
                    (static_cast<double>(gaming.rxBytes) * 8.0) / duration / 1e6;
                const double goodputVideoUeMbps =
                    (static_cast<double>(video.rxBytes) * 8.0) / duration / 1e6;
                double avgAoiFtpMs = (http.aoiSamples > 0) ? (http.aoiSumMs / http.aoiSamples) : 0.0;
                double avgAoiGamingMs =
                    (gaming.aoiSamples > 0) ? (gaming.aoiSumMs / gaming.aoiSamples) : 0.0;
                double avgAoiVideoMs =
                    (video.aoiSamples > 0) ? (video.aoiSumMs / video.aoiSamples) : 0.0;
                const auto& prb = g_prbStats[i];
                const auto& envLos = g_envStatsByUe[i][ENV_LOS];
                const auto& envNlos = g_envStatsByUe[i][ENV_NLOS];
                const double losTimeS = envLos.dwellTimeS;
                const double nlosTimeS = envNlos.dwellTimeS;
                const double envTimeS = losTimeS + nlosTimeS;
                const double losRatio = (envTimeS > 0.0) ? (losTimeS / envTimeS) : 0.0;
                const double nlosRatio = (envTimeS > 0.0) ? (nlosTimeS / envTimeS) : 0.0;
                const double thrLosMbps =
                    (losTimeS > 0.0) ? (static_cast<double>(envLos.rxBytes) * 8.0 / losTimeS / 1e6) : 0.0;
                const double thrNlosMbps =
                    (nlosTimeS > 0.0) ? (static_cast<double>(envNlos.rxBytes) * 8.0 / nlosTimeS / 1e6) : 0.0;
                const double avgAoiLosMs =
                    (envLos.aoiSamples > 0) ? (envLos.aoiSumMs / envLos.aoiSamples) : 0.0;
                const double avgAoiNlosMs =
                    (envNlos.aoiSamples > 0) ? (envNlos.aoiSumMs / envNlos.aoiSamples) : 0.0;
                double bytesPerPrb =
                    (prb.prbTotal > 0) ? (static_cast<double>(totalBytes) / prb.prbTotal) : 0.0;
                double avgSinr = (prb.sinrSamples > 0) ? (prb.sinrSumDb / prb.sinrSamples) : -100.0;
                out << "ue=" << i << ",class=" << ueTrafficProfiles[i].trafficClass
                    << ",thrMbps=" << thrMbps
                    << ",goodputFtpMbps=" << goodputFtpUeMbps
                    << ",goodputGamingMbps=" << goodputGamingUeMbps
                    << ",goodputVideoMbps=" << goodputVideoUeMbps
                    << ",aoiNodeMs=" << ComputeUeKpiAoiMs(i)
                    << ",packetAoiFtpMeanMs=" << avgAoiFtpMs
                    << ",packetAoiGamingMeanMs=" << avgAoiGamingMs
                    << ",packetAoiVideoMeanMs=" << avgAoiVideoMs
                    << ",avgSinrDb=" << avgSinr
                    << ",prbTotal=" << prb.prbTotal
                    << ",bytesPerPrb=" << bytesPerPrb
                    << ",losRatio=" << losRatio << ",nlosRatio=" << nlosRatio
                    << ",thrLosMbps=" << thrLosMbps << ",thrNlosMbps=" << thrNlosMbps
                    << ",aoiLosMs=" << avgAoiLosMs << ",aoiNlosMs=" << avgAoiNlosMs
                    << "\n";
            }
            out.flush();
            NS_LOG_UNCOND("Saved summaryFile: " << summaryFile);
        }
        else
        {
            NS_LOG_UNCOND("Failed to open summaryFile: " << summaryFile);
        }
    }

    if (g_metricsTraceStream && g_metricsTraceStream->is_open())
    {
        g_metricsTraceStream->flush();
        g_metricsTraceStream->close();
    }
    if (g_aoiTraceStream && g_aoiTraceStream->is_open())
    {
        g_aoiTraceStream->flush();
        g_aoiTraceStream->close();
    }
    if (g_aoiStateTraceStream && g_aoiStateTraceStream->is_open())
    {
        g_aoiStateTraceStream->flush();
        g_aoiStateTraceStream->close();
    }
    if (g_appStateTraceStream && g_appStateTraceStream->is_open())
    {
        g_appStateTraceStream->flush();
        g_appStateTraceStream->close();
    }
    if (g_burstStateTraceStream && g_burstStateTraceStream->is_open())
    {
        g_burstStateTraceStream->flush();
        g_burstStateTraceStream->close();
    }
    if (g_causeTraceStream && g_causeTraceStream->is_open())
    {
        g_causeTraceStream->flush();
        g_causeTraceStream->close();
    }
    if (g_sinrTraceStream && g_sinrTraceStream->is_open())
    {
        g_sinrTraceStream->flush();
        g_sinrTraceStream->close();
    }
    if (g_trajectoryTraceStream && g_trajectoryTraceStream->is_open())
    {
        g_trajectoryTraceStream->flush();
        g_trajectoryTraceStream->close();
    }
    if (g_dppScoreTraceStream && g_dppScoreTraceStream->is_open())
    {
        g_dppScoreTraceStream->flush();
        g_dppScoreTraceStream->close();
    }
    if (g_dppPosteriorTraceStream && g_dppPosteriorTraceStream->is_open())
    {
        g_dppPosteriorTraceStream->flush();
        g_dppPosteriorTraceStream->close();
    }
    if (g_dppTrafficStateTraceStream && g_dppTrafficStateTraceStream->is_open())
    {
        g_dppTrafficStateTraceStream->flush();
        g_dppTrafficStateTraceStream->close();
    }

    Simulator::Destroy();
    return 0;
}
