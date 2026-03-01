// Scenario: Single-cell NR (sub-6GHz) with UMi+Buildings, shadowing/fading enabled.
// Mixed traffic: bursty + background (OnOff UDP). Collect per-UE/per-traffic-type AoI,
// per-UE throughput, and PRB utilization (estimated by bytes share).
// codex resume 019c7023-f3e2-7e00-acea-5d1b1b73577d

#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
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
#include "ns3/nr-mac-scheduler-ofdma-rr.h"
#include "ns3/nr-mac-scheduler-ns3.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/seq-ts-size-header.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AoiPrbUrbanOnOff");

enum TrafficType : uint8_t
{
    BURST = 0,
    BACKGROUND = 1,
    TRAFFIC_TYPES = 2
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
};

static std::vector<UeStats> g_ueStats;
static std::vector<PrbStats> g_prbStats;

static Ptr<BwpManagerGnb> g_gnbBwpMgr;
static std::unordered_map<uint16_t, Ptr<BwpManagerUe>> g_ueMgrByRnti;
static uint8_t g_lowBwpId = 0;
static uint8_t g_highBwpId = 1;
static uint8_t g_initialBwpId = g_highBwpId;
static uint8_t g_mcsLowThreshold = 8;
static uint8_t g_mcsHighThreshold = 12;
static uint32_t g_mcsSwitchCount = 5;
static double g_minSwitchIntervalMs = 0.0;
static bool g_enableMcsSwitch = false;

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

// RL/OpenGym state
static bool g_enableOpenGym = true;
static bool g_enableRlBwpControl = true;
static bool g_enableRlMcsControl = true;
static uint32_t g_openGymPort = 5555;
static double g_envStepTime = 0.02; // 20 ms
static double g_simTime = 2.0;
static double g_appStartTime = 0.3;
static double g_switchDelayMsCfg = 0.1;
static double g_queueNormBytes = 200000.0;
static double g_rewardLambdaSwitch = 0.05;
static double g_rewardLambdaQueue = 0.20;
static double g_rewardLambdaDelay = 0.10;
static uint8_t g_rlInitialMcs = 10;

static std::vector<std::array<uint64_t, TRAFFIC_TYPES>> g_txBytesByType;
static std::vector<std::array<double, TRAFFIC_TYPES>> g_lastDeliveredAoiMs;
static std::vector<std::array<double, TRAFFIC_TYPES>> g_lastRxTimeByType;
static std::vector<double> g_lastCqiByUe;
static std::vector<double> g_lastSinrDbByUe;
static std::vector<double> g_lastMcsByUe;
static std::vector<uint8_t> g_currentBwpByUe;
static std::vector<double> g_switchCooldownUntilSByUe;
static std::vector<uint64_t> g_stepAssignedPrbByUe;
static std::vector<uint64_t> g_stepTbCountByUe;
static std::vector<uint8_t> g_targetMcsByUe;

static std::vector<Ptr<NrMacSchedulerNs3>> g_dlSchedulers;

static uint32_t g_lastIntervalSwitchCount = 0;
static uint32_t g_nextIntervalSwitchCount = 0;
static double g_lastMeanAoiMs = 0.0;
static double g_lastQueueOverflowRatio = 0.0;
static double g_lastMeanPrbUtility = 0.0;

static void
SwitchUeBwp(uint16_t rnti, uint8_t targetBwp)
{
    if (!g_gnbBwpMgr)
    {
        return;
    }
    if (g_gnbBwpMgr->IsSwitching(rnti))
    {
        return;
    }
    auto ueIt = g_ueMgrByRnti.find(rnti);
    if (ueIt == g_ueMgrByRnti.end())
    {
        return;
    }
    g_gnbBwpMgr->ForceUeBwp(rnti, targetBwp);
    ueIt->second->ForceActiveBwp(targetBwp);
    g_mcsStateByRnti[rnti].lastSwitchTime = Simulator::Now().GetSeconds();
    auto idxIt = g_ueIdxByRnti.find(rnti);
    if (idxIt != g_ueIdxByRnti.end() && idxIt->second < g_currentBwpByUe.size())
    {
        uint32_t ueIdx = idxIt->second;
        g_currentBwpByUe[ueIdx] = targetBwp;
        g_switchCooldownUntilSByUe[ueIdx] =
            Simulator::Now().GetSeconds() + (g_switchDelayMsCfg / 1000.0);
    }
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
        g_lastMcsByUe[idxIt->second] = static_cast<double>(info.m_mcs);
    }
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
    if (g_enableRlMcsControl && ueIdx < g_targetMcsByUe.size())
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
RxWithSeqTsSize(uint32_t ueIdx,
                uint8_t type,
                Ptr<const Packet> p,
                const Address&,
                const Address&,
                const SeqTsSizeHeader& header)
{
    if (ueIdx >= g_ueStats.size() || type >= TRAFFIC_TYPES)
    {
        return;
    }
    auto& st = g_ueStats[ueIdx].flow[type];
    st.rxBytes += p->GetSize();
    st.rxPkts++;
    double aoiMs = (Simulator::Now() - header.GetTs()).GetMilliSeconds();
    st.aoiSumMs += aoiMs;
    st.aoiSamples++;
    st.lastRxTime = Simulator::Now().GetSeconds();
    g_lastDeliveredAoiMs[ueIdx][type] = aoiMs;
    g_lastRxTimeByType[ueIdx][type] = st.lastRxTime;
}

static void
TxWithSeqTsSize(uint32_t ueIdx,
                uint8_t type,
                Ptr<const Packet> p,
                const Address&,
                const Address&,
                const SeqTsSizeHeader&)
{
    if (ueIdx >= g_txBytesByType.size() || type >= TRAFFIC_TYPES)
    {
        return;
    }
    g_txBytesByType[ueIdx][type] += p->GetSize();
}

static void
OnRxPacketTraceUe(uint32_t ueIdx, RxPacketTraceParams params)
{
    if (ueIdx >= g_prbStats.size())
    {
        return;
    }
    g_prbStats[ueIdx].prbTotal += params.m_rbAssignedNum;
    g_prbStats[ueIdx].tbBytes += params.m_tbSize;
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
    }
    if (ueIdx < g_stepAssignedPrbByUe.size())
    {
        g_stepAssignedPrbByUe[ueIdx] += params.m_rbAssignedNum;
        g_stepTbCountByUe[ueIdx]++;
        g_lastCqiByUe[ueIdx] = static_cast<double>(params.m_cqi);
        if (params.m_sinr > 0.0)
        {
            g_lastSinrDbByUe[ueIdx] = 10.0 * std::log10(params.m_sinr);
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
    return std::max(0.0, std::min(1.0, v));
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
ComputeQueueProxyBytes(uint32_t ueIdx)
{
    if (ueIdx >= g_txBytesByType.size() || ueIdx >= g_ueStats.size())
    {
        return 0.0;
    }
    uint64_t tx = g_txBytesByType[ueIdx][BURST] + g_txBytesByType[ueIdx][BACKGROUND];
    uint64_t rx = g_ueStats[ueIdx].flow[BURST].rxBytes + g_ueStats[ueIdx].flow[BACKGROUND].rxBytes;
    if (tx <= rx)
    {
        return 0.0;
    }
    return static_cast<double>(tx - rx);
}

static double
ComputeCurrentAoiMs(uint32_t ueIdx, uint8_t type)
{
    if (ueIdx >= g_lastDeliveredAoiMs.size() || type >= TRAFFIC_TYPES)
    {
        return 0.0;
    }
    double nowS = Simulator::Now().GetSeconds();
    double lastRxS = g_lastRxTimeByType[ueIdx][type];
    if (lastRxS <= 0.0)
    {
        return std::max(0.0, (nowS - g_appStartTime) * 1000.0);
    }
    double deliveredAoiMs = g_lastDeliveredAoiMs[ueIdx][type];
    return std::max(0.0, deliveredAoiMs + (nowS - lastRxS) * 1000.0);
}

static double
ComputeUeMeanAoiMs(uint32_t ueIdx)
{
    double aoiBurst = ComputeCurrentAoiMs(ueIdx, BURST);
    double aoiBg = ComputeCurrentAoiMs(ueIdx, BACKGROUND);
    return 0.5 * (aoiBurst + aoiBg);
}

static double
ComputeUePrbUtility(uint32_t ueIdx)
{
    if (ueIdx >= g_stepTbCountByUe.size() || g_stepTbCountByUe[ueIdx] == 0)
    {
        return 0.0;
    }
    uint8_t bwp = GetCurrentUeBwp(ueIdx);
    uint32_t totalPrb = 1;
    if (bwp < g_totalPrbByBwp.size())
    {
        totalPrb = std::max<uint32_t>(1u, g_totalPrbByBwp[bwp]);
    }
    double avgAssigned =
        static_cast<double>(g_stepAssignedPrbByUe[ueIdx]) / static_cast<double>(g_stepTbCountByUe[ueIdx]);
    return Clamp01(avgAssigned / static_cast<double>(totalPrb));
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

#ifdef HAVE_OPENGYM
static Ptr<OpenGymSpace>
MyGetObservationSpace()
{
    const uint32_t featuresPerUe = 7;
    uint32_t obsDim = featuresPerUe * static_cast<uint32_t>(g_ueStats.size());
    std::vector<uint32_t> shape = {obsDim};
    return CreateObject<OpenGymBoxSpace>(0.0f, 1.0f, shape, TypeNameGet<float>());
}

static Ptr<OpenGymSpace>
MyGetActionSpace()
{
    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    if (numUes <= 1)
    {
        return CreateObject<OpenGymDiscreteSpace>(6);
    }
    std::vector<uint32_t> shape = {numUes};
    return CreateObject<OpenGymBoxSpace>(0.0f, 5.0f, shape, TypeNameGet<uint32_t>());
}

static bool
MyGetGameOver()
{
    return Simulator::Now().GetSeconds() >= g_simTime;
}

static Ptr<OpenGymDataContainer>
MyGetObservation()
{
    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    std::vector<uint32_t> shape = {7u * numUes};
    Ptr<OpenGymBoxContainer<float>> box = CreateObject<OpenGymBoxContainer<float>>(shape);
    double nowS = Simulator::Now().GetSeconds();

    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        double queueProxy = ComputeQueueProxyBytes(ueIdx);
        double queueNorm = Clamp01(queueProxy / std::max(1.0, g_queueNormBytes));
        double cqiNorm = Clamp01(g_lastCqiByUe[ueIdx] / 15.0);
        double sinrNorm = Clamp01((g_lastSinrDbByUe[ueIdx] + 10.0) / 40.0);
        double mcsNorm = Clamp01(g_lastMcsByUe[ueIdx] / 27.0);
        double bwpMode = static_cast<double>(GetCurrentUeBwp(ueIdx) == g_highBwpId ? 1.0 : 0.0);
        double prbUtility = ComputeUePrbUtility(ueIdx);
        double cooldownRemainMs =
            std::max(0.0, (g_switchCooldownUntilSByUe[ueIdx] - nowS) * 1000.0);
        double cooldownNorm = Clamp01(cooldownRemainMs / std::max(1e-6, g_switchDelayMsCfg));

        box->AddValue(static_cast<float>(queueNorm));
        box->AddValue(static_cast<float>(cqiNorm));
        box->AddValue(static_cast<float>(sinrNorm));
        box->AddValue(static_cast<float>(mcsNorm));
        box->AddValue(static_cast<float>(bwpMode));
        box->AddValue(static_cast<float>(prbUtility));
        box->AddValue(static_cast<float>(cooldownNorm));
    }
    return box;
}

static float
MyGetReward()
{
    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    if (numUes == 0)
    {
        return 0.0f;
    }

    double meanAoiMs = 0.0;
    double queueOverflowCnt = 0.0;
    double meanPrbUtility = 0.0;

    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        meanAoiMs += ComputeUeMeanAoiMs(ueIdx);
        if (ComputeQueueProxyBytes(ueIdx) > g_queueNormBytes)
        {
            queueOverflowCnt += 1.0;
        }
        meanPrbUtility += ComputeUePrbUtility(ueIdx);
    }
    meanAoiMs /= numUes;
    meanPrbUtility /= numUes;
    double queueOverflowRatio = queueOverflowCnt / numUes;

    double delayRatio = g_switchDelayMsCfg / std::max(1e-6, g_envStepTime * 1000.0);
    double reward = -meanAoiMs;
    reward -= g_rewardLambdaSwitch * static_cast<double>(g_lastIntervalSwitchCount);
    reward -= g_rewardLambdaQueue * queueOverflowRatio;
    reward -= g_rewardLambdaDelay * delayRatio * static_cast<double>(g_lastIntervalSwitchCount);

    g_lastMeanAoiMs = meanAoiMs;
    g_lastQueueOverflowRatio = queueOverflowRatio;
    g_lastMeanPrbUtility = meanPrbUtility;
    return static_cast<float>(reward);
}

static std::string
MyGetExtraInfo()
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "mean_aoi_ms=" << g_lastMeanAoiMs << "|"
        << "queue_overflow_ratio=" << g_lastQueueOverflowRatio << "|"
        << "mean_prb_utility=" << g_lastMeanPrbUtility << "|"
        << "switch_count=" << g_lastIntervalSwitchCount;
    return oss.str();
}

static bool
MyExecuteActions(Ptr<OpenGymDataContainer> action)
{
    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    std::vector<uint32_t> codes(numUes, 1); // hold + delta 0

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
    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        uint32_t code = std::min<uint32_t>(5u, codes[ueIdx]);
        uint8_t bwpCmd = static_cast<uint8_t>(code / 3u);
        uint8_t deltaIdx = static_cast<uint8_t>(code % 3u);
        int8_t delta = (deltaIdx == 0) ? -1 : (deltaIdx == 2 ? +1 : 0);

        if (g_enableRlMcsControl && ueIdx < g_targetMcsByUe.size())
        {
            int target = static_cast<int>(g_targetMcsByUe[ueIdx]) + delta;
            target = std::max(0, std::min(27, target));
            g_targetMcsByUe[ueIdx] = static_cast<uint8_t>(target);
        }

        if (g_enableRlBwpControl && bwpCmd == 1 && ueIdx < g_rntiByUeIdx.size() && g_rntiByUeIdx[ueIdx] != 0)
        {
            uint16_t rnti = g_rntiByUeIdx[ueIdx];
            uint8_t current = GetCurrentUeBwp(ueIdx);
            uint8_t target = (current == g_lowBwpId) ? g_highBwpId : g_lowBwpId;
            SwitchUeBwp(rnti, target);
            switchCount++;
        }
    }

    if (g_enableRlMcsControl && !g_targetMcsByUe.empty())
    {
        ApplyPerUeMcsOverridesToSchedulers();
    }
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
    openGym->NotifyCurrentState();
    g_lastIntervalSwitchCount = g_nextIntervalSwitchCount;
    g_nextIntervalSwitchCount = 0;
    std::fill(g_stepAssignedPrbByUe.begin(), g_stepAssignedPrbByUe.end(), 0);
    std::fill(g_stepTbCountByUe.begin(), g_stepTbCountByUe.end(), 0);
    Simulator::Schedule(Seconds(g_envStepTime), &ScheduleNextStateRead, openGym);
}
#endif

int
main(int argc, char* argv[])
{
    uint32_t numUes = 10;
    double simTime = g_simTime;
    double appStart = g_appStartTime;
    double lowFreqHz = 700e6;
    double highFreqHz = 3.5e9;
    double lowBandwidthHz = 20e6;
    double highBandwidthHz = 80e6;
    double gnbTxPowerDbm = 20.0;
    double ueDistance = 20.0;
    double ueSpacing = 3.0;
    uint32_t startJitterMs = 50;
    double ueSpeed = 1.0;
    bool enableMobility = true;
    double switchDelayMs = g_switchDelayMsCfg;

    // Bursty traffic
    double burstRateMbps = 100.0;
    uint32_t burstPktSize = 1200;
    double burstOnMs = 100.0;
    double burstOffMs = 150.0;

    // Background traffic
    double backgroundRateKbps = 500.0;
    uint32_t backgroundPktSize = 500;

    // Channel dynamics
    bool enableShadowing = true;
    double channelUpdateMs = 10.0;

    std::string errorModel = "ns3::NrEesmCcT1";

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
    cmd.AddValue("ueSpacing", "UE spacing along x-axis (m)", ueSpacing);
    cmd.AddValue("startJitterMs", "App start jitter (ms)", startJitterMs);
    cmd.AddValue("ueSpeed", "UE speed along x-axis (m/s)", ueSpeed);
    cmd.AddValue("enableMobility", "Enable UE mobility", enableMobility);
    cmd.AddValue("initialBwpId", "Initial BWP id (0=low,1=high)", g_initialBwpId);
    cmd.AddValue("mcsLowThreshold", "Switch to low BWP if MCS <= threshold", g_mcsLowThreshold);
    cmd.AddValue("mcsHighThreshold", "Switch to high BWP if MCS >= threshold", g_mcsHighThreshold);
    cmd.AddValue("mcsSwitchCount", "Consecutive MCS samples required to switch", g_mcsSwitchCount);
    cmd.AddValue("minSwitchIntervalMs", "Minimum time between switches (ms)", g_minSwitchIntervalMs);
    cmd.AddValue("enableMcsSwitch", "Enable MCS-based BWP switching", g_enableMcsSwitch);
    cmd.AddValue("switchDelayMs", "BWP switching delay (ms)", switchDelayMs);
    cmd.AddValue("enableOpenGym", "Enable OpenGym interface (ns3-gym)", g_enableOpenGym);
    cmd.AddValue("openGymPort", "OpenGym TCP port", g_openGymPort);
    cmd.AddValue("envStepTime", "OpenGym env step time (s)", g_envStepTime);
    cmd.AddValue("enableRlBwpControl", "Use RL actions for BWP switching", g_enableRlBwpControl);
    cmd.AddValue("enableRlMcsControl", "Use RL actions for MCS delta control", g_enableRlMcsControl);
    cmd.AddValue("rlInitialMcs", "Initial fixed DL MCS for RL control", g_rlInitialMcs);
    cmd.AddValue("queueNormBytes", "Queue normalization/overflow threshold in bytes", g_queueNormBytes);
    cmd.AddValue("rewardLambdaSwitch", "Reward penalty weight for switch count", g_rewardLambdaSwitch);
    cmd.AddValue("rewardLambdaQueue", "Reward penalty weight for queue overflow ratio", g_rewardLambdaQueue);
    cmd.AddValue("rewardLambdaDelay", "Reward penalty weight for switch delay burden", g_rewardLambdaDelay);
    cmd.AddValue("burstRateMbps", "Bursty traffic rate (Mbps) during ON", burstRateMbps);
    cmd.AddValue("burstPktSize", "Bursty packet size (bytes)", burstPktSize);
    cmd.AddValue("burstOnMs", "Bursty ON duration (ms)", burstOnMs);
    cmd.AddValue("burstOffMs", "Bursty OFF duration (ms)", burstOffMs);
    cmd.AddValue("backgroundRateKbps", "Background traffic rate (Kbps)", backgroundRateKbps);
    cmd.AddValue("backgroundPktSize", "Background packet size (bytes)", backgroundPktSize);
    cmd.AddValue("enableShadowing", "Enable shadowing in pathloss", enableShadowing);
    cmd.AddValue("channelUpdateMs", "3GPP channel model update period (ms)", channelUpdateMs);
    cmd.Parse(argc, argv);

    g_simTime = simTime;
    g_appStartTime = appStart;
    g_switchDelayMsCfg = switchDelayMs;

    if (g_enableOpenGym && g_enableMcsSwitch)
    {
        NS_LOG_UNCOND("OpenGym enabled: disabling threshold-based MCS switching to avoid policy conflicts.");
        g_enableMcsSwitch = false;
    }
    g_ueStats.assign(numUes, UeStats{});
    g_prbStats.assign(numUes, PrbStats{});
    g_rntiByUeIdx.assign(numUes, 0);
    g_txBytesByType.assign(numUes, {0, 0});
    g_lastDeliveredAoiMs.assign(numUes, {0.0, 0.0});
    g_lastRxTimeByType.assign(numUes, {0.0, 0.0});
    g_lastCqiByUe.assign(numUes, 0.0);
    g_lastSinrDbByUe.assign(numUes, 0.0);
    g_lastMcsByUe.assign(numUes, static_cast<double>(g_rlInitialMcs));
    g_currentBwpByUe.assign(numUes, g_initialBwpId);
    g_switchCooldownUntilSByUe.assign(numUes, 0.0);
    g_stepAssignedPrbByUe.assign(numUes, 0);
    g_stepTbCountByUe.assign(numUes, 0);
    g_targetMcsByUe.assign(numUes, g_rlInitialMcs);
    g_ueIdxByRnti.clear();
    g_dlSchedulers.clear();
    g_totalPrbByBwp.clear();
    g_lastIntervalSwitchCount = 0;
    g_nextIntervalSwitchCount = 0;

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    gnbPos->Add(Vector(0.0, 0.0, 10.0));
    gnbMobility.SetPositionAllocator(gnbPos);
    gnbMobility.Install(gnbNodes);

    MobilityHelper ueMobility;
    if (enableMobility)
    {
        ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    }
    else
    {
        ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    }
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numUes; ++i)
    {
        double x = ueDistance + i * ueSpacing;
        double y = -5.0 + (i % 5) * 2.5;
        uePos->Add(Vector(x, y, 1.5));
    }
    ueMobility.SetPositionAllocator(uePos);
    ueMobility.Install(ueNodes);
    if (enableMobility)
    {
        for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
        {
            Ptr<ConstantVelocityMobilityModel> cv =
                ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
            if (cv)
            {
                cv->SetVelocity(Vector(ueSpeed, 0.0, 0.0));
            }
        }
    }

    // Place a building block between gNB and UEs (UMi + Buildings condition).
    Ptr<Building> building = CreateObject<Building>();
    building->SetBoundaries(Box(30.0, 60.0, -20.0, 20.0, 0.0, 20.0));
    BuildingsHelper::Install(gnbNodes);
    BuildingsHelper::Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Buildings", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(enableShadowing));
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(channelUpdateMs)));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf lowConf(lowFreqHz, lowBandwidthHz, 1);
    CcBwpCreator::SimpleOperationBandConf highConf(highFreqHz, highBandwidthHz, 1);
    OperationBandInfo bandLow = ccBwpCreator.CreateOperationBandContiguousCc(lowConf);
    OperationBandInfo bandHigh = ccBwpCreator.CreateOperationBandContiguousCc(highConf);
    channelHelper->AssignChannelsToBands({bandLow, bandHigh});
    auto allBwps = CcBwpCreator::GetAllBwps({bandLow, bandHigh});

    nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaRR::GetTypeId());
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(g_enableRlMcsControl));
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(false));
    if (g_enableRlMcsControl)
    {
        nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(g_rlInitialMcs));
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
            g_gnbBwpMgr->SetAttributeSwitchingDelay(MilliSeconds(switchDelayMs));
        }
        for (uint8_t bwpId = 0; bwpId < gnbNetDev->GetCcMapSize(); ++bwpId)
        {
            Ptr<NrGnbMac> gnbMac = gnbNetDev->GetMac(bwpId);
            if (gnbMac)
            {
                gnbMac->TraceConnectWithoutContext("DlScheduling",
                                                   MakeCallback(&DlSchedulingTrace));
            }
            Ptr<NrMacSchedulerNs3> sched =
                DynamicCast<NrMacSchedulerNs3>(gnbNetDev->GetScheduler(bwpId));
            g_dlSchedulers.push_back(sched);
            if (g_enableRlMcsControl && sched)
            {
                sched->SetAttribute("FixedMcsDl", BooleanValue(true));
                sched->SetAttribute("StartingMcsDl", UintegerValue(g_rlInitialMcs));
            }
            Ptr<NrGnbPhy> gnbPhy = gnbNetDev->GetPhy(bwpId);
            uint32_t totalPrb = gnbPhy ? gnbPhy->GetRbNum() : 1u;
            g_totalPrbByBwp.push_back(totalPrb);
        }
    }

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

    // Application setup: bursty + background per UE
    ApplicationContainer serverApps;
    ApplicationContainer burstApps;
    ApplicationContainer backgroundApps;

    Ptr<UniformRandomVariable> startJitter = CreateObject<UniformRandomVariable>();

    for (uint32_t i = 0; i < numUes; ++i)
    {
        uint16_t burstPort = 5000 + i;
        uint16_t bgPort = 6000 + i;

        PacketSinkHelper burstSinkHelper("ns3::UdpSocketFactory",
                                         InetSocketAddress(Ipv4Address::GetAny(), burstPort));
        burstSinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer burstSinkApp = burstSinkHelper.Install(ueNodes.Get(i));
        serverApps.Add(burstSinkApp);

        PacketSinkHelper bgSinkHelper("ns3::UdpSocketFactory",
                                      InetSocketAddress(Ipv4Address::GetAny(), bgPort));
        bgSinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer bgSinkApp = bgSinkHelper.Install(ueNodes.Get(i));
        serverApps.Add(bgSinkApp);

        Ptr<NrEpcTft> burstTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter burstPf;
        burstPf.localPortStart = burstPort;
        burstPf.localPortEnd = burstPort;
        burstTft->Add(burstPf);
        NrEpsBearer burstBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
        nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(i), burstBearer, burstTft);

        Ptr<NrEpcTft> bgTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter bgPf;
        bgPf.localPortStart = bgPort;
        bgPf.localPortEnd = bgPort;
        bgTft->Add(bgPf);
        NrEpsBearer bgBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
        nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(i), bgBearer, bgTft);

        OnOffHelper burst("ns3::UdpSocketFactory",
                          InetSocketAddress(ueIfaces.GetAddress(i), burstPort));
        burst.SetAttribute("DataRate", DataRateValue(DataRate(burstRateMbps * 1e6)));
        burst.SetAttribute("PacketSize", UintegerValue(burstPktSize));
        burst.SetAttribute("OnTime",
                           StringValue("ns3::ConstantRandomVariable[Constant=" +
                                       std::to_string(burstOnMs / 1000.0) + "]"));
        burst.SetAttribute("OffTime",
                           StringValue("ns3::ConstantRandomVariable[Constant=" +
                                       std::to_string(burstOffMs / 1000.0) + "]"));
        burst.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer burstApp = burst.Install(remoteHost);
        burstApps.Add(burstApp);

        OnOffHelper bg("ns3::UdpSocketFactory",
                       InetSocketAddress(ueIfaces.GetAddress(i), bgPort));
        bg.SetAttribute("DataRate", DataRateValue(DataRate(backgroundRateKbps * 1000.0)));
        bg.SetAttribute("PacketSize", UintegerValue(backgroundPktSize));
        bg.SetAttribute("OnTime",
                        StringValue("ns3::ConstantRandomVariable[Constant=" +
                                    std::to_string(simTime) + "]"));
        bg.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        bg.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer bgApp = bg.Install(remoteHost);
        backgroundApps.Add(bgApp);

        Ptr<Application> burstSrc = burstApp.Get(0);
        Ptr<Application> bgSrc = bgApp.Get(0);
        if (burstSrc)
        {
            burstSrc->TraceConnectWithoutContext("TxWithSeqTsSize",
                                                 MakeBoundCallback(&TxWithSeqTsSize, i, BURST));
        }
        if (bgSrc)
        {
            bgSrc->TraceConnectWithoutContext("TxWithSeqTsSize",
                                              MakeBoundCallback(&TxWithSeqTsSize, i, BACKGROUND));
        }

        Ptr<PacketSink> burstSink = DynamicCast<PacketSink>(burstSinkApp.Get(0));
        Ptr<PacketSink> bgSink = DynamicCast<PacketSink>(bgSinkApp.Get(0));
        if (burstSink)
        {
            burstSink->TraceConnectWithoutContext("RxWithSeqTsSize",
                                                  MakeBoundCallback(&RxWithSeqTsSize, i, BURST));
        }
        if (bgSink)
        {
            bgSink->TraceConnectWithoutContext("RxWithSeqTsSize",
                                               MakeBoundCallback(&RxWithSeqTsSize, i, BACKGROUND));
        }

        double jitter = startJitter->GetInteger(0, startJitterMs) / 1000.0;
        Time startTime = Seconds(appStart + jitter);
        Time stopTime = Seconds(simTime);
        burstApp.Start(startTime);
        burstApp.Stop(stopTime);
        bgApp.Start(startTime);
        bgApp.Stop(stopTime);

        burstSinkApp.Start(Seconds(appStart * 0.5));
        burstSinkApp.Stop(stopTime);
        bgSinkApp.Start(Seconds(appStart * 0.5));
        bgSinkApp.Stop(stopTime);
    }

#ifdef HAVE_OPENGYM
    Ptr<OpenGymInterface> openGym;
    if (g_enableOpenGym)
    {
        openGym = CreateObject<OpenGymInterface>(g_openGymPort);
        openGym->SetGetActionSpaceCb(MakeCallback(&MyGetActionSpace));
        openGym->SetGetObservationSpaceCb(MakeCallback(&MyGetObservationSpace));
        openGym->SetGetGameOverCb(MakeCallback(&MyGetGameOver));
        openGym->SetGetObservationCb(MakeCallback(&MyGetObservation));
        openGym->SetGetRewardCb(MakeCallback(&MyGetReward));
        openGym->SetGetExtraInfoCb(MakeCallback(&MyGetExtraInfo));
        openGym->SetExecuteActionsCb(MakeCallback(&MyExecuteActions));
        Simulator::Schedule(Seconds(g_envStepTime), &ScheduleNextStateRead, openGym);
    }
#else
    if (g_enableOpenGym)
    {
        NS_ABORT_MSG("OpenGym requested but ns3/opengym-module.h is unavailable in this build.");
    }
#endif

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

#ifdef HAVE_OPENGYM
    if (openGym)
    {
        openGym->NotifySimulationEnd();
    }
#endif

    double duration = simTime - appStart;
    if (duration <= 0.0)
    {
        duration = simTime;
    }

    uint64_t totalPrbAll = 0;
    uint64_t totalBytesAll = 0;
    uint64_t totalBurstBytesAll = 0;
    uint64_t totalBgBytesAll = 0;
    uint64_t totalTbCountAll = 0;
    uint64_t totalTbErrorAll = 0;
    double totalMcsSumAll = 0.0;
    uint64_t totalMcsCountAll = 0;
    double totalTblerSumAll = 0.0;
    uint64_t totalTblerCountAll = 0;

    NS_LOG_UNCOND("\n=== Per-UE AoI/Throughput/PRB Summary ===");
    for (uint32_t i = 0; i < numUes; ++i)
    {
        const auto& burst = g_ueStats[i].flow[BURST];
        const auto& bg = g_ueStats[i].flow[BACKGROUND];
        uint64_t totalBytes = burst.rxBytes + bg.rxBytes;
        double thrMbps = (totalBytes * 8.0) / duration / 1e6;
        double avgAoiBurstMs = (burst.aoiSamples > 0) ? (burst.aoiSumMs / burst.aoiSamples) : 0.0;
        double avgAoiBgMs = (bg.aoiSamples > 0) ? (bg.aoiSumMs / bg.aoiSamples) : 0.0;

        const auto& prb = g_prbStats[i];
        double bytesPerPrb =
            (prb.prbTotal > 0) ? (static_cast<double>(totalBytes) / prb.prbTotal) : 0.0;
        double burstPrbEst = 0.0;
        double bgPrbEst = 0.0;
        if (totalBytes > 0 && prb.prbTotal > 0)
        {
            burstPrbEst = prb.prbTotal * (static_cast<double>(burst.rxBytes) / totalBytes);
            bgPrbEst = prb.prbTotal * (static_cast<double>(bg.rxBytes) / totalBytes);
        }
        double avgMcs = (prb.mcsCount > 0) ? (prb.mcsSum / prb.mcsCount) : 0.0;
        double bler = (prb.tbCount > 0) ? (static_cast<double>(prb.tbError) / prb.tbCount) : 0.0;
        double avgTbler =
            (prb.tblerCount > 0) ? (prb.tblerSum / prb.tblerCount) : 0.0;

        NS_LOG_UNCOND(std::fixed << std::setprecision(3)
                                 << "UE" << i
                                 << " thr=" << thrMbps << " Mbps"
                                 << " AoI(burst)=" << avgAoiBurstMs << " ms"
                                 << " AoI(bg)=" << avgAoiBgMs << " ms"
                                 << " avgMcs=" << avgMcs
                                 << " bler=" << bler
                                 << " avgTbler=" << avgTbler
                                 << " PRB(total)=" << prb.prbTotal
                                 << " bytes/PRB=" << bytesPerPrb
                                 << " estPRB(burst)=" << burstPrbEst
                                 << " estPRB(bg)=" << bgPrbEst);

        totalPrbAll += prb.prbTotal;
        totalBytesAll += totalBytes;
        totalBurstBytesAll += burst.rxBytes;
        totalBgBytesAll += bg.rxBytes;
        totalTbCountAll += prb.tbCount;
        totalTbErrorAll += prb.tbError;
        totalMcsSumAll += prb.mcsSum;
        totalMcsCountAll += prb.mcsCount;
        totalTblerSumAll += prb.tblerSum;
        totalTblerCountAll += prb.tblerCount;
    }

    double bytesPerPrbAll =
        (totalPrbAll > 0) ? (static_cast<double>(totalBytesAll) / totalPrbAll) : 0.0;
    double burstPrbEstAll = 0.0;
    double bgPrbEstAll = 0.0;
    double avgMcsAll = (totalMcsCountAll > 0) ? (totalMcsSumAll / totalMcsCountAll) : 0.0;
    double blerAll =
        (totalTbCountAll > 0) ? (static_cast<double>(totalTbErrorAll) / totalTbCountAll) : 0.0;
    double avgTblerAll =
        (totalTblerCountAll > 0) ? (totalTblerSumAll / totalTblerCountAll) : 0.0;
    if (totalBytesAll > 0 && totalPrbAll > 0)
    {
        burstPrbEstAll = totalPrbAll * (static_cast<double>(totalBurstBytesAll) / totalBytesAll);
        bgPrbEstAll = totalPrbAll * (static_cast<double>(totalBgBytesAll) / totalBytesAll);
    }

    NS_LOG_UNCOND("\n=== Aggregate PRB (bytes-share estimate) ===");
    NS_LOG_UNCOND(std::fixed << std::setprecision(3)
                             << "avgMcs=" << avgMcsAll
                             << " bler=" << blerAll
                             << " avgTbler=" << avgTblerAll
                             << " PRB(total)=" << totalPrbAll
                             << " bytes/PRB=" << bytesPerPrbAll
                             << " estPRB(burst)=" << burstPrbEstAll
                             << " estPRB(bg)=" << bgPrbEstAll);

    Simulator::Destroy();
    return 0;
}
