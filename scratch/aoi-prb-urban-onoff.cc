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
#include "ns3/nr-mac-scheduler-ofdma-aequitas.h"
#include "ns3/nr-mac-scheduler-ofdma-pf.h"
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
#include <cctype>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
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

struct PerUeTrafficProfile
{
    // Background (VoIP-like superposition)
    double backgroundRateKbps{1800.0};
    uint32_t backgroundPktSize{160};
    // Bursty classes: HTTP / real-time video / real-time gaming
    double httpRateMbps{6.0};
    uint32_t httpPktSize{1000};
    double httpOnMinMs{40.0};
    double httpOnMaxMs{120.0};
    double httpOffMinMs{300.0};
    double httpOffMaxMs{800.0};
    double videoRateMbps{4.0};
    uint32_t videoPktSize{1100};
    double videoOnMinMs{120.0};
    double videoOnMaxMs{300.0};
    double videoOffMinMs{120.0};
    double videoOffMaxMs{260.0};
    double gamingRateMbps{1.5};
    uint32_t gamingPktSize{180};
    double gamingOnMinMs{20.0};
    double gamingOnMaxMs{60.0};
    double gamingOffMinMs{40.0};
    double gamingOffMaxMs{120.0};
    std::string trafficClass{"legacy"};
};

struct PendingTxChunk
{
    double txTimeS{0.0};
    uint64_t bytes{0};
};

static std::vector<UeStats> g_ueStats;
static std::vector<PrbStats> g_prbStats;

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

// RL/OpenGym state
static bool g_enableOpenGym = true;
static bool g_enableRlBwpControl = true;
static bool g_enableRlMcsControl = true;
static bool g_rlDebug = false;
static uint32_t g_openGymPort = 5555;
static double g_envStepTime = 0.02; // 20 ms
static double g_simTime = 2.0;
static double g_appStartTime = 0.3;
static double g_switchDelayMsCfg = 10.0;
static double g_queueNormBytes = 200000.0;
static double g_rewardLambdaSwitch = 0.0001;
static double g_rewardLambdaQueue = 0.20;
static double g_rewardLambdaDelay = 0.10;
static uint8_t g_rlInitialMcs = 10;
static double g_prbDemandC0 = 5.0;
static double g_prbDemandC1 = 2.0;

static std::vector<std::array<uint64_t, TRAFFIC_TYPES>> g_txBytesByType;
static std::vector<std::array<double, TRAFFIC_TYPES>> g_lastDeliveredAoiMs;
static std::vector<std::array<double, TRAFFIC_TYPES>> g_lastRxTimeByType;
static std::vector<double> g_lastCqiByUe;
static std::vector<double> g_lastSinrDbByUe;
static std::vector<double> g_lastMcsByUe;
static std::vector<double> g_lastRequestedMcsOffsetByUe;
static std::vector<double> g_lastBwpSwitchTimeSByUe;
static std::vector<double> g_rlcDlQueueBytesByUe;
static std::vector<double> g_prevRewardQueueBytesByUe;
static std::vector<uint8_t> g_currentBwpByUe;
static std::vector<double> g_switchCooldownUntilSByUe;
static std::vector<uint64_t> g_stepAssignedPrbByUe;
static std::vector<uint64_t> g_stepTbBytesByUe;
static std::vector<uint64_t> g_stepTbCountByUe;
static std::vector<uint64_t> g_stepTbErrorByUe;
static std::vector<uint64_t> g_lastStepAssignedPrbByUe;
static std::vector<double> g_lastStepSpectralEfficiencyByUe;
static std::vector<double> g_lastBwpStepSpectralEfficiency;
static std::vector<double> g_lastBwpAvgMcs;
static std::vector<uint8_t> g_targetMcsByUe;

static std::vector<Ptr<NrMacSchedulerNs3>> g_dlSchedulers;
static std::vector<Ptr<NrMacSchedulerOfdmaAequitas>> g_aequitasSchedulers;

static uint32_t g_lastIntervalSwitchCount = 0;
static uint32_t g_nextIntervalSwitchCount = 0;
static double g_lastMeanAoiMs = 0.0;
static double g_lastQueueOverflowRatio = 0.0;
static double g_lastMeanPrbUtility = 0.0;
static double g_lastMeanRewardAoiTerm = 0.0;
static double g_lastMeanRewardThrTerm = 0.0;
static double g_lastMeanRewardPdrTerm = 0.0;
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
static std::vector<double> g_lastRewardPrevQueueBytesByUe;
static std::vector<double> g_lastRewardPrevAoiMsByUe;
static std::vector<uint64_t> g_lastRewardArrivedBytesByUe;
static std::vector<uint64_t> g_lastRewardDeliveredBytesByUe;
static std::vector<uint32_t> g_lastActualSwitchByUe;
static std::vector<uint32_t> g_nextActualSwitchByUe;
static std::vector<std::deque<PendingTxChunk>> g_pendingTxChunksByUe;
static std::vector<double> g_holAgeMsByUe;
static double g_lastPendingTxReconcileTimeS = -1.0;

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
static std::string g_burstStateTraceFile = "";
static std::unique_ptr<std::ofstream> g_burstStateTraceStream;

static std::string g_schedulerPolicy = "rr"; // rr|pf|aequitas

// Baseline policy configuration
static std::string g_bwpBaseline = "none"; // none|dt|dqn|aequitas
static double g_aequitasDeadlineMs = 100.0;
static bool g_aequitasEnableMcsSelection = true;
static uint32_t g_policySwitchCooldownSteps = 2;
static std::vector<uint32_t> g_policyCooldownStepsByUe;

// DT-like predictive baseline parameters
static double g_dtDelayTargetMs = 80.0;
static double g_dtQueueHigh = 0.70;
static double g_dtQueueLow = 0.20;
static double g_dtPredictGainAoi = 0.8;
static double g_dtPredictGainQueue = 0.8;
static double g_dtEmaAlpha = 0.25;
static std::vector<double> g_dtEmaAoiMsByUe;
static std::vector<double> g_dtEmaQueueNormByUe;

// DQN-like baseline parameters
static uint32_t g_dqnHistoryLen = 4;
static uint32_t g_dqnHiddenSize = 32;
static uint32_t g_dqnReplayCapacity = 6000;
static uint32_t g_dqnBatchSize = 64;
static uint32_t g_dqnWarmup = 300;
static uint32_t g_dqnTargetSync = 100;
static double g_dqnLr = 1e-3;
static double g_dqnGamma = 0.98;
static double g_dqnEpsStart = 1.0;
static double g_dqnEpsEnd = 0.05;
static double g_dqnEpsDecay = 0.999;
static double g_dqnDelayTargetMs = 80.0;
static double g_dqnThrTargetMbps = 2.0;
static double g_dqnLatencyUeRatio = 0.5;
static double g_dqnAlpha = 0.7; // delay weight for latency-sensitive UEs
static double g_dqnBeta = 0.3;  // throughput weight for latency-sensitive UEs
static bool g_rlDrqnProfile = false;

static std::vector<std::deque<std::array<double, 7>>> g_dqnFeatureHistoryByUe;
static std::vector<std::vector<double>> g_dqnPrevStateByUe;
static std::vector<int> g_dqnPrevActionByUe;
static std::vector<bool> g_dqnHasPrevByUe;

struct DqnTransition
{
    std::vector<double> s;
    int a{0};
    double r{0.0};
    std::vector<double> sNext;
};

class SimpleDqnAgent
{
  public:
    SimpleDqnAgent(uint32_t stateDim,
                   uint32_t hiddenDim,
                   double lr,
                   double gamma,
                   uint32_t replayCapacity,
                   uint32_t batchSize,
                   uint32_t warmup,
                   uint32_t targetSync,
                   double epsStart,
                   double epsEnd,
                   double epsDecay)
        : m_stateDim(stateDim),
          m_hiddenDim(hiddenDim),
          m_lr(lr),
          m_gamma(gamma),
          m_replayCapacity(replayCapacity),
          m_batchSize(batchSize),
          m_warmup(warmup),
          m_targetSync(targetSync),
          m_eps(epsStart),
          m_epsEnd(epsEnd),
          m_epsDecay(epsDecay)
    {
        std::normal_distribution<double> nd(0.0, 0.05);
        m_w1.assign(m_hiddenDim * m_stateDim, 0.0);
        m_b1.assign(m_hiddenDim, 0.0);
        m_w2.assign(2 * m_hiddenDim, 0.0);
        m_b2.assign(2, 0.0);
        for (auto& w : m_w1)
        {
            w = nd(m_rng);
        }
        for (auto& w : m_w2)
        {
            w = nd(m_rng);
        }
        SyncTarget();
    }

    int SelectAction(const std::vector<double>& state)
    {
        std::uniform_real_distribution<double> ur(0.0, 1.0);
        if (ur(m_rng) < m_eps)
        {
            std::uniform_int_distribution<int> ui(0, 1);
            return ui(m_rng);
        }
        auto q = Forward(state, m_w1, m_b1, m_w2, m_b2);
        return (q[1] > q[0]) ? 1 : 0;
    }

    void Observe(const DqnTransition& tr)
    {
        if (tr.s.empty() || tr.sNext.empty())
        {
            return;
        }
        if (m_replay.size() >= m_replayCapacity)
        {
            m_replay.pop_front();
        }
        m_replay.push_back(tr);
        m_step++;
        if (m_eps > m_epsEnd)
        {
            m_eps = std::max(m_epsEnd, m_eps * m_epsDecay);
        }
        if (m_replay.size() >= m_warmup)
        {
            TrainOneBatch();
            if (m_targetSync > 0 && m_step % m_targetSync == 0)
            {
                SyncTarget();
            }
        }
    }

  private:
    std::array<double, 2> Forward(const std::vector<double>& s,
                                  const std::vector<double>& w1,
                                  const std::vector<double>& b1,
                                  const std::vector<double>& w2,
                                  const std::vector<double>& b2) const
    {
        std::vector<double> h(m_hiddenDim, 0.0);
        for (uint32_t i = 0; i < m_hiddenDim; ++i)
        {
            double z = b1[i];
            for (uint32_t j = 0; j < m_stateDim; ++j)
            {
                z += w1[i * m_stateDim + j] * s[j];
            }
            h[i] = (z > 0.0) ? z : 0.0;
        }
        std::array<double, 2> q = {b2[0], b2[1]};
        for (uint32_t a = 0; a < 2; ++a)
        {
            for (uint32_t i = 0; i < m_hiddenDim; ++i)
            {
                q[a] += w2[a * m_hiddenDim + i] * h[i];
            }
        }
        return q;
    }

    void TrainOneBatch()
    {
        std::uniform_int_distribution<uint32_t> ui(0, static_cast<uint32_t>(m_replay.size() - 1));
        uint32_t bs = std::min<uint32_t>(m_batchSize, static_cast<uint32_t>(m_replay.size()));
        for (uint32_t bi = 0; bi < bs; ++bi)
        {
            const auto& tr = m_replay[ui(m_rng)];
            std::vector<double> h(m_hiddenDim, 0.0);
            std::vector<double> z(m_hiddenDim, 0.0);
            for (uint32_t i = 0; i < m_hiddenDim; ++i)
            {
                z[i] = m_b1[i];
                for (uint32_t j = 0; j < m_stateDim; ++j)
                {
                    z[i] += m_w1[i * m_stateDim + j] * tr.s[j];
                }
                h[i] = (z[i] > 0.0) ? z[i] : 0.0;
            }

            std::array<double, 2> q = {m_b2[0], m_b2[1]};
            for (uint32_t a = 0; a < 2; ++a)
            {
                for (uint32_t i = 0; i < m_hiddenDim; ++i)
                {
                    q[a] += m_w2[a * m_hiddenDim + i] * h[i];
                }
            }

            auto qNext = Forward(tr.sNext, m_w1Target, m_b1Target, m_w2Target, m_b2Target);
            double target = tr.r + m_gamma * std::max(qNext[0], qNext[1]);
            double err = q[tr.a] - target;
            double gradQ = 2.0 * err;

            for (uint32_t i = 0; i < m_hiddenDim; ++i)
            {
                m_w2[tr.a * m_hiddenDim + i] -= m_lr * gradQ * h[i];
            }
            m_b2[tr.a] -= m_lr * gradQ;

            std::vector<double> gradH(m_hiddenDim, 0.0);
            for (uint32_t i = 0; i < m_hiddenDim; ++i)
            {
                gradH[i] = gradQ * m_w2[tr.a * m_hiddenDim + i];
                if (z[i] <= 0.0)
                {
                    gradH[i] = 0.0;
                }
            }
            for (uint32_t i = 0; i < m_hiddenDim; ++i)
            {
                for (uint32_t j = 0; j < m_stateDim; ++j)
                {
                    m_w1[i * m_stateDim + j] -= m_lr * gradH[i] * tr.s[j];
                }
                m_b1[i] -= m_lr * gradH[i];
            }
        }
    }

    void SyncTarget()
    {
        m_w1Target = m_w1;
        m_b1Target = m_b1;
        m_w2Target = m_w2;
        m_b2Target = m_b2;
    }

  private:
    uint32_t m_stateDim{0};
    uint32_t m_hiddenDim{0};
    double m_lr{1e-3};
    double m_gamma{0.98};
    uint32_t m_replayCapacity{5000};
    uint32_t m_batchSize{64};
    uint32_t m_warmup{200};
    uint32_t m_targetSync{100};
    uint64_t m_step{0};
    double m_eps{1.0};
    double m_epsEnd{0.05};
    double m_epsDecay{0.999};
    std::mt19937 m_rng{7};

    std::vector<double> m_w1;
    std::vector<double> m_b1;
    std::vector<double> m_w2;
    std::vector<double> m_b2;

    std::vector<double> m_w1Target;
    std::vector<double> m_b1Target;
    std::vector<double> m_w2Target;
    std::vector<double> m_b2Target;

    std::deque<DqnTransition> m_replay;
};

static std::unique_ptr<SimpleDqnAgent> g_dqnAgent;

static bool
SwitchUeBwp(uint16_t rnti, uint8_t targetBwp)
{
    if (!g_gnbBwpMgr)
    {
        g_nextSwitchRejectMgrMissingCount++;
        return false;
    }
    if (g_gnbBwpMgr->IsSwitching(rnti))
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

    if (g_aoiTraceStream && g_aoiTraceStream->is_open() &&
        ueIdx == static_cast<uint32_t>(std::max<int32_t>(0, g_aoiTraceUe)))
    {
        (*g_aoiTraceStream) << std::fixed << std::setprecision(6) << Simulator::Now().GetSeconds()
                            << "," << ueIdx << "," << static_cast<uint32_t>(type) << ","
                            << p->GetSize() << "," << aoiMs << "\n";
    }
}

static void
TxWithSeqTsSize(uint32_t ueIdx,
                uint8_t type,
                Ptr<const Packet> p,
                const Address&,
                const Address&,
                const SeqTsSizeHeader& header)
{
    if (ueIdx >= g_txBytesByType.size() || type >= TRAFFIC_TYPES)
    {
        return;
    }
    g_txBytesByType[ueIdx][type] += p->GetSize();
    if (ueIdx < g_pendingTxChunksByUe.size())
    {
        g_pendingTxChunksByUe[ueIdx].push_back({header.GetTs().GetSeconds(), p->GetSize()});
    }
}

static void
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
        g_stepTbBytesByUe[ueIdx] += params.m_tbSize;
        g_stepTbCountByUe[ueIdx]++;
        if (params.m_corrupt)
        {
            g_stepTbErrorByUe[ueIdx]++;
        }
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
    if (!std::isfinite(v))
    {
        return 0.0;
    }
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
ComputeDlRlcQueueBytes(uint32_t ueIdx)
{
    if (ueIdx >= g_rlcDlQueueBytesByUe.size())
    {
        return 0.0;
    }
    return std::max(0.0, g_rlcDlQueueBytesByUe[ueIdx]);
}

static void
DlBufferReportTrace(uint16_t rnti, uint8_t lcid, uint32_t bytes, uint16_t bwpId)
{
    (void)lcid;
    (void)bwpId;

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

    // Use the scheduler-observed DL RLC queue size directly as the queue backlog proxy.
    g_rlcDlQueueBytesByUe[ueIdx] = static_cast<double>(bytes);
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
    double mcs = (ueIdx < g_lastMcsByUe.size()) ? g_lastMcsByUe[ueIdx] : static_cast<double>(g_rlInitialMcs);
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
    uint64_t rxBytes = g_ueStats[ueIdx].flow[BURST].rxBytes + g_ueStats[ueIdx].flow[BACKGROUND].rxBytes;
    double durationS = std::max(1e-6, Simulator::Now().GetSeconds() - g_appStartTime);
    return static_cast<double>(rxBytes) * 8.0 / durationS / 1e6;
}

static uint64_t
ComputeUeTxBytes(uint32_t ueIdx)
{
    if (ueIdx >= g_txBytesByType.size())
    {
        return 0u;
    }
    return g_txBytesByType[ueIdx][BURST] + g_txBytesByType[ueIdx][BACKGROUND];
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

static uint64_t
ComputeUeStepArrivedBytes(uint32_t ueIdx)
{
    uint64_t currentTxBytes = ComputeUeTxBytes(ueIdx);
    uint64_t prevTxBytes = (ueIdx < g_prevRewardTxBytesByUe.size()) ? g_prevRewardTxBytesByUe[ueIdx] : 0u;
    return (currentTxBytes >= prevTxBytes) ? (currentTxBytes - prevTxBytes) : 0u;
}

static void
ReconcilePendingTxHolAgeCaches()
{
    double nowS = Simulator::Now().GetSeconds();
    if (std::abs(nowS - g_lastPendingTxReconcileTimeS) < 1e-9)
    {
        return;
    }
    g_lastPendingTxReconcileTimeS = nowS;

    for (uint32_t ueIdx = 0; ueIdx < g_pendingTxChunksByUe.size(); ++ueIdx)
    {
        uint64_t deliveredBytes = (ueIdx < g_stepTbBytesByUe.size()) ? g_stepTbBytesByUe[ueIdx] : 0u;
        auto& q = g_pendingTxChunksByUe[ueIdx];
        while (deliveredBytes > 0 && !q.empty())
        {
            if (q.front().bytes <= deliveredBytes)
            {
                deliveredBytes -= q.front().bytes;
                q.pop_front();
            }
            else
            {
                q.front().bytes -= deliveredBytes;
                deliveredBytes = 0u;
            }
        }
        if (ueIdx < g_holAgeMsByUe.size())
        {
            g_holAgeMsByUe[ueIdx] = q.empty() ? 0.0 : std::max(0.0, (nowS - q.front().txTimeS) * 1000.0);
        }
    }
}

static void
RefreshStepSpectralEfficiencyCaches()
{
    ReconcilePendingTxHolAgeCaches();
    for (uint32_t ueIdx = 0; ueIdx < g_lastStepSpectralEfficiencyByUe.size(); ++ueIdx)
    {
        g_lastStepSpectralEfficiencyByUe[ueIdx] = ComputeUeStepSpectralEfficiency(ueIdx);
    }

    std::fill(g_lastBwpStepSpectralEfficiency.begin(), g_lastBwpStepSpectralEfficiency.end(), 0.0);
    std::vector<double> bwpAssignedPrb(g_totalPrbByBwp.size(), 0.0);
    std::vector<double> bwpDeliveredBits(g_totalPrbByBwp.size(), 0.0);
    std::vector<double> bwpMcsSum(g_totalPrbByBwp.size(), 0.0);
    std::vector<uint32_t> bwpMcsCount(g_totalPrbByBwp.size(), 0u);

    for (uint32_t ueIdx = 0; ueIdx < g_stepAssignedPrbByUe.size() && ueIdx < g_lastStepSpectralEfficiencyByUe.size();
         ++ueIdx)
    {
        uint8_t bwp = GetCurrentUeBwp(ueIdx);
        if (bwp >= g_totalPrbByBwp.size())
        {
            continue;
        }
        bwpAssignedPrb[bwp] += static_cast<double>(g_stepAssignedPrbByUe[ueIdx]);
        bwpDeliveredBits[bwp] +=
            static_cast<double>((ueIdx < g_stepTbBytesByUe.size()) ? g_stepTbBytesByUe[ueIdx] : 0u) * 8.0;
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
}

static double
LogScaleNonNegative(double v)
{
    return std::log1p(std::max(0.0, v));
}

static double
SignedLogScale(double v)
{
    if (v > 0.0)
    {
        return std::log1p(v);
    }
    if (v < 0.0)
    {
        return -std::log1p(-v);
    }
    return 0.0;
}

static double
SymmetricUnitNorm(double v)
{
    return 2.0 * Clamp01(v) - 1.0;
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
        meanAoiMs += ComputeUeMeanAoiMs(ueIdx);
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

static uint8_t
EstimateBaselineDlMcsForUe(uint32_t ueIdx, uint8_t bwpTarget)
{
    if (ueIdx >= g_rntiByUeIdx.size())
    {
        return g_rlInitialMcs;
    }
    uint16_t rnti = g_rntiByUeIdx[ueIdx];
    if (rnti == 0)
    {
        return g_rlInitialMcs;
    }
    if (bwpTarget >= g_dlSchedulers.size() || !g_dlSchedulers[bwpTarget])
    {
        return (ueIdx < g_lastMcsByUe.size()) ? static_cast<uint8_t>(std::lround(g_lastMcsByUe[ueIdx]))
                                              : g_rlInitialMcs;
    }
    return g_dlSchedulers[bwpTarget]->EstimateDlMcsFromCurrentCqiForRnti(rnti);
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
        if (g_rlDebug)
        {
            NS_LOG_UNCOND("[rl-debug] switch reject ue=" << ueIdx << " reason=index_oob");
        }
        return false;
    }
    if (!CanSwitchByCooldown(ueIdx))
    {
        g_nextSwitchRejectCooldownCount++;
        if (g_rlDebug)
        {
            NS_LOG_UNCOND("[rl-debug] switch reject ue="
                          << ueIdx << " reason=cooldown current=" << unsigned(GetCurrentUeBwp(ueIdx))
                          << " target=" << unsigned(targetBwp)
                          << " cooldown=" << g_policyCooldownStepsByUe[ueIdx]);
        }
        return false;
    }
    uint16_t rnti = g_rntiByUeIdx[ueIdx];
    if (rnti == 0)
    {
        g_nextSwitchRejectNoRntiCount++;
        if (g_rlDebug)
        {
            NS_LOG_UNCOND("[rl-debug] switch reject ue=" << ueIdx << " reason=no_rnti");
        }
        return false;
    }
    uint8_t current = GetCurrentUeBwp(ueIdx);
    if (current == targetBwp)
    {
        g_nextSwitchRejectSameTargetCount++;
        if (g_rlDebug)
        {
            NS_LOG_UNCOND("[rl-debug] switch reject ue="
                          << ueIdx << " reason=same_target current=" << unsigned(current)
                          << " target=" << unsigned(targetBwp));
        }
        return false;
    }
    if (!SwitchUeBwp(rnti, targetBwp))
    {
        if (g_rlDebug)
        {
            NS_LOG_UNCOND("[rl-debug] switch reject ue="
                          << ueIdx << " reason=switch_apply_failed current=" << unsigned(current)
                          << " target=" << unsigned(targetBwp) << " rnti=" << rnti);
        }
        return false;
    }
    if (g_rlDebug)
    {
        NS_LOG_UNCOND("[rl-debug] switch success ue="
                      << ueIdx << " current=" << unsigned(current) << " target="
                      << unsigned(targetBwp) << " rnti=" << rnti);
    }
    g_policyCooldownStepsByUe[ueIdx] = g_policySwitchCooldownSteps;
    return true;
}

static std::array<double, 7>
BuildDqnFeature(uint32_t ueIdx)
{
    double queueNorm = Clamp01(ComputeDlRlcQueueBytes(ueIdx) / std::max(1.0, g_queueNormBytes));
    double aoiNorm = Clamp01(ComputeUeMeanAoiMs(ueIdx) / std::max(1.0, g_dqnDelayTargetMs));
    double thrNorm = Clamp01(ComputeUeThroughputMbps(ueIdx) / std::max(1e-6, g_dqnThrTargetMbps));
    double prbUtility = Clamp01(ComputeUePrbUtility(ueIdx));
    double cqiNorm = 0.0;
    double sinrNorm = 0.0;
    if (ueIdx < g_lastCqiByUe.size())
    {
        cqiNorm = Clamp01(g_lastCqiByUe[ueIdx] / 15.0);
    }
    if (ueIdx < g_lastSinrDbByUe.size())
    {
        sinrNorm = Clamp01((g_lastSinrDbByUe[ueIdx] + 10.0) / 40.0);
    }
    double bwpNorm = Clamp01(GetCurrentUeBwp(ueIdx) == g_highBwpId ? 1.0 : 0.0);
    return {queueNorm, aoiNorm, thrNorm, prbUtility, cqiNorm, sinrNorm, bwpNorm};
}

static double
ComputeUeStepBler(uint32_t ueIdx)
{
    if (ueIdx >= g_stepTbCountByUe.size() || ueIdx >= g_stepTbErrorByUe.size())
    {
        return 0.0;
    }
    uint64_t tbCount = g_stepTbCountByUe[ueIdx];
    if (tbCount == 0)
    {
        return 0.0;
    }
    return static_cast<double>(g_stepTbErrorByUe[ueIdx]) / static_cast<double>(tbCount);
}

static std::array<double, 4>
ComputeDrqnLiteBwpMetrics()
{
    std::array<double, 4> metrics = {0.0, 0.0, 0.0, 0.0}; // active0, bler0, active1, bler1
    std::array<double, 2> active = {0.0, 0.0};
    std::array<double, 2> blerSum = {0.0, 0.0};
    std::array<double, 2> blerCount = {0.0, 0.0};
    double numUes = std::max<uint32_t>(1u, static_cast<uint32_t>(g_ueStats.size()));
    for (uint32_t ueIdx = 0; ueIdx < g_ueStats.size(); ++ueIdx)
    {
        uint8_t bwp = GetCurrentUeBwp(ueIdx);
        if (bwp > 1)
        {
            continue;
        }
        active[bwp] += 1.0;
        double tbCount = (ueIdx < g_stepTbCountByUe.size()) ? static_cast<double>(g_stepTbCountByUe[ueIdx]) : 0.0;
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
BuildDrqnLiteObservation(uint32_t ueIdx, const std::array<double, 4>& bwpMetrics)
{
    double blerNorm = Clamp01(ComputeUeStepBler(ueIdx));
    double dropRate = 0.0;
    double arrivedBytes = static_cast<double>(ComputeUeStepArrivedBytes(ueIdx));
    double deliveredBytes = (ueIdx < g_stepTbBytesByUe.size()) ? static_cast<double>(g_stepTbBytesByUe[ueIdx]) : 0.0;
    if (arrivedBytes > 0.0)
    {
        dropRate = Clamp01(std::max(0.0, arrivedBytes - deliveredBytes) / arrivedBytes);
    }
    double queueNorm = Clamp01(ComputeDlRlcQueueBytes(ueIdx) / std::max(1.0, g_queueNormBytes));
    double channelNorm = 0.0;
    if (ueIdx < g_lastCqiByUe.size())
    {
        channelNorm = Clamp01(g_lastCqiByUe[ueIdx] / 15.0);
    }
    double delayNorm = Clamp01(ComputeUeMeanAoiMs(ueIdx) / std::max(1.0, g_dqnDelayTargetMs));
    double txSizeNorm = Clamp01(deliveredBytes / std::max(1.0, g_queueNormBytes));
    double incomingNorm = Clamp01(arrivedBytes / std::max(1.0, g_queueNormBytes));
    uint8_t currentBwp = GetCurrentUeBwp(ueIdx);
    double bwp0 = (currentBwp == g_lowBwpId) ? 1.0 : 0.0;
    double bwp1 = (currentBwp == g_highBwpId) ? 1.0 : 0.0;
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
            bwpMetrics[3]};
}

static std::vector<double>
BuildDqnStateFromHistory(uint32_t ueIdx)
{
    const uint32_t featDim = 7;
    std::vector<double> state(g_dqnHistoryLen * featDim, 0.0);
    if (ueIdx >= g_dqnFeatureHistoryByUe.size())
    {
        return state;
    }
    auto& hist = g_dqnFeatureHistoryByUe[ueIdx];
    while (hist.size() > g_dqnHistoryLen)
    {
        hist.pop_front();
    }
    uint32_t offset = static_cast<uint32_t>(g_dqnHistoryLen - hist.size());
    for (uint32_t t = 0; t < hist.size(); ++t)
    {
        for (uint32_t k = 0; k < featDim; ++k)
        {
            state[(offset + t) * featDim + k] = hist[t][k];
        }
    }
    return state;
}

static double
ComputeDrqnLiteReward(uint32_t ueIdx)
{
    double delayNorm = Clamp01(ComputeUeMeanAoiMs(ueIdx) / std::max(1.0, g_dqnDelayTargetMs));
    double thrNorm = Clamp01(ComputeUeThroughputMbps(ueIdx) / std::max(1e-6, g_dqnThrTargetMbps));

    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    uint32_t latencyUes = static_cast<uint32_t>(std::round(g_dqnLatencyUeRatio * numUes));
    bool isLatencySensitive = (ueIdx < latencyUes);
    double lambda = isLatencySensitive ? g_dqnAlpha : (1.0 - g_dqnAlpha);
    double mu = isLatencySensitive ? g_dqnBeta : (1.0 - g_dqnBeta);
    return -(lambda * delayNorm + mu * (1.0 - thrNorm));
}

static double
ComputeDqnReward(uint32_t ueIdx)
{
    double delayNorm = Clamp01(ComputeUeMeanAoiMs(ueIdx) / std::max(1.0, g_dqnDelayTargetMs));
    double thrNorm = Clamp01(ComputeUeThroughputMbps(ueIdx) / std::max(1e-6, g_dqnThrTargetMbps));

    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    uint32_t latencyUes = static_cast<uint32_t>(std::round(g_dqnLatencyUeRatio * numUes));
    bool isLatencySensitive = (ueIdx < latencyUes);
    double lambda = isLatencySensitive ? g_dqnAlpha : (1.0 - g_dqnAlpha);
    double mu = isLatencySensitive ? g_dqnBeta : (1.0 - g_dqnBeta);
    return -(lambda * delayNorm + mu * (1.0 - thrNorm));
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
RunDqnPolicyStep()
{
    uint32_t switchCount = 0;
    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        auto feature = BuildDqnFeature(ueIdx);
        g_dqnFeatureHistoryByUe[ueIdx].push_back(feature);
        auto state = BuildDqnStateFromHistory(ueIdx);

        if (g_dqnHasPrevByUe[ueIdx])
        {
            DqnTransition tr;
            tr.s = g_dqnPrevStateByUe[ueIdx];
            tr.a = g_dqnPrevActionByUe[ueIdx];
            tr.r = ComputeDqnReward(ueIdx);
            tr.sNext = state;
            g_dqnAgent->Observe(tr);
        }

        int action = g_dqnAgent->SelectAction(state); // 0=low, 1=high
        uint8_t targetBwp = (action == 1) ? g_highBwpId : g_lowBwpId;
        if (TrySwitchUeToBwp(ueIdx, targetBwp))
        {
            switchCount++;
        }

        g_dqnPrevStateByUe[ueIdx] = std::move(state);
        g_dqnPrevActionByUe[ueIdx] = action;
        g_dqnHasPrevByUe[ueIdx] = true;
    }
    return switchCount;
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
    else if (ShouldDriveAequitasScheduler())
    {
        ApplyAequitasInputsToSchedulers();
    }
    else if (g_bwpBaseline == "dqn" && g_dqnAgent)
    {
        switchCount = RunDqnPolicyStep();
    }
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

#ifdef HAVE_OPENGYM
static Ptr<OpenGymSpace>
MyGetObservationSpace()
{
    const uint32_t featuresPerUe = g_rlDrqnProfile ? 13u : 6u;
    uint32_t obsDim = featuresPerUe * static_cast<uint32_t>(g_ueStats.size());
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
    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
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
    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    RefreshStepSpectralEfficiencyCaches();
    uint32_t featuresPerUe = g_rlDrqnProfile ? 13u : 6u;
    std::vector<uint32_t> shape = {featuresPerUe * numUes};
    Ptr<OpenGymBoxContainer<float>> box = CreateObject<OpenGymBoxContainer<float>>(shape);
    auto bwpMetrics = ComputeDrqnLiteBwpMetrics();

    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        if (g_rlDrqnProfile)
        {
            auto obs = BuildDrqnLiteObservation(ueIdx, bwpMetrics);
            for (double v : obs)
            {
                box->AddValue(static_cast<float>(v));
            }
            continue;
        }
        double queueBytes = ComputeDlRlcQueueBytes(ueIdx);
        double cqi = (ueIdx < g_lastCqiByUe.size()) ? g_lastCqiByUe[ueIdx] : 0.0;
        double mcsOffset =
            (ueIdx < g_lastRequestedMcsOffsetByUe.size()) ? g_lastRequestedMcsOffsetByUe[ueIdx] : 0.0;
        double bwpMode = (GetCurrentUeBwp(ueIdx) == g_highBwpId) ? 1.0 : -1.0;
        double timeSinceLastSwitchMs =
            (ueIdx < g_lastBwpSwitchTimeSByUe.size())
                ? std::max(0.0, (Simulator::Now().GetSeconds() - g_lastBwpSwitchTimeSByUe[ueIdx]) * 1000.0)
                : 0.0;
        double aoiMs = ComputeUeMeanAoiMs(ueIdx);

        box->AddValue(static_cast<float>(bwpMode));
        box->AddValue(static_cast<float>(std::max(-2.0, std::min(2.0, mcsOffset))));
        box->AddValue(static_cast<float>(SymmetricUnitNorm(cqi / 15.0)));
        box->AddValue(static_cast<float>(SymmetricUnitNorm(queueBytes / std::max(1.0, g_queueNormBytes))));
        box->AddValue(static_cast<float>(SymmetricUnitNorm(timeSinceLastSwitchMs / std::max(1.0, g_dqnDelayTargetMs))));
        box->AddValue(static_cast<float>(SymmetricUnitNorm(aoiMs / std::max(1.0, g_dqnDelayTargetMs))));
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

    RefreshStepSpectralEfficiencyCaches();

    if (g_rlDrqnProfile)
    {
        double rewardSum = 0.0;
        for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
        {
            rewardSum += ComputeDrqnLiteReward(ueIdx);
        }
        return static_cast<float>(rewardSum / static_cast<double>(numUes));
    }
    g_lastMeanAoiMs = 0.0;
    double rewardSum = 0.0;
    double aoiPenaltySum = 0.0;
    double goodputTermSum = 0.0;
    double auxTermSum = 0.0;
    double seTermSum = 0.0;
    double switchPenaltySum = 0.0;

    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        double currentAoiMs = ComputeUeMeanAoiMs(ueIdx);
        g_lastMeanAoiMs += currentAoiMs;
        double currentQueueBytes = ComputeDlRlcQueueBytes(ueIdx);
        double prevQueueBytes =
            (ueIdx < g_prevRewardQueueBytesByUe.size() && g_prevRewardQueueBytesByUe[ueIdx] > 0.0)
                ? g_prevRewardQueueBytesByUe[ueIdx]
                : currentQueueBytes;
        uint64_t currentTxBytes = ComputeUeTxBytes(ueIdx);
        uint64_t prevTxBytes = (ueIdx < g_prevRewardTxBytesByUe.size()) ? g_prevRewardTxBytesByUe[ueIdx] : 0u;
        uint64_t arrivedBytesRaw = (currentTxBytes >= prevTxBytes) ? (currentTxBytes - prevTxBytes) : 0u;
        uint64_t deliveredBytesRaw = (ueIdx < g_stepTbBytesByUe.size()) ? g_stepTbBytesByUe[ueIdx] : 0u;
        double aoiPenalty = -std::log1p(std::max(0.0, currentAoiMs));
        double goodputMbps = static_cast<double>(deliveredBytesRaw) * 8.0 / std::max(1e-6, g_envStepTime) / 1.0e6;
        double goodputTerm = std::log1p(std::max(0.0, goodputMbps));
        double auxTerm = -g_rewardLambdaQueue * std::log1p(currentQueueBytes / std::max(prevQueueBytes, 1e-6));
        double seReward = 0.0;
        double actualSwitch =
            (ueIdx < g_lastActualSwitchByUe.size()) ? static_cast<double>(g_lastActualSwitchByUe[ueIdx]) : 0.0;
        double switchPenalty = (actualSwitch > 0.5) ? -g_rewardLambdaSwitch : 0.0;
        double reward = aoiPenalty + goodputTerm + auxTerm + switchPenalty;
        rewardSum += reward;
        aoiPenaltySum += aoiPenalty;
        goodputTermSum += goodputTerm;
        auxTermSum += auxTerm;
        seTermSum += seReward;
        switchPenaltySum += switchPenalty;
        if (ueIdx < g_lastRewardPrevQueueBytesByUe.size())
        {
            g_lastRewardPrevQueueBytesByUe[ueIdx] = prevQueueBytes;
        }
        if (ueIdx < g_lastRewardArrivedBytesByUe.size())
        {
            g_lastRewardArrivedBytesByUe[ueIdx] = arrivedBytesRaw;
        }
        if (ueIdx < g_lastRewardDeliveredBytesByUe.size())
        {
            g_lastRewardDeliveredBytesByUe[ueIdx] = deliveredBytesRaw;
        }
        if (ueIdx < g_prevRewardQueueBytesByUe.size())
        {
            g_prevRewardQueueBytesByUe[ueIdx] = currentQueueBytes;
        }
        if (ueIdx < g_prevRewardTxBytesByUe.size())
        {
            g_prevRewardTxBytesByUe[ueIdx] = currentTxBytes;
        }
    }
    g_lastMeanAoiMs /= numUes;
    g_lastMeanRewardAoiTerm = aoiPenaltySum / static_cast<double>(numUes);
    g_lastMeanRewardThrTerm = goodputTermSum / static_cast<double>(numUes);
    g_lastMeanRewardPdrTerm = auxTermSum / static_cast<double>(numUes);
    g_lastMeanRewardSeTerm = seTermSum / static_cast<double>(numUes);
    g_lastMeanRewardSwitchPenalty = switchPenaltySum / static_cast<double>(numUes);
    g_lastMeanRewardTotal = rewardSum / static_cast<double>(numUes);
    return static_cast<float>(g_lastMeanRewardTotal);
}

static std::string
MyGetExtraInfo()
{
    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    RefreshStepSpectralEfficiencyCaches();
    double meanThrMbps = 0.0;
    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        meanThrMbps += ComputeUeThroughputMbps(ueIdx);
    }
    if (numUes > 0)
    {
        meanThrMbps /= numUes;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "mean_aoi_ms=" << g_lastMeanAoiMs << "|"
        << "queue_overflow_ratio=" << g_lastQueueOverflowRatio << "|"
        << "mean_prb_utility=" << g_lastMeanPrbUtility << "|"
        << "switch_count=" << g_lastIntervalSwitchCount << "|"
        << "requested_bwp0_count=" << g_lastRequestedBwp0Count << "|"
        << "requested_bwp1_count=" << g_lastRequestedBwp1Count << "|"
        << "switch_reject_cooldown=" << g_lastSwitchRejectCooldownCount << "|"
        << "switch_reject_no_rnti=" << g_lastSwitchRejectNoRntiCount << "|"
        << "switch_reject_same_target=" << g_lastSwitchRejectSameTargetCount << "|"
        << "switch_reject_mgr_busy=" << g_lastSwitchRejectMgrBusyCount << "|"
        << "switch_reject_mgr_missing=" << g_lastSwitchRejectMgrMissingCount << "|"
        << "switch_reject_uemgr_missing=" << g_lastSwitchRejectUeMgrMissingCount << "|"
        << "mean_thr_mbps=" << meanThrMbps << "|"
        << "reward_aoi_penalty=" << g_lastMeanRewardAoiTerm << "|"
        << "reward_goodput_term=" << g_lastMeanRewardThrTerm << "|"
        << "reward_aux_term=" << g_lastMeanRewardPdrTerm << "|"
        << "reward_se_term=" << g_lastMeanRewardSeTerm << "|"
        << "reward_switch_penalty=" << g_lastMeanRewardSwitchPenalty << "|"
        << "reward_total=" << g_lastMeanRewardTotal << "|"
        << "bwp0_avg_mcs=" << ((g_lowBwpId < g_lastBwpAvgMcs.size()) ? g_lastBwpAvgMcs[g_lowBwpId] : 0.0) << "|"
        << "bwp1_avg_mcs=" << ((g_highBwpId < g_lastBwpAvgMcs.size()) ? g_lastBwpAvgMcs[g_highBwpId] : 0.0) << "|"
        << "bwp0_se=" << ((g_lowBwpId < g_lastBwpStepSpectralEfficiency.size()) ? g_lastBwpStepSpectralEfficiency[g_lowBwpId] : 0.0) << "|"
        << "bwp1_se=" << ((g_highBwpId < g_lastBwpStepSpectralEfficiency.size()) ? g_lastBwpStepSpectralEfficiency[g_highBwpId] : 0.0) << "|"
        << "bwp0_total_prb=" << ((g_lowBwpId < g_totalPrbByBwp.size()) ? g_totalPrbByBwp[g_lowBwpId] : 0u) << "|"
        << "bwp1_total_prb=" << ((g_highBwpId < g_totalPrbByBwp.size()) ? g_totalPrbByBwp[g_highBwpId] : 0u);

    for (uint32_t ueIdx = 0; ueIdx < numUes; ++ueIdx)
    {
        double stepGoodputMbps =
            ((ueIdx < g_lastRewardDeliveredBytesByUe.size()) ? static_cast<double>(g_lastRewardDeliveredBytesByUe[ueIdx]) : 0.0) *
            8.0 / std::max(1e-6, g_envStepTime) / 1.0e6;
        double aoiBurstMs = ComputeCurrentAoiMs(ueIdx, BURST);
        double aoiBgMs = ComputeCurrentAoiMs(ueIdx, BACKGROUND);
        oss << "|ue" << ueIdx << "_thr_mbps=" << ComputeUeThroughputMbps(ueIdx) << "|ue" << ueIdx
            << "_goodput_mbps=" << stepGoodputMbps << "|ue" << ueIdx
            << "_aoi_ms=" << ComputeUeMeanAoiMs(ueIdx) << "|ue" << ueIdx
            << "_prev_aoi_ms="
            << ((ueIdx < g_lastRewardPrevAoiMsByUe.size()) ? g_lastRewardPrevAoiMsByUe[ueIdx] : 0.0)
            << "|ue" << ueIdx
            << "_aoi_burst_ms=" << aoiBurstMs << "|ue" << ueIdx
            << "_aoi_bg_ms=" << aoiBgMs << "|ue" << ueIdx
            << "_bler=" << ComputeUeStepBler(ueIdx) << "|ue" << ueIdx
            << "_hol_age_ms=" << ((ueIdx < g_holAgeMsByUe.size()) ? g_holAgeMsByUe[ueIdx] : 0.0) << "|ue"
            << ueIdx
            << "_se="
            << ((ueIdx < g_lastStepSpectralEfficiencyByUe.size()) ? g_lastStepSpectralEfficiencyByUe[ueIdx]
                                                                  : 0.0)
            << "|ue" << ueIdx << "_assigned_prb="
            << ((ueIdx < g_lastStepAssignedPrbByUe.size()) ? static_cast<double>(g_lastStepAssignedPrbByUe[ueIdx])
                                                           : 0.0)
            << "|ue" << ueIdx << "_arrived_bytes="
            << ((ueIdx < g_lastRewardArrivedBytesByUe.size()) ? static_cast<double>(g_lastRewardArrivedBytesByUe[ueIdx])
                                                              : 0.0)
            << "|ue" << ueIdx << "_delivered_bytes="
            << ((ueIdx < g_lastRewardDeliveredBytesByUe.size())
                    ? static_cast<double>(g_lastRewardDeliveredBytesByUe[ueIdx])
                    : 0.0)
            << "|ue" << ueIdx << "_queue_bytes=" << ComputeDlRlcQueueBytes(ueIdx)
            << "|ue" << ueIdx << "_prev_queue_bytes="
            << ((ueIdx < g_lastRewardPrevQueueBytesByUe.size()) ? g_lastRewardPrevQueueBytesByUe[ueIdx] : 0.0)
            << "|ue" << ueIdx << "_actual_switch="
            << ((ueIdx < g_lastActualSwitchByUe.size()) ? static_cast<double>(g_lastActualSwitchByUe[ueIdx]) : 0.0);
    }
    return oss.str();
}

static bool
MyExecuteActions(Ptr<OpenGymDataContainer> action)
{
    uint32_t numUes = static_cast<uint32_t>(g_ueStats.size());
    std::vector<uint32_t> codes(numUes, 2); // force BWP 0 + delta 0

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
        uint32_t code = std::min<uint32_t>(g_enableRlMcsControl ? 9u : 1u, codes[ueIdx]);
        uint8_t bwpTarget = 0;
        uint8_t deltaIdx = 2;
        if (g_enableRlMcsControl)
        {
            bwpTarget = static_cast<uint8_t>(code / 5u);
            deltaIdx = static_cast<uint8_t>(code % 5u);
        }
        else
        {
            bwpTarget = static_cast<uint8_t>(code);
        }
        int8_t delta = 0;
        switch (deltaIdx)
        {
        case 0:
            delta = -2;
            break;
        case 1:
            delta = -1;
            break;
        case 3:
            delta = +1;
            break;
        case 4:
            delta = +2;
            break;
        default:
            delta = 0;
            break;
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
            uint8_t targetBwpId = (bwpTarget == 0) ? g_lowBwpId : g_highBwpId;
            int baseMcs = static_cast<int>(EstimateBaselineDlMcsForUe(ueIdx, targetBwpId));
            int target = baseMcs + delta;
            target = std::max(0, std::min(27, target));
            g_targetMcsByUe[ueIdx] = static_cast<uint8_t>(target);
        }

        if (g_rlDebug)
        {
            uint8_t current = (ueIdx < g_currentBwpByUe.size()) ? GetCurrentUeBwp(ueIdx) : 0;
            uint8_t target = (bwpTarget == 0) ? g_lowBwpId : g_highBwpId;
            uint8_t targetBwpId = (bwpTarget == 0) ? g_lowBwpId : g_highBwpId;
            uint8_t baseMcs = EstimateBaselineDlMcsForUe(ueIdx, targetBwpId);
            uint8_t targetMcs = (ueIdx < g_targetMcsByUe.size()) ? g_targetMcsByUe[ueIdx] : 0;
            NS_LOG_UNCOND("[rl-debug] action ue="
                          << ueIdx << " code=" << code << " current_bwp=" << unsigned(current)
                          << " target_bwp=" << unsigned(target) << " base_mcs=" << unsigned(baseMcs)
                          << " mcs_delta=" << int(delta)
                          << " target_mcs=" << unsigned(targetMcs)
                          << " prb_utility=" << ComputeUePrbUtility(ueIdx)
                          << " thr_mbps=" << ComputeUeThroughputMbps(ueIdx)
                          << " aoi_ms=" << ComputeUeMeanAoiMs(ueIdx)
                          << " queue_bytes=" << ComputeDlRlcQueueBytes(ueIdx));
        }

        if (g_enableRlBwpControl && ueIdx < g_rntiByUeIdx.size() && g_rntiByUeIdx[ueIdx] != 0)
        {
            uint8_t current = GetCurrentUeBwp(ueIdx);
            uint8_t target = (bwpTarget == 0) ? g_lowBwpId : g_highBwpId;
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
    if (g_rlDebug)
    {
        NS_LOG_UNCOND("[rl-debug] action-summary requested_bwp0="
                      << requestedBwp0Count << " requested_bwp1=" << requestedBwp1Count
                      << " actual_switches=" << switchCount);
    }
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
    if (g_rlDebug)
    {
        NS_LOG_UNCOND("[rl-debug] interval-summary switches="
                      << g_lastIntervalSwitchCount
                      << " req0=" << g_lastRequestedBwp0Count
                      << " req1=" << g_lastRequestedBwp1Count
                      << " rej_cooldown=" << g_lastSwitchRejectCooldownCount
                      << " rej_same_target=" << g_lastSwitchRejectSameTargetCount
                      << " rej_mgr_busy=" << g_lastSwitchRejectMgrBusyCount
                      << " mean_prb=" << g_lastMeanPrbUtility
                      << " mean_aoi=" << g_lastMeanAoiMs
                      << " reward_aoi_penalty=" << g_lastMeanRewardAoiTerm
                      << " reward_goodput=" << g_lastMeanRewardThrTerm
                      << " reward_se=" << g_lastMeanRewardSeTerm
                      << " reward_switch=" << g_lastMeanRewardSwitchPenalty
                      << " reward=" << g_lastMeanRewardTotal);
    }
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
    double lowFreqHz = 3.5e9;
    double highFreqHz = 6e9;
    double lowBandwidthHz = 20e6;
    double highBandwidthHz = 100e6;
    double gnbTxPowerDbm = 40.0;
    double ueDistance = 20.0;
    double ueSpacing = 3.0;
    uint32_t startJitterMs = 50;
    double ueSpeed = 1.0;
    bool enableMobility = true;
    double switchDelayMs = g_switchDelayMsCfg;

    // Background + burst traffic
    double backgroundRateKbps = 1800.0;
    uint32_t backgroundPktSize = 160;
    double httpRateMbps = 6.0;
    uint32_t httpPktSize = 1000;
    double httpOnMinMs = 40.0;
    double httpOnMaxMs = 120.0;
    double httpOffMinMs = 300.0;
    double httpOffMaxMs = 800.0;
    double videoRateMbps = 4.0;
    uint32_t videoPktSize = 1100;
    double videoOnMinMs = 120.0;
    double videoOnMaxMs = 300.0;
    double videoOffMinMs = 120.0;
    double videoOffMaxMs = 260.0;
    double gamingRateMbps = 1.5;
    uint32_t gamingPktSize = 180;
    double gamingOnMinMs = 20.0;
    double gamingOnMaxMs = 60.0;
    double gamingOffMinMs = 40.0;
    double gamingOffMaxMs = 120.0;
    std::string trafficModel = "legacy"; // legacy|mixed
    double mixedLightRatio = 0.4;
    double mixedModerateRatio = 0.4;
    double mixedHeavyRatio = 0.2;

    // Channel dynamics
    bool enableShadowing = true;
    double channelUpdateMs = 50.0;
    std::string summaryFile = "";
    std::string metricsTraceFile = "";
    std::string aoiTraceFile = "";
    int32_t aoiTraceUe = 0;
    std::string burstStateTraceFile = "";

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
    cmd.AddValue("trafficModel", "Traffic model: legacy|mixed", trafficModel);
    cmd.AddValue("mixedLightRatio", "UE ratio for light mixed-traffic class", mixedLightRatio);
    cmd.AddValue("mixedModerateRatio", "UE ratio for moderate mixed-traffic class", mixedModerateRatio);
    cmd.AddValue("mixedHeavyRatio", "UE ratio for heavy mixed-traffic class", mixedHeavyRatio);
    cmd.AddValue("ueSpeed", "UE speed along x-axis (m/s)", ueSpeed);
    cmd.AddValue("enableMobility", "Enable UE mobility", enableMobility);
    cmd.AddValue("initialBwpId", "Initial BWP id (0=low,1=high)", g_initialBwpId);
    cmd.AddValue("mcsLowThreshold", "Switch to low BWP if MCS <= threshold", g_mcsLowThreshold);
    cmd.AddValue("mcsHighThreshold", "Switch to high BWP if MCS >= threshold", g_mcsHighThreshold);
    cmd.AddValue("mcsSwitchCount", "Consecutive MCS samples required to switch", g_mcsSwitchCount);
    cmd.AddValue("minSwitchIntervalMs", "Minimum time between switches (ms)", g_minSwitchIntervalMs);
    cmd.AddValue("enableMcsSwitch", "Enable MCS-based BWP switching", g_enableMcsSwitch);
    cmd.AddValue("enableHarqReTx", "Enable HARQ retransmissions in the NR scheduler", g_enableHarqReTx);
    cmd.AddValue("switchDelayMs", "BWP switching delay (ms)", switchDelayMs);
    cmd.AddValue("enableOpenGym", "Enable OpenGym interface (ns3-gym)", g_enableOpenGym);
    cmd.AddValue("openGymPort", "OpenGym TCP port", g_openGymPort);
    cmd.AddValue("envStepTime", "OpenGym env step time (s)", g_envStepTime);
    cmd.AddValue("enableRlBwpControl", "Use RL actions for BWP switching", g_enableRlBwpControl);
    cmd.AddValue("enableRlMcsControl", "Use RL actions for MCS delta control", g_enableRlMcsControl);
    cmd.AddValue("rlDebug", "Enable verbose RL action/switch debug logs", g_rlDebug);
    cmd.AddValue("rlInitialMcs", "Initial fixed DL MCS for RL control", g_rlInitialMcs);
    cmd.AddValue("prbDemandC0", "Linear PRB-demand proxy intercept in bytes/PRB", g_prbDemandC0);
    cmd.AddValue("prbDemandC1", "Linear PRB-demand proxy slope in bytes/PRB per MCS", g_prbDemandC1);
    cmd.AddValue("queueNormBytes", "Queue normalization/overflow threshold in bytes", g_queueNormBytes);
    cmd.AddValue("rewardLambdaSwitch", "Reward penalty weight for switch count", g_rewardLambdaSwitch);
    cmd.AddValue("rewardLambdaQueue", "Reward penalty weight for queue overflow ratio", g_rewardLambdaQueue);
    cmd.AddValue("rewardLambdaDelay", "Reward penalty weight for switch delay burden", g_rewardLambdaDelay);
    cmd.AddValue("bwpBaseline",
                 "Built-in baseline policy when OpenGym is disabled: none|dt|dqn|aequitas",
                 g_bwpBaseline);
    cmd.AddValue("schedulerPolicy",
                 "NR MAC scheduler policy: rr|pf|aequitas",
                 g_schedulerPolicy);
    cmd.AddValue("aequitasDeadlineMs",
                 "AoI deadline (ms) supplied to the Aequitas scheduler baseline.",
                 g_aequitasDeadlineMs);
    cmd.AddValue("aequitasEnableMcsSelection",
                 "Enable Aequitas-specific MCS selection; if false, use AoI-aware ordering only.",
                 g_aequitasEnableMcsSelection);
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
    cmd.AddValue("dqnHistoryLen", "DQN baseline history window length", g_dqnHistoryLen);
    cmd.AddValue("dqnHiddenSize", "DQN baseline hidden layer size", g_dqnHiddenSize);
    cmd.AddValue("dqnReplayCapacity", "DQN baseline replay capacity", g_dqnReplayCapacity);
    cmd.AddValue("dqnBatchSize", "DQN baseline minibatch size", g_dqnBatchSize);
    cmd.AddValue("dqnWarmup", "DQN baseline warmup transitions before training", g_dqnWarmup);
    cmd.AddValue("dqnTargetSync", "DQN baseline target sync interval (steps)", g_dqnTargetSync);
    cmd.AddValue("dqnLr", "DQN baseline learning rate", g_dqnLr);
    cmd.AddValue("dqnGamma", "DQN baseline discount factor", g_dqnGamma);
    cmd.AddValue("dqnEpsStart", "DQN baseline epsilon start", g_dqnEpsStart);
    cmd.AddValue("dqnEpsEnd", "DQN baseline epsilon end", g_dqnEpsEnd);
    cmd.AddValue("dqnEpsDecay", "DQN baseline epsilon decay multiplier", g_dqnEpsDecay);
    cmd.AddValue("dqnDelayTargetMs", "DQN baseline delay normalization target (ms)", g_dqnDelayTargetMs);
    cmd.AddValue("dqnThrTargetMbps", "DQN baseline throughput normalization target (Mbps)", g_dqnThrTargetMbps);
    cmd.AddValue("dqnLatencyUeRatio", "Fraction of UEs treated as latency-sensitive [0,1]", g_dqnLatencyUeRatio);
    cmd.AddValue("dqnAlpha", "Delay weight for latency-sensitive UEs", g_dqnAlpha);
    cmd.AddValue("dqnBeta", "Throughput weight for latency-sensitive UEs", g_dqnBeta);
    cmd.AddValue("rlDrqnProfile", "Use DRQN state/action/reward profile for OpenGym RL", g_rlDrqnProfile);
    cmd.AddValue("backgroundRateKbps", "Background traffic rate (Kbps)", backgroundRateKbps);
    cmd.AddValue("backgroundPktSize", "Background packet size (bytes)", backgroundPktSize);
    cmd.AddValue("httpRateMbps", "HTTP-like burst ON rate (Mbps)", httpRateMbps);
    cmd.AddValue("httpPktSize", "HTTP-like burst packet size (bytes)", httpPktSize);
    cmd.AddValue("httpOnMinMs", "HTTP-like burst ON minimum duration (ms)", httpOnMinMs);
    cmd.AddValue("httpOnMaxMs", "HTTP-like burst ON maximum duration (ms)", httpOnMaxMs);
    cmd.AddValue("httpOffMinMs", "HTTP-like burst OFF minimum duration (ms)", httpOffMinMs);
    cmd.AddValue("httpOffMaxMs", "HTTP-like burst OFF maximum duration (ms)", httpOffMaxMs);
    cmd.AddValue("videoRateMbps", "Video-like burst ON rate (Mbps)", videoRateMbps);
    cmd.AddValue("videoPktSize", "Video-like burst packet size (bytes)", videoPktSize);
    cmd.AddValue("videoOnMinMs", "Video-like burst ON minimum duration (ms)", videoOnMinMs);
    cmd.AddValue("videoOnMaxMs", "Video-like burst ON maximum duration (ms)", videoOnMaxMs);
    cmd.AddValue("videoOffMinMs", "Video-like burst OFF minimum duration (ms)", videoOffMinMs);
    cmd.AddValue("videoOffMaxMs", "Video-like burst OFF maximum duration (ms)", videoOffMaxMs);
    cmd.AddValue("gamingRateMbps", "Gaming-like burst ON rate (Mbps)", gamingRateMbps);
    cmd.AddValue("gamingPktSize", "Gaming-like burst packet size (bytes)", gamingPktSize);
    cmd.AddValue("gamingOnMinMs", "Gaming-like burst ON minimum duration (ms)", gamingOnMinMs);
    cmd.AddValue("gamingOnMaxMs", "Gaming-like burst ON maximum duration (ms)", gamingOnMaxMs);
    cmd.AddValue("gamingOffMinMs", "Gaming-like burst OFF minimum duration (ms)", gamingOffMinMs);
    cmd.AddValue("gamingOffMaxMs", "Gaming-like burst OFF maximum duration (ms)", gamingOffMaxMs);
    cmd.AddValue("enableShadowing", "Enable shadowing in pathloss", enableShadowing);
    cmd.AddValue("channelUpdateMs", "3GPP channel model update period (ms)", channelUpdateMs);
    cmd.AddValue("summaryFile", "Append run summary to this file path", summaryFile);
    cmd.AddValue("metricsTraceFile", "Write interval metrics CSV to this file path", metricsTraceFile);
    cmd.AddValue("aoiTraceFile",
                 "Write per-packet AoI trace CSV for the selected UE to this file path.",
                 aoiTraceFile);
    cmd.AddValue("aoiTraceUe", "UE index for per-packet AoI tracing.", aoiTraceUe);
    cmd.AddValue("burstStateTraceFile",
                 "Write burst source-side On/Off state trace CSV for the selected UE to this file path.",
                 burstStateTraceFile);
    cmd.Parse(argc, argv);

    g_simTime = simTime;
    g_appStartTime = appStart;
    g_switchDelayMsCfg = switchDelayMs;
    g_bwpBaseline = ToLower(g_bwpBaseline);
    g_schedulerPolicy = ToLower(g_schedulerPolicy);
    trafficModel = ToLower(trafficModel);
    if (g_bwpBaseline != "none" && g_bwpBaseline != "dt" && g_bwpBaseline != "dqn" &&
        g_bwpBaseline != "aequitas")
    {
        NS_ABORT_MSG("Invalid bwpBaseline. Supported values: none|dt|dqn|aequitas");
    }
    if (g_schedulerPolicy != "rr" && g_schedulerPolicy != "pf" && g_schedulerPolicy != "aequitas")
    {
        NS_ABORT_MSG("Invalid schedulerPolicy. Supported values: rr|pf|aequitas");
    }
    if (trafficModel != "legacy" && trafficModel != "mixed")
    {
        NS_ABORT_MSG("Invalid trafficModel. Supported values: legacy|mixed");
    }
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
    g_dqnLatencyUeRatio = std::max(0.0, std::min(1.0, g_dqnLatencyUeRatio));
    g_dtQueueLow = std::max(0.0, std::min(1.0, g_dtQueueLow));
    g_dtQueueHigh = std::max(0.0, std::min(1.0, g_dtQueueHigh));
    if (g_dtQueueLow > g_dtQueueHigh)
    {
        std::swap(g_dtQueueLow, g_dtQueueHigh);
    }

    if (g_bwpBaseline != "none" && g_enableOpenGym)
    {
        NS_LOG_UNCOND("bwpBaseline is set; forcing enableOpenGym=false for standalone baseline mode.");
        g_enableOpenGym = false;
    }
    if (!g_enableOpenGym)
    {
        g_enableRlBwpControl = false;
        g_enableRlMcsControl = false;
    }

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
    g_lastRequestedMcsOffsetByUe.assign(numUes, 0.0);
    g_lastBwpSwitchTimeSByUe.assign(numUes, 0.0);
    g_rlcDlQueueBytesByUe.assign(numUes, 0.0);
    g_prevRewardQueueBytesByUe.assign(numUes, 0.0);
    g_currentBwpByUe.assign(numUes, g_initialBwpId);
    g_switchCooldownUntilSByUe.assign(numUes, 0.0);
    g_stepAssignedPrbByUe.assign(numUes, 0);
    g_stepTbBytesByUe.assign(numUes, 0);
    g_stepTbCountByUe.assign(numUes, 0);
    g_stepTbErrorByUe.assign(numUes, 0);
    g_lastStepAssignedPrbByUe.assign(numUes, 0);
    g_lastStepSpectralEfficiencyByUe.assign(numUes, 0.0);
    g_lastBwpStepSpectralEfficiency.assign(2, 0.0);
    g_lastBwpAvgMcs.assign(2, 0.0);
    g_prevRewardAoiMsByUe.assign(numUes, 0.0);
    g_prevRewardThrMbpsByUe.assign(numUes, 0.0);
    g_prevRewardQueueBytesByUe.assign(numUes, 0.0);
    g_prevRewardTxBytesByUe.assign(numUes, 0u);
    g_lastRewardPrevQueueBytesByUe.assign(numUes, 0.0);
    g_lastRewardPrevAoiMsByUe.assign(numUes, 0.0);
    g_lastRewardArrivedBytesByUe.assign(numUes, 0u);
    g_lastRewardDeliveredBytesByUe.assign(numUes, 0u);
    g_lastActualSwitchByUe.assign(numUes, 0);
    g_nextActualSwitchByUe.assign(numUes, 0);
    g_pendingTxChunksByUe.assign(numUes, {});
    g_holAgeMsByUe.assign(numUes, 0.0);
    g_lastPendingTxReconcileTimeS = -1.0;
    g_targetMcsByUe.assign(numUes, g_rlInitialMcs);
    g_policyCooldownStepsByUe.assign(numUes, 0);
    g_dtEmaAoiMsByUe.assign(numUes, 0.0);
    g_dtEmaQueueNormByUe.assign(numUes, 0.0);
    g_dqnFeatureHistoryByUe.assign(numUes, {});
    g_dqnPrevStateByUe.assign(numUes, {});
    g_dqnPrevActionByUe.assign(numUes, 0);
    g_dqnHasPrevByUe.assign(numUes, false);
    g_dqnAgent.reset();
    if (!g_enableOpenGym && g_bwpBaseline == "dqn")
    {
        uint32_t stateDim = 7u * g_dqnHistoryLen;
        g_dqnAgent = std::make_unique<SimpleDqnAgent>(stateDim,
                                                      g_dqnHiddenSize,
                                                      g_dqnLr,
                                                      g_dqnGamma,
                                                      g_dqnReplayCapacity,
                                                      g_dqnBatchSize,
                                                      g_dqnWarmup,
                                                      g_dqnTargetSync,
                                                      g_dqnEpsStart,
                                                      g_dqnEpsEnd,
                                                      g_dqnEpsDecay);
    }
    g_ueIdxByRnti.clear();
    g_dlSchedulers.clear();
    g_totalPrbByBwp.clear();
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
    g_metricsTraceFile = metricsTraceFile;
    g_aoiTraceFile = aoiTraceFile;
    g_aoiTraceUe = aoiTraceUe;
    g_burstStateTraceFile = burstStateTraceFile;
    g_metricsTraceStream.reset();
    g_aoiTraceStream.reset();
    g_burstStateTraceStream.reset();
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
        }
        else
        {
            NS_LOG_UNCOND("Failed to open aoiTraceFile: " << g_aoiTraceFile);
            g_aoiTraceStream.reset();
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

    const uint32_t gridCols = std::max<uint32_t>(1, static_cast<uint32_t>(std::ceil(std::sqrt(numUes))));
    const uint32_t gridRows = std::max<uint32_t>(1, static_cast<uint32_t>(std::ceil(static_cast<double>(numUes) /
                                                                                    static_cast<double>(gridCols))));
    const double xSpan = std::max(ueDistance * 2.0, (gridCols - 1) * ueSpacing * 1.5 + 20.0);
    const double ySpan = std::max(ueDistance * 1.5, (gridRows - 1) * ueSpacing * 1.8 + 20.0);
    const double xMin = gnbX - xSpan / 2.0;
    const double yMin = gnbY - ySpan / 2.0;

    MobilityHelper ueMobility;
    if (enableMobility)
    {
        ueMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                    "Bounds",
                                    RectangleValue(Rectangle(xMin, xMin + xSpan, yMin, yMin + ySpan)),
                                    "Speed",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1.0]"),
                                    "Distance",
                                    DoubleValue(std::max(1.0, ueSpeed * 5.0)));
    }
    else
    {
        ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    }
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numUes; ++i)
    {
        const uint32_t row = i / gridCols;
        const uint32_t col = i % gridCols;
        const double xBase = xMin + col * (xSpan / std::max<uint32_t>(1, gridCols - 1));
        const double yBase = yMin + row * (ySpan / std::max<uint32_t>(1, gridRows - 1));
        const double xJitter = ((i % 3) - 1) * 0.35 * ueSpacing;
        const double yJitter = (((i / 3) % 3) - 1) * 0.35 * ueSpacing;
        const double x = std::min(xMin + xSpan - 0.5, std::max(xMin + 0.5, xBase + xJitter));
        const double y = std::min(yMin + ySpan - 0.5, std::max(yMin + 0.5, yBase + yJitter));
        uePos->Add(Vector(x, y, 1.5));
    }
    ueMobility.SetPositionAllocator(uePos);
    ueMobility.Install(ueNodes);

    // Keep only a light set of obstructions so FR2 remains usable under load
    // while still seeing occasional blockage events across the whole UE cloud.
    Ptr<Building> building0 = CreateObject<Building>();
    building0->SetBoundaries(Box(-24.0, -12.0, -10.0, 4.0, 0.0, 16.0));
    building0->SetBuildingType(Building::Residential);
    building0->SetExtWallsType(Building::ConcreteWithWindows);
    building0->SetNFloors(3);
    building0->SetNRoomsX(1);
    building0->SetNRoomsY(1);

    Ptr<Building> building1 = CreateObject<Building>();
    building1->SetBoundaries(Box(10.0, 22.0, 6.0, 18.0, 0.0, 18.0));
    building1->SetBuildingType(Building::Residential);
    building1->SetExtWallsType(Building::ConcreteWithWindows);
    building1->SetNFloors(3);
    building1->SetNRoomsX(1);
    building1->SetNRoomsY(1);

    Ptr<Building> building2 = CreateObject<Building>();
    building2->SetBoundaries(Box(18.0, 32.0, -20.0, -8.0, 0.0, 14.0));
    building2->SetBuildingType(Building::Residential);
    building2->SetExtWallsType(Building::ConcreteWithWindows);
    building2->SetNFloors(2);
    building2->SetNRoomsX(1);
    building2->SetNRoomsY(1);

    Ptr<Building> building3 = CreateObject<Building>();
    building3->SetBoundaries(Box(-8.0, 4.0, 12.0, 24.0, 0.0, 18.0));
    building3->SetBuildingType(Building::Residential);
    building3->SetExtWallsType(Building::ConcreteWithWindows);
    building3->SetNFloors(3);
    building3->SetNRoomsX(1);
    building3->SetNRoomsY(1);

    Ptr<Building> building4 = CreateObject<Building>();
    building4->SetBoundaries(Box(-6.0, 8.0, -26.0, -14.0, 0.0, 16.0));
    building4->SetBuildingType(Building::Residential);
    building4->SetExtWallsType(Building::ConcreteWithWindows);
    building4->SetNFloors(3);
    building4->SetNRoomsX(1);
    building4->SetNRoomsY(1);

    Ptr<Building> building5 = CreateObject<Building>();
    building5->SetBoundaries(Box(28.0, 40.0, 10.0, 22.0, 0.0, 20.0));
    building5->SetBuildingType(Building::Residential);
    building5->SetExtWallsType(Building::ConcreteWithWindows);
    building5->SetNFloors(4);
    building5->SetNRoomsX(1);
    building5->SetNRoomsY(1);

    Ptr<Building> building6 = CreateObject<Building>();
    building6->SetBoundaries(Box(-30.0, -18.0, 8.0, 20.0, 0.0, 14.0));
    building6->SetBuildingType(Building::Residential);
    building6->SetExtWallsType(Building::ConcreteWithWindows);
    building6->SetNFloors(2);
    building6->SetNRoomsX(1);
    building6->SetNRoomsY(1);

    BuildingsHelper::Install(gnbNodes);
    BuildingsHelper::Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    beamformingHelper->SetAttribute("BeamformingPeriodicity", TimeValue(MilliSeconds(50)));
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

    if (g_schedulerPolicy == "aequitas" || g_bwpBaseline == "aequitas")
    {
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaAequitas::GetTypeId());
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
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(false));
    nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(g_rlInitialMcs));

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
            g_aequitasSchedulers.push_back(DynamicCast<NrMacSchedulerOfdmaAequitas>(sched));
            if (sched)
            {
                sched->TraceConnectWithoutContext("DlBufferReport",
                                                  MakeCallback(&DlBufferReportTrace));
            }
            if (g_enableRlMcsControl && sched)
            {
                sched->SetAttribute("FixedMcsDl", BooleanValue(false));
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

    // Application setup: background + (HTTP/video/gaming)-like bursts per UE
    ApplicationContainer serverApps;
    ApplicationContainer httpApps;
    ApplicationContainer videoApps;
    ApplicationContainer gamingApps;
    ApplicationContainer backgroundApps;

    Ptr<UniformRandomVariable> startJitter = CreateObject<UniformRandomVariable>();
    std::vector<PerUeTrafficProfile> ueTrafficProfiles(numUes);

    if (trafficModel == "legacy")
    {
        for (uint32_t i = 0; i < numUes; ++i)
        {
            ueTrafficProfiles[i].backgroundRateKbps = backgroundRateKbps;
            ueTrafficProfiles[i].backgroundPktSize = backgroundPktSize;
            ueTrafficProfiles[i].httpRateMbps = httpRateMbps;
            ueTrafficProfiles[i].httpPktSize = httpPktSize;
            ueTrafficProfiles[i].httpOnMinMs = httpOnMinMs;
            ueTrafficProfiles[i].httpOnMaxMs = httpOnMaxMs;
            ueTrafficProfiles[i].httpOffMinMs = httpOffMinMs;
            ueTrafficProfiles[i].httpOffMaxMs = httpOffMaxMs;
            ueTrafficProfiles[i].videoRateMbps = videoRateMbps;
            ueTrafficProfiles[i].videoPktSize = videoPktSize;
            ueTrafficProfiles[i].videoOnMinMs = videoOnMinMs;
            ueTrafficProfiles[i].videoOnMaxMs = videoOnMaxMs;
            ueTrafficProfiles[i].videoOffMinMs = videoOffMinMs;
            ueTrafficProfiles[i].videoOffMaxMs = videoOffMaxMs;
            ueTrafficProfiles[i].gamingRateMbps = gamingRateMbps;
            ueTrafficProfiles[i].gamingPktSize = gamingPktSize;
            ueTrafficProfiles[i].gamingOnMinMs = gamingOnMinMs;
            ueTrafficProfiles[i].gamingOnMaxMs = gamingOnMaxMs;
            ueTrafficProfiles[i].gamingOffMinMs = gamingOffMinMs;
            ueTrafficProfiles[i].gamingOffMaxMs = gamingOffMaxMs;
            ueTrafficProfiles[i].trafficClass = "legacy";
        }
    }
    else
    {
        std::vector<uint32_t> ueOrder;
        ueOrder.reserve(numUes);
        for (uint32_t i = 0; i < numUes; ++i)
        {
            ueOrder.push_back(i);
        }
        std::mt19937 trafficRng(12345);
        std::shuffle(ueOrder.begin(), ueOrder.end(), trafficRng);

        uint32_t numLight = static_cast<uint32_t>(std::round(numUes * (mixedLightRatio / mixedRatioSum)));
        uint32_t numModerate =
            static_cast<uint32_t>(std::round(numUes * (mixedModerateRatio / mixedRatioSum)));
        numLight = std::min(numLight, numUes);
        numModerate = std::min(numModerate, numUes - numLight);

        for (uint32_t rank = 0; rank < numUes; ++rank)
        {
            uint32_t ueIdx = ueOrder[rank];
            auto& p = ueTrafficProfiles[ueIdx];
            if (rank < numLight)
            {
                p.trafficClass = "light";
                p.backgroundRateKbps = backgroundRateKbps * 0.8;
                p.backgroundPktSize = backgroundPktSize;
                p.httpRateMbps = httpRateMbps * 0.7;
                p.httpPktSize = httpPktSize;
                p.httpOnMinMs = 30.0;
                p.httpOnMaxMs = 90.0;
                p.httpOffMinMs = 420.0;
                p.httpOffMaxMs = 1000.0;
                p.videoRateMbps = videoRateMbps * 0.7;
                p.videoPktSize = videoPktSize;
                p.videoOnMinMs = 90.0;
                p.videoOnMaxMs = 220.0;
                p.videoOffMinMs = 180.0;
                p.videoOffMaxMs = 340.0;
                p.gamingRateMbps = gamingRateMbps * 0.8;
                p.gamingPktSize = gamingPktSize;
                p.gamingOnMinMs = 15.0;
                p.gamingOnMaxMs = 45.0;
                p.gamingOffMinMs = 60.0;
                p.gamingOffMaxMs = 150.0;
            }
            else if (rank < numLight + numModerate)
            {
                p.trafficClass = "moderate";
                p.backgroundRateKbps = backgroundRateKbps;
                p.backgroundPktSize = backgroundPktSize;
                p.httpRateMbps = httpRateMbps;
                p.httpPktSize = httpPktSize;
                p.httpOnMinMs = httpOnMinMs;
                p.httpOnMaxMs = httpOnMaxMs;
                p.httpOffMinMs = httpOffMinMs;
                p.httpOffMaxMs = httpOffMaxMs;
                p.videoRateMbps = videoRateMbps;
                p.videoPktSize = videoPktSize;
                p.videoOnMinMs = videoOnMinMs;
                p.videoOnMaxMs = videoOnMaxMs;
                p.videoOffMinMs = videoOffMinMs;
                p.videoOffMaxMs = videoOffMaxMs;
                p.gamingRateMbps = gamingRateMbps;
                p.gamingPktSize = gamingPktSize;
                p.gamingOnMinMs = gamingOnMinMs;
                p.gamingOnMaxMs = gamingOnMaxMs;
                p.gamingOffMinMs = gamingOffMinMs;
                p.gamingOffMaxMs = gamingOffMaxMs;
            }
            else
            {
                p.trafficClass = "heavy";
                p.backgroundRateKbps = backgroundRateKbps * 1.15;
                p.backgroundPktSize = backgroundPktSize;
                p.httpRateMbps = httpRateMbps * 1.25;
                p.httpPktSize = httpPktSize;
                p.httpOnMinMs = 40.0;
                p.httpOnMaxMs = 130.0;
                p.httpOffMinMs = 220.0;
                p.httpOffMaxMs = 650.0;
                p.videoRateMbps = videoRateMbps * 1.25;
                p.videoPktSize = videoPktSize;
                p.videoOnMinMs = 140.0;
                p.videoOnMaxMs = 330.0;
                p.videoOffMinMs = 90.0;
                p.videoOffMaxMs = 220.0;
                p.gamingRateMbps = gamingRateMbps * 1.2;
                p.gamingPktSize = gamingPktSize;
                p.gamingOnMinMs = 25.0;
                p.gamingOnMaxMs = 70.0;
                p.gamingOffMinMs = 35.0;
                p.gamingOffMaxMs = 100.0;
            }
        }
    }

    for (uint32_t i = 0; i < numUes; ++i)
    {
        const auto& traffic = ueTrafficProfiles[i];
        uint16_t httpPort = 5000 + i;
        uint16_t videoPort = 6000 + i;
        uint16_t gamingPort = 7000 + i;
        uint16_t bgPort = 8000 + i;

        PacketSinkHelper httpSinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), httpPort));
        httpSinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer httpSinkApp = httpSinkHelper.Install(ueNodes.Get(i));
        serverApps.Add(httpSinkApp);
        PacketSinkHelper videoSinkHelper("ns3::UdpSocketFactory",
                                         InetSocketAddress(Ipv4Address::GetAny(), videoPort));
        videoSinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer videoSinkApp = videoSinkHelper.Install(ueNodes.Get(i));
        serverApps.Add(videoSinkApp);
        PacketSinkHelper gamingSinkHelper("ns3::UdpSocketFactory",
                                          InetSocketAddress(Ipv4Address::GetAny(), gamingPort));
        gamingSinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer gamingSinkApp = gamingSinkHelper.Install(ueNodes.Get(i));
        serverApps.Add(gamingSinkApp);

        PacketSinkHelper bgSinkHelper("ns3::UdpSocketFactory",
                                      InetSocketAddress(Ipv4Address::GetAny(), bgPort));
        bgSinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer bgSinkApp = bgSinkHelper.Install(ueNodes.Get(i));
        serverApps.Add(bgSinkApp);

        Ptr<NrEpcTft> httpTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter httpPf;
        httpPf.localPortStart = httpPort;
        httpPf.localPortEnd = httpPort;
        httpTft->Add(httpPf);
        NrEpsBearer httpBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
        nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(i), httpBearer, httpTft);
        Ptr<NrEpcTft> videoTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter videoPf;
        videoPf.localPortStart = videoPort;
        videoPf.localPortEnd = videoPort;
        videoTft->Add(videoPf);
        NrEpsBearer videoBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
        nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(i), videoBearer, videoTft);
        Ptr<NrEpcTft> gamingTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter gamingPf;
        gamingPf.localPortStart = gamingPort;
        gamingPf.localPortEnd = gamingPort;
        gamingTft->Add(gamingPf);
        NrEpsBearer gamingBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
        nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(i), gamingBearer, gamingTft);

        Ptr<NrEpcTft> bgTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter bgPf;
        bgPf.localPortStart = bgPort;
        bgPf.localPortEnd = bgPort;
        bgTft->Add(bgPf);
        NrEpsBearer bgBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
        nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(i), bgBearer, bgTft);

        auto makeBurstApp = [&](uint16_t port,
                                double rateMbps,
                                uint32_t pktSize,
                                double onMinMs,
                                double onMaxMs,
                                double offMinMs,
                                double offMaxMs) {
            OnOffHelper app("ns3::UdpSocketFactory", InetSocketAddress(ueIfaces.GetAddress(i), port));
            app.SetAttribute("DataRate", DataRateValue(DataRate(rateMbps * 1e6)));
            app.SetAttribute("PacketSize", UintegerValue(pktSize));
            app.SetAttribute("OnTime",
                             StringValue("ns3::UniformRandomVariable[Min=" +
                                         std::to_string(std::max(1e-3, onMinMs / 1000.0)) + "|Max=" +
                                         std::to_string(std::max(onMinMs, onMaxMs) / 1000.0) + "]"));
            app.SetAttribute("OffTime",
                             StringValue("ns3::UniformRandomVariable[Min=" +
                                         std::to_string(std::max(1e-3, offMinMs / 1000.0)) + "|Max=" +
                                         std::to_string(std::max(offMinMs, offMaxMs) / 1000.0) + "]"));
            app.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
            return app.Install(remoteHost);
        };
        ApplicationContainer httpApp = makeBurstApp(httpPort,
                                                    traffic.httpRateMbps,
                                                    traffic.httpPktSize,
                                                    traffic.httpOnMinMs,
                                                    traffic.httpOnMaxMs,
                                                    traffic.httpOffMinMs,
                                                    traffic.httpOffMaxMs);
        ApplicationContainer videoApp = makeBurstApp(videoPort,
                                                     traffic.videoRateMbps,
                                                     traffic.videoPktSize,
                                                     traffic.videoOnMinMs,
                                                     traffic.videoOnMaxMs,
                                                     traffic.videoOffMinMs,
                                                     traffic.videoOffMaxMs);
        ApplicationContainer gamingApp = makeBurstApp(gamingPort,
                                                      traffic.gamingRateMbps,
                                                      traffic.gamingPktSize,
                                                      traffic.gamingOnMinMs,
                                                      traffic.gamingOnMaxMs,
                                                      traffic.gamingOffMinMs,
                                                      traffic.gamingOffMaxMs);
        httpApps.Add(httpApp);
        videoApps.Add(videoApp);
        gamingApps.Add(gamingApp);

        OnOffHelper bg("ns3::UdpSocketFactory",
                       InetSocketAddress(ueIfaces.GetAddress(i), bgPort));
        bg.SetAttribute("DataRate", DataRateValue(DataRate(traffic.backgroundRateKbps * 1000.0)));
        bg.SetAttribute("PacketSize", UintegerValue(traffic.backgroundPktSize));
        bg.SetAttribute("OnTime",
                        StringValue("ns3::ConstantRandomVariable[Constant=" +
                                    std::to_string(simTime) + "]"));
        bg.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        bg.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer bgApp = bg.Install(remoteHost);
        backgroundApps.Add(bgApp);

        Ptr<Application> httpSrc = httpApp.Get(0);
        Ptr<Application> videoSrc = videoApp.Get(0);
        Ptr<Application> gamingSrc = gamingApp.Get(0);
        Ptr<Application> bgSrc = bgApp.Get(0);
        if (httpSrc)
        {
            httpSrc->TraceConnectWithoutContext("TxWithSeqTsSize",
                                                MakeBoundCallback(&TxWithSeqTsSize, i, BURST));
        }
        if (videoSrc)
        {
            videoSrc->TraceConnectWithoutContext("TxWithSeqTsSize",
                                                 MakeBoundCallback(&TxWithSeqTsSize, i, BURST));
            videoSrc->TraceConnectWithoutContext("OnOffState",
                                                 MakeBoundCallback(&BurstOnOffStateTrace, i));
        }
        if (gamingSrc)
        {
            gamingSrc->TraceConnectWithoutContext("TxWithSeqTsSize",
                                                  MakeBoundCallback(&TxWithSeqTsSize, i, BURST));
        }
        if (bgSrc)
        {
            bgSrc->TraceConnectWithoutContext("TxWithSeqTsSize",
                                              MakeBoundCallback(&TxWithSeqTsSize, i, BACKGROUND));
        }

        Ptr<PacketSink> httpSink = DynamicCast<PacketSink>(httpSinkApp.Get(0));
        Ptr<PacketSink> videoSink = DynamicCast<PacketSink>(videoSinkApp.Get(0));
        Ptr<PacketSink> gamingSink = DynamicCast<PacketSink>(gamingSinkApp.Get(0));
        Ptr<PacketSink> bgSink = DynamicCast<PacketSink>(bgSinkApp.Get(0));
        if (httpSink)
        {
            httpSink->TraceConnectWithoutContext("RxWithSeqTsSize",
                                                 MakeBoundCallback(&RxWithSeqTsSize, i, BURST));
        }
        if (videoSink)
        {
            videoSink->TraceConnectWithoutContext("RxWithSeqTsSize",
                                                  MakeBoundCallback(&RxWithSeqTsSize, i, BURST));
        }
        if (gamingSink)
        {
            gamingSink->TraceConnectWithoutContext("RxWithSeqTsSize",
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
        httpApp.Start(startTime);
        httpApp.Stop(stopTime);
        videoApp.Start(startTime);
        videoApp.Stop(stopTime);
        gamingApp.Start(startTime);
        gamingApp.Stop(stopTime);
        bgApp.Start(startTime);
        bgApp.Stop(stopTime);

        httpSinkApp.Start(Seconds(appStart * 0.5));
        httpSinkApp.Stop(stopTime);
        videoSinkApp.Start(Seconds(appStart * 0.5));
        videoSinkApp.Stop(stopTime);
        gamingSinkApp.Start(Seconds(appStart * 0.5));
        gamingSinkApp.Stop(stopTime);
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

    if (!g_enableOpenGym)
    {
        if (ShouldDriveAequitasScheduler())
        {
            ApplyAequitasInputsToSchedulers();
        }
        Simulator::Schedule(Seconds(g_envStepTime), &ScheduleBaselinePolicyStep);
    }

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
    double runMeanThrMbps = 0.0;
    double runMeanAoiMs = 0.0;
    double runMeanPrbUtility = 0.0;
    double runQueueOverflowRatio = 0.0;
    double runMeanSwitchCount = 0.0;
    if (g_intervalMetrics.samples > 0)
    {
        double denom = static_cast<double>(g_intervalMetrics.samples);
        runMeanThrMbps = g_intervalMetrics.meanThrMbpsSum / denom;
        runMeanAoiMs = g_intervalMetrics.meanAoiMsSum / denom;
        runMeanPrbUtility = g_intervalMetrics.meanPrbUtilitySum / denom;
        runQueueOverflowRatio = g_intervalMetrics.queueOverflowRatioSum / denom;
        runMeanSwitchCount = g_intervalMetrics.switchCountSum / denom;
    }
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
                             << "runMeanThr=" << runMeanThrMbps << " Mbps "
                             << "runMeanAoI=" << runMeanAoiMs << " ms "
                             << "runMeanPrbUtility=" << runMeanPrbUtility << " "
                             << "runQueueOverflowRatio=" << runQueueOverflowRatio << " "
                             << "runMeanSwitchCount=" << runMeanSwitchCount << " "
                             << "avgMcs=" << avgMcsAll
                             << " bler=" << blerAll
                             << " avgTbler=" << avgTblerAll
                             << " PRB(total)=" << totalPrbAll
                             << " bytes/PRB=" << bytesPerPrbAll
                             << " estPRB(burst)=" << burstPrbEstAll
                             << " estPRB(bg)=" << bgPrbEstAll);

    if (!summaryFile.empty())
    {
        std::ofstream out(summaryFile, std::ios::app);
        if (out.is_open())
        {
            out << std::fixed << std::setprecision(6);
            out << "simTime=" << simTime << ",appStart=" << appStart << ",numUes=" << numUes
                << ",bwpBaseline=" << g_bwpBaseline << ",trafficModel=" << trafficModel
                << ",backgroundRateKbps=" << backgroundRateKbps
                << ",httpRateMbps=" << httpRateMbps
                << ",videoRateMbps=" << videoRateMbps
                << ",gamingRateMbps=" << gamingRateMbps
                << ",duration=" << duration << ",avgMcs=" << avgMcsAll << ",bler=" << blerAll
                << ",avgTbler=" << avgTblerAll << ",runMeanThrMbps=" << runMeanThrMbps
                << ",runMeanAoiMs=" << runMeanAoiMs
                << ",runMeanPrbUtility=" << runMeanPrbUtility
                << ",runQueueOverflowRatio=" << runQueueOverflowRatio
                << ",runMeanSwitchCount=" << runMeanSwitchCount << ",prbTotal=" << totalPrbAll
                << ",bytesPerPrb=" << bytesPerPrbAll << ",estPrbBurst=" << burstPrbEstAll
                << ",estPrbBg=" << bgPrbEstAll << "\n";
            for (uint32_t i = 0; i < numUes; ++i)
            {
                const auto& burst = g_ueStats[i].flow[BURST];
                const auto& bg = g_ueStats[i].flow[BACKGROUND];
                uint64_t totalBytes = burst.rxBytes + bg.rxBytes;
                double thrMbps = (totalBytes * 8.0) / duration / 1e6;
                double avgAoiBurstMs =
                    (burst.aoiSamples > 0) ? (burst.aoiSumMs / burst.aoiSamples) : 0.0;
                double avgAoiBgMs = (bg.aoiSamples > 0) ? (bg.aoiSumMs / bg.aoiSamples) : 0.0;
                const auto& prb = g_prbStats[i];
                double bytesPerPrb =
                    (prb.prbTotal > 0) ? (static_cast<double>(totalBytes) / prb.prbTotal) : 0.0;
                out << "ue=" << i << ",thrMbps=" << thrMbps << ",aoiBurstMs=" << avgAoiBurstMs
                    << ",aoiBgMs=" << avgAoiBgMs << ",prbTotal=" << prb.prbTotal
                    << ",bytesPerPrb=" << bytesPerPrb << "\n";
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
    if (g_burstStateTraceStream && g_burstStateTraceStream->is_open())
    {
        g_burstStateTraceStream->flush();
        g_burstStateTraceStream->close();
    }

    Simulator::Destroy();
    return 0;
}
