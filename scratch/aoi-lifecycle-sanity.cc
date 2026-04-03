#include "ns3/core-module.h"
#include "ns3/network-module.h"

#include <array>
#include <map>
#include <unordered_map>
#include <vector>

using namespace ns3;

namespace
{

constexpr uint8_t TRAFFIC_TYPES = 3;

class AoiIdentityTag : public Tag
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::AoiLifecycleSanityIdentityTag").SetParent<Tag>().AddConstructor<AoiIdentityTag>();
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

    void SetTxTime(Time txTime)
    {
        m_txTimeNs = static_cast<uint64_t>(txTime.GetNanoSeconds());
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

struct OutstandingAoiPacketInfo
{
    uint32_t ueIdx{0};
    uint8_t trafficType{0};
    uint64_t enqueueTimeNs{0};
    uint64_t txTimeNs{0};
};

uint64_t g_nextPacketId = 1;
std::unordered_map<uint64_t, OutstandingAoiPacketInfo> g_outstandingById;
std::vector<std::array<std::multimap<uint64_t, uint64_t>, TRAFFIC_TYPES>> g_outstandingByUeType;
std::vector<std::multimap<uint64_t, uint64_t>> g_outstandingByUe;
uint32_t g_failures = 0;

void
TagAoiIdentity(const Ptr<Packet>& p, uint32_t ueIdx, uint8_t trafficType)
{
    AoiIdentityTag tag;
    tag.SetPacketId(g_nextPacketId++);
    tag.SetUeIdx(ueIdx);
    tag.SetTrafficType(trafficType);
    tag.SetTxTime(Simulator::Now());
    p->ReplacePacketTag(tag);
}

bool
ExtractAoiIdentity(Ptr<const Packet> p,
                   uint64_t* packetId,
                   uint32_t* ueIdx,
                   uint8_t* trafficType,
                   Time* txTime)
{
    AoiIdentityTag tag;
    if (!p->PeekPacketTag(tag))
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
    if (trafficType)
    {
        *trafficType = tag.GetTrafficType();
    }
    if (txTime)
    {
        *txTime = tag.GetTxTime();
    }
    return true;
}

void
RegisterOutstandingAoiPacket(uint64_t packetId, uint32_t ueIdx, uint8_t trafficType, Time enqueueTime, Time txTime)
{
    if (ueIdx >= g_outstandingByUeType.size() || trafficType >= TRAFFIC_TYPES)
    {
        return;
    }
    if (g_outstandingById.find(packetId) != g_outstandingById.end())
    {
        return;
    }

    OutstandingAoiPacketInfo info;
    info.ueIdx = ueIdx;
    info.trafficType = trafficType;
    info.enqueueTimeNs = static_cast<uint64_t>(enqueueTime.GetNanoSeconds());
    info.txTimeNs = static_cast<uint64_t>(txTime.GetNanoSeconds());
    g_outstandingById.emplace(packetId, info);
    g_outstandingByUeType[ueIdx][trafficType].emplace(info.enqueueTimeNs, packetId);
    g_outstandingByUe[ueIdx].emplace(info.enqueueTimeNs, packetId);
}

bool
UnregisterOutstandingAoiPacket(uint64_t packetId)
{
    auto it = g_outstandingById.find(packetId);
    if (it == g_outstandingById.end())
    {
        return false;
    }
    OutstandingAoiPacketInfo info = it->second;

    auto& byType = g_outstandingByUeType[info.ueIdx][info.trafficType];
    auto typeRange = byType.equal_range(info.enqueueTimeNs);
    for (auto pos = typeRange.first; pos != typeRange.second; ++pos)
    {
        if (pos->second == packetId)
        {
            byType.erase(pos);
            break;
        }
    }

    auto& byUe = g_outstandingByUe[info.ueIdx];
    auto ueRange = byUe.equal_range(info.enqueueTimeNs);
    for (auto pos = ueRange.first; pos != ueRange.second; ++pos)
    {
        if (pos->second == packetId)
        {
            byUe.erase(pos);
            break;
        }
    }

    g_outstandingById.erase(it);
    return true;
}

void
OnEnqueue(Ptr<const Packet> p)
{
    uint64_t packetId = 0;
    uint32_t ueIdx = 0;
    uint8_t trafficType = 0;
    Time txTime = Seconds(0);
    if (!ExtractAoiIdentity(p, &packetId, &ueIdx, &trafficType, &txTime))
    {
        return;
    }
    RegisterOutstandingAoiPacket(packetId, ueIdx, trafficType, Simulator::Now(), txTime);
}

void
OnDelivered(Ptr<const Packet> p)
{
    uint64_t packetId = 0;
    if (!ExtractAoiIdentity(p, &packetId, nullptr, nullptr, nullptr))
    {
        return;
    }
    UnregisterOutstandingAoiPacket(packetId);
}

void
OnFinalDrop(Ptr<const Packet> p)
{
    OnDelivered(p);
}

double
ComputeClassAoiMs(uint32_t ueIdx, uint8_t trafficType)
{
    if (ueIdx >= g_outstandingByUeType.size() || trafficType >= TRAFFIC_TYPES)
    {
        return 0.0;
    }
    const auto& outstanding = g_outstandingByUeType[ueIdx][trafficType];
    if (outstanding.empty())
    {
        return 0.0;
    }
    return (Simulator::Now() - NanoSeconds(outstanding.begin()->first)).GetMilliSeconds();
}

double
ComputeNodeAoiMs(uint32_t ueIdx)
{
    if (ueIdx >= g_outstandingByUe.size())
    {
        return 0.0;
    }
    const auto& outstanding = g_outstandingByUe[ueIdx];
    if (outstanding.empty())
    {
        return 0.0;
    }
    return (Simulator::Now() - NanoSeconds(outstanding.begin()->first)).GetMilliSeconds();
}

void
ExpectNear(const std::string& label, double actual, double expected, double toleranceMs = 1e-6)
{
    if (std::fabs(actual - expected) > toleranceMs)
    {
        ++g_failures;
        std::cout << "FAIL " << label << " expected=" << expected << " actual=" << actual << std::endl;
        return;
    }
    std::cout << "PASS " << label << " = " << actual << std::endl;
}

void
ExpectNodeAndClass(const std::string& prefix,
                   uint32_t ueIdx,
                   uint8_t trafficType,
                   double expectedNodeMs,
                   double expectedClassMs)
{
    ExpectNear(prefix + ".node", ComputeNodeAoiMs(ueIdx), expectedNodeMs);
    ExpectNear(prefix + ".class", ComputeClassAoiMs(ueIdx, trafficType), expectedClassMs);
}

} // namespace

int
main(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    cmd.Parse(argc, argv);

    g_outstandingByUeType.assign(2, {});
    g_outstandingByUe.assign(2, {});

    Ptr<Packet> deliverPkt = Create<Packet>(100);
    Simulator::Schedule(MilliSeconds(0), &TagAoiIdentity, deliverPkt, 0u, 0u);
    Simulator::Schedule(MilliSeconds(5), &ExpectNodeAndClass, "deliver.pre_enqueue", 0u, 0u, 0.0, 0.0);
    Simulator::Schedule(MilliSeconds(10), &OnEnqueue, deliverPkt);
    Simulator::Schedule(MilliSeconds(25), &ExpectNodeAndClass, "deliver.inflight", 0u, 0u, 15.0, 15.0);
    Simulator::Schedule(MilliSeconds(40), &OnDelivered, deliverPkt);
    Simulator::Schedule(MilliSeconds(45), &ExpectNodeAndClass, "deliver.post_rx", 0u, 0u, 0.0, 0.0);

    Ptr<Packet> noEnqueuePkt = Create<Packet>(100);
    Simulator::Schedule(MilliSeconds(50), &TagAoiIdentity, noEnqueuePkt, 0u, 1u);
    Simulator::Schedule(MilliSeconds(70), &ExpectNodeAndClass, "no_enqueue", 0u, 1u, 0.0, 0.0);

    Ptr<Packet> rlcDropPkt = Create<Packet>(100);
    Simulator::Schedule(MilliSeconds(80), &TagAoiIdentity, rlcDropPkt, 0u, 0u);
    Simulator::Schedule(MilliSeconds(90), &OnEnqueue, rlcDropPkt);
    Simulator::Schedule(MilliSeconds(100), &ExpectNodeAndClass, "rlc_drop.before_drop", 0u, 0u, 10.0, 10.0);
    Simulator::Schedule(MilliSeconds(120), &OnFinalDrop, rlcDropPkt);
    Simulator::Schedule(MilliSeconds(125), &ExpectNodeAndClass, "rlc_drop.after_drop", 0u, 0u, 0.0, 0.0);

    Ptr<Packet> harqDropPkt = Create<Packet>(100);
    Simulator::Schedule(MilliSeconds(130), &TagAoiIdentity, harqDropPkt, 0u, 2u);
    Simulator::Schedule(MilliSeconds(140), &OnEnqueue, harqDropPkt);
    Simulator::Schedule(MilliSeconds(175), &ExpectNodeAndClass, "harq_drop.before_drop", 0u, 2u, 35.0, 35.0);
    Simulator::Schedule(MilliSeconds(180), &OnFinalDrop, harqDropPkt);
    Simulator::Schedule(MilliSeconds(185), &ExpectNodeAndClass, "harq_drop.after_drop", 0u, 2u, 0.0, 0.0);

    Ptr<Packet> class0Pkt = Create<Packet>(100);
    Ptr<Packet> class1Pkt = Create<Packet>(100);
    Simulator::Schedule(MilliSeconds(200), &TagAoiIdentity, class0Pkt, 0u, 0u);
    Simulator::Schedule(MilliSeconds(210), &OnEnqueue, class0Pkt);
    Simulator::Schedule(MilliSeconds(220), &TagAoiIdentity, class1Pkt, 0u, 1u);
    Simulator::Schedule(MilliSeconds(230), &OnEnqueue, class1Pkt);
    Simulator::Schedule(MilliSeconds(240), []() {
        ExpectNear("multi_class.node.oldest", ComputeNodeAoiMs(0), 30.0);
        ExpectNear("multi_class.class0", ComputeClassAoiMs(0, 0), 30.0);
        ExpectNear("multi_class.class1", ComputeClassAoiMs(0, 1), 10.0);
    });
    Simulator::Schedule(MilliSeconds(250), &OnDelivered, class0Pkt);
    Simulator::Schedule(MilliSeconds(255), []() {
        ExpectNear("multi_class.after_class0_rx.node", ComputeNodeAoiMs(0), 25.0);
        ExpectNear("multi_class.after_class0_rx.class0", ComputeClassAoiMs(0, 0), 0.0);
        ExpectNear("multi_class.after_class0_rx.class1", ComputeClassAoiMs(0, 1), 25.0);
    });
    Simulator::Schedule(MilliSeconds(260), &OnFinalDrop, class1Pkt);
    Simulator::Schedule(MilliSeconds(265), []() {
        ExpectNear("multi_class.after_all_clear.node", ComputeNodeAoiMs(0), 0.0);
        ExpectNear("multi_class.after_all_clear.class1", ComputeClassAoiMs(0, 1), 0.0);
    });

    Ptr<Packet> otherUePkt = Create<Packet>(100);
    Simulator::Schedule(MilliSeconds(300), &TagAoiIdentity, otherUePkt, 1u, 0u);
    Simulator::Schedule(MilliSeconds(305), &OnEnqueue, otherUePkt);
    Simulator::Schedule(MilliSeconds(320), []() {
        ExpectNear("other_ue.ue0.node", ComputeNodeAoiMs(0), 0.0);
        ExpectNear("other_ue.ue1.node", ComputeNodeAoiMs(1), 15.0);
    });
    Simulator::Schedule(MilliSeconds(330), &OnDelivered, otherUePkt);
    Simulator::Schedule(MilliSeconds(335), []() {
        ExpectNear("other_ue.after_rx.ue1.node", ComputeNodeAoiMs(1), 0.0);
    });

    Simulator::Stop(MilliSeconds(350));
    Simulator::Run();
    Simulator::Destroy();

    if (g_failures > 0)
    {
        std::cout << "AoI lifecycle sanity failed: " << g_failures << " checks" << std::endl;
        return 1;
    }

    std::cout << "AoI lifecycle sanity passed" << std::endl;
    return 0;
}
