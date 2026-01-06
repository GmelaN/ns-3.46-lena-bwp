# NR BWP 강제 스위치/지연/큐잉 동작 가이드 (v1)

본 문서는 이번 커밋으로 추가/변경된 BWP 스위치 기능을 한눈에 이해할 수 있도록 정리한 매뉴얼입니다. gNB/UE 양쪽 BWP Manager 동작, 스위칭 지연, 큐잉 정책, UL/DL 동작, 예제 사용법을 포함합니다.

## 1. 핵심 변경점 요약
- **강제 BWP 스위치**: `BwpManagerGnb::ForceUeBwp(rnti, bwpId)`, `BwpManagerUe::ForceActiveBwp(bwpId)`로 UE별 BWP를 강제 지정.
- **스위칭 지연**: `SwitchingDelay` 속성(Time). 강제 명령 후 지정 시간 동안 전환 가드 적용.
- **전환 가드 중 큐잉**: 지연 구간에 들어온 DL PDU, UL BSR, SR을 큐에 적재 후 전환 완료 시 플러시. (UL CE는 단순 무시)
- **활성 MAC 필터**: gNB MAC이 비활성 UE의 DCI/DATA를 무시해 DL 스케줄링을 차단.
- **UL 라우팅 강제**: BSR/SR/PDUs를 강제 BWP로 전달.
- **예제**: `scratch/bwp-switch-multi.cc`에서 다중 UE, 좁/넓 BWP, 스위칭 지연, UL/DL 트래픽, 에너지 추정까지 포함.

## 2. 주요 클래스와 속성
### BwpManagerGnb (contrib/nr/model/bwp-manager-gnb.{h,cc})
- `ForceUeBwp(uint16_t rnti, uint8_t bwpId)`: UE별 강제 BWP 설정. 즉시 모든 MAC 비활성 → 지연 후 목표 BWP만 활성.
- `SwitchingDelay` (Attribute, Time): 강제 명령 후 적용까지의 지연 시간.
- `GetForcedUeBwp(rnti)`: 강제 BWP 조회 (없으면 UINT8_MAX).
- 내부 맵:
  - `m_forcedUeBwp`: 강제 BWP
  - `m_switchingUntil`: 전환 가드 종료 시각
  - `m_pendingDlPdu` / `m_pendingBsr` / `m_pendingSr`: 가드 중 큐잉
- 라우팅/필터:
  - DL 데이터: `DoTransmitPdu`에서 가드 시 큐잉, 완료 후 강제 BWP로 전송
  - UL BSR: `DoTransmitBufferStatusReport`에서 가드 시 큐잉
  - UL SR: `DoUlReceiveSr`에서 가드 시 큐잉
  - DL 스케줄링 필터: `NrGnbMac::DoSchedConfigIndication` 내부에서 비활성 UE 무시 (기존 변경사항)

### BwpManagerUe (contrib/nr/model/bwp-manager-ue.{h,cc})
- `ForceActiveBwp(uint8_t bwpId)`: UE 측 강제 BWP 설정. 지연 후 활성화.
- `SwitchingDelay` (Attribute, Time): UE 측 적용 지연.
- 내부 상태:
  - `m_switchingUntil`: 전환 가드 종료 시각
  - `m_pendingBsr`: 가드 중 BSR 큐
- BSR 전송: 가드 중이면 큐잉, 종료 후 자동 플러시.

### NrGnbMac (DL 필터)
- `DoSchedConfigIndication`에서 UE 비활성 시 DL DCI/DATA 무시 (활성 BWP만 스케줄).

## 3. 스위칭 시퀀스 (시나리오)
1) 상위 로직/예제에서 UE RNTI를 알아낸 후 `ForceUeBwp(rnti, targetBwp)`와 `ForceActiveBwp(targetBwp)`를 호출.
2) gNB: 모든 MAC에서 해당 UE 비활성 → `SwitchingDelay` 후 강제 BWP만 활성 → 큐에 있던 DL PDU/BSR/SR 플러시.
3) UE: `SwitchingDelay` 후 활성 BWP 갱신, 큐에 있던 BSR 플러시.
4) DL: 비활성 MAC은 스케줄 무시, 활성 BWP에서만 DCI/DATA 생성.
5) UL: BSR/SR/PDUs는 강제 BWP로 라우팅.

## 4. 전환 가드 동안 트래픽 처리 정책
- DL PDU: 큐잉 후 전환 완료 시 재전송.
- UL BSR/SR: 큐잉 후 전환 완료 시 재전송.
- UL CE(기타): 단순 무시(간단화). 필요 시 확장 가능.
- 스위칭 지연 기본값 0ms, 속성으로 조정 가능.

## 5. 사용 방법 (API 예시)
```cpp
Ptr<BwpManagerGnb> gnbMgr = DynamicCast<BwpManagerGnb>(gnb->GetComponentCarrierManager());
gnbMgr->SetAttribute("SwitchingDelay", TimeValue(MilliSeconds(2)));
// UE 측도 속성으로 설정 가능 (helper로 세트하거나 객체 직접 접근)

// RNTI 확보 후 강제 스위치
gnbMgr->ForceUeBwp(rnti, 1);
ueBwpMgr->ForceActiveBwp(1);
```

## 6. 예제: scratch/bwp-switch-multi.cc
- 구성: 1 gNB, 4 UE, 좁은 BWP(5 MHz), 넓은 BWP(80 MHz), 스위칭 지연 2 ms.
- 트래픽: DL/UL UDP 각각 1400B @ 0.5ms, 시작 50 ms, 종료 10 s.
- 스위치 패턴: UE0~3에 대해 초기 BWP/전환 시점 배열로 지정(스위칭 시퀀스 확장).
- 출력:
  - 스위칭 로그(비활성→지연→활성)
  - DL/UL throughput/mean delay (FlowMonitor)
  - 단순 전력 추정(정적 BWP 전력 모델 예시)

실행:
```
./ns3 run bwp-switch-multi -- --stop-on-failure=1
```

## 7. 확장 포인트
- **UL/DL 분리 강제**: 현재 UL/DL 동일 BWP(TDD 형태). 방향별 강제 맵을 나누면 FDD처럼 동작 가능.
- **큐 정책**: 현재 FIFO 즉시 플러시. 큐 크기 제한, 우선순위, 타임아웃 등 세분화 가능.
- **전력 모델**: 정적 예시 대신 PHY 상태 기반 DeviceEnergyModel 연계(향후 DRX 연동).
- **스위칭 지연**: 지금은 고정 지연. TTI/슬롯 단위 정밀 제어, 가드 중 블랭킹 정책 등 추가 가능.
- **UL CE 큐잉**: 현재 미지원. 필요 시 동일 구조로 큐잉/플러시 구현.

## 8. 알려진 제약/주의사항
- 스위칭 중 UL CE(예: 특정 MAC CE)는 무시됨.
- 가드 구간 동안 DL/UL이 지연만큼 밀림(큐잉). 실험 시 지연을 0으로 두면 즉시 스위치와 비교 가능.
- 에너지 추정은 예제 내 단순 계산이며 실제 전력 프로파일과 다를 수 있음.

## 9. 빠른 체크리스트
- RNTI 확보 후 `ForceUeBwp`/`ForceActiveBwp` 동시 호출? ✅
- `SwitchingDelay` 설정 확인? ✅
- UL/DL 모두 한 BWP 강제임을 인지? ✅
- 큐잉/플러시로 인한 지연 허용 가능한지? ✅

## 10. 참고 파일 경로
- gNB BWP Manager: `contrib/nr/model/bwp-manager-gnb.{h,cc}`
- UE BWP Manager: `contrib/nr/model/bwp-manager-ue.{h,cc}`
- MAC 활성 필터: `contrib/nr/model/nr-gnb-mac.cc` (`DoSchedConfigIndication`)
- 테스트/데모: `scratch/bwp-switch-multi.cc`

