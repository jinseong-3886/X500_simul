# field_jetson_px4 — 실기체(Jetson + PX4) 작업 백업 (2026-11-13 ~ 11-17)

> 기존 저장소 파일은 하나도 수정하지 않고, 이 폴더에만 새로 추가한 작업물입니다.
> 기체: X500 V2 + KOA FC(**PX4 1.16**) + Jetson Orin Nano + D435i. FC 제어는 ArduPilot GUIDED 대신 **PX4 OFFBOARD**로 바꿈.

## 폴더 구성
| 폴더 | 내용 |
|---|---|
| `px4/` | PX4용 미션 코드와 시뮬 |
| `px4/fly_mission_px4.py` | `sim_gazebo/fly_mission.py`의 PX4 OFFBOARD판. yaw 부호 버그와 `--tgt-h` 거리 계산을 수정하고, `--mocap`(VISION_POSITION_ESTIMATE)·지오펜스를 넣었으며, 조종자가 조종을 넘겨받을 때 LAND로 맞서지 않게 함 |
| `px4/gazebo_sim/` | PX4 SITL + Gazebo 시뮬(`README.md` 참고), clean/patched 결과 영상·로그 |
| `px4/sitl/` | 초기 SITL 연동·추종 테스트 |
| `px4/fc_check.py` | 실제 FC 파라미터 읽기 전용 점검 |
| `px4/fc_backup/` | 킬 스위치 설정 전 **FC 전체 파라미터 백업(960개)**, 파라미터 설정·RC 확인·킬 스위치 확인 스크립트, 프로펠러 없이 한 킬 테스트 로그 |
| `patch_print_test_20261113/` | 첫 실물 패치 테스트. 태블릿 화면은 r=0.46으로 **작동**, A4 원본 인쇄는 r≈1.0으로 **실패** |
| `color_chart_20261116/` | 프린터 색 보정용 A3 색상표(242칸, ArUco 마커). 인쇄소 안내는 `README.md` |
| `print_precomp_patch_20261116/` | 인쇄가 어둡게 나오는 걸 미리 상쇄하려고 밝기 보정(gamma 0.65/0.8)한 패치와 깨끗한 이미지 쌍 + 생성 스크립트 |

## 현재까지 상태 요약
- **킬 스위치 완료(11-16)**: SWA → FC ch7, `RC_MAP_KILL_SW=7`. 바꾼 파라미터는 이것 하나. 프로펠러 없이 시동 건 상태에서 킬 테스트 통과
  - 비행 모드 스위치 SWC = ch5 (위 Stabilized / 가운데 Altitude / 아래 Position)
  - ch6은 스로틀을 그대로 따라가므로 사용 금지
- **전원**: 드론 배터리로 Jetson + 카메라 + YOLO 구동 시 평균 7.3 W, 전원 저하(brown-out) 0회
- **패치**: 밝게 보정(g0.65)한 A4 인쇄를 0.42 m(A2 기준 0.83 m)에서 측정 → 박스 291→160 px, r=0.55로 **작동**
  - 아직 할 일: 1.5 m급에서 확인, 색상표 기반 재학습
- **보드**: 그림 59.4×39.6 cm를 A2 폼보드에 부착, 중심 높이 1.0 m, 이륙 지점에서 2.5 m, `--tgt-h 0.396`
- **모션캡처**: XEROmotion HALO + HELIX. 스트리밍 프로토콜과 축 방향은 아직 확인 중
