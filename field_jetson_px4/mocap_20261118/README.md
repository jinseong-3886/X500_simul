# 모캡 실험 (2026-11-18, 13:00–16:00) 명령어 모음

파일 3개 (Jetson에서는 `~/mocap/` 에 복사해서 사용)
| 파일 | 하는 일 |
|---|---|
| `helix_rx.py` | HELIX 수신 테스트 (화면 출력 + CSV). 노트북/Jetson 공통 |
| `fc_params.py` | FC 파라미터 백업 / 모캡용 설정 / 되돌리기 / 확인 |
| `mocap_bridge.py` | 모캡 위치를 FC로 50 Hz 전송 + FC 연결을 UDP로 중계 (미션 코드, QGC용) |

PC 시뮬(PX4 SITL)에서 검증 완료 (11-18 12:30): 50 Hz 송신, 누락 0, FC 추정 위치가 모캡 값과 1 cm 안으로 일치.
브리지를 통한 파라미터 읽기·백업(1091개)·설정도 정상 동작.

## ① 데이터 수신 (13:00–13:30)
```
# 노트북
pip install ~/HELIX_Python_SDK-4.1.0.7412/HELIX_Python_SDK-4.1.0.7412/dist/helixpy-3.0.1-py3-none-any.whl
python3 helix_rx.py -s 192.168.0.66

# Jetson (USB로 접속해서 와이파이 연결 후)
pip3 install --user helixpy-3.0.1-py3-none-any.whl
python3 ~/mocap/helix_rx.py -s 192.168.0.66
```
통과: `수신 ~100 Hz`, 강체 `추적중`, `마지막 프레임`이 수십 ms 이하.

## ② 축 확인 (13:30–14:00)
1. 드론 앞을 보드 쪽으로 향하게 놓고 `helix_rx.py` 를 실행
2. 드론을 **앞으로 30 cm** 옮겼을 때 어느 값이 +/− 로 바뀌는지 확인
   - x 가 커짐 → `--fwd +x`
   - x 가 작아짐 → `--fwd -x`
   - y 가 커지거나 작아지면 → `+y` / `-y`
3. `--fwd` 를 넣고 다시 실행해서 확인: 앞으로 → `앞(N)` 증가, 오른쪽으로 → `오른쪽(E)` 증가, 들어 올림 → `위` 증가
4. **위로 들었는데 z 가 안 커지면 중단**하고 저(Claude)를 불러 주세요 (Z-up 이 아님)

## ③ FC 연결 (14:00–14:40, 프로펠러 없음)
```
python3 fc_params.py backup                  # 전체 백업
python3 fc_params.py set-mocap               # 자동 백업 후 모캡용 4개 값 설정
# → FC 재부팅 (배터리 뽑았다 꽂기)
python3 mocap_bridge.py -s 192.168.0.66 --body <강체이름> --fwd <②결과> --forward udpout:<노트북IP>:14550
```
- 브리지는 시작할 때 **드론이 앞(보드 쪽)을 보고 있어야** 해요. 그 방향이 yaw 0으로 맞춰져요.
- 화면의 `모캡`과 `FC추정` 숫자가 같이 움직이면 통과예요. 노트북 QGC 지도로도 확인할 수 있어요.
- **추적 끊김 시험**: 마커를 손으로 가리기 → `모캡 추적 끊김 - 송신 중단` 표시 → QGC에서 위치 경고가 뜨는지 확인

되돌리기: `python3 fc_params.py restore ~/fc_backup/params_<시각>.txt` 실행 후 재부팅

## 미션 코드와 같이 쓸 때 (오늘은 선택)
브리지가 FC 시리얼을 잡고 있으니, 미션 코드는 UDP로 접속하고 **모캡 송신은 끔**:
```
python3 fly_mission_px4.py --conn udpin:0.0.0.0:14551 --mocap none ...
```
