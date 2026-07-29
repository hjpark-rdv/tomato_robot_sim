# PCAN-USB를 `can2`로 사용하기

PEAK PCAN-USB를 연결한 뒤 아래 명령을 실행합니다. 기본 비트레이트는
500 kbit/s입니다.

```bash
cd /root/farmily_tomato/ros2_ws/src/farmily_uv_lift
sudo ./setup_can2.sh
candump can2
```

다른 비트레이트를 쓰는 CAN 버스라면 첫 번째 인자로 bit/s 값을 지정합니다.

```bash
sudo ./setup_can2.sh 250000
```

인터페이스 상태와 에러 카운터는 다음 명령으로 확인할 수 있습니다.

```bash
ip -details -statistics link show can2
```

USB를 분리하거나 재부팅하면 커널이 장치를 다시 `can0` 같은 이름으로 만들 수
있으므로 `setup_can2.sh`를 다시 실행합니다.
