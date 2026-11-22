# 실행

```bash
source ~/2026-AutoRace/source_all_ws.bash
ros2 launch tensor_bringup tensor_bringup.launch.py lane_mode:=deep lane_speed:=0.8
```

차선 단독은 `~/2026-AutoRace/lane_fusion_ws/deep.sh real`,
시뮬레이터 연결은 `~/2026-AutoRace/simul/lane.sh`입니다.
자세한 구성·빌드·튜닝은 [deep 차선 안내](../lane_fusion_ws/README.md)를 참고하세요.
