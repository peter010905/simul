#!/bin/bash
# Double-click on the Mac: start the osp-jazzy VM (if needed), start the physical robot
# control board on port 8767 (if not running) and open it in the browser.
# It never sends a motion command; driving starts only from the board's start button.
set -u
board=http://192.168.252.2:8767/
vm=osp-jazzy

echo "== 1. 로봇 랜선 어댑터 확인 (en5, USB 10/100 LAN)"
if ! ifconfig -l | tr ' ' '\n' | grep -qx en5; then
  echo "en5 어댑터가 없습니다. 로봇 랜선을 USB 10/100 LAN 어댑터로 Mac에 연결한 뒤 다시 실행하세요."
  echo "(VM 네트워크는 en5에 연결되도록 설정되어 있습니다.)"
  read -r -p "Enter를 누르면 닫힙니다." _
  exit 1
fi

echo "== 2. VM 시작"
state=$(multipass info "$vm" --format csv 2>/dev/null | awk -F, 'NR==2{print $2}')
if [ "$state" != "Running" ]; then
  multipass start "$vm" || { echo "VM 시작 실패. multipass list로 상태를 확인하세요."; read -r -p "Enter" _; exit 1; }
fi

echo "== 3. 로봇 제어 보드 시작"
multipass exec "$vm" -- bash -c '
if systemctl --user is-active --quiet osp-robot-board; then echo "보드가 이미 실행 중입니다."; exit 0; fi
systemctl --user reset-failed osp-robot-board 2>/dev/null
systemd-run --user --unit=osp-robot-board --working-directory=/home/ubuntu/simul bash -c "source /opt/ros/jazzy/setup.bash && source /home/ubuntu/face_intent_flow_v4r2_ws/install/setup.bash && export ROS_DOMAIN_ID=4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/simul/robot_profiles/wired_flow_v3/fastdds_vm.xml && exec python3 -u dashboard/robot_board.py --domain 4 --host 0.0.0.0 --port 8767 --enable-control --control-workspace /home/ubuntu/face_intent_flow_v4r2_ws"
' </dev/null

echo "== 4. 보드 응답 대기"
for _ in $(seq 1 30); do
  if curl -s -m 2 -o /dev/null "$board"; then break; fi
  sleep 1
done

echo "== 5. 읽기 전용 점검 (로봇을 움직이지 않습니다)"
cd "$(dirname "$0")" && ./robot.sh check || echo "점검 실패: 로봇 전원과 랜선을 확인하세요."

open "$board"
echo "보드를 열었습니다: $board"
echo "정책 목록에서 'Mixed v1.5.2.57 · 벽 보기 + 짧은 순회 · LiDAR 5m (실험)'을 고르고 구동 버튼을 누르세요."
read -r -p "Enter를 누르면 닫힙니다." _
