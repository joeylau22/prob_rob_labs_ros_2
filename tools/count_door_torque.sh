#!/bin/bash
log=$(mktemp)
ros2 topic echo /hinged_glass_door/torque > "$log" &
echo_pid=$!
sleep 2

for i in $(seq 1 100); do
  echo "request $i"
  ros2 topic pub -1 /door_open std_msgs/msg/Empty
  if [ "$i" -lt 100 ]; then
    sleep 10
  fi
done

sleep 1
kill "$echo_pid" 2>/dev/null
wait "$echo_pid" 2>/dev/null
n=$(grep -c 'data: 5.0' "$log" || true)
echo "torque == 5.0: $n / 100"
rm -f "$log"
