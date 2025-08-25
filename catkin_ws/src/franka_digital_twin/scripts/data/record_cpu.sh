#!/bin/bash


LOG_FILE="gazebo_cpu_log_specific.txt"


if [ -f "$LOG_FILE" ]; then
    > "$LOG_FILE"
fi

echo "正在记录 Gazebo 进程的 CPU 利用率，每秒一次..."


for i in $(seq 1 60); do

  cpu_usage=$(ps -C gzserver -o %cpu --no-headers | awk '{s+=$1} END {print s}')
  

  if [ -z "$cpu_usage" ]; then
      cpu_usage=0.0
  fi
  

  echo "$(date +%s),$cpu_usage" >> "$LOG_FILE"
  

  sleep 1
done

echo "数据记录完成，请查看 $LOG_FILE 文件。"
