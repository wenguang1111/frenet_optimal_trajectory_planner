#!/bin/bash
# filepath: cvae/utils/monitor_resources.sh

LOG_DIR="$1"
INTERVAL="${2:-5}"

# mkdir -p "$LOG_DIR"

# Combined log file
LOG_FILE="$LOG_DIR/system_usage.log"

# Write header
echo "timestamp,cpu_percent,ram_used_gb,ram_total_gb,ram_percent,gpu_util,gpu_mem_used_mb,gpu_mem_total_mb" > "$LOG_FILE"

while true; do
    timestamp=$(date "+%Y-%m-%d %H:%M:%S")
    
    # CPU usage
    cpu_percent=$(top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print 100 - $1}')
    
    # RAM usage
    ram_info=$(free -g | grep Mem)
    ram_total=$(echo $ram_info | awk '{print $2}')
    ram_used=$(echo $ram_info | awk '{print $3}')
    ram_percent=$(echo $ram_info | awk '{printf "%.2f", ($3/$2)*100}')
    
    # GPU usage
    gpu_info=$(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits)
    gpu_util=$(echo $gpu_info | awk -F',' '{print $1}' | xargs)
    gpu_mem_used=$(echo $gpu_info | awk -F',' '{print $2}' | xargs)
    gpu_mem_total=$(echo $gpu_info | awk -F',' '{print $3}' | xargs)
    
    echo "$timestamp,$cpu_percent,$ram_used,$ram_total,$ram_percent,$gpu_util,$gpu_mem_used,$gpu_mem_total" >> "$LOG_FILE"
    
    sleep "$INTERVAL"
done