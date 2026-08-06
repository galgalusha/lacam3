#!/bin/bash

# Target the actual executable, not the wrapper script
PROCESS_NAME="build/main"

echo "Waiting for $PROCESS_NAME to launch..."

# Fast-poll until the process is found
PID=""
while [ -z "$PID" ]; do
    # head -n 1 ensures we only grab one PID if there are multiple matches
    PID=$(pgrep -f "$PROCESS_NAME" | head -n 1)
    sleep 0.05
done

echo "Found $PROCESS_NAME running with PID: $PID"
echo "Time         | VmSize      | VmRSS"
echo "---------------------------------------"

# Monitor loop
while kill -0 "$PID" 2>/dev/null; do
    # Suppress errors on grep in case the process dies exactly while reading
    VM_SIZE=$(grep -E "^VmSize:" /proc/"$PID"/status 2>/dev/null | awk '{print $2, $3}')
    VM_RSS=$(grep -E "^VmRSS:" /proc/"$PID"/status 2>/dev/null | awk '{print $2, $3}')
    
    NOW=$(date "+%H:%M:%S.%3N")
    
    if [ -n "$VM_SIZE" ]; then
        echo "$NOW | Size: $VM_SIZE | RSS: $VM_RSS"
    fi
    
    # Poll every 100ms
    sleep 0.1 
done

echo "Process $PID terminated."
