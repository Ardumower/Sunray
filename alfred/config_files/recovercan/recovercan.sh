#!/bin/bash

# CAN Interface Monitor Script
# Monitors can0 interface and restarts it if no communication detected within 5 seconds

INTERFACE="can0"
TIMEOUT=5
LOG_FILE="/var/log/can-monitor.log"
PID_FILE="/var/run/can-monitor.pid"

# Function to log messages
log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

# Function to check if interface exists
check_interface() {
    while true; do
        if ! ip link show "$INTERFACE" &>/dev/null; then
            log_message "ERROR: Interface $INTERFACE does not exist"
            sleep 2 
            #exit 1
        else 
            return 
        fi
    done
}

# Function to restart CAN interface
restart_can_interface() {
    log_message "No CAN communication detected within ${TIMEOUT}s, restarting $INTERFACE"
    
    # Bring interface down
    if sudo ip link set "$INTERFACE" down; then
        log_message "Interface $INTERFACE brought down successfully"
    else
        log_message "ERROR: Failed to bring down interface $INTERFACE"
        return 1
    fi
    
    # Brief pause
    sleep 1
    
    # Bring interface up
    if sudo ip link set "$INTERFACE" up; then
        log_message "Interface $INTERFACE brought up successfully"
    else
        log_message "ERROR: Failed to bring up interface $INTERFACE"
        return 1
    fi
    
    # Additional pause to allow interface to stabilize
    sleep 2
}

# Function to monitor CAN traffic
monitor_can_traffic() {
    # Use timeout to limit candump execution time
    # candump will output CAN frames if there's traffic
    timeout "$TIMEOUT" candump "$INTERFACE" 2>/dev/null | head -1
}

# Function to cleanup on exit
cleanup() {
    log_message "CAN monitor service stopping"
    rm -f "$PID_FILE"
    exit 0
}

# Set up signal handlers
trap cleanup SIGTERM SIGINT

# Main function
main() {
    log_message "Starting CAN monitor service for interface $INTERFACE"
    
    # Store PID
    echo $$ > "$PID_FILE"
    
    # Check if interface exists
    check_interface
    
    while true; do
        # Monitor CAN traffic with timeout
        if traffic=$(monitor_can_traffic); then
            if [ -n "$traffic" ]; then
                log_message "CAN traffic detected: $traffic"
            else
                log_message "No CAN traffic detected within ${TIMEOUT}s timeout"
                restart_can_interface
            fi
        else
            log_message "candump command failed or timed out"
            restart_can_interface
        fi
        
        # Brief pause before next check
        sleep 1
    done
}

# Check if running as root or with sudo capabilities
if [ "$EUID" -ne 0 ] && ! sudo -n true 2>/dev/null; then
    echo "This script requires root privileges or passwordless sudo access"
    exit 1
fi

# Check if required tools are available
for tool in candump ip; do
    if ! command -v "$tool" &>/dev/null; then
        echo "ERROR: Required tool '$tool' not found. Please install can-utils package."
        exit 1
    fi
done

# Run main function
main
