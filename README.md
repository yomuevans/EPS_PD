# EPS_PD

To-Do List: AFDEVSAT EPS Firmware Features
==========================================

This document outlines the remaining features and ideas to be implemented in the AFDEVSAT CubeSat Electrical Power System (EPS) firmware. Each feature is firmware-only, requiring no hardware changes, and focuses on enhancing power distribution, fault management, diagnostics, and reliability. Priorities are assigned based on impact and feasibility, with implementation notes to guide development.

To-Do Items
-----------

### 1\. Advanced Fault Diagnostics and Reporting

*   **Description**: Classify fault types (e.g., overcurrent, overtemperature) based on eFuse signals and log detailed data (e.g., fault duration, type).
    
*   **Priority**: High
    
*   **Implementation Notes**:
    
    *   Update FaultLogEntry in eps\_faults.h to include fault\_type and fault\_duration.
        
    *   Analyze eFuse signals (e.g., TPS259621DDAT FLT pulse duration for overtemperature).
        
    *   Modify LogFault in eps\_faults.c to classify and store fault types.
        
    *   Example: if (fault\_pulse\_duration > 95ms) fault\_type = FAULT\_TYPE\_OVERTEMP;
        

### 2\. Power Line Current Limiting and Monitoring

*   **Description**: Monitor subsystem currents via ADC and disable power lines if thresholds are exceeded (e.g., UHF > 1500 mA).
    
*   **Priority**: High
    
*   **Implementation Notes**:
    
    *   Define current limits in ssp\_telemetry.h (e.g., MAX\_CURRENT\_UHF 1500).
        
    *   Check currents in SSP\_UpdateTelemetryAndParameters and call SSP\_DisablePowerLine if exceeded.
        
    *   Log events via LogFault.
        

### 3\. Automated Subsystem Health Checks

*   **Description**: Periodically ping subsystems (e.g., every 5 minutes) to detect early failures.
    
*   **Priority**: Medium
    
*   **Implementation Notes**:
    
    *   Add a health check routine in main.c to send PING commands via SSP.
        
    *   Track responses in a subsystem\_health array.
        
    *   Log failures using LogFault.
        

### 4\. Dynamic Power Management Based on Battery Levels

*   **Description**: Adjust power distribution based on battery voltage (e.g., disable non-critical systems below 7V).
    
*   **Priority**: High
    
*   **Implementation Notes**:
    
    *   Read battery voltage from ADC in ssp\_telemetry.c.
        
    *   Define thresholds (e.g., BATTERY\_VOLTAGE\_LOW 7000).
        
    *   Disable non-critical lines in ssp\_power.c.
        

### 5\. Enhanced Telemetry with Historical Data

*   **Description**: Store telemetry snapshots (e.g., every 15 minutes) in EEPROM for trend analysis.
    
*   **Priority**: Medium
    
*   **Implementation Notes**:
    
    *   Add a telemetry history buffer in ssp\_telemetry.c.
        
    *   Save snapshots to EEPROM using EEPROM\_Write.
        
    *   Implement an SSP command (GET\_HISTORICAL\_TELEMETRY) to retrieve data.
        

### 6\. Watchdog Timer for Firmware Reliability

*   **Description**: Reset EPS if firmware becomes unresponsive using STM32 IWDG.
    
*   **Priority**: High
    
*   **Implementation Notes**:
    
    *   Configure IWDG in main.c with a 10-second timeout.
        
    *   Call HAL\_IWDG\_Refresh in the main loop.
        

### 7\. Secure Command Execution with Authentication

*   **Description**: Require an authentication code for critical commands (e.g., KEN for mission termination).
    
*   **Priority**: Medium
    
*   **Implementation Notes**:
    
    *   Add authentication check in SSP\_HandleCommand in ssp\_command.c.
        
    *   Example: if (data\[0\] != AUTH\_CODE) return SSP\_RESP\_NACK;.
        

### 8\. Fault Recovery Prioritization

*   **Description**: Prioritize fault recovery for critical subsystems (e.g., OBC, CCU) over non-critical ones (e.g., Payload).
    
*   **Priority**: Medium
    
*   **Implementation Notes**:
    
    *   Define a FaultPriority enum in eps\_faults.h.
        
    *   Sort recovery attempts in HandleFaults based on priority.
        

### 9\. Power Cycling for Stuck Subsystems

*   **Description**: Automatically power-cycle subsystems that remain unresponsive after a timeout (e.g., 10 minutes).
    
*   **Priority**: Medium
    
*   **Implementation Notes**:
    
    *   Track unresponsiveness in main.c using a timer.
        
    *   Toggle power lines using SSP\_SetPowerLine and SSP\_DisablePowerLine.
        

### 10\. Adaptive Power Budgeting

*   **Description**: Adjust power distribution based on operational mode (e.g., disable X-Band in Normal mode).
    
*   **Priority**: Medium
    
*   **Implementation Notes**:
    
    *   Add mode-aware logic in ssp\_power.c.
        
    *   Update power lines based on mode changes received via SSP.
        

### 11\. Overvoltage Protection via Software

*   **Description**: Monitor voltages via ADC and disable power lines if overvoltage thresholds are exceeded (e.g., 3.6V for 3.3V rail).
    
*   **Priority**: High
    
*   **Implementation Notes**:
    
    *   Define thresholds in ssp\_telemetry.h (e.g., MAX\_VOLTAGE\_3V3 3600).
        
    *   Check in main.c and disable lines if exceeded.
        

### 12\. Periodic EEPROM Health Checks

*   **Description**: Test EEPROM read/write integrity periodically (e.g., every 24 hours).
    
*   **Priority**: Medium
    
*   **Implementation Notes**:
    
    *   Add a test routine in main.c to write and verify a pattern.
        
    *   Log failures using LogFault.
        

### 13\. Command Logging for Debugging

*   **Description**: Log all incoming SSP commands to EEPROM for post-mission analysis.
    
*   **Priority**: Low
    
*   **Implementation Notes**:
    
    *   Add a CommandLogEntry struct in ssp\_command.h.
        
    *   Log commands in SSP\_HandleCommand using EEPROM\_Write.
        

### 14\. Autonomous Mode Transitions

*   **Description**: Transition to modes (e.g., Safe Mode) based on conditions like low battery voltage.
    
*   **Priority**: High
    
*   **Implementation Notes**:
    
    *   Monitor conditions in main.c (e.g., battery voltage < 6.5V).
        
    *   Send SSP mode change commands (SM \[0x15\]).
        

### 15\. Real-Time Power Consumption Reporting

*   **Description**: Calculate and report subsystem power consumption (P = V × I) in real-time telemetry.
    
*   **Priority**: High
    
*   **Implementation Notes**:
    
    *   Compute power in ssp\_telemetry.c using ADC readings.
        
    *   Include in GOSTM (0x25) response packets.
        

### 16\. On-Orbit Power Distribution Reconfiguration

*   **Description**: Reassign power to subsystems via ground commands (implemented in eps\_power\_manager.c).
    
*   **Priority**: Implemented
    
*   **Notes**: Already implemented; allows dynamic power policy updates via SSP.
    

### 17\. Subsystem Power Usage Profiling

*   **Description**: Track per-subsystem power usage (average, peak) for analysis (implemented in eps\_power\_manager.c).
    
*   **Priority**: Implemented
    
*   **Notes**: Already implemented; provides historical power data.
    

### 18\. SSP Heartbeat-to-Power Mapping

*   **Description**: Disable power to subsystems missing SSP heartbeats (implemented in eps\_power\_manager.c).
    
*   **Priority**: Implemented
    
*   **Notes**: Already implemented; isolates non-responsive subsystems.
    

### 19\. Soft Power Ramp-Up

*   **Description**: Enable power lines sequentially to reduce inrush current (implemented in eps\_power\_manager.c).
    
*   **Priority**: Implemented
    
*   **Notes**: Already implemented; uses 500ms delays between power-ups.
    

### 20\. Mode-Dependent Load Shedding

*   **Description**: Disable non-critical subsystems based on mode and power availability (implemented in eps\_power\_manager.c).
    
*   **Priority**: Implemented
    
*   **Notes**: Already implemented; sheds optional loads when voltage is low.
    

### 21\. Solar Input-Aware Load Management

*   **Description**: Throttle power based on real-time solar input.
    
*   **Priority**: Medium
    
*   **Implementation Notes**:
    
    *   Calculate solar power in ssp\_telemetry.c using ADC readings.
        
    *   Throttle optional loads in eps\_power\_manager.c if demand exceeds supply.
        

### 22\. Predictive Battery Reserve Estimation

*   **Description**: Predict when battery reserve will reach a minimum and trigger actions (e.g., mode change).
    
*   **Priority**: High
    
*   **Implementation Notes**:
    
    *   Estimate runtime in eps\_power\_manager.c using SOC and current draw.
        
    *   Trigger Safe Mode if runtime < 10 minutes.
        

### 23\. Minimum Reserve Enforcer

*   **Description**: Prevent enabling high-demand subsystems if battery SOC is below a reserve (e.g., 20%).
    
*   **Priority**: Medium
    
*   **Implementation Notes**:
    
    *   Check SOC in ssp\_power.c before enabling lines.
        
    *   Log refusals using LogFault.
        

### 24\. Load Scheduling Based on Orbit Position

*   **Description**: Activate subsystems based on orbital position (e.g., enable cameras in sunlight).
    
*   **Priority**: Medium
    
*   **Implementation Notes**:
    
    *   Use RTC or GPS data to determine orbit position.
        
    *   Schedule in eps\_power\_manager.c using IsInSunlightWindow.
        

### 25\. Evaluate Load Shedding

*   **Description**: Disable optional loads when voltage drops critically (implemented in eps\_power\_manager.c).
    
*   **Priority**: Implemented
    
*   **Notes**: Already implemented; part of mode-dependent load shedding.
    

### 26\. Manage Solar Input Distribution

*   **Description**: Match subsystem usage to available solar power.
    
*   **Priority**: Medium
    
*   **Implementation Notes**:
    
    *   Extend eps\_power\_manager.c to calculate solar power.
        
    *   Throttle optional loads if demand exceeds solar input.
        

### 27\. Predict Battery Depletion

*   **Description**: Calculate runtime and enter Safe Mode if critically low.
    
*   **Priority**: High
    
*   **Implementation Notes**:
    
    *   Implement in eps\_power\_manager.c using SOC and current draw.
        
    *   Trigger mode change via SSP (SM).
        

### 28\. Enforce Min Reserve Before Enable

*   **Description**: Prevent enabling power lines below minimum SOC (e.g., 20%).
    
*   **Priority**: Medium
    
*   **Implementation Notes**:
    
    *   Add SOC check in ssp\_power.c before enabling lines.
        
    *   Log refusals using LogFault.
        

### 29\. Orbit-Based Load Control

*   **Description**: Schedule loads (e.g., cameras) during sunlight passes (implemented in eps\_power\_manager.c).
    
*   **Priority**: Implemented
    
*   **Notes**: Already implemented; uses IsInSunlightWindow.
    

Implementation Plan
-------------------

### High Priority (Critical Features)

*   **Advanced Fault Diagnostics and Reporting**: Enhance fault logging for better diagnostics.
    
*   **Power Line Current Limiting and Monitoring**: Prevent overcurrent scenarios.
    
*   **Dynamic Power Management Based on Battery Levels**: Ensure power availability.
    
*   **Watchdog Timer for Firmware Reliability**: Guarantee system recovery.
    
*   **Overvoltage Protection via Software**: Add redundant safety layer.
    
*   **Autonomous Mode Transitions**: Enable self-preservation.
    
*   **Real-Time Power Consumption Reporting**: Improve telemetry insights.
    
*   **Predictive Battery Reserve Estimation**: Prevent battery depletion.
    
*   **Predict Battery Depletion**: Enhance autonomous safety.
    

### Medium Priority (Enhancements)

*   **Automated Subsystem Health Checks**: Proactive failure detection.
    
*   **Enhanced Telemetry with Historical Data**: Support long-term analysis.
    
*   **Secure Command Execution with Authentication**: Improve security.
    
*   **Fault Recovery Prioritization**: Optimize recovery efforts.
    
*   **Power Cycling for Stuck Subsystems**: Automate recovery.
    
*   **Adaptive Power Budgeting**: Mode-based efficiency.
    
*   **Periodic EEPROM Health Checks**: Ensure data integrity.
    
*   **Solar Input-Aware Load Management**: Optimize solar power usage.
    
*   **Minimum Reserve Enforcer**: Protect battery reserves.
    
*   **Enforce Min Reserve Before Enable**: Enhance reserve enforcement.
    
*   **Load Scheduling Based on Orbit Position**: Schedule based on orbit.
    
*   **Manage Solar Input Distribution**: Balance solar power usage.
    

### Low Priority (Optional)

*   **Command Logging for Debugging**: Support post-mission analysis.
    

Notes
-----

*   **Testing**: Simulate faults, low battery conditions, and mode changes to verify each feature.
    
*   **Dependencies**: Ensure GetBatterySOC, GetADCReading, and SSP communication are functional.
    
*   **Resource Constraints**: Monitor RAM/Flash usage on STM32L496VGT6 to avoid overflow.