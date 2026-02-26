# Pre-Run Checklist

**Objective:** Ensure all systems, safety protocols, communication channels, and equipment are in full readiness before starting a run (either testing or mission).

## Actions:
### Safety Checks
1) Inspect robot for any hardware damage or misalignment.#

2) Test emergency stop mechanisms on both hardware and software levels via `ros2 service call /emergency_stop std_srvs/Trigger`

### Communications Check

#### Network Configuration:

- Verify DDS domain ID and that all nodes are discoverable and communicating over the correct network interfaces via `export ROS_DOMAIN_ID=0` and `ros2 node list`

#### Human-robot Communication:

- Verify that operator interfaces are responsive and can communicate with the robot.

### System Readiness

#### Hardware Check:

- Confirm all hardware is powered on and functioning (actuators, sensors, cameras).

- Run basic hardware tests to verify that sensors (LiDAR, cameras, GPS, IMU) are publishing data to ROS2 topics via `ros2 topic echo /robot/camera/image` and `ros2 topic echo /robot/imu/data`

#### Software/Node Check:

Verify that the necessary ROS2 nodes are running and communicating (robot controller, navigation stack, etc.) via `ros2 node list` and `ros2 topic list`.

#### Launch Configuration:

- Ensure that all launch files and configurations are correct (check if all parameters are loaded properly).

- Test core functionality with a mock mission sequence task (e.g., robot move forward 1 meter).

## Test Scripts and Recovery Plans:

- Ensure all test scripts are up-to-date and stored in the github.

- Review recovery plans in case of system failure.

# MCC Callout Script for Autonomy Attempt (Start/End/Fail)

Objective: TODO

## Callout Script: Start of Autonomy Attempt

TODO

## Callout Script: End of Autonomy Attempt (Success/Failure)

Objective: TODO

Success:

TODO

Failure:

TODO

## Callout Script: Anomaly Detection

Objective: TODO

# Clear Anomaly and Abort Handling Procedure

Objective: Define a clear and structured process for handling anomalies or faults during the run, with a focus on aborting missions when necessary.

## Anomaly Handling Procedure

### Detection

Continuously monitor system health, sensors, and communication links.

Any abnormal behavior (e.g., unexpected node shutdown, sensor data failure) should trigger an alert:

`ros2 topic echo /robot/health_status`
`ros2 service call /robot/diagnostics`

### Assessment

Review the data and determine if the anomaly is critical (requires abort) or non-critical (can be resolved with minor adjustments).

Team members should report findings via the designated communication channels.

### Recovery Plan

Attempt soft recovery (e.g., restarting a node, re-sending a command):

`ros2 node restart <node_name>`
`ros2 service call /restart_task <parameters>`

If recovery fails, initiate hard recovery or abort procedure.

## Abort Handling Procedure

### Decision

If the anomaly causes too much wastage (time, regolith), or if it compromises the safety of the mission, initiate an immediate abort.

### Execution

TODO

Use the emergency stop function for all systems:

`ros2 service call /emergency_stop std_srvs/Trigger`

### Post-Abort

Ensure all robot systems are powered off or in a safe state.

# Team Role Assignment for Dry Runs

Objective: Define roles and responsibilities for each team member during dry runs and ensure clear communication.

## Roles

TODO

# Dry Run Completion Process

Objective: Confirm that all tasks are successfully completed using the runbook only, with any gaps or issues logged and fixed.

## Dry Run Execution

### Start Dry Run:

Execute the dry run based on the procedures outlined in the runbook.

Follow the same steps as mission-day, including safety, communication, and system readiness.

### Monitor for Gaps:

During the dry run, observe for any issues or gaps in procedures, technology, or communication that might cause problems during the mission.

Log any gaps, including steps missed or errors in the procedure.

## Review and Feedback

### Post-Dry Run Review

Ensure each issue is clearly documented and assigned for fixing.

### Log and Fix Gaps

Log identified gaps or issues into a project management tool or shared document.

Create github issues to let members assign themselves to task and update runbook as necessary.

### Confirm Readiness:

Once all gaps are addressed and fixed, perform another dry run to confirm that the system operates as expected.

# Conclusion

Dry Run Complete: The runbook is considered complete when one full dry run is executed using the runbook only, all issues are logged, and the identified gaps are resolved.