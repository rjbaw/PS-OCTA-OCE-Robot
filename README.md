# Issues and Possible Solutions

## Symptom: Displayed Values Are Incorrect or Robot Is Non-Responsive
The robot’s displayed values are nonsensical or corrupted - clicking on next/previous button
and the robot will not move.

### Potential Fix:
1. Ensure that everything is running smoothly:
Ensure that the robot is in Remote Control mode and powered on, ROS2 driver is running inside WSL, and correct Labview version (2024) is running.
There should be no red color log messages.
2. Check Robot Connection:
To check if robot connection is working properly, try clicking on the Next/Previous/Reset buttons on the Labview interface. The robot should move accordingly.
If the robot driver appears to be disconnected, restart it and verify that the connection is stable.
3. Try restarting:
If restarting Labview does not work, restart the Robot. If all fails see "Planning error" section.

## Symptom: Planning Error / Planning request aborted is displayed on Labview status log
The robot’s path or motion planning is failing, potentially causing it to remain stationary or move incorrectly.

### Potential Fixes:
- Reset the Robot to Default State:
Move the robot manually or using the pendant so that its joints are as close as possible to their default (home) positions. Once in a near-default pose, click "Reset" on the Labview interface.
- Run Reset Script:
From your workstation, run the ./reset.sh script several times.
Check the terminal output for any error messages and ensure that the script completes successfully each time.
You can exit script if the robot returns to default state.
- Use the Robot Pendant to Locate the Zero Position:
On the robot’s pendant, navigate to the Installation section.
Identify the robot’s zeroth position.
Manually jog or position the robot joints so that each joint matches the zeroth position as closely as possible. After the robot reaches the position, run reset script again.
After the robot is aligned to the zero position, run the ./reset.sh script a few more times to ensure all settings are properly initialized.

## Symptom: Labview crash while auto-focusing 
While performing the auto focusing process through the Labview interface, the interface might crash unexpectedly. 
There is no solution for this issue yet.

### Potential Fixes:
- Use task manager to end Labview process and the associated Matlab process.
- This error and velocity error displayed on robot pendant is still unsolved. 
Robot should fall into disabled mode automatically when encountering this erro. 
Just hit enable and ignore for now.
> Error message from the robot tablet :
> ‘FAULT
> Wrist 2BCS06A1 : joint : Not Stopping fast enough
> Explanation : joint was unable to come to a full stop fast enough
> …’

## Symptom: Robot moves in unexpected ways when trying to autofocus
When autofocusing the robot might move upwards to reorient a few joints to reach the desired position. 
This happens when the robot encounters a singular solution during planning, therefore there are multiple joint solutions to reach that position. 

### Potential Fixes:
Until robot planning issues are solved, it is recommended to perform operations not far from the default state (Reset button position).
- Hit CTRL-C through the WSL interface to immediately stop robot movement and use freedrive/reset button to move the robot back into position.

## Symptom: Robot is being "loose"
Robot sometimes loses its ability to remain stable/static. This is due to incorrect TCP configuration.

### Potential Fixes:
Recalibrate the robot's Tool Center Point (TCP) both in the robot pendant and in the ROS2 driver. 
Make sure that the new TCP configuration (installation) is saved and loaded correctly in the robot pendant.
For the ROS2 driver, relevant files are location in the `urdf` folder.
