# Pre Requisite
## Items Required:
- Computer with Windows OS
- Small `minus` screw driver
- Small `plus` screw driver
- Long black cable to power the gripper.[Image Attacked]
- White cable to connect the controller to the computer.[Image Attached]
![Items Image](/alex_externals/docs/ag_gripper_images/items_required.png)

## Install the dh gripper debugging software:
- Navigate to [DH Software Link](https://en.dh-robotics.com/service/software)
- Download DH Electric Gripper Debugging Software
- ![Installation Image](/alex_externals/docs/ag_gripper_images/dh-software_installation.png)
- Install the Software to the computer

## Steps:
1. Connect the Black long cable to the White cable as follows:

    |Black Cable Name|White Connection Point|
    |----|----|
    |485A|T/R+|
    |485B|T/R-|
    ![Connection Image](/alex_externals/docs/ag_gripper_images/communication_connection.png)

2. Unmount the VI0 and GI0 adapter from the controller box.
    ![Adapter Location Image](/alex_externals/docs/ag_gripper_images/controller_adapter_.png)
    ![Adapter Location Image](/alex_externals/docs/ag_gripper_images/unmounted_adapter.png)

3. Connect the power and ground from the long black cable to the previously removed adapter.
    |Black Cable Name|Adapter|
    |---- | ----|
    |24V|VI0|
    |GND|GI0|
    ![Power Connection](/alex_externals/docs/ag_gripper_images/power_adapter_connection.png)

4. Mount the two adapter back into the controller. 
    > NOTE: Connect `GND` first
    ![Power Connection](/alex_externals/docs/ag_gripper_images/mount_adapter_.png)

5. Connect the white tagged cable of the gripper extender to gripper input.
    ![Entender Connection](/alex_externals/docs/ag_gripper_images/entender_connection.png)
6. Unplug the extender from the robot, and connect it to the long black cable which connects to the power supply.
    ![Long cable connection](/alex_externals/docs/ag_gripper_images/long_cable_connection.png)
7. Plug the USB port of the White cable to the Computer.
8. Start the application
    ![DH Application](/alex_externals/docs/ag_gripper_images/dh_application.png)
9. In the console, you should see `Scanned COM port:...`
10. Click on `Auto-Connect`
11. You can close the `Oscilloscope`.
    ![Oscilloscope](/alex_externals/docs/ag_gripper_images/oscilloscope.png)
12. Navigate to the I/O Param pannel.
13. From the drop down, switch off the I/O Mode.
    ![switch off io mode](/alex_externals/docs/ag_gripper_images/io_params.png)
14. You can modify the group 1, group 2, etc. 
    |Type|Value|
    |----|----|
    |Maximum position|1000(Open)|
    |Minimum position|0(Close)|
    |Minimum Force|100%|
    |Minumim Force|20%|
15. You can test all the groups by clicking the `Test` button below each groups.
16. To save the current configuration, return the I/O mode back to `ON` from the drop down. 
17. Click on `Save`, and wait for the saving to be completed. Then click update. 
18. Now you can disconnect the gripper from the application. To disconnect, click `Connection` on the top bar > Then select disconnect. 
    ![switch off io mode](/alex_externals/docs/ag_gripper_images/disconnect.png)
19. Disconnect the gripper connection fromt the long cable back to the robot. 
20. You can use the `Flange Ditital Output` control button inside `Status` Section to move the gripper. 
    |Index 1|Index 2|  Group Position | 
    |-------|-------|-----------------|
    |   0   |   0   |     Group 1     |
    |   1   |   0   |     Group 2     |
    |   0   |   1   |     Group 3     |
    |   1   |   1   |     Group 4     |



### Defaults:
While creating this document, the set group values are given below:

| Position | Force  |  Group Position | 
|----------|--------|-----------------|
|   1000   |   100  |     Group 1     |
|   550    |   20   |     Group 2     |
|   400    |   20   |     Group 3     |
|   0      |   20   |     Group 4     |


# THE END
