# FRC Match Log Review Guide – Team 1405

Guide is WIP, has not been tested!

This guide explains how to download `.wpilog` files from the robot after a competition match and load them into AdvantageScope for telemetry review and match replay.

## Prerequisites

- A laptop with SSH access to the roboRIO
- AdvantageScope installed: https://docs.advantagescope.org/
- `.wpilog` files generated during the match using WPILib logging or AdvantageKit

## Step 1: Download Logs from the Robot

After the match, connect your laptop to the robot’s network (via USB or Ethernet tether) and follow these steps:

1. SSH into the roboRIO:
    
    ssh lvuser@roborio-1405-FRC.local

2. Navigate to the log directory:

    cd /home/lvuser

3. List available `.wpilog` files:


    ls *.wpilog

Note: If a USB stick was plugged into the roboRIO, logs may be saved to `/media/sda1/` or `/media/sdb1/`.

4. Transfer the file to your laptop using `scp`:

    scp lvuser@roborio-1405-FRC.local:/home/lvuser/<filename>.wpilog .

Replace `<filename>` with the actual log file name.

## Step 2: Load the Log into AdvantageScope

1. Open AdvantageScope on your laptop.
2. Go to `File > Open Log(s)...`
3. Select the `.wpilog` file you downloaded.
4. Use the interface to explore telemetry:
- View subsystem data
- Analyze command lifecycles
- Inspect drivetrain paths, sensor readings, and more
- Sync with match video if available

## Optional: Organize Logs by Match

Rename logs for clarity:


mv FRC_20250830_154700.wpilog QualMatch3_RedAlliance.wpilog

## Troubleshooting

- No logs found: Ensure logging was enabled in your robot code using `DataLogManager` or `@Logged` annotations.
- Can't connect to roboRIO: Try using the IP address `10.14.05.2` instead of mDNS.
- AdvantageScope not showing data: Confirm that your fields are properly annotated or published to NetworkTables.

## Resources

- AdvantageScope Documentation: https://docs.advantagescope.org/
- WPILib Logging Guide: https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/data-logging.html
- AdvantageKit (optional): https://docs.advantagekit.org/
