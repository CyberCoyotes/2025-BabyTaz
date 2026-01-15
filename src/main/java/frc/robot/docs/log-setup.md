when is the fastest way to export those logs to my computer for analysis?

The fastest way depends on your workflow, but here are the best methods ranked by speed:

1. USB Drive (Fastest - Recommended)
On roboRIO:

AdvantageKit logs are saved to /home/lvuser/logs/ on the roboRIO
The logs are .wpilog files
Steps:

Plug USB drive into roboRIO (USB port on the robot)
SSH into roboRIO:
ssh admin@10.30.3.2  # Replace with your team number

Copy logs to USB:
# Find USB mount point
ls /media/

# Copy logs
cp /home/lvuser/logs/*.wpilog /media/usb0/

Eject USB and plug into your computer
Time: ~30 seconds per match log

2. SCP/SFTP Over USB (Fast)
If connected via USB tethering to roboRIO:

# From your computer
scp admin@10.30.3.2:/home/lvuser/logs/*.wpilog ~/Desktop/FRC-Logs/

Or use WinSCP (Windows) / Cyberduck (Mac):

Connect to 10.30.3.2 via SFTP
Navigate to /home/lvuser/logs/
Drag and drop files
Time: ~1-2 minutes for multiple logs

3. Radio WiFi (Slower but Convenient)
If connected to robot WiFi:

# SCP over WiFi
scp admin@roboRIO-3032-FRC.local:/home/lvuser/logs/*.wpilog ~/Desktop/FRC-Logs/

Or use AdvantageScope's built-in downloader:

Open AdvantageScope
File → Download Logs
Enter roboRIO address: roboRIO-3032-FRC.local or 10.30.3.2
Select logs to download
Time: ~3-5 minutes depending on WiFi

4. FTP (If Configured)
Some teams set up an FTP server on the roboRIO for automatic log downloading.

5. During Match (Real-Time)
NetworkTables streaming (not for storage, just live viewing):

Use AdvantageScope in live mode
Connect to robot: 10.30.3.2:5800 (NetworkTables)
View data in real-time
Note: This doesn't save logs, just views live data
Recommended Workflow for Competition
Between Matches (Fastest):
Option A - Automated Script:
Create a script download_logs.sh:

#!/bin/bash
ROBOT_IP="10.30.3.2"  # Change to your team number
DEST_DIR="~/Desktop/FRC-Logs/$(date +%Y-%m-%d)"

mkdir -p "$DEST_DIR"
scp admin@$ROBOT_IP:/home/lvuser/logs/*.wpilog "$DEST_DIR/"
echo "Logs downloaded to $DEST_DIR"

Run after each match:

./download_logs.sh

Option B - AdvantageScope Auto-Download:

Open AdvantageScope
File → Preferences → Download
Set roboRIO address: 10.30.3.2
Enable "Auto-download after disconnect"
At Competition (Fastest Manual Method):
After match: Plug USB into roboRIO
In pit: Remove USB, plug into laptop
Drag logs from USB to laptop folder
Analyze in AdvantageScope while USB goes back to robot
Time: < 1 minute per match

Pro Tips
Faster Log Collection:
1. Log Rotation (Prevent huge files):
Your AdvantageKit is probably already configured to rotate logs, but verify in your Robot.java:

Logger.recordMetadata("EventName", "Competition");
// Logs are auto-named with timestamp

2. Only Download What You Need:

# Download only today's logs
scp admin@10.30.3.2:/home/lvuser/logs/FRC_$(date +%Y%m%d)*.wpilog ~/Desktop/

3. Wireless File Sync (Advanced):
Set up rsync to automatically sync logs when connected:

# Add to ~/.ssh/config
Host roborio
    HostName 10.30.3.2
    User admin

# Then just:
rsync -av roborio:/home/lvuser/logs/ ~/FRC-Logs/

Viewing Logs Without Downloading:
AdvantageScope can open logs directly over network:

File → Open Robot
Enter 10.30.3.2:5800
View live data (NetworkTables)
Or browse /home/lvuser/logs/ remotely
Summary
Fastest method: USB drive directly plugged into roboRIO → ~30 sec/match

Most convenient: SCP over USB/WiFi → ~1-2 min

Competition workflow: USB drive in robot, swap after each match, analyze in pit

Your state machine logs are already being saved automatically to .wpilog files, so you just need to get them off the robot!

