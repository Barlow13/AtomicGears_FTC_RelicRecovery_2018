click “File”, then “Settings…”.
In the settings panel:

Expand the “Tools” item
Click “External Tools” to open the External Tools list. The list is normally empty.
For each of the following tools below, click the [+] button, and fill in the provided fields (leaving all others unchanged). Once each tool screen is filled-in, click “OK” to save. (Note: The “Program” field is the same, so the value can be cut and pasted to avoid re-typing.)
Once all tools have been added, click the main settings panel’s “OK” button to save.

#“Enable ADB over TCP/IP”

###Field	Value
#####Name:	Enable ADB over TCP/IP
#####Program:	$ModuleSdkPath$/platform-tools/adb
#####Parameters:	tcpip 5555

#“Connect to ADB over WiFi Direct”

###Field	Value
#####Name:	Connect to ADB over WiFi Direct
#####Program:	$ModuleSdkPath$/platform-tools/adb
#####Parameters:	connect 192.168.49.1:5555

Once the above External Tools are added, connecting to an FTC robot wirelessly to program it and debug it is trivial (after having used USB to program the device at least once4). After each reboot of the Android device (and it doesn’t hurt any other time), follow these steps:

```
* Connect the robot’s Android device to the PC via USB.
* Ensure that a file (such as an OpMode) is open and the cursor is in that window5.
* Click Tools → External Tools → “Enable ADB over TCP/IP” to enable ADB.
* Disconnect the USB cable from the Android device and ensure the computer is connected to the WiFi Direct network with its WiFi adapter.
* Ensure the “FTC Robot Controller” app is running on the Android device.
* Click Tools → External Tools → “Connect to ADB over WiFi Direct” to connect to ADB.
* After the above steps, it may be useful to open the Android Monitor 
and ensure the connected device is selected in the device drop down (there may be a “[DISCONNECTED]” entry as well,
from the previous USB connection; it is not the correct one).
```

Android Studio should work exactly the same as when it’s connected via USB: the “Run” [▶] button should transfer new APKs to the Android device, and Android Monitor will work even while the robot is running.