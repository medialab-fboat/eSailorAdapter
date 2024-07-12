# actuators-code-just-arduino-mega-version
Folder with code and dependencies to be uploaded to the Arduino Mega board of the FBoat.

Folder Contents:

.pio: Folder containing dependencies used during the development of the main code file (RudderCheck.cpp).
.vscode: Folder containing files that add the PlatformIO extension to VS Code (the settingssettings file needs to be customized).
include: Contains the RudderCheck.hpp file and the \mavlink\mavilink.h file, which should be referenced in the main code file (RudderCheck.cpp).
lib: Contains the UN178MotorDriver\UN178Driver.hpp file, which should be referenced in the main code file (RudderCheck.cpp).
src: Contains the code to be uploaded to the Arduino Mega of the FBoat.
