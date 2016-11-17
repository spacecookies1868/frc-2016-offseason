Space Cookies Programming 2017 Guidelines
=======

This document contains framework, style, and check-in guidelines.

------------------------------------------------------------
Framework <a name="framework"></a>
=======
src
--------
####MainProgram
- Drives the program. Uses the IterativeRobot base class from the WPI Library.

####RobotModel
- Creates objects of everything on the robot (actuators, sensors, etc.)

####ControlBoard
- Reads driver station button and joystick values
- Sets them to desired states of drivetrain, superstructure, and autonomous

####Ports
- Contains the robot and driver station ports

src/Logger
--------
####Logger
- Logs state (`LogState()`) and action (`LogAction()`). Uses the macro `LOG(myRobot, stateName, state)` for `LogAction()`
	
####INI configuration file
- Sets up ini file. See `ini_config` for ini files.

####Debugging
- Sets up `DoPeriodic`

src/Autonomous
--------
####Commands
- WaitCommand, DriveStraightCommand, etc.
	
####Modes
- Contains our various autonomous modes by putting the commands into a queue

src/Vision
--------
####CameraController
- 

src/Controllers
--------
####DriveController
- Code for driving the robot

####SuperstructureController
- Code for moving the superstructure

####Style of controllers:
- Constructor takes in `RobotModel* myRobot, RemoteControl* myHumanControl` (except for `AutonomousController`)
- Have the following methods: `Init()`, `Reset()`, `Update(double currTimeSec, double deltaTimeSec)`, `RefreshIni()`

------------------------------------------------------------
Style Guidelines <a name="style"></a>
=======
Follow these guidelines to ensure consistency across our code.

Constructors
--------
Initialize all private instance variables in constructors.

Header Files
--------
All header files should have `#define` guards
````
	#ifndef FILE_NAME_H
	#define FILE_NAME_H
	...
	#endif		// SRC_FOLDER_FILE_NAME_H
````

Naming Conventions
--------
Private instance variables: `camelCaseFirstLetterLowercase`
Macros, consts (like ports): `CAPITAL_LETTERS`
Methods and classes: `CamelCaseFirstLetterUppercase`
-- Class names should be nouns. 
-- Accessor methods typically start with `Get`, mutator methods typically start with `Set`.

------------------------------------------------------------
Check-in Guidelines <a name="check-in"></a>
=======
Follow these guidelines for clarity and quality control.

Checklist:
- Are you only checking in the files you changed? Make sure to check the diffs between your code and the code in the repo.
- Have you added comments to your code?
- Has a leader and/or mentor checked your code?
- Have you written a descriptive commit message with the following details?:
	- The changes you made
	- Whether you tested the code. If so, please describe how the code ran.
	- What needs to be done on the code
	- Sign your name, the names of people working with you, and the person/people who checked your code.

------------------------------------------------------------

Remember to write a detailed blog post on our [internal build season website](https://sites.google.com/site/scbuildseason2017/programming)!
