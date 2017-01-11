Space Cookies Programming 2017 Guidelines
=======

This document contains [framework](#framework), [style](#style), and [check-in](#check-in) guidelines.

------------------------------------------------------------
Framework <a name="framework"></a>
=======
src
--------
####MainProgram
- Drives the program. Inherits from the IterativeRobot base class from the WPI Library.

####RobotModel
- Instantiates everything on the robot (actuators, sensors, etc.)

####ControlBoard
- Reads driver station button and joystick values
- Sets them to desired states of drivetrain, superstructure, and autonomous

####Ports
- Contains the robot and driver station ports

####Logger
- Logs state (`LogState()`) and action (`LogAction()`). Uses the macro `LOG(myRobot, stateName, state)` for `LogAction()`

src/Autonomous
--------
####/Commands
- Contains command classes (`WaitCommand`, `DriveStraightCommand`, etc.)

#####AutonomousCommand
- Abstract class, contains the `Init()`, `Update()`, `IsDone()` virtual methods

####/Modes
- Contains our various autonomous modes by putting the commands into a queue

#####BlankMode
- The robot just sits

src/Vision
--------

src/Controllers
--------
####Controller
- Abstract class, contains the `Init()`, `Reset()`, `Update(double currTimeSec, double deltaTimeSec)`, `RefreshIni()` virtual methods

####DriveController
- Code for driving the robot

####SuperstructureController
- Code for moving the superstructure

ext
--------
####ini

####navx

------------------------------------------------------------
Style Guidelines <a name="style"></a>
=======
Follow these guidelines to ensure consistency across our code.

Constructors
--------
- Initialize all private instance variables in constructors in roughly the same order they were declared in the header file.

Header Files
--------
- All header files should have `#define` guards
````
	#ifndef FILE_NAME_H
	#define FILE_NAME_H
	...
	#endif		// SRC_FOLDER_FILE_NAME_H
````
- `#include` the header files in this order (if applicable): `WPILib.h`, external libraries specific to robotics (like `navx/AHRS.h`), other header files in the order they appear in your source tree, and libraries specific to C++ (like `<iostream>`)
- Only `#include` the header files you need

Pointers
--------
- Declare pointers like `type *ptrName;`. If you're declaring more than one pointer, do `type *ptrName1, *ptrName2;`.

Formatting
--------
- Put the starting curly brace on the same line (like `Method() {`)
- Try to group similar methods and variables together. Separate the groups with empty lines.
- In header files, generally methods go before variables
- Methods should be written in this order: Get, Set, Reset

Switch statements
--------
Format:
````
switch (condition) {
case ABC:
    statements;
    break;
case DEF:
    statements;
    break;
default:
    statements;
    break;
}
````

Naming Conventions
--------
- Private instance variables: `lowerCamelCase`
- Macros, consts (like ports): `CAPITAL_LETTERS`
- Methods and classes: `UpperCamelCase`
- Enum-list: `kInit`, `kIdle`, etc.

- Abbreviate only when necessary
- Class names should be nouns. 
- Accessor methods typically start with `Get`, mutator methods typically start with `Set`.

Other
--------
- Use `double` (not `float`) for decimal values
- Always use curly braces for if statements, for loops, etc.

------------------------------------------------------------
Check-in Guidelines <a name="check-in"></a>
=======
Follow these guidelines for clarity and quality control.

Checklist:
- [ ] Are you only checking in the files you changed? Make sure to check the diffs between your code and the code in the repo.
- [ ] Does the code compile?
- [ ] Have you added descriptive and concise comments to your code?
- [ ] Does the code comply with our [style guidelines](#style)?
- [ ] Has a leader and/or mentor checked your code?
- [ ] Have you written a descriptive commit message with the following details?:
	- The changes you made
	- Whether you tested the code and how it ran on the robot
	- Next steps / what needs to be done on the code
	- Sign your name, the names of people working with you, and the person/people who checked your code.

------------------------------------------------------------

Remember to write a detailed blog post on our [internal build season website](https://sites.google.com/site/scbuildseason2017/programming)!
