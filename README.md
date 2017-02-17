Space Cookies Programming 2017 Guidelines
=======

This document contains [framework](#framework), [style](#style), and [check-in](#check-in) guidelines.

------------------------------------------------------------
Framework <a name="framework"></a>
=======
See the current documentation [here](http://htmlpreview.github.com/spacecookies1868/frc2017/master/documentation/html/index.html).

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
- Instance variables: `lowerCamelCase`
- Member instance variable: 'lowerCaseCamel_'. Don't forget the underscore!
- Macros, consts (like ports): `CAPITAL_LETTERS`
- Methods and classes: `UpperCamelCase`
- Enum-list: `kInit`, `kIdle`, etc.

- Abbreviate only when necessary
- Class names should be nouns. 
- Accessor methods typically start with `Get`, mutator methods typically start with `Set`.

Unit and Coordinate Conventions
--------
All distances are in feet, all angles are in degrees.
+x is forward, +y is left. This means that positive degrees are counterclockwise.

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
