# Assignment 2

### Requirement

Open Hero_Canon from Keil, diagram should include only the files in the Application/User folder.

1. Draw the function and variable dependencies, meaning which function called which functions, and which variable is created by which functions or declared at which file. Recommend to download Xmind Zen and Draw.io desktop app (not website) to draw the diagram. 
2. For each timer interrupt, please specify which timer it is, the frequecy setup, the the interrupt prioroty (NVIC). 
3. Descibe your understanding of each function by observe how their variables are interrelated.
4. Submission: TBD.

### Other hardware details in Hero robot

1. `CAN` and `UART` are communication protocols. The remote control receiver talks to the STM32 via DBUS, which is also implemented by UART. CAN is used for talking to the chassis, gimbal and trigger motors. The tigger motor determines the shooting frequency. Every motor corresponds a Electronic Speed Controller (ESC), and yet our ESC is kinda smart because it can talk over CAN and translate the data into motor accelration control, and encoder raw data back to the angular position and the speed of the motor.
2. For controlling brushless DC (BLDC) motors, `PWM` may also be used - in hero_2018, on the friction wheels, whose speed positively correlate the shooting speed. 
3. Also there are stepper motors for lifting the arm of the hero robot in `stepper.c` and `lifter.c`. The bullet collection is implemented in `collect.c`.
4. `PID` theory and code realization is introduced in the last training. Please consult your teammates if you had no idea what it is. Discussions are always encouraged within the ee_training group!

### Tips for reading the code

1. Open the project from Keil. The function needed to include in the diagram should should only be in the files in the Application/User folder. Any function in other folders are considered too low level, and you are not supposed to care.
2. `DMA` stands for "Direct Memory Access," enable peripheral devices to write directly into memory, instead of interrupting the cpu. Usually used when the interrupt frequency is too high and CPU cannot handle it.
3. `extern` means "the variable is declared somewhere else," usually used when interrupt functions write the value into the variable, but they are used somewhere else.
4. Usually the `struct`, `typedef`, `enum` and global variables are declared in the .h header file instead the .c file. In C, every .c corresponds a .h. If you have no idea what header file is, please check out the moodle tutorials. 
5. The STM32 schemtaics, [bbs.robomaster.com] and [robomaster.com] may have more details, discussions and posts that help faciltate a better understanding. 
6. Suggest you first start with a pen and paper, and `imu.c`, which we already have covered twice in the training. Write down the function names and try to link them by "Go to definitions of xxx." Then go to "STM32f4xxxx_it.c" to find all the interrupt request handlers. Gradually map the program by `controltask.c`, which is supposed to be the highest level of the program. 
7. Please search for "software diagramming technique" on the internet. Alright I will also try to find some good resources some time later, but Google is always your best teacher. :) 