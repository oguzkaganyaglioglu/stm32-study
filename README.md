# STM32 STUDY REPOSITORY
In this repository, I publish the projects I made while learning STM32.
I am currently using the blue pill board.That's why I'm writing these projects for that board. But projects should work on any stm32f103-based board.
### blank_project

This is a blank project. I use this when creating a new project.

### blink

This is a the hello world of the embedded systems. This projects blinks the on board led.

### led_button

**This projects requires you to connect a button at the pin PC15.**
If you push the button the led turns on until you release the button.

### button_irq

**This projects requires you to connect a button at the pin PC15.**
<br/>
Every time you press the button, the  LED toggles. In this project, an interrupt is used to detect the button pressed.

### stm32f1xx_drivers

In this project, I am developing the drivers that I use in the other project.
<br/>
I use directory junction to point drivers folder in other projects to drivers folder in this project. Therefore, when I make a change in the drivers folder, it is available in all other projects.
<br/>
If you think there is a better way to do that than directory junctions you can send your idea to me through e-mail.
<br/>
e-mail: me@oguzkagan.xyz