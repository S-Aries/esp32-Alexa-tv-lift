# esp32-Alexa-tv-lift

This projet is for a tv lift that is designed to be concealed in a piece of furnature such as a kitchen counter. and when called upon 
will lift the tv into it viewable poition and then rotate the tv to its preset locationand turn the tv on.

I wanted this project to function with the following abilities: 
voice comnand via amazon echo/ alexa;
function with tv remote;
hall effect home and limit switches to prevent position loss in case of malfunction or power outage;
tv remote to keep its current functionality;
minimal power consumption, using the sleep function on the A4988 stepper driver, and esp32.

this project incorporates a linear actuator, and a stepper motor, esp32, and esp8622-01, not to mention an amazon echo, tv and 
its remote.

it is important to note that I am not utilizing a web server to comunicate instructions between the esp32 and the echo.
Instead I used fuaxmo to emulate a Belkin WEMO.
