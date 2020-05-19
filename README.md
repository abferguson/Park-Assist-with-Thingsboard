# Park-Assist-with-Thingsboard
An ESP8266-based device using ultrasonic sensor to help driver park car at the correct position

I ran across other maker's park assist devices and decided it might be useful for my sister. She has a tendency to park too close to the back wall of her garage resulting in damage to the wall. It is useful and it does work. I make two of these for myself.

My goal was to use the HC-SR04 ultrasonic sensor to make the measurements coupled with a 5mm LED to provide visual feedback:
  1) Turn the LED on when the car was within range of the sensor
  2) Blink the LED white at a faster rate the closer the car was to the sensor
  3) Change the LED color to red, with no blinking, when the car reaches the park position.
  4) Provide a momentary contact button on the unit to provide distance setting capability.
  5) Store the park position distance onboard the unit using SPIFFS, in case of reboot or power failure.
  
The park assist unit is also integrated with a ThingsBoard server.  Via ThingsBoard, the unit can be monitored for the latest network parameters, sensor readings, reboots and for errors.  The park stop distance used by the unit can also be changed through ThingsBoard.

The unit above was later modified to add a Neopixel LED strip to aid the visual feedback. Others before me have done this and it look like a good way to play with Neopixels and provide useful functionality to the unit.  Concept: As the car gets closer to the sensor, fewer LEDs are lit on the strip.  At the stop position all Neopixel LEDs are turned red.  Like the 5mm LED, the Neopixel LEDs are not lit when the the car is out of range of the sensor.

Additional LED effects:
After five seconds of being in the 'park' position, the 5mm LED brighness is reduced.  Additionally, a random Neopixel animation is executed for amusement for a few seconds.  the Neopixels are then turned off.

Firmware environment: Arduino IDE

Hardware: (see Fritzing file for circuit)
 1 x NodeMCU v1.0 (ESP-12E) microcontroller
 
 1 x 5mm RGB LED
 
 3 x 470 Ohm resistor (for RGB LED)
 
 1 x 10k Ohm resistor (momentary switch)
 
 1 x momentary contact tactile button
 
 1 x HC-SR04 ultrasonic sensor
 
 1 x WS2912b LED strip (I use 21 LEDs. ESP8266 can power these).
 
 1 x 470 Ohm resistor (WS2912b data line)
 
 1 x 4 channel level converter (HC-SR04 trigger and echo lines, WS2812b data line)
 
 1 x 1000 uF capacitor (for WS2812B LED strip)
 
