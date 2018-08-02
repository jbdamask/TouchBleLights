# TouchBleLights

Code for Adafruit Feather Bluefruit 32u4 with MPR121 capacitive touch breakout. This code allows for the control of attached NeoPixels by either touch or Bluetooth Low Energy. Touch events are packaged into a payload and written to UART TX for use by listeners.

## Materials

* [Feather Bluefruit 32u4](https://www.adafruit.com/product/2829)
* [MPR121](https://www.adafruit.com/product/1982)
* [NeoPixles](https://www.adafruit.com/product/1376?length=1)
* [Buttons (Optional. If you use these, the MPR121 becomes a button hub)](https://www.adafruit.com/product/367)



## Connecting devices

While this is pretty cool by itself, it gets really fun when you connect several. This can be done
via my [PiHubBle](https://github.com/jbdamask/PiHubBle) or [PiHubBleAwsIoT](https://github.com/jbdamask/PiHubBleAwsIoT) projects.

## ToDo
* Eventually, clean up the code. As I've been writing while learning and experimenting, this is a freaking mess.