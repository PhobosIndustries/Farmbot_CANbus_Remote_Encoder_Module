# Farmbot_CANbus_Remote_Encoder_Module
A simple controller for managing a local rotary encoder and communications to the host via CANbus.

This module runs on an arduino nano, communicates to the host via an MCP2515 CANbus transceiver and signal states to the operator by a single WS2812B RGB LED (Adafruit NeoPixel). Each device is addressed by a 4 way dip switch.

Dependencies:
CAN library by Sandeep Mistry - https://github.com/sandeepmistry/arduino-CAN
Adafruit NeoPixel Library - https://github.com/adafruit/Adafruit_NeoPixel
Encoder library by Paul Stoffregen - https://github.com/PaulStoffregen/Encoder/
