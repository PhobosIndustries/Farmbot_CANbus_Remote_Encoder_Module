void LEDaction()
{
  // Show the 4 colours
  _time = millis();
  if(_time - lastLEDFlash >= LEDFlashInterval)
  {
    // Update timestamp
    lastLEDFlash = _time;

    // Pick the colour
    switch(LEDsequence) {
      case first:
        // Change the LED colour
        statusLED.setPixelColor(0, colour1);

        // Toggle
        LEDsequence = second;
        
        break;
      case second:
        // Change the LED colour
        statusLED.setPixelColor(0, colour2);

        // Toggle
        LEDsequence = third;
        
        break;
      case third:
        // Change the LED colour
        statusLED.setPixelColor(0, colour3);

        // Toggle
        LEDsequence = fourth;
        
        break;
      case fourth:
        // Change the LED colour
        statusLED.setPixelColor(0, colour4);

        // Toggle
        LEDsequence = first;
        
        break;
      default:
        break;      
    }

    // Display
    statusLED.show();
  }
}
