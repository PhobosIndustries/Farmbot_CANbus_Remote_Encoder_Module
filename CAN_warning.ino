void CANwarning() {
  // Lose synchronisation
  synchronised = false;

  // Warning delay
  _time = millis();
  if(_time - lastWarning > warningInterval)
  {
    // Save time
    lastWarning = _time;

    // Limit the number of warning reattempts
    if(warningNumber < warningLimit)
    {
      // Warn master ('W')
      CAN.beginPacket(localAddress);
      CAN.write(warning);
      CAN.endPacket();

      // Increment warning counter
      warningNumber++;
    }
    else
    {
      // Send error message once ('E')
      if(!errorMode)
      {
        CAN.beginPacket(localAddress);
        CAN.write(error);
        CAN.endPacket();
      }

      // Too many warnings - go into error mode
      errorMode = true;
    }
  }
}
