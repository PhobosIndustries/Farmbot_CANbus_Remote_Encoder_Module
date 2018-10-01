void decodeCAN()  {
  // Look for device ID or broadcast
  if((uint8_t)CAN.packetId() != localAddress && (uint8_t)CAN.packetId() != broadcastAddress)
  {
    // Not for this device, move on
    Serial.print("Message not for this device: PacketId = ");
    Serial.print(CAN.packetId());
    Serial.print(". First byte: ");
    Serial.println(CAN.peek());
    
    return;
  }
  else if(!initialised && (CAN.peek() != initialise))
  {
    // Catch all other message requests and declare that this device is uninitialised
    Serial.println("Uninitialised!");

    // Send request for initialisation
    CAN.beginPacket(localAddress);
    CAN.write(uninitialised);
    CAN.endPacket();

    return;
  }
  else
  {
    Serial.println("Message received");
    
    // Is this a basic Remote Transmission Request
    if(CAN.packetRtr())
    {
      Serial.println("RTR received");
      // Build message
      buildMessage(position/4);

      // Send return message
      CAN.beginPacket(localAddress);
      CAN.write(packet, 4);
      CAN.endPacket();

      // Declare synchronisation
      synchronised = true;

      // Clear warning and error modes
      errorMode = false;
      warningNumber = 0;

      // Save position
      lastPosition = position;
      return;
    }
    else
    {
      // Message is an instruction
      instruction = CAN.read();
      
      switch(instruction)  {
        case initialise:
        case 'i': // Initialise
  
          Serial.println("Initialised");
          
          initialised = true;
          
          // Clear warning and error modes
          errorMode = false;
          warningNumber = 0;
  
          // Set encoder to 0
          rotaryEncoder.write(0);

          // Report to master with OK ('K')
          CAN.beginPacket(localAddress);
          CAN.write(OK);
          CAN.endPacket();

          // Declare synchronisation
          synchronised = true;
          
          break;
  
        case setPosition:
        case 's': // Set position
          // Read remaining data
  
          // Find the transmitted value (next 4 bytes)
          dataValue = CAN.read();
          dataValue = (dataValue << 8) + CAN.read();
          dataValue = (dataValue << 8) + CAN.read();
          dataValue = (dataValue << 8) + CAN.read();
  
          // Is the value within acceptable range
          if(dataValue >= minPosition && dataValue <= maxPosition)
          {
            // Set encoder to the new value
            rotaryEncoder.write(dataValue);

            // Declare synchronisation
            synchronised = true;

            // Report to master with OK ('K')
            CAN.beginPacket(localAddress);
            CAN.write(OK);
            CAN.endPacket();
          }
          else
          {
            // Reject out of bounds value

            // Report to master with error ('E')
            CAN.beginPacket(localAddress);
            CAN.write(error);
            CAN.endPacket();
          }
  
          break;

        /*
        case reset:
        case 'r': // Reset
          // Respond with OK message ('K')
          CAN.beginPacket(localAddress);
          CAN.write(OK);
          CAN.endPacket();

          // Reboot this controller
          // - method unknown -

          break;
        */
          
        default:
          Serial.print("Unknown instruction type: ");
          Serial.println(instruction);
          break;
      }
      
    }
  }
}


void buildMessage(int32_t value) {
  /*
  // Is current position positive, negative or zero
  if(value > 0)
  {
    // Positive
    packet[0] = (byte)(value >> 24);
    packet[1] = (byte)(value >> 16);
    packet[2] = (byte)(value >> 8);
    packet[3] = (byte)(value);
  }
  else if(value < 0)
  {
    // Negative
    packet[0] = ~(byte)(value >> 24);
    packet[1] = ~(byte)(value >> 16);
    packet[2] = ~(byte)(value >> 8);
    packet[3] = ~(byte)(value - 1);
  }
  else
  {
    // Zero
    packet[0] = 0;
    packet[1] = 0;
    packet[2] = 0;
    packet[3] = 0;
  }
  */
  packet[0] = (byte)(value >> 24);
  packet[1] = (byte)(value >> 16);
  packet[2] = (byte)(value >> 8);
  packet[3] = (byte)(value);
}
