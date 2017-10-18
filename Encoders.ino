void initEncoders() {
  
   // Set slave selects as outputs
   pinMode(slaveSelectEnc1, OUTPUT);
   pinMode(slaveSelectEnc2, OUTPUT);
   pinMode(slaveSelectEnc3, OUTPUT);
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
   digitalWrite(slaveSelectEnc1,HIGH);
   digitalWrite(slaveSelectEnc2,HIGH);
   digitalWrite(slaveSelectEnc3,HIGH);
   SPI.begin();
  
   digitalWrite(slaveSelectEnc1,LOW);        // Begin SPI conversation
   SPI.transfer(0x88);                       // Write to MDR0
   SPI.transfer(0x03);                       // Configure to 4 byte mode
   digitalWrite(slaveSelectEnc1,HIGH);       // Terminate SPI conversation 


   digitalWrite(slaveSelectEnc2,LOW);        // Begin SPI conversation
   SPI.transfer(0x88);                       // Write to MDR0
   SPI.transfer(0x03);                       // Configure to 4 byte mode
   digitalWrite(slaveSelectEnc2,HIGH);       // Terminate SPI conversation 
  
  
   digitalWrite(slaveSelectEnc3,LOW);        // Begin SPI conversation
   SPI.transfer(0x88);                       // Write to MDR0
   SPI.transfer(0x03);                       // Configure to 4 byte mode
   digitalWrite(slaveSelectEnc3,HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder) {
  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;  
  
  // Read encoder 1
  if (encoder == 1) 
  { 
    
    digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  }
  
  // Read encoder 2
  else if (encoder == 2) {
    digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                      // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
  }
  //Read Encoder 3
  else if (encoder == 3) {
    digitalWrite(slaveSelectEnc3,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                      // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc3,HIGH);     // Terminate SPI conversation 
  }
  
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  
  return count_value;
}

void clearEncoderCount() {
    
  // Set encoder1's data register to 0
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(50);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation   
  
  // Set encoder2's data register to 0
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(50);  // provides some breathing room between SPI conversations
  
  // Set encoder2's current data register to center
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
  
  
  // Set encoder3's data register to 0
  digitalWrite(slaveSelectEnc3,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc3,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(50);  // provides some breathing room between SPI conversations
  
  // Set encoder3's current data register to center
  digitalWrite(slaveSelectEnc3,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc3,HIGH);     // Terminate SPI conversation   
}
