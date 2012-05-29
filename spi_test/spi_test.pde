// Use SPI port number 1
HardwareSPI spi(1);

void setup() {
  // Turn on the SPI port
  // Frequency: 18MHZ (fastest available)
  // Bit Order: Big-endian
  // Mode: CPOL = 0, CPHA = 1 (high clock polarity, falling-edge trigger)
  spi.begin(SPI_18MHZ, MSBFIRST, SPI_MODE_2);
}

void loop() {
  // Send 245 over SPI, and wait for a response.
  spi.write(1);
  spi.write(2);
  spi.write(3);
  spi.write(4);
  byte response = spi.read();
  // Print out the response received.
  SerialUSB.print("response: ");
  SerialUSB.println(response, DEC);
  
  response = 0;
  
  toggleLED();
  delay(100);
}
