int ssPin = 10;
int ldacPin = 12;
int ledPin1 = 17;

// triangular wave variables
uint16 value = 0x7FFF;
int direction = 1;

// Use SPI port number 1
HardwareSPI spi(1);

void setup() {
  pinMode(ledPin1, OUTPUT);  
  // Turn on the SPI port
  spi.begin(SPI_2_25MHZ, MSBFIRST, 2);
  pinMode(ssPin, OUTPUT);  
  pinMode(ldacPin, OUTPUT);  
  digitalWrite(ledPin1, LOW);  
  digitalWrite(ssPin, HIGH);  
  digitalWrite(ldacPin, HIGH);  
  spi_init();
}

void loop() {
  
  // DC (constant) voltage
  /*spi_writeToDac(0x3FFF);
  delayMicroseconds(10);*/ 
  
  // Triangular wave
  if(value >= 0x8FFF)
    direction = -1;
  else if(value <= 0x6FFF)
    direction = 1;
  if(direction == 1)
    value += 0x0100;
  else
    value -= 0x0100;
  spi_writeToDac(value);
  delayMicroseconds(10);
  
}

void spi_init()
{
  // Software reset
  spi.write(0x28);  
  spi.write(0x00);  
  spi.write(0x01);
  // Set LDAC to Asynchronous mode
  spi.write(0x30);  
  spi.write(0x00);  
  spi.write(0x00);    
  // Enable Internal Reference
  /*spi.write(0x38);  
  spi.write(0x00);  
  spi.write(0x01);*/
}


void spi_writeToDac(uint16 val)
{
  digitalWrite(ssPin, LOW);  
  spi.write(0x07);
  spi.write(val >> 8);
  spi.write(val);
  //uint8 response = spi.read();
  delayMicroseconds(6);
  digitalWrite(ssPin, HIGH);
  digitalWrite(ldacPin, LOW);  
  delayMicroseconds(1);
  digitalWrite(ldacPin, HIGH);  
}
