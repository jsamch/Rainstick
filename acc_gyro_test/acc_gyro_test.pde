#include <Wire.h>
#include <i2c.h>

#define LIS331_ADDR_ACC 0x18
#define LIS331_DEVID 0x32
#define LIS331_ADDR_GYRO 0x68
#define L3G4200D_ID 0xD3


void setup() { 
  
  // Set up the built-in LED pin as an output:
  pinMode(BOARD_LED_PIN, OUTPUT);
     
  Serial1.begin(9600);
    
  SerialUSB.println("Hello"); 
  i2c_master_enable(I2C1,I2C_BUS_RESET | I2C_FAST_MODE);
         
  ls331_init();
  
}


void loop() {
  
  int16 x_a, y_a, z_a, x_g, y_g, z_g;
  float xf_g, yf_g, zf_g;
  
  int samplenum = 0;

  while(1)
  {
    toggleLED();
    delay(100);
    
    samplenum++;
    
    x_a = ls331_read(0x28, LIS331_ADDR_ACC);
    y_a = ls331_read(0x2A, LIS331_ADDR_ACC);
    z_a = ls331_read(0x2C, LIS331_ADDR_ACC);
    
    x_g = ls331_read(0x28, LIS331_ADDR_GYRO);
    y_g = ls331_read(0x2A, LIS331_ADDR_GYRO);
    z_g = ls331_read(0x2C, LIS331_ADDR_GYRO);
    
    xf_g = x_g*8.75/1000.0;
    yf_g = y_g*8.75/1000.0;
    zf_g = z_g*8.75/1000.0;
    
    SerialUSB.print(samplenum);
    SerialUSB.print(": Acc=[");
    SerialUSB.print(x_a,DEC);
    SerialUSB.print(",");
    SerialUSB.print(y_a,DEC);
    SerialUSB.print(",");
    SerialUSB.print(z_a,DEC);
    SerialUSB.print("],\t");
    SerialUSB.print("Gyro=[");
    SerialUSB.print(xf_g,1);
    SerialUSB.print(",");
    SerialUSB.print(yf_g,1);
    SerialUSB.print(",");
    SerialUSB.print(zf_g,1);
    SerialUSB.print("]\n");    
  }
}


void ls331_init()
{
  uint8 buf[2] = {0x0f, 0x0f};  
  uint8 dev_id;
  i2c_msg msg[1];

  msg[0].addr = LIS331_ADDR_ACC;
  msg[0].flags =0;
  msg[0].length =1;
  msg[0].data= buf;
  i2c_master_xfer(I2C1, msg, 1, 0);

  msg[0].addr = LIS331_ADDR_ACC;  
  msg[0].flags = I2C_MSG_READ;
  msg[0].length =1;
  msg[0].data= buf;
  i2c_master_xfer(I2C1, msg, 1, 0);
  dev_id = buf[0];  
  
  if (dev_id != LIS331_DEVID) {
    SerialUSB.println("Error, incorrect LIS331 devid!");
    SerialUSB.println("Halting program, hit reset...");
  }
  
  //ls331_write_acc(0x20, 0x2f);
  ls331_write(0x20, 0x2f, LIS331_ADDR_ACC);
  
  /* Take out of power down mode,  200 Hz sampling */
  ls331_write(0x20, 0x6f, LIS331_ADDR_GYRO);
  /* HPF Normal mode, HPCF3@0.05Hz*/
  ls331_write(0x21, 0x28, LIS331_ADDR_GYRO);
  /* HPF Enable */
  ls331_write(0x24, 0x11, LIS331_ADDR_GYRO);
}


void ls331_write(uint8 reg, uint8 data, int16 addr)
{
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[2];
 
  msg_data = {reg,data};
  msgs[0].addr = addr;
  msgs[0].flags = 0; // write
  msgs[0].length = 2;
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C1, msgs, 1,0); 
}


uint16 ls331_read(uint8 reg, int16 addr)
{
  uint8 buf[2] = {reg|0x80,0x80|reg+1};  
  i2c_msg msg[3];

  msg[0].addr = addr;
  msg[0].flags =0;
  msg[0].length = 1;
  msg[0].data= buf;

  msg[1].addr = addr;  
  msg[1].flags = I2C_MSG_READ;
  msg[1].length = 2;
  msg[1].data= buf;

  i2c_master_xfer(I2C1, msg, 2, 0);
 
  return buf[1]<<8|buf[0];  
}

