#include <Wire.h>
#include <i2c.h>
#include "gravity.h"
#include "wavelet.h"

#define LIS331_ADDR_ACC 0x18
#define LIS331_ADDR_GYRO 0x68
#define LIS331_DEVID 0x32


//--------------------------------------------------
// MAIN  
//--------------------------------------------------
static int length = 300*(MULT_FACTOR/1000);
static int friction = 0;
static int bounce = 3;   // use 1/10 of this value
static int mass = 16;
static int rolling_noise = 1;
static int output_video = 0;
static int pot; // potentiometer, reads 0-3975, but gives 0-10 to outside
static int record = 0;
uint32 start_time = 0, end_time, delay_micros;
int time_error;
int samplenum = 0;
//--------------------------------------------------


//--------------------------------------------------
// I/O  
//--------------------------------------------------
int ledPin1 = 17;
int ledPin2 = 18;
int ledPin3 = 19;
int ledPin4 = 20;
//--------------------------------------------------


//--------------------------------------------------
// PHYSICS  
//--------------------------------------------------
static unsigned int tick_counter, sec_counter = 0;
static unsigned int error_counter = 0;
static int enable_print = 0;
static int print_ticks = 0;
static int acc, acc_old;
static int speed = 0, speed_old;
static float distance = 0; 
static unsigned int normspeed = 0;
static unsigned int end_reached = 0;
static float temp_float1, sin_theta;
static int wavelet_ind = 0;
static int output = 0;
int acc_offset = 0;
int calibrate_samplenum;
boolean calibrate_flag = 0;
//--------------------------------------------------
 
 
//--------------------------------------------------
// SENSOR DATA  
//--------------------------------------------------
int16 x_a, y_a, z_a, x_g, y_g, z_g;
float xf_g, yf_g, zf_g;
//-------------------------------------------------- 


//--------------------------------------------------
// SPI/DAC
//--------------------------------------------------
int ssPin = 10;
int ldacPin = 12;
uint16 spiData = 0x7FFF;
// Use SPI port number 1
HardwareSPI spi(1);
//-------------------------------------------------- 


//--------------------------------------------------
// TIMER INTERRUPTS
//--------------------------------------------------
HardwareTimer timer(2);
//--------------------------------------------------


//--------------------------------------------------
// MAIN
//--------------------------------------------------
   
void setup() { 
  
  // Set up LEDs and SPI pins as outputs
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(ledPin1, OUTPUT);  
  pinMode(ledPin2, OUTPUT);  
  pinMode(ledPin3, OUTPUT);  
  pinMode(ledPin4, OUTPUT);  
  pinMode(ssPin, OUTPUT);  
  pinMode(ldacPin, OUTPUT);  
     
  // Set up serial port
  Serial1.begin(115200);
  
  // Set up I2C
  i2c_master_enable(I2C1,I2C_BUS_RESET | I2C_FAST_MODE);       
  ls331_init();
  
  // Set up SPI
  // NOTE: The Slave Select delay in spi_writeToDac is hard-coded based on the clock frequency.  Changing the clock frequency
  // from 2.25MHz will require adjusting the delay as indicated in spi_writeToDac.
  spi.begin(SPI_2_25MHZ, MSBFIRST, 2);
  digitalWrite(ssPin, HIGH);  
  digitalWrite(ldacPin, HIGH);  
  spi_init();
  
  // Set up Timer Interrupts
  // Pause the timer while we're configuring it
  timer.pause();
  // Set up period
  timer.setPeriod(1000000/UPDATE_RATE); // in microseconds
  // Set up an interrupt on channel 1
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  timer.attachCompare1Interrupt(handler);
  // Refresh the timer's count, prescale, and overflow
  timer.refresh();
  // Start the timer counting
  timer.resume();
  
  digitalWrite(ledPin1, HIGH);  
  digitalWrite(ledPin2, LOW);  
  digitalWrite(ledPin3, HIGH);  
  digitalWrite(ledPin4, HIGH);  
}

void loop() {

  while(1)
  {
    
    delayMicroseconds(1000000/UPDATE_RATE);
    toggleLED();
    
    // UPDATE RATE CONTROL (replaced by timer interrupts)
    /*time_error = start_time - samplenum*4000;
    end_time = micros();
    delay_micros = end_time - start_time;
    delayMicroseconds(1000000/UPDATE_RATE - delay_micros - time_error);
    start_time = micros();*/
    
    // check the console for and inputted character
    update_terminal_input();
    
    // terminal output
    if(samplenum % 1 == 0)
    {
      // SAMPLING/UPDATE RATE TEST OUTPUT
      /*SerialUSB.print(samplenum);
      SerialUSB.print(": start = ");
      SerialUSB.print(start_time);      
      //SerialUSB.print(", \terror = ");
      //SerialUSB.print(time_error);  
      SerialUSB.print(", \tdelay = ");
      SerialUSB.print(delay_micros);     
      SerialUSB.print("]\n");*/
      
      // PHYSICS PLOT OUTPUT (rolling noise, speed, distance)
      SerialUSB.print(output);
      SerialUSB.print(", ");
      SerialUSB.print(speed);   
      SerialUSB.print(", ");
      SerialUSB.print(distance);      
      SerialUSB.print("\n");
      
      // PHYSICS TEST OUTPUT
      /*SerialUSB.print(samplenum);
      SerialUSB.print(": sin_theta = ");
      SerialUSB.print(sin_theta);      
      SerialUSB.print(",    \tacc = ");
      SerialUSB.print(acc);
      SerialUSB.print(", \tspeed = ");
      SerialUSB.print(speed);   
      SerialUSB.print(", \tdist = ");
      SerialUSB.print(distance);      
      SerialUSB.print("]\n");*/
      
      // GYRO & ACC TEST OUTPUT
      /*SerialUSB.print(samplenum);
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
      SerialUSB.print("]\n"); */
    }
  }
}

// (END MAIN)
//--------------------------------------------------


//--------------------------------------------------
// INTERRUPT HANDLING
//--------------------------------------------------

void handler(void) {
  
  samplenum++;
    
  // ACC READINGS
  x_a = ls331_read(0x28, LIS331_ADDR_ACC);
  y_a = ls331_read(0x2A, LIS331_ADDR_ACC);
  z_a = ls331_read(0x2C, LIS331_ADDR_ACC);
    
  // GYRO READINGS
  x_g = ls331_read(0x28, LIS331_ADDR_GYRO);
  y_g = ls331_read(0x2A, LIS331_ADDR_GYRO);
  z_g = ls331_read(0x2C, LIS331_ADDR_GYRO);
    
  xf_g = x_g*8.75/1000.0;
  yf_g = y_g*8.75/1000.0;
  zf_g = z_g*8.75/1000.0;
   
  update_physics();
  
}

// (END INTERRUPT HANDLING)
//--------------------------------------------------


//--------------------------------------------------
// MENUS
//--------------------------------------------------

void update_terminal_input() 
{
  
  if (SerialUSB.available() != 0) {
    int new_rcvd_char;
    
    new_rcvd_char = SerialUSB.read();
    
    switch (new_rcvd_char) {
      case 'c':
        calibrate();
        break;
      /*
      case 'm':
        main_menu();
        break;
      case 'q':
        set_alltestsequence();
        print_testsequence();
        break;
      case 't':
        start_testcases();
        break;
      case 'p':
     	print_debugdata();
        break;
      case 'i':
      	put_mass(mass);
        printf("  Impact ON\n");
        break;
      case 'e':
      	printf("  err: [%d]\n", get_error_counter());
        break;
      case 'v':
      	toggle_outputvideo();
        break;
      case 'o':
        printf("pot: raw %d, reads %d\n", pot, get_pot());
    	break;
      case 'r':
      	printf("real ball\n");
      	rolling_noise = 0;
      	mass = 0;
        break;
      case 's':
      	printf("simulated ball\n");
      	rolling_noise = 1;
      	mass = 16;
        break;
      case 'l':
      	printf("set length: ");
      	ret_val = getInteger3(&length_local);
      	if (ret_val == 0)
        {
      	  put_length(length_local);
          printf("\n%d\n",get_length());
      	} 
        else
        {
      	  printf("error getting integer!\n");
        }
        break;
      case 'h':
      	printf("haptic cues:");
      	ret_val = getInteger3(&cues);
        if (ret_val == 0) 
        {
          printf("\n%d\n",cues);
          switch(cues) 
          {
            // 1 for impact only, 2 for rolling only, 3 for both
      	    case 1: mass = 16; rolling_noise = 0; break;
            case 2: mass = 0;  rolling_noise = 1; break;
      	    case 3: mass = 16; rolling_noise = 1; break;
      	    default: break;        	
          }
      	} 
        else
        {
          printf("error getting integer!\n");
        }
        break;    	
      case 'z':
      	record = 1;
      	printf("\nstart recording\n");
        break;
      case 'x':
      	record = 0;
    	printf("\nstop recording\n");        	
        break;
      case 'b':
      	printf("bounce:");
      	ret_val = getInteger3(&local_bounce);
        if (ret_val == 0) {
           printf("\n%d\n",local_bounce);
      	} else
      	  printf("error getting integer!\n");
        put_bounce(local_bounce);
        break;*/
      default:
      	SerialUSB.print("m for menu, q for sequencing, t for testcases, \np for debug print, e for error, v for video, o for pot\n");
      	SerialUSB.print("r for real, s for simulate, \n l+nb for length (nb in multiples of 20), h+1/2/3 for impact, rolling or both\n");
     	SerialUSB.print("b for turn bounce on/off\n");
     	SerialUSB.print("z to start recording, x to stop recording\n");
        print_params();
        break;
    }
  }   
}

// (END MENUS)
//--------------------------------------------------


//--------------------------------------------------
// PHYSICS
//--------------------------------------------------

void calibrate()
{
   calibrate_flag = 1;
   calibrate_samplenum = 0;
   acc_offset = 0;
}

void update_physics()
{
  acc_old = acc;
  speed_old = speed;

  // update acceleration based on x axis acceleration
  sin_theta = (float)x_a/17000;
  acc = sin_theta*7*MULT_FACTOR - acc_offset;
  
  // calibration
  if(calibrate_flag)
  {
    acc_offset = (int)((acc + calibrate_samplenum * acc_offset) / calibrate_samplenum);
    calibrate_samplenum++;
    if(calibrate_samplenum == UPDATE_RATE)
      calibrate_flag = 0;
  }
  
  // update speed. 1/UPDATE_RATE is time
  speed = speed + acc/UPDATE_RATE;

  // update distance. 1/UPDATE_RATE is time
  distance = distance + (float)speed/UPDATE_RATE;

  if ( (distance > length || distance < 0) ||
    ((distance == length || distance == 0) && end_reached == 0))  
  {
      	    
    // end reached
    
    // reset distance
    if (distance >= length) 
      distance = length;	        
    if (distance <= 0)
      distance = 0;
      
    // send impact
    if (end_reached == 1) {
      // end already reached, impact already sent, so return DAC to zero
      normspeed = 0;
    } else {
      // first time end reached
      normspeed = ABS(speed)/80;
      output = mass*normspeed;
    }
    speed = speed*bounce*(-0.1);
    
    end_reached = 1;
	        	        
  } 
  else if ( distance < length && distance > 0)
  {
    
    // rolling noise here
    
    end_reached = 0;
    normspeed = 0;
    output = wavelet[wavelet_ind];         
	        
  }
  else
  {
    output = 0; // prevent tick when starts to roll?
  }
    
  wavelet_ind = ((int)distance)%WAVELET_LEN; 
}

// (END PHYSICS)
//--------------------------------------------------


//--------------------------------------------------
// I2C
//--------------------------------------------------

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

// (END I2C)
//--------------------------------------------------


//--------------------------------------------------
// SPI
//--------------------------------------------------
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

// Ignores the 4 MSBs
void spi_writeToDac(uint16 val)
{
  val = val << 4;
  digitalWrite(ssPin, LOW);  
  // 0x07 --> write to input registers of both DACs (A and B)
  spi.write(0x07);
  spi.write(val >> 8);
  spi.write(val);
  //uint8 response = spi.read();
  // 6 microsecond delay to ensure that the Slave Select pin stays low until the 24th bit is transmitted
  // (only works with 2.25MHz clock frequency)
  delayMicroseconds(6);
  digitalWrite(ssPin, HIGH);
  digitalWrite(ldacPin, LOW);  
  delayMicroseconds(1);
  digitalWrite(ldacPin, HIGH);  
}

// (END SPI)
//--------------------------------------------------


//--------------------------------------------------
// UTILITIES
//--------------------------------------------------
void print_params(void)
{
  SerialUSB.print("\nLength: ");
  SerialUSB.print(length);
  SerialUSB.print("mm, Friction: ");
  SerialUSB.print(friction);
  SerialUSB.print(", Bounce: ");
  SerialUSB.print(bounce);
  SerialUSB.print("/10, Mass(imipact): ");
  SerialUSB.print(mass);
  SerialUSB.print("");
  SerialUSB.print(rolling_noise);
  SerialUSB.print("\n");
}
//--------------------------------------------------
