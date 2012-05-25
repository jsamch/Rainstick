#include "gravity.h"
#include <math.h>
#include "wavelet.h"

  static unsigned int tick_counter, sec_counter = 0;
  static unsigned int error_counter = 0;
  static int enable_print = 0;
  static int print_ticks = 0;
  
  static float sin_theta_mid = -0.440;
  static float sin_theta_raw = 0.0;
  
  static int acc, acc_old;
  static int speed, speed_old, distance = 0; 
  static unsigned int normspeed = 0;
  static unsigned int end_reached = 0;
  static float temp_float1, sin_theta;
  static int wavelet_ind = 0;
  
  
  void update_physics() {
  	
    int bounce, mass;
    int length;
    
    length = get_reallength();
    bounce = get_bounce();
    mass = get_mass();
    	
    acc_old = acc;
    speed_old = speed;
  
    // update acceleration based on x axis acceleration
    sin_theta = -(float)1/17000;
    acc = sin_theta*7*MULT_FACTOR;
    
    // update speed. 1/UPDATE_RATE is time
    speed = speed + (acc)/(UPDATE_RATE);
  
    // update distance. 1/UPDATE_RATE is time
    distance = distance + (speed)/(UPDATE_RATE);
  
    if ( (distance > length || distance < 0) ||
      ((distance == length || distance == 0) && end_reached == 0))  {
        	    
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
        /*DAC12_0DAT = mass*normspeed;*/
      }
      speed = speed*bounce*(-0.1);
      
      end_reached = 1;
  	        	        
    } else if ( distance < length && distance > 0) {
      
      // rolling noise here
      
      end_reached = 0;
      normspeed = 0;
      /*DAC12_0DAT = get_rolling()*get_pot()*wavelet[wavelet_ind];*/          
  	        
    } else {
      /*DAC12_0DAT = 0; // prevent tick when starts to roll?*/
    }
      
    wavelet_ind = (distance)%WAVELET_LEN; 
  }
  
  
  
  /*void print_debugdata() {
  
  	// Printing and Display
  	//if ( (print_ticks == 1 || speed) && enable_print ) {
  	if ( enable_print ) {
  	      
  	  int sin_theta_int = (int)(sin_theta*MULT_FACTOR);
  	  int sin_theta_mid_int = (int)(sin_theta_mid*MULT_FACTOR);
  	  int short_acc = (int)acc;
  	  printf("\n");
  	  if (enable_print) {
  	    printf("sin:%6d mid:%3d a:%6d v:%5d d:%5d i:%2d e:%2d",
  	            sin_theta_int, sin_theta_mid_int, 
  	            short_acc, speed, distance, wavelet_ind,
  	            error_counter);
  	    //if (normspeed)
  	      //printf("norm:%3d", normspeed);
  	  }
  	    //printf("e:%2d", error_counter);
  	}
  }*/
  
  
  /*interrupt (TIMERA1_VECTOR) TIMERA_ISR(void) {
    static unsigned int rising_cap, falling_cap = 0;
    int timer_ccr;
  
  		TACTL &= ~TAIFG;
  		if (tick_counter == UPDATE_RATE-1) {
  		  tick_counter = 0; // return to zero every second
            sec_counter++;
            print_debugdata();
  		} else {
  		  tick_counter++;
  		}
  		
  		// calculate physics [UPDATE_RATE] times per second
  	    update_physics();
  
  	    if ((TACTL & TAIFG) != 0)
  	    	error_counter++;
  }*/
  
  
  
  unsigned int get_tick_counter() {
  	return tick_counter;
  }
  
  unsigned int get_sec_counter() {
  	return sec_counter;
  }
  
  unsigned int get_error_counter() {
  	unsigned int error_local = error_counter;
  	reset_error_counter();
  	return error_local;
  }
  
  void reset_error_counter() {
      _DINT();
      error_counter = 0;
      _EINT();
  }
 
  void toggle_enableprint() {
      enable_print = ~enable_print;
  }
  void set_zero(void)	{
  	sin_theta_mid = sin_theta_raw;
  }
  
  long int get_acc(void) {
  	return acc/(MULT_FACTOR/1000);
  }
  // returns 1000*sin(theta)
  int get_sin_angle(void) {
  	return (int)(sin_theta*1000);
  }
  // returns distance of the ball in mm
  int get_distance(void) {
  	return (int)(distance/(MULT_FACTOR/1000));
  }
