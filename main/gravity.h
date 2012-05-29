#ifndef _GRAVITY_H
#define _GRAVITY_H

/*#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <io.h>*/

#define ABS(a) (((a) < 0) ? -(a) : (a))
#define SGN(a) (((a) < 0) ? -(1) : (1))
#define SGN_F(a) (((a) < 0) ? -(1.0) : (1.0))


typedef enum{
  SLIDING = 0,
  ROLLING = 1
} moving_mode_t;


#define UPDATE_RATE 250 // number of update per sec
#define TESTCASE_NUMBER 15
#define MULT_FACTOR 5000


void gen_init(void);
void ADC12_init(void);
void DAC12_init(void);
void TA_init(unsigned int update_rate);
void TB_init(unsigned int update_rate);
void UART_init(void);

int uart_check_received(void);
void uart_reset_received(void);
int uart_get_received(void);
int putchar(int ch);
int getchar(void);
int getInteger3(int * rcvd_integer);


//main.c
void set_zero(void);
int get_length(void);      
void put_length(int ln);
int get_reallength(void);
int get_friction(void);
void put_friction(int fr);
int get_bounce(void);  
void put_bounce(int bn);
int get_mass(void);  
void put_mass(int m);
void rolling_on(void);
void rolling_off(void);
void set_sliding(void);
void set_rolling(void);
void toggle_outputvideo(void);
int get_pot(void);
int get_rolling(void);

void main_menu(void);

//utilities.c
void display_volume(unsigned int);
void print_params(void);
void flash_leds(void) ;
void treatButtons();

//testcases.c
void set_alltestsequence(void);
void print_testsequence(void);
void init_testsequence(void);
void start_testcases(void);

//physics.c
void update_physics(void);
unsigned int get_tick_counter();
unsigned int get_sec_counter();
unsigned int get_error_counter();
void reset_error_counter();
void toggle_enableprint();
void print_debugdata();
long int get_acc();
int get_sin_angle(void);
int get_distance(void);

#endif


