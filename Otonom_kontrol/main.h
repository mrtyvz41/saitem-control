/*
 * main.h
 *
 *  Created on: 17 Tem 2020
 *      Author:
 */
#ifndef MAIN_H_
#define MAIN_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/debug.h"
#include "driverlib/timer.h"

#define PWM_FREQUENCY 55


#define GPIO_PIN_LEFT           GPIO_PIN_7

#define GPIO_PIN_RIGHT          GPIO_PIN_4

#define GPIO_PIN_BRAKE          GPIO_PIN_3

#define GPIO_PIN_SPEEDUP        GPIO_PIN_6  //GPIO_PIN_6

#define GPIO_PIN_SPEEDDOWN      GPIO_PIN_3 //GPIO_PIN_3

#define GPIO_PIN_ONOFF          GPIO_PIN_2

#define GPIO_PIN_RELAY          GPIO_PIN_2 // RELAY OUTPUT

//PWM CONFIGS //

#define Speed_output            PWM_OUT_5

#define Steer_output            PWM_OUT_4

#define Brake_output            PWM_OUT_2

#define Brake_PWM_PIN           GPIO_PIN_4

#define Steer_PWM_PIN           GPIO_PIN_4

#define Speed_PWM_PIN           GPIO_PIN_5

#define Brake_PWM_Config        GPIO_PB4_M0PWM2

#define Steer_PWM_Config        GPIO_PE4_M0PWM4

#define Speed_PWM_Config        GPIO_PE5_M0PWM5


bool buttonLeft = false;
bool stateButtonLeft = false;

bool buttonRight = false;
bool stateButtonRight = false;

bool buttonBrake = false;
bool stateButtonBrake = false;

bool buttonSpeedUp = false;
bool stateButtonSpeedup = false;

bool buttonSpeedDown = false;
bool stateButtonSpeedDown = false;

bool button_on_off = false;
bool stateButton_on_off = false;
bool relay_state = false;

int counterr = 0;
int elsecounter = 0;

int break_counter = 0;

bool break_flag = false;

uint32_t break_value;

bool steer_flag = false;

bool speed_flag = false;

bool tick = false; //timer variable

bool start_flag;

// VARIABLE //

volatile uint32_t lastvalue_brake = 26; //
volatile uint32_t lastvalue_steer = 26; //
volatile uint32_t lastvalue_speed = 1;  //

uint32_t ui32Load;           // PWM  doluluk oranini(Duty cycle) belirlemek �zere kullan�lan degisken
uint32_t ui32PWMClock;       // PWM osilatorunun clock degeri

char buffer[100];
int  uart_counter;
int  brake_counter;
int  steer_degree = 74;


int  speed = 1;
char rcv_ch;  // uart flag


// METOD DEFINES //
void interrupt_uart(void);
void control_gpio_init(void);
void pwm_init(void);
void uart_init(void);
void uart_parse(void);
void brake_control(uint32_t);
void steer_control(uint32_t);
void speed_control(uint32_t);
void gpio_interrupt_init(void);
void IntGPIOdHandler(void);
void init_timerHardware (void);
void IntTimer1 (void);


#endif /* MAIN_H_ */

