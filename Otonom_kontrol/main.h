/*
 * main.h
 *
 *  Created on: 17 Tem 2020
 *      Author: CELIL
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

// PWM FREQUENCY //
#define PWM_FREQUENCY 55

// CONTROLLER PINS //
#define GPIO_PIN_LEFT           GPIO_PIN_7

#define GPIO_PIN_RIGHT          GPIO_PIN_4

#define GPIO_PIN_BRAKE          GPIO_PIN_3

#define GPIO_PIN_SPEEDUP        GPIO_PIN_6

#define GPIO_PIN_SPEEDDOWN      GPIO_PIN_3

#define GPIO_PIN_ONOFF          GPIO_PIN_2


// RELAY PIN //
#define GPIO_PIN_RELAY          GPIO_PIN_2


// PWM CONFIGS //
#define Speed_output            PWM_OUT_5

#define Steer_output            PWM_OUT_4

#define Brake_output            PWM_OUT_2

#define Brake_PWM_PIN           GPIO_PIN_4

#define Steer_PWM_PIN           GPIO_PIN_4

#define Speed_PWM_PIN           GPIO_PIN_5

#define Brake_PWM_Config        GPIO_PB4_M0PWM2

#define Steer_PWM_Config        GPIO_PE4_M0PWM4

#define Speed_PWM_Config        GPIO_PE5_M0PWM5

// MAIN UART PINS & CONFIGS //
#define SYSCTL_PERIPH_MAIN_UART SYSCTL_PERIPH_UART0
#define MAIN_UART_INT           INT_UART0
#define MAIN_UART_BASE          UART0_BASE
#define MAIN_UART_RX            GPIO_PA0_U0RX
#define MAIN_UART_TX            GPIO_PA1_U0TX
#define MAIN_UART_PORT          GPIO_PORTA_BASE
#define MAIN_UART_RX_PIN        GPIO_PIN_0
#define MAIN_UART_TX_PIN        GPIO_PIN_1


// XBEE UART PINS & CONFIGS //
#define SYSCTL_PERIPH_XBEE_UART SYSCTL_PERIPH_UART3
#define XBEE_UART_INT           INT_UART3
#define XBEE_UART_BASE          UART3_BASE
#define XBEE_UART_RX            GPIO_PC6_U3RX
#define XBEE_UART_TX            GPIO_PC7_U3TX
#define XBEE_UART_PORT          GPIO_PORTC_BASE
#define XBEE_UART_RX_PIN        GPIO_PIN_6
#define XBEE_UART_TX_PIN        GPIO_PIN_7

// CONTROLLER BUTTONS //
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

uint32_t break_value;


// CONTROLLER BUTTONS FLAGS //
bool break_flag = false;

bool steer_flag = false;

bool speed_flag = false;

bool start_flag;

bool uart_flag = false; // If it's false xbee gonna be working vice verse main gonna be working.

// Timer flag //
bool tick = false;


///////////////////////////////////////
//                                   //
//             VARIABLES             //
//                                   //
///////////////////////////////////////

/*      PWM duty cycle values        */
volatile uint32_t lastvalue_brake = 26; //
volatile uint32_t lastvalue_steer = 26; //
volatile uint32_t lastvalue_speed = 1;  //


uint32_t ui32Load;           // PWM  doluluk oranini(Duty cycle) belirlemek uzere kullanilan degisken
uint32_t ui32PWMClock;       // PWM osilatorunun clock degeri


char buffer_main[100];
char buffer_xbee[100];
int  uart_counter_main;
int  uart_counter_xbee;
int  brake_counter;
int  steer_degree = 74;


int  speed = 1;
char rcv_ch;  // uart flag


///////////////////////////////////////
//                                   //
//         METOD DEFINES             //
//                                   //
///////////////////////////////////////
void control_gpio_init(void);
void pwm_init(void);
void interrupt_Main_uart(void);
void main_uart_init(bool);
void uart_parse(void);
void brake_control(uint32_t);
void steer_control(uint32_t);
void speed_control(uint32_t);
void gpio_interrupt_init(void);
void IntGPIOdHandler(void);
void init_timerHardware (void);
void IntTimer1 (void);
void xbee_uart_init(bool);
void interrupt_xbee_uart(void);


#endif /* MAIN_H_ */

