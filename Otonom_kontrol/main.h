
#ifndef MAIN_H_
#define MAIN_H_


#define PWM_FREQUENCY 55

#define PERIPH_SPEED_NEG            SYSCTL_PERIPH_GPIOB
#define BASE_SPEED_NEG              GPIO_PORTB_BASE
#define PIN_SPEED_NEG               GPIO_PIN_3

#define PERIPH_SPEED_POS            SYSCTL_PERIPH_GPIOB
#define BASE_SPEED_POS              GPIO_PORTB_BASE
#define PIN_SPEED_POS               GPIO_PIN_6

#define PERIPH_BREAK                SYSCTL_PERIPH_GPIOA //A PORT
#define BASE_BREAK                  GPIO_PORTA_BASE // A PORT
#define PIN_BREAK                   GPIO_PIN_2 // PIN 2

#define PERIPH_STEER_LEFT           SYSCTL_PERIPH_GPIOB
#define BASE_STEER_LEFT             GPIO_PORTB_BASE
#define PIN_STEER_LEFT              GPIO_PIN_7

#define PERIPH_STEER_RIGHT          SYSCTL_PERIPH_GPIOA
#define BASE_STEER_RIGHT            GPIO_PORTA_BASE
#define PIN_STEER_RIGHT             GPIO_PIN_4

#define PERIPH_ON_OFF               SYSCTL_PERIPH_GPIOA
#define BASE_ON_OFF                 GPIO_PORTA_BASE
#define PIN_ON_OFF                  GPIO_PIN_3

#define PERIPH_RELAY               SYSCTL_PERIPH_GPIOC
#define BASE_RELAY                 GPIO_PORTC_BASE
#define PIN_RELAY                  GPIO_PIN_4

#define PERIPH_RELAY_SINYAL        SYSCTL_PERIPH_GPIOB
#define BASE_RELAY_SINYAL          GPIO_PORTB_BASE
#define PIN_RELAY_SINYAL           GPIO_PIN_2

char xbee_buffer[200];
char main_buffer[200];
int uart_counter = 0;
int uart_counter_m = 0;
bool autonom_state = false;
bool relay_state = false;
bool relay_state_u = false;
int steer_degree = 0;
void IntXbee(void);
void mainUart(void);
/****/
void speed_up(void);
void speed_down(void);
void turn_left(void);
void turn_right(void);
void brake(void);
void relay_on(void);
void relay_off(void);
    //
    //                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 200);
    //                PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT , true);
    //
    //                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 200);
    //                PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT , true);
    //
    //                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 200);
    //                PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT , true);

#endif /* MAIN_H_ */
