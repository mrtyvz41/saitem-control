
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

char xbee_buffer[200];
int uart_counter = 0;
void IntXbee(void);
/****/
void speed_up(void);
void speed_down(void);
void turn_left(void);
void turn_right(void);
void brake(void);

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
