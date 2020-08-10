#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"

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

#include <string.h>

typedef struct delays{

    bool state;
    uint8_t count;
    uint8_t set;
    uint32_t counter;

}Delay;

Delay isSpeedUpTimer;
Delay isSpeedDownTimer;
Delay isLeftSteerTimer;
Delay isRightSteerTimer;


typedef struct buttons{

    bool value;
    bool state;

}Button;

Button isSpeedUpBtn;
Button isSpeedDownBtn;
Button isBreakBtn;
Button isSteerLeftBtn;
Button isSteerRightBtn;


bool tick = false;
uint16_t speed = 0;
uint16_t SteerPos = 74;
uint16_t BreakPos = 26;

int brake_counter = 0;

uint32_t ui32Load;
uint32_t ui32PWMClock;

void IntTimer1 ( void ){
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    tick = true;
}

void initHandle ( void ){

//** button config **//
    SysCtlPeripheralEnable(PERIPH_SPEED_NEG);
    GPIOPinTypeGPIOInput(BASE_SPEED_NEG, PIN_SPEED_NEG);
    GPIOPadConfigSet(BASE_SPEED_NEG, PIN_SPEED_NEG, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    SysCtlPeripheralEnable(PERIPH_SPEED_POS);
    GPIOPinTypeGPIOInput(BASE_SPEED_POS, PIN_SPEED_POS);
    GPIOPadConfigSet(BASE_SPEED_POS, PIN_SPEED_POS, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    SysCtlPeripheralEnable(PERIPH_BREAK);
    GPIOPinTypeGPIOInput(BASE_BREAK, PIN_BREAK);
    GPIOPadConfigSet(BASE_BREAK, PIN_BREAK, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    SysCtlPeripheralEnable(PERIPH_STEER_LEFT);
    GPIOPinTypeGPIOInput(BASE_STEER_LEFT, PIN_STEER_LEFT);
    GPIOPadConfigSet(BASE_STEER_LEFT, PIN_STEER_LEFT, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    SysCtlPeripheralEnable(PERIPH_STEER_RIGHT);
    GPIOPinTypeGPIOInput(BASE_STEER_RIGHT, PIN_STEER_RIGHT);
    GPIOPadConfigSet(BASE_STEER_RIGHT, PIN_STEER_RIGHT, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    SysCtlPeripheralEnable(PERIPH_ON_OFF);
    GPIOPinTypeGPIOInput(BASE_ON_OFF, PIN_ON_OFF);
    GPIOPadConfigSet(BASE_ON_OFF, PIN_ON_OFF, GPIO_STRENGTH_6MA, GPIO_PIN_TYPE_STD_WPU);

    SysCtlPeripheralEnable(PERIPH_RELAY);
    GPIOPinTypeGPIOInput(BASE_RELAY, PIN_RELAY);
    GPIOPadConfigSet(BASE_RELAY, PIN_RELAY, GPIO_STRENGTH_6MA, GPIO_PIN_TYPE_STD_WPU);

    SysCtlPeripheralEnable(PERIPH_RELAY_SINYAL);
    GPIOPinTypeGPIOOutput(BASE_RELAY_SINYAL, PIN_RELAY_SINYAL);
    GPIOPadConfigSet(BASE_RELAY_SINYAL, PIN_RELAY_SINYAL, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

//** button config end **//

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinConfigure(GPIO_PE5_M0PWM5);

    GPIOPinTypePWM( GPIO_PORTB_BASE, GPIO_PIN_4 );
    GPIOPinTypePWM( GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5 );

    PWMGenConfigure(PWM0_BASE, PWM_GEN_1 , PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2 , PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    ui32PWMClock = SysCtlClockGet()/64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 400); //Max Voltaj degeri
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32Load);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 0);

    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT , false);
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT , false);
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT , false);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/20);
    TimerIntRegister(TIMER1_BASE, TIMER_A, IntTimer1);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER1_BASE, TIMER_A);

    /**** UART 3 CONFIG ******/
    SysCtlPeripheralEnable( SYSCTL_PERIPH_UART3);
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOC);

    GPIOPinConfigure( GPIO_PC7_U3TX);
    GPIOPinConfigure( GPIO_PC6_U3RX);
    GPIOPinTypeUART( GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    IntDisable(INT_UART3);
    UARTDisable( UART3_BASE);
    UARTClockSourceSet( UART3_BASE, UART_CLOCK_PIOSC );

    UARTConfigSetExpClk( UART3_BASE, 16000000, 9600 , UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE );
    UARTIntEnable( UART3_BASE, UART_INT_RX | UART_INT_RT );

    UARTIntRegister( UART3_BASE, IntXbee );
    UARTEnable( UART3_BASE );





    IntEnable( INT_UART3);
    //*** END **//


}
void IntXbee(void){

    char rcv_data;
    uint32_t xbee_status;
    xbee_status = UARTIntStatus(UART3_BASE, true);

    UARTIntClear(UART3_BASE, xbee_status);
    rcv_data = UARTCharGetNonBlocking(UART3_BASE);

    if((uint32_t)rcv_data != (-1)){

        xbee_buffer[uart_counter++] = rcv_data;


        if(xbee_buffer[0] == '#'){

            if(xbee_buffer[2] == '$'){
                uart_counter = 0;
                memset(xbee_buffer, 0, sizeof(xbee_buffer));
            }
            if(xbee_buffer[1] == 'i'){
                 speed_up();
            }
            if(xbee_buffer[1] == 'g'){
                speed_down();
            }
            if(xbee_buffer[1] == 'a'){
                turn_left();
            }
            if(xbee_buffer[1] == 's'){
                turn_right();
            }
            if(xbee_buffer[1] == 'f') brake();
        }
        else{
            uart_counter = 0;
            memset(xbee_buffer, 0, sizeof(xbee_buffer));
        }


    }

}
void brake(void){
   speed = 0;
   PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT , false);

   PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT , true);
   PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (32 * ui32Load/1000));

   SysCtlDelay(SysCtlClockGet()/3);


   PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (109 * ui32Load/1000));
   PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT , false);
}
void turn_right(void){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ((SteerPos-=16) * ui32Load/1000) );
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT , true);
}
void turn_left(void){
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ((SteerPos+=16) * ui32Load/1000) );
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT , true);
}
void speed_up(void){
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT , true);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (speed+=20) );
}
void speed_down(void){

    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT , true);
    speed-=20;
    if(speed > 5000){
            speed = 0;
        }
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, speed );

}
void ButtonState ( void ){

    if(!GPIOPinRead(BASE_RELAY, PIN_RELAY)){
            relay_state = true;
        }
        else{
            relay_state = false;
        }


    if(isSpeedUpBtn.value != GPIOPinRead(BASE_SPEED_POS, PIN_SPEED_POS)){

         if(!(isSpeedUpBtn.value = GPIOPinRead(BASE_SPEED_POS, PIN_SPEED_POS))){

             isSpeedUpTimer.set = 4;

             isSpeedUpBtn.state = false;
         }else{
             if( isSpeedUpBtn.state == false){

                 isSpeedUpTimer.set = 0;
                 isSpeedUpBtn.state = true;
             }
         }
     }

    if(isSpeedDownBtn.value != GPIOPinRead(BASE_SPEED_NEG, PIN_SPEED_NEG)){

         if(!(isSpeedDownBtn.value = GPIOPinRead(BASE_SPEED_NEG, PIN_SPEED_NEG))){

             isSpeedDownTimer.set = 4;

             isSpeedDownBtn.state = false;
         }else{
             if( isSpeedDownBtn.state == false){

                 isSpeedDownTimer.set = 0;
                 isSpeedDownBtn.state = true;
             }
         }
     }


    /***BRAKE***/
    if(!GPIOPinRead(BASE_BREAK, PIN_BREAK)){
        ///****///
        if(brake_counter >= 2000){
              speed = 0;
              PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT , false);

              PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (32 * ui32Load/1000));
              PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT , true);

              isBreakBtn.state = false;
         }

         brake_counter++;
    }
    else{

        //if(!stateButtonBrake){
            /*CODES*/
                         PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (109 * ui32Load/1000));
                         PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT , true);

                         brake_counter = 0;
        //}
    }

//    if(isBreakBtn.value != GPIOPinRead(BASE_BREAK, PIN_BREAK)){
//
//         if(!(isBreakBtn.value = GPIOPinRead(BASE_BREAK, PIN_BREAK)) ){
//
//             if(brake_counter >= 3){
//                     speed = 0;
//                  PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT , false);
//
//                  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (32 * ui32Load/1000));
//                  PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT , true);
//
//                  isBreakBtn.state = false;
//             }
//
//             brake_counter++;
//         }
//         else{
//             if( isBreakBtn.state == false){
//
//                 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (109 * ui32Load/1000));
//                 PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT , true);
//
//                 brake_counter = 0;
//
//                 isBreakBtn.state = true;
//             }
//         }
//     }

    if(isSteerLeftBtn.value != GPIOPinRead(BASE_STEER_LEFT, PIN_STEER_LEFT)){

         if(!(isSteerLeftBtn.value = GPIOPinRead(BASE_STEER_LEFT, PIN_STEER_LEFT))){
             isLeftSteerTimer.set = 1;
             isSteerLeftBtn.state = false;
         }else{
             if( isSteerLeftBtn.state == false){
                 isLeftSteerTimer.set = 0;
                 isSteerLeftBtn.state = true;
             }
         }
     }

    if(isSteerRightBtn.value != GPIOPinRead(BASE_STEER_RIGHT, PIN_STEER_RIGHT)){

         if(!(isSteerRightBtn.value = GPIOPinRead(BASE_STEER_RIGHT, PIN_STEER_RIGHT))){
             isRightSteerTimer.set = 1;
             isSteerRightBtn.state = false;
         }else{
             if( isSteerRightBtn.state == false){
                 isRightSteerTimer.set = 0;
                 isSteerRightBtn.state = true;
             }
         }
     }
}

int main(void)

{
    SysCtlClockSet( SYSCTL_SYSDIV_4
                    | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ );

    initHandle();

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (SteerPos * ui32Load/1000) );
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT , true);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (BreakPos * ui32Load/1000) );
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT , true);

    isSpeedUpTimer.set = 0;
    isSpeedDownTimer.set = 0;
    isLeftSteerTimer.set = 0;
    isRightSteerTimer.set = 0;




	while(1){

	    ButtonState();
	    if(relay_state){
            GPIOPinWrite(BASE_RELAY_SINYAL, PIN_RELAY_SINYAL, PIN_RELAY_SINYAL);
        }
        else{
            GPIOPinWrite(BASE_RELAY_SINYAL, PIN_RELAY_SINYAL, 0);
        }

        if(tick){
            tick = false;

            if( isSpeedUpTimer.set != 0 ){
                if( ++isSpeedUpTimer.count >= isSpeedUpTimer.set ){
                    isSpeedUpTimer.count = 0;
                    isSpeedUpTimer.state = !isSpeedUpTimer.state;

                    if( isSpeedUpTimer.state ){
                        if( speed < 400 ){
                            speed_up();
                        }else{
                            isSpeedUpTimer.set = 0;
                        }
                    }
                }
            }

            if( isSpeedDownTimer.set != 0 ){
                if( ++isSpeedDownTimer.count >= isSpeedDownTimer.set ){
                    isSpeedDownTimer.count = 0;
                    isSpeedDownTimer.state = !isSpeedDownTimer.state;

                    if( isSpeedDownTimer.state ){
                        if( speed > 10 ){
                            speed_down();
                        }else{
                            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0 );
                            PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT , false);
                            isSpeedDownTimer.set = 0;
                        }
                    }
                }
            }


            /* STEER */

            if( isLeftSteerTimer.set != 0 ){
                if( ++isLeftSteerTimer.count >= isLeftSteerTimer.set ){
                    isLeftSteerTimer.count = 0;
                    isLeftSteerTimer.state = !isLeftSteerTimer.state;

                    if( isLeftSteerTimer.state ){

                        if( SteerPos < 134 ){
                            turn_left();
                        }else{
                            isRightSteerTimer.set = 0;
                        }
                    }
                }
            }

            if( isRightSteerTimer.set != 0 ){
                if( ++isRightSteerTimer.count >= isRightSteerTimer.set ){
                    isRightSteerTimer.count = 0;
                    isRightSteerTimer.state = !isRightSteerTimer.state;

                    if( isRightSteerTimer.state ){
                        if( SteerPos > 26 ){
                            turn_right();
                        }else{
                            isLeftSteerTimer.set = 0;
                        }
                    }
                }
            }
        }


	}

}
