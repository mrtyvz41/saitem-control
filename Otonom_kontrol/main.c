
/**
 * main.c
 * Created on: 17 Tem 2020
 * LAST_AUTOR: CELIL
 */

#include "main.h"

int main(void)
{

//Write here a code that can control the flag status to your own desire
////////////////////////////////////////////
    main_uart_init(uart_flag);
    xbee_uart_init(uart_flag);
///////////////////////////////////////////

    control_gpio_init();
    pwm_init();
    gpio_interrupt_init();
    init_timerHardware();
    steer_control(74);
    //speed_control(1);
    //GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_RELAY, GPIO_PIN_RELAY);
    while (1)
    {

//if(start_flag){

        buttonSpeedUp = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_SPEEDUP);
        buttonRight = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_RIGHT);
        buttonBrake = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_BRAKE);
        buttonLeft = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_LEFT);
        buttonSpeedDown = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_SPEEDDOWN);
        button_on_off = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_ONOFF);

        // PWMPulseWidthSet(PWM0_BASE, Speed_output, lastvalue_speed * ui32Load/1000);

///////////////***********************/////////////////////////////
        /***LEFT***/

        if (!buttonLeft)
        {
            ///****///
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 2);
            //if(stateButtonLeft){
            /*CODES*/
            steer_degree += 5;
            if (steer_degree > 134)
            {
                steer_degree = 134;
            }
            steer_control(steer_degree);

            //SysCtlDelay(SysCtlClockGet()/100);
            //}
            stateButtonLeft = false;

        }
        else
        {

            if (!stateButtonLeft)
            {
                /*CODES*/
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

            }
            stateButtonLeft = true;
        }

///////////////***********************/////////////////////////////
        /***RIGHT***/

        if (!buttonRight)
        {
            ///****///
            //if(stateButtonRight){
            /*CODES*/
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
            steer_degree -= 5;
            if (steer_degree < 26)
            {
                steer_degree = 26;
            }
            steer_control(steer_degree);

            //SysCtlDelay(SysCtlClockGet()/100);

            stateButtonRight = false;

            //}
        }
        else
        {
            if (!stateButtonRight)
            {
                /*CODES*/
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

                stateButtonRight = true;
            }
        }

/////////////////***********************/////////////////////////////
        /***BRAKE***/
        if (!buttonBrake)
        {
            ///****///
            if (break_counter > 2000)
            {
//                /*CODES*/
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 8);
                break_value = 32;
                brake_control(break_value);
                SysCtlDelay(SysCtlClockGet() / 100);
                stateButtonBrake = false;
            }
            break_counter++;

        }
        else
        {

            //if(!stateButtonBrake){
            /*CODES*/
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
            break_value = 109;
            brake_control(break_value);
            SysCtlDelay(SysCtlClockGet() / 100);

            //PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
            stateButtonBrake = true;
            break_counter = 0;
            //}
        }
//
/////////////////***********************/////////////////////////////
        /***SPEED+***/
        if (!buttonSpeedUp)
        {
            ///****///
            if (stateButtonSpeedup)
            {
                /*CODES*/

                speed += 50;
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, 12);

                if (speed > 1000)
                {
                    speed = 1;
                }
                speed_control(speed);
            }
            stateButtonSpeedup = false;

        }
        else
        {

            if (!stateButtonSpeedup)
            {
                /*CODES*/
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0);

            }
            stateButtonSpeedup = true;
        }
/////////////////***********************/////////////////////////////
        /***SPEED-***/
        if (!buttonSpeedDown)
        {
            /****/            //
            if (stateButtonSpeedDown)
            {
                /*CODES*/

                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, 10);

                speed -= 25;

                if (speed < 1)
                {
                    speed = 1;
                }
                speed_control(speed);
            }
            stateButtonSpeedDown = false;

        }
        else
        {
            if (!stateButtonSpeedDown)
            {
                /*CODES*/
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, 0);

            }
            stateButtonSpeedDown = true;
        }

    }
}

void pwm_init(void)
{

    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
                    | SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypePWM(GPIO_PORTB_BASE, Brake_PWM_PIN);
    GPIOPinTypePWM(GPIO_PORTE_BASE, Steer_PWM_PIN);
    GPIOPinTypePWM(GPIO_PORTE_BASE, Speed_PWM_PIN);

    GPIOPinConfigure(Speed_PWM_Config);
    GPIOPinConfigure(Steer_PWM_Config);
    GPIOPinConfigure(Brake_PWM_Config);

    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

    PWMGenConfigure(PWM0_BASE, PWM_GEN_2,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32Load);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);

    PWMPulseWidthSet(PWM0_BASE, Brake_output, 1);
    PWMPulseWidthSet(PWM0_BASE, Steer_output, 1);
    PWMPulseWidthSet(PWM0_BASE, Speed_output, 1);

    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);

    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

}
void main_uart_init(bool flag)
{
    if(flag == true)
    {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_MAIN_UART);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    IntDisable(MAIN_UART_INT);
    UARTDisable(MAIN_UART_BASE);

    GPIOPinConfigure(MAIN_UART_RX);
    GPIOPinConfigure(MAIN_UART_TX);
    GPIOPinTypeUART(MAIN_UART_PORT, MAIN_UART_RX_PIN | MAIN_UART_TX_PIN);

    UARTClockSourceSet(MAIN_UART_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(
            MAIN_UART_BASE, SysCtlClockGet(), 115200,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTFIFODisable(MAIN_UART_BASE);
    UARTIntEnable(MAIN_UART_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(MAIN_UART_BASE, interrupt_Main_uart);

    UARTEnable(MAIN_UART_BASE);
    IntEnable(MAIN_UART_INT);
    }
    else{
        IntDisable(MAIN_UART_INT);
        UARTDisable(MAIN_UART_BASE);
    }

}
void interrupt_Main_uart(void)
{
    uint32_t IntStatus;

    IntStatus = UARTIntStatus(MAIN_UART_BASE, true);
    UARTIntClear(MAIN_UART_BASE, IntStatus);

    buffer_main[uart_counter_main++] = UARTCharGetNonBlocking(MAIN_UART_BASE);;
    if (sizeof(buffer_main) == sizeof(char) * 6 && buffer_main[0] == '*')
    {
        on_load(buffer_main);
        uart_counter_main = 0;
    }
}
void on_load(char *message)
{
    steer_degree = atoi((char*) message[1]) * 10 + atoi((char*) message[2]);
    if (message[3] == '1')
    {
        break_flag = true;
    }
    else
    {
        break_flag = false;
    }
    speed = atoi((char*) message[4]) * 10 + atoi((char*) message[5]);
}

void xbee_uart_init(bool flag)
{
    if(flag == false)
    {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_XBEE_UART);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    IntDisable(XBEE_UART_INT);
    UARTDisable(XBEE_UART_BASE);

    GPIOPinConfigure(XBEE_UART_RX);
    GPIOPinConfigure(XBEE_UART_TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, XBEE_UART_RX_PIN | XBEE_UART_TX_PIN);

    UARTClockSourceSet(XBEE_UART_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(
            XBEE_UART_BASE, SysCtlClockGet(), 115200,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTFIFODisable(XBEE_UART_BASE);
    UARTIntEnable(XBEE_UART_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(XBEE_UART_BASE, interrupt_xbee_uart);
    UARTEnable(XBEE_UART_BASE);
    IntEnable(XBEE_UART_BASE);
    }
    else{
        IntDisable(XBEE_UART_INT);
        UARTDisable(XBEE_UART_BASE);
    }
}
void interrupt_xbee_uart(void)
{
    uint32_t IntStatus;

    IntStatus = UARTIntStatus(XBEE_UART_BASE, true);
    UARTIntClear(XBEE_UART_BASE, IntStatus);

    buffer_xbee[uart_counter_xbee++] = UARTCharGetNonBlocking(XBEE_UART_BASE);;
    if (sizeof(buffer_xbee) == sizeof(char) * 6 && buffer_xbee[0] == '*')
    {
        on_load(buffer_xbee);
        uart_counter_xbee = 0;
    }
}

void control_gpio_init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
                          GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    GPIOPinTypeGPIOInput(
            GPIO_PORTA_BASE,
            GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
    GPIOPinTypeGPIOInput(
            GPIO_PORTB_BASE,
            GPIO_PIN_5 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_0 | GPIO_PIN_4
                    | GPIO_PIN_6 | GPIO_PIN_7);

    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4,
                     GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU); //pull-up pin 4
    GPIOPadConfigSet(
            GPIO_PORTB_BASE,
            GPIO_PIN_5 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_0 | GPIO_PIN_4,
            GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU); //pull-up pin 4
    GPIOPadConfigSet(
            GPIO_PORTA_BASE,
            GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6,
            GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU); //pull-up pin 4

}

void brake_control(uint32_t ui8Adjust_0)
{

    if (lastvalue_brake < ui8Adjust_0)
    {
        for (lastvalue_brake; lastvalue_brake < ui8Adjust_0; lastvalue_brake++)
        {
            PWMPulseWidthSet(PWM0_BASE, Brake_output,
                             lastvalue_brake * ui32Load / 1000);
            PWMPulseWidthSet(PWM0_BASE, Speed_output, 0 * ui32Load / 1000);
            SysCtlDelay(SysCtlClockGet() / 1000);
        }
    }
    else if (lastvalue_brake > ui8Adjust_0)
    {
        for (lastvalue_brake; lastvalue_brake > ui8Adjust_0; lastvalue_brake--)
        {
            PWMPulseWidthSet(PWM0_BASE, Brake_output,
                             lastvalue_brake * ui32Load / 1000);
            PWMPulseWidthSet(PWM0_BASE, Speed_output, 0 * ui32Load / 1000);
            SysCtlDelay(SysCtlClockGet() / 1000);
        }
    }
    else if (lastvalue_brake == ui8Adjust_0)
    {
        PWMPulseWidthSet(PWM0_BASE, Brake_output,
                         lastvalue_brake * ui32Load / 1000);
    }
}

void steer_control(uint32_t ui8Adjust_0)
{
    if (lastvalue_steer < ui8Adjust_0)
    {
        for (lastvalue_steer; lastvalue_steer < ui8Adjust_0; lastvalue_steer++)
        {
            PWMPulseWidthSet(PWM0_BASE, Steer_output,
                             lastvalue_steer * ui32Load / 1000);
            SysCtlDelay(SysCtlClockGet() / 1000);
        }
    }
    else if (lastvalue_steer > ui8Adjust_0)
    {
        for (lastvalue_steer; lastvalue_steer > ui8Adjust_0; lastvalue_steer--)
        {
            PWMPulseWidthSet(PWM0_BASE, Steer_output,
                             lastvalue_steer * ui32Load / 1000);
            SysCtlDelay(SysCtlClockGet() / 1000);
        }
    }
    else if (lastvalue_steer == ui8Adjust_0)
    {
        PWMPulseWidthSet(PWM0_BASE, Steer_output,
                         lastvalue_steer * ui32Load / 1000);
    }
}

void speed_control(uint32_t rpm)
{

    if (lastvalue_speed < rpm)
    {
        for (lastvalue_speed; lastvalue_speed < rpm; lastvalue_speed++)
        {
            PWMPulseWidthSet(PWM0_BASE, Speed_output,
                             lastvalue_speed * ui32Load / 1000);
            SysCtlDelay(SysCtlClockGet() / 1000);
        }
    }
    else if (lastvalue_speed > rpm)
    {
        for (lastvalue_speed; lastvalue_speed > rpm; lastvalue_speed--)
        {
            PWMPulseWidthSet(PWM0_BASE, Speed_output,
                             lastvalue_speed * ui32Load / 1000);
            SysCtlDelay(SysCtlClockGet() / 1000);
        }
    }
    else if (lastvalue_speed == rpm)
    {
        PWMPulseWidthSet(PWM0_BASE, Speed_output,
                         lastvalue_speed * ui32Load / 1000);
    }
}

void gpio_interrupt_init(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);

    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA,
                     GPIO_PIN_TYPE_STD_WPU);

    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_2);

    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_LOW_LEVEL);

    IntPrioritySet(INT_GPIOA, 0);

    IntRegister(INT_GPIOA, IntGPIOdHandler);

    IntEnable(INT_GPIOA);

    IntMasterEnable();
}
void IntGPIOdHandler(void)
{

    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_2);
    int status = GPIOIntStatus(GPIO_PORTA_BASE, true);

    button_on_off = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2);

    if (status == 4 && GPIO_INT_PIN_2 == GPIO_INT_PIN_2)
    {
        if (start_flag == false)
        {
            start_flag = true;
            SysCtlDelay(4000000);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 2);
            SysCtlDelay(4000000);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

        }
        else if (start_flag == true)
        {
            start_flag = false;
            SysCtlDelay(4000000);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 2);
            SysCtlDelay(8000000);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
            SysCtlDelay(8000000);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 2);
            SysCtlDelay(8000000);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
        }
    }

}

void IntTimer1(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    tick = true;
}

void init_timerHardware(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 20);
    TimerIntRegister(TIMER1_BASE, TIMER_A, IntTimer1);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER1_BASE, TIMER_A);
}
