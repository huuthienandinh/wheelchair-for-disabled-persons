/*
 * main.c
 * pin_map:
 * 			PE3, PE2 ------------>Joystick
 *			PA2 ----------------->SW
 * 			PF1, PC4 ------------> Motor
 * 			PF2, PC5 ------------>DIR
 *			PF3 -----------------> Buzzer
 *			PA2 -----------------> SW_Light
 *			PA3 -----------------> Enable Sensor
 *			PC6 -----------------> Light
 *			PE1 -----------------> Bat.
 *			PA4, PA6, PA7 -------> BAT WARNING
 */
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "MPU6050.h" // self-writing
#include "I2C.h" // self-writing

#define		Step_dutycycle		0.045
int readSW, readSensor, readLed, flag_Sensor, flag_BAT, warning_BAT, sort;
uint32_t ulPeriod1, ulPeriod2, joystick[], BAT[];
float dutycycle1, dutycycle2;

void ReadJoystick();
void InitPWM();
void Motion();
void Check_input();
void ReadBAT();

// vector stop of first timer (timer0)
void Sensor(void)
{
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
if(flag_Sensor == 1)
	ReadMPU();
}
// vector stop of second timer (timer1)
void BATT(void)
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	if(warning_BAT ==1 && sort == 0)
	{
	GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_6, GPIO_PIN_6);
	SysCtlDelay(SysCtlClockGet()/3/20);
	GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3, 0);
	GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_6, 0);
	SysCtlDelay(SysCtlClockGet()/3/20);
	GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_6, GPIO_PIN_6);
	SysCtlDelay(SysCtlClockGet()/3/20);
	GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3, 0);
	GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_6, 0);
	}
}
// main
void main(void) {
	//20MHz
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    // enable ports
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	// declare of 3 pins used signals
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);// het
   // declare adc to read joystick and battery
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1 | ADC_CTL_IE| ADC_CTL_END);
   ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 0);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 0);
    ADCIntClear(ADC0_BASE, 3); 
   // init pwm
    InitPWM();
    // init i2c to communicate with sensors
    initI2C();
    // setup sensor
	Setup_MPU6050();
	//config timer0 = sensor
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	uint32_t ui32PeriodA=SysCtlClockGet()/50;
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32PeriodA-1);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER0_BASE, TIMER_A);
	// config timer1 = BAT
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	uint32_t ui32PeriodB=SysCtlClockGet();
	TimerLoadSet(TIMER1_BASE, TIMER_A, ui32PeriodB-1);
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER1_BASE, TIMER_A);
	IntMasterEnable(); 
    while(1)
    	ReadJoystick(); 
    	Motion(); // control 2 engine motion
    	ReadBAT(); 
    	Check_input(); 
    }
}

void Check_input()
{

	readLed = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
	readSW = GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_2);
	readSensor = GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_3);
	if((readSensor & GPIO_PIN_3) == 0)
	{
		flag_Sensor = 1;
	if(kalAngleX >=30 || kalAngleY >=30 || kalAngleX <=-30 || kalAngleY <=-30)
	{
		GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3, GPIO_PIN_3);
		SysCtlDelay(SysCtlClockGet()/3);
		GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3, 0);
		SysCtlDelay(SysCtlClockGet()/3);
		sort = 1;
	}
	else sort = 0;
	}
	else
	{
		sort = 0;
		flag_Sensor = 0;
		kalAngleX = 0;
		kalAngleY = 0;
	}
	if(BAT[0] <= 3300 && sort == 0)//3390)
	{
		flag_BAT = 1;
		warning_BAT = 1;
	}
	else
	{
		flag_BAT = 0;
		warning_BAT = 0;
	}
	if(BAT[0] > 3700)
	{
    	GPIOPinWrite(GPIO_PORTE_BASE,  GPIO_PIN_5, GPIO_PIN_5);
    	GPIOPinWrite(GPIO_PORTE_BASE,  GPIO_PIN_4, GPIO_PIN_4);
    	GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_7, GPIO_PIN_7);
	}
	if(BAT[0] <=3700 && BAT[0] >= 3400)
	{
    	GPIOPinWrite(GPIO_PORTE_BASE,  GPIO_PIN_5, 0);
    	GPIOPinWrite(GPIO_PORTE_BASE,  GPIO_PIN_4, GPIO_PIN_4);
    	GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_7, GPIO_PIN_7);
	}
	if(BAT[0] < 3400)
	{
    	GPIOPinWrite(GPIO_PORTE_BASE,  GPIO_PIN_5, 0);
    	GPIOPinWrite(GPIO_PORTE_BASE,  GPIO_PIN_4, 0);
    	GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_7, GPIO_PIN_7);
	}

	if((readLed & GPIO_PIN_4) == 0 && flag_BAT == 0)
	{

		GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_6, 0);
	}
	else
		GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_6, GPIO_PIN_6);


	 if((readSW & GPIO_PIN_2) == 0)
	{

		GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3, GPIO_PIN_3);
	}
	else
	{
		GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3, 0);
	}
}

void ReadBAT()
{
	 // Trigger the ADC conversion.
	        //
	        ADCProcessorTrigger(ADC0_BASE, 3);


	        //
	        // Wait for conversion to be completed.
	        //
	        while(!ADCIntStatus(ADC0_BASE, 3, false))
	        {
	        }

	        //
	        // Clear the ADC interrupt flag.
	        //
	        ADCIntClear(ADC0_BASE, 3);

	        //
	        // Read ADC Value.
	        //
	        ADCSequenceDataGet(ADC0_BASE, 3, BAT);
	      // SysCtlDelay(SysCtlClockGet() / 12);
}
void ReadJoystick()
{
	 // Trigger the ADC conversion.
	        //
	        ADCProcessorTrigger(ADC0_BASE, 0);


	        //
	        // Wait for conversion to be completed.
	        //
	        while(!ADCIntStatus(ADC0_BASE, 0, false))
	        {
	        }

	        //
	        // Clear the ADC interrupt flag.
	        //
	        ADCIntClear(ADC0_BASE, 0);

	        //
	        // Read ADC Value.
	        //
	        ADCSequenceDataGet(ADC0_BASE, 0, joystick);
	      // SysCtlDelay(SysCtlClockGet() / 12);
}

void InitPWM()
{
	//Configure PWM clock to match system
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	//Enable the peripherals used by this program.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);//Tiva Launchpad has 2 modules (0 and 1) and module 1 covers led pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	ulPeriod1 = SysCtlClockGet()/10000;//PWM frequency 10kHz
	ulPeriod2 = SysCtlClockGet()/10000;

	//Configure PF1 pins as PWM
	GPIOPinConfigure(GPIO_PF1_M1PWM5);
	GPIOPinConfigure(GPIO_PC4_M0PWM6);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
	GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);

	//Configure PWM Options
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ulPeriod1);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ulPeriod2);

	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ulPeriod1*1/100);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, ulPeriod2*1/100);

	//Turn on the Output pins
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
	//Enable the PWM generator
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);
	//Dir
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);

	GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_2,0);
	GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3, 0);
	GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_5,0);
	GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_6,0);
	//Config Buttons
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0x4C4F434B;	//Unlocks the GPIO Commit (GPIOCR) register for write access PF0
	    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = GPIO_PIN_4;
		GPIODirModeSet(GPIO_PORTF_BASE,  GPIO_PIN_4, GPIO_DIR_MODE_IN);
		GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		GPIODirModeSet(GPIO_PORTA_BASE,  GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4, GPIO_DIR_MODE_IN);
		GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2| GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void Motion()
{
	if(joystick[0] < 1500)
	{
		GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_2, GPIO_PIN_2);
		dutycycle1 = (2000 - joystick[0])*Step_dutycycle;
		GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_5, GPIO_PIN_5);
		dutycycle2 = (2000 - joystick[0])*Step_dutycycle;
		dutycycle2+=10;
		dutycycle1+=5;
	}
	else if(joystick[0] > 2500)
	{
		GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_2, 0);
		dutycycle1 = (joystick[0] -2000)*Step_dutycycle;
		GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_5, 0);
		dutycycle2 = (joystick[0] - 2000)*Step_dutycycle;
		dutycycle2+=5;
	}

	else if(joystick[1] < 1500)
		{
		GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_2, 0);
		dutycycle1 = (2000 - joystick[1])*Step_dutycycle;
		GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_5, GPIO_PIN_5);
		dutycycle2 = (2000 - joystick[1])*Step_dutycycle;
		dutycycle2+=10;
		dutycycle1 +=5;
		}
		else if(joystick[1] > 2500)
		{
			GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_2, GPIO_PIN_2);
			dutycycle1 = (joystick[1] -2000)*Step_dutycycle;
			GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_5, 0);
			dutycycle2 = (joystick[1] - 2000)*Step_dutycycle;
			dutycycle2+=5;
		}
		else
		{
			dutycycle1 = 1;
			dutycycle2 = 1;
		}
		if(dutycycle2 < 1)
			dutycycle2 =1;
		if(dutycycle1 < 1)
			dutycycle1 =1;
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ulPeriod1*dutycycle1/100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, ulPeriod2*dutycycle2/100);
}




