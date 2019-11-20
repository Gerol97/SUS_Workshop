/*
// AUFGABE 3.1 ==========================================================================================================================

#include<stdint.h>


// Warteschleife
void delay(void)
{
    uint32_t i=50000;
    while(i) {i--;}
}
int main(void)
{
    uint32_t* address_RCGCGPIO;
    uint32_t* address_GPIO_DIR;
    uint32_t* address_GPIO_DATA;
    uint32_t* address_GPIO_DEN;
    // Adresse RCGCGPIO Register
    address_RCGCGPIO = (uint32_t*) 0x400FE608;
    // Adresse DATA Register mit Bitmaskierung für Pin1 (LED_R), Pin2 (LED_B) und Pin3 (LED_G)
    address_GPIO_DATA = (uint32_t*) 0x40025038;
    // Adresse DEN Register
    address_GPIO_DEN = (uint32_t*) 0x4002551C;
    // Adresse DIR Register
    address_GPIO_DIR = (uint32_t*) 0x40025400;
    // Peripherie Port F aktivieren
 *address_RCGCGPIO = *address_RCGCGPIO | (0x20);
    // setze Pin als Ausgang
 *address_GPIO_DIR = *address_GPIO_DIR| (0x0E);
    // setze Pin als Digital
 *address_GPIO_DEN = *address_GPIO_DEN| (0x0E);
    while(1) {
        // Setze Pin auf High Pegel
 *address_GPIO_DATA =  (0x04);
        delay();
        // Setze Pin auf Low Pegel
 *address_GPIO_DATA =  (0x00);
        delay();
    }
}
 */
/*

// AUFGABE 4.1 ==========================================================================================================================

#include<stdint.h> // Definition des Typs "int"
#include<stdbool.h> // Definition des Typs "bool"
#include"inc/hw_memmap.h" // Definition der Registeradressen
#include"inc/hw_types.h" // Definition der framework makros
#include"driverlib/gpio.h"
#include"driverlib/sysctl.h"

void delay(void)
{
    uint32_t i=50000;
    while(i) {i--;}
}

int main(void)
{
    //Enable GPIO Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //Set GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 as Digital Output
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    while(1)
    {
        //GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 High
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0xFF);
        delay();

        //GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 Low
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);
        delay();
    }
}
 */
/*

// AUFGABE 4.2 ==========================================================================================================================

#include<stdint.h> // Definition des Typs "int"
#include<stdbool.h> // Definition des Typs "bool"
#include"inc/hw_memmap.h" // Definition der Registeradressen
#include"inc/hw_types.h" // Definition der framework makros
#include"driverlib/gpio.h"
#include"driverlib/sysctl.h"

void delay1(void)
{
    uint32_t i=500000;
    while(i) {i--;}
}

void delay2(void)
{
    uint32_t k=500000;
    while(k) {k--;}
}

int main(void)
{
    //Enable GPIO Port B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //Set GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 as Digital Output
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                          | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    while(1)
    {
        //Set GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 High
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0xFF);
        delay2();

        //Set GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 Low
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
        delay1();

    }
}
 */
/*

// AUFGABE 5 ==========================================================================================================================

#include<stdint.h>
#include<stdbool.h>
#include"inc/hw_ints.h"
#include"inc/hw_memmap.h"
#include"inc/hw_types.h"
#include"driverlib/gpio.h"
#include"driverlib/sysctl.h"
#include"driverlib/timer.h"
#include"driverlib/interrupt.h"

//speichert die Zeit seit Systemstart in ms
uint32_t systemTime_ms;

void InterruptHandlerTimer0A (void)
{
    // loesche das timer interrupt flag, um ein direktes, erneutes Aufrufen zu vermeiden
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // erhoehe den ms-Zaehler um 1 ms
    systemTime_ms++;
}

void clockSetup(void)
{
        uint32_t timerPeriod;
        //konfiguriere clock
        SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
        //aktiviere Periphie fuer den timer
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        //konfiguriere timer als 32 bit timer in periodischem Modus
        TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
        //setze die Variable timerPeriod auf die Anzahl der Perioden, um ein jede ms ein timeout zu erzeugen
        timerPeriod = (SysCtlClockGet()/1000);
        //uebergebe die Variable timerPeriod an den TIMER-0-A
        TimerLoadSet(TIMER0_BASE, TIMER_A, timerPeriod-1);
        //registriere die Funktion InterruptHandlerTimer0A als Interrupt-Serviceroutine
        TimerIntRegister(TIMER0_BASE, TIMER_A, &(InterruptHandlerTimer0A));
        //aktivie die Interrupt auf TIMER-0-A
        IntEnable(INT_TIMER0A);
        //erzeuge einen Interrupt, wenn TIMER-0-A ein timeout erzeugt
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        //alle Interrupt werden aktiviert
        IntMasterEnable();
        //starte das Zaehlen des Timers
        TimerEnable(TIMER0_BASE, TIMER_A);
}

void delay_ms(uint32_t waitTime)
{
    //saving systemTime to another variable
    uint32_t Time = systemTime_ms;
    //waiting loop in ms
    while(1)
    {
        if(Time == (systemTime_ms-waitTime))
        {
            break;
        }
    }
}

int main(void)
{
    systemTime_ms = 0;
    clockSetup();
    //Enable GPIO PORTB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //Set GPIO PORTB 0-7 as Digital Output
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                          | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    while(1)
    {
        //Set GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 High
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0xFF);
        delay_ms(500);

        //Set GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 Low
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
        delay_ms(500);
    }
}
 */
/*

//AUFGABE 6.1(1)==========================================================================================================================

#include<stdint.h>
#include<stdbool.h>
#include"inc/hw_ints.h"
#include"inc/hw_memmap.h"
#include"inc/hw_types.h"
#include"driverlib/gpio.h"
#include"driverlib/sysctl.h"
#include"driverlib/timer.h"
#include"driverlib/interrupt.h"

//speichert die Zeit seit Systemstart in ms
uint32_t systemTime_ms;

void InterruptHandlerTimer0A (void)
{
    // loesche das timer interrupt flag, um ein direktes, erneutes Aufrufen zu vermeiden
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // erhoehe den ms-Zaehler um 1 ms
    systemTime_ms++;
}

void clockSetup(void)
{
        uint32_t timerPeriod;
        //konfiguriere clock
        SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
        //aktiviere Periphie fuer den timer
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        //konfiguriere timer als 32 bit timer in periodischem Modus
        TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
        //setze die Variable timerPeriod auf die Anzahl der Perioden, um ein jede ms ein timeout zu erzeugen
        timerPeriod = (SysCtlClockGet()/1000);
        //uebergebe die Variable timerPeriod an den TIMER-0-A
        TimerLoadSet(TIMER0_BASE, TIMER_A, timerPeriod-1);
        //registriere die Funktion InterruptHandlerTimer0A als Interrupt-Serviceroutine
        TimerIntRegister(TIMER0_BASE, TIMER_A, &(InterruptHandlerTimer0A));
        //aktivie die Interrupt auf TIMER-0-A
        IntEnable(INT_TIMER0A);
        //erzeuge einen Interrupt, wenn TIMER-0-A ein timeout erzeugt
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        //alle Interrupt werden aktiviert
        IntMasterEnable();
        //starte das Zaehlen des Timers
        TimerEnable(TIMER0_BASE, TIMER_A);
}

int main(void)
{
    //Enable GPIO PORT B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //Enable GPIO PORT C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    // Taster als Eingang mit Pull-up Widerstand schalten
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE,GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_7,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    //Set Port B Pin 7 as output
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);
    systemTime_ms = 0;
    clockSetup();

    while(1)
    {
        //Taster gedrueckt
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00);

        if (GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_7)==0)
        {
            //auf Tastendruck reagieren
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0xFF);

            while (GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_7)==0)
            {
                // auf Öffnen des Tasters reagieren
                if (GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_7)!=0)
                {
                    break;
                }
            }
        }
    }
}
 */
/*

//AUFGABE 6.1(2)==========================================================================================================================

#include<stdint.h>
#include<stdbool.h>
#include"inc/hw_ints.h"
#include"inc/hw_memmap.h"
#include"inc/hw_types.h"
#include"driverlib/gpio.h"
#include"driverlib/sysctl.h"
#include"driverlib/timer.h"
#include"driverlib/interrupt.h"

//speichert die Zeit seit Systemstart in ms
uint32_t systemTime_ms;

void InterruptHandlerTimer0A (void)
{
    // loesche das timer interrupt flag, um ein direktes, erneutes Aufrufen zu vermeiden
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // erhoehe den ms-Zaehler um 1 ms
    systemTime_ms++;
}

void clockSetup(void)
{
        uint32_t timerPeriod;
        //konfiguriere clock
        SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
        //aktiviere Periphie fuer den timer
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        //konfiguriere timer als 32 bit timer in periodischem Modus
        TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
        //setze die Variable timerPeriod auf die Anzahl der Perioden, um ein jede ms ein timeout zu erzeugen
        timerPeriod = (SysCtlClockGet()/1000);
        //uebergebe die Variable timerPeriod an den TIMER-0-A
        TimerLoadSet(TIMER0_BASE, TIMER_A, timerPeriod-1);
        //registriere die Funktion InterruptHandlerTimer0A als Interrupt-Serviceroutine
        TimerIntRegister(TIMER0_BASE, TIMER_A, &(InterruptHandlerTimer0A));
        //aktivie die Interrupt auf TIMER-0-A
        IntEnable(INT_TIMER0A);
        //erzeuge einen Interrupt, wenn TIMER-0-A ein timeout erzeugt
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        //alle Interrupt werden aktiviert
        IntMasterEnable();
        //starte das Zaehlen des Timers
        TimerEnable(TIMER0_BASE, TIMER_A);
}

int main(void)
{
    //Enable GPIO PORT B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //Enable GPIO PORT C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    //Set Port C Pin 7 as input with pull-up resistance.
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE,GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_7,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    //Set Port B Pin 7 as output
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);
    systemTime_ms = 0;
    clockSetup();

    uint32_t number = 0;
    while(1)
    {
        //if switch is on add 1 to the number
        if (GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)==0)
        {
            number++;
        }
        //if the number is an even number turn off the LED
        if(number%2==0)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,0x00);
        }
        //if the number is an odd number turn on the LED
        else
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,0xFF);
        }
    }
}
 */
/*

//AUFGABE 6.1(3)==========================================================================================================================

#include<stdint.h>
#include<stdbool.h>
#include"inc/hw_ints.h"
#include"inc/hw_memmap.h"
#include"inc/hw_types.h"
#include"driverlib/gpio.h"
#include"driverlib/sysctl.h"
#include"driverlib/timer.h"
#include"driverlib/interrupt.h"

//speichert die Zeit seit Systemstart in ms
uint32_t systemTime_ms;

void InterruptHandlerTimer0A (void)
{
    // loesche das timer interrupt flag, um ein direktes, erneutes Aufrufen zu vermeiden
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // erhoehe den ms-Zaehler um 1 ms
    systemTime_ms++;
}

void clockSetup(void)
{
        uint32_t timerPeriod;
        //konfiguriere clock
        SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
        //aktiviere Periphie fuer den timer
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
        //konfiguriere timer als 32 bit timer in periodischem Modus
        TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
        //setze die Variable timerPeriod auf die Anzahl der Perioden, um ein jede ms ein timeout zu erzeugen
        timerPeriod = (SysCtlClockGet()/1000);
        //uebergebe die Variable timerPeriod an den TIMER-0-A
        TimerLoadSet(TIMER0_BASE, TIMER_A, timerPeriod-1);
        //registriere die Funktion InterruptHandlerTimer0A als Interrupt-Serviceroutine
        TimerIntRegister(TIMER0_BASE, TIMER_A, &(InterruptHandlerTimer0A));
        //aktivie die Interrupt auf TIMER-0-A
        IntEnable(INT_TIMER0A);
        //erzeuge einen Interrupt, wenn TIMER-0-A ein timeout erzeugt
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        //alle Interrupt werden aktiviert
        IntMasterEnable();
        //starte das Zaehlen des Timers
        TimerEnable(TIMER0_BASE, TIMER_A);
}

int main(void)
{
    //Enable GPIO PORT B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //Enable GPIO PORT F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //Set Port F Pin 4 as input with pull-up resistance.
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    //Set Port B Pin 7 as output
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);
    systemTime_ms = 0;
    clockSetup();

    uint32_t number = 0;
    while(1)
    {
        //if switch is on add 1 to the number
        if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)==0)
        {
            number++;
        }
        //if the number is an even number turn off the LED
        if(number%2==0)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,0x00);
        }
        //if the number is an odd number turn on the LED
        else
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,0xFF);
        }
    }
}
 */
/*

//AUFGABE 6.2==========================================================================================================================

#include<stdint.h>
#include<stdbool.h>
#include"inc/hw_ints.h"
#include"inc/hw_memmap.h"
#include"inc/hw_types.h"
#include"driverlib/gpio.h"
#include"driverlib/sysctl.h"
#include"driverlib/timer.h"
#include"driverlib/interrupt.h"

//speichert die Zeit seit Systemstart in ms
uint32_t systemTime_ms;

void InterruptHandlerTimer0A (void)
{
    // loesche das timer interrupt flag, um ein direktes, erneutes Aufrufen zu vermeiden
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // erhoehe den ms-Zaehler um 1 ms
    systemTime_ms++;
}

void clockSetup(void)
{
    uint32_t timerPeriod;
    //konfiguriere clock
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    //aktiviere Periphie fuer den timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    //konfiguriere timer als 32 bit timer in periodischem Modus
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //setze die Variable timerPeriod auf die Anzahl der Perioden, um ein jede ms ein timeout zu erzeugen
    timerPeriod = (SysCtlClockGet()/1000);
    //uebergebe die Variable timerPeriod an den TIMER-0-A
    TimerLoadSet(TIMER0_BASE, TIMER_A, timerPeriod-1);
    //registriere die Funktion InterruptHandlerTimer0A als Interrupt-Serviceroutine
    TimerIntRegister(TIMER0_BASE, TIMER_A, &(InterruptHandlerTimer0A));
    //aktivie die Interrupt auf TIMER-0-A
    IntEnable(INT_TIMER0A);
    //erzeuge einen Interrupt, wenn TIMER-0-A ein timeout erzeugt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //alle Interrupt werden aktiviert
    IntMasterEnable();
    //starte das Zaehlen des Timers
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void delay_ms(uint32_t waitTime)
{
    //saving systemTime to another variable
    uint32_t Time = systemTime_ms;
    //waiting loop in ms
    while(1)
    {
        if(Time == (systemTime_ms-waitTime))
        {
            break;
        }
    }
}

volatile uint32_t Value=0x01;

// Interruptroutine
void ex_int_handler(void) {
    // Auf Tastendruck reagieren
    Value=Value*2;
    // Interrupt-Flag loeschen
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
}

void main(void)
{
    clockSetup();
    // Peripherie aktivieren
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Set Port B Pin 0-7 as output
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                          | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    // Taster mit Pullup
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Pin mit Interrupt
    GPIOIntDisable(GPIO_PORTF_BASE,GPIO_PIN_4);
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTF_BASE, ex_int_handler);
    GPIOIntEnable(GPIO_PORTF_BASE,GPIO_INT_PIN_4);

    // Endlosschleife
    while(1) {
        //Taster gedrueckt
        if (Value==0x100)
        {
            Value=0x01;
        }

        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, Value);
        delay_ms(50);

        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
        delay_ms(50);
    }
}
 */
/*

//AUFGABE 7.2==========================================================================================================================

#include<stdint.h>
#include<stdbool.h>
#include"math.h"
#include"inc/hw_ints.h"
#include"inc/hw_memmap.h"
#include"inc/hw_types.h"
#include"driverlib/gpio.h"
#include"driverlib/sysctl.h"
#include"driverlib/timer.h"
#include"driverlib/adc.h"
#include"driverlib/interrupt.h"
#include"driverlib/fpu.h"

//speichert die Zeit seit Systemstart in ms
uint32_t systemTime_ms;

void InterruptHandlerTimer0A (void)
{
    // loesche das timer interrupt flag, um ein direktes, erneutes Aufrufen zu vermeiden
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // erhoehe den ms-Zaehler um 1 ms
    systemTime_ms++;
}

void clockSetup(void)
{
    uint32_t timerPeriod;
    //konfiguriere clock
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    //aktiviere Periphie fuer den timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    //konfiguriere timer als 32 bit timer in periodischem Modus
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //setze die Variable timerPeriod auf die Anzahl der Perioden, um ein jede ms ein timeout zu erzeugen
    timerPeriod = (SysCtlClockGet()/1000);
    //uebergebe die Variable timerPeriod an den TIMER-0-A
    TimerLoadSet(TIMER0_BASE, TIMER_A, timerPeriod-1);
    //registriere die Funktion InterruptHandlerTimer0A als Interrupt-Serviceroutine
    TimerIntRegister(TIMER0_BASE, TIMER_A, &(InterruptHandlerTimer0A));
    //aktivie die Interrupt auf TIMER-0-A
    IntEnable(INT_TIMER0A);
    //erzeuge einen Interrupt, wenn TIMER-0-A ein timeout erzeugt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //alle Interrupt werden aktiviert
    IntMasterEnable();
    //starte das Zaehlen des Timers
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void delay_ms(uint32_t waitTime)
{
    //saving systemTime to another variable
    uint32_t Time = systemTime_ms;
    //waiting loop in ms
    while(1)
    {
        if(Time == (systemTime_ms-waitTime))
        {
            break;
        }
    }
}

void main(void)
{
    //using floating-point unit (Fliesskommazahl)
    FPUEnable();
    FPUStackingEnable();

    clockSetup();
    systemTime_ms=0;

    // Peripherie ADC aktivieren
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    //Enable GPIO Port B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //Set GPIO Port B Pins 0-7 as Output
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
                          | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    // PIN PE2 ADC Funktion zuweisen
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_2);

    // ADC konfigurieren
    ADCSequenceConfigure(ADC0_BASE,1,ADC_TRIGGER_PROCESSOR,0);  // Prozessor als Trigger Quelle
    ADCSequenceStepConfigure(ADC0_BASE,1,0,ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END); // AI1 abtasten/Interrupt erzeugen bei Ende/letzter Schritt
    ADCSequenceEnable(ADC0_BASE,1); // ADC Sequenz 1 aktivieren

    uint32_t ui32ADC0Value;

    float Resistance;
    //   float *p=&Widerstand;

    while(1)
    {
        IntMasterDisable();

        // Eingang abfragen
        ADCIntClear(ADC0_BASE,1);       // evtl vorhandene ADC Interrupts loeschen
        ADCProcessorTrigger(ADC0_BASE,1);   // Konvertierung beginnen
        while(!ADCIntStatus(ADC0_BASE,1,false));    // warten bis Konvertierung abgeschlossen
        ADCSequenceDataGet(ADC0_BASE,1,&ui32ADC0Value); // Wert auslesen

        IntMasterEnable();
        systemTime_ms=0;
        clockSetup();

        //Calculating the Tesistance from the ADC value
        Resistance = (1000.0/(4095.0/ui32ADC0Value-1));

        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
        delay_ms(20);

        if (Resistance>100 && Resistance<=300)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x01);
        }
        else if(Resistance>300 && Resistance<=500)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x02);
        }
        else if(Resistance>500 && Resistance<=1000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x04);
        }
        else if(Resistance>1000 && Resistance<=3000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x08);
        }
        else if(Resistance>3000 && Resistance<=5000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x10);
        }
        else if(Resistance>5000 && Resistance<=9000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x20);
        }
        else if(Resistance>9000 && Resistance<=13000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x40);
        }
        else if(Resistance>13000 && Resistance<=20000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x80);
        }
        delay_ms(20);
    }
}
 */


//AUFGABE 8.1==========================================================================================================================

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"

// Makros
#define FSAMPLE  440000
#define BUFFER_SIZE 1000

// globale Variable
int32_t buffer_sample[BUFFER_SIZE];     //Quadratische Signale
uint32_t i_sample = 0;
int32_t buffer_sample_sum = 0;          // momentaner Pegel
int Babun=0;

// Prototypen
void ADC_int_handler(void);

int main(void)
{
    // SystemClock konfigurieren
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    uint32_t ui32Period = SysCtlClockGet()/FSAMPLE;

    // Peripherie aktivieren
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // GPIO konfigurieren
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    //Timer0 konfigurieren
    TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
    TimerControlTrigger(TIMER0_BASE,TIMER_A,true);
    TimerEnable(TIMER0_BASE,TIMER_A);

    // ADC konfigurieren
    ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_RATE_FULL,1);

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE,3);
    ADCIntRegister(ADC0_BASE,3,ADC_int_handler);
    ADCIntEnable(ADC0_BASE,3);

    while(1)
    {

    }
}

uint32_t Value=0;
int32_t buffer_sample_sum_check;
int32_t diff=0;

// Interrupt handler
void ADC_int_handler(void)
{
    ADCIntClear(ADC0_BASE, 3);  // delete interrupt flag
    //Start Converting
    ADCProcessorTrigger(ADC0_BASE,3);
    //Waiting for converting to complete
    while(!ADCIntStatus(ADC0_BASE,3,false));
    //Reading the Value
    ADCSequenceDataGet(ADC0_BASE,3,&Value);

    //Add the current sum of every 1000 values
    buffer_sample_sum=buffer_sample_sum+Value*Value-buffer_sample[i_sample];

    //Add the last sum of every 1000 values
    if(Babun==1)
    {
        buffer_sample_sum_check=buffer_sample_sum-Value*Value+buffer_sample[i_sample];
        diff= buffer_sample_sum-buffer_sample_sum_check;
    }
    buffer_sample[i_sample]=Value*Value;

    //Comparing the current sum and the last sum
    if(Babun==1)
    {
        if(diff<0)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
            SysCtlDelay(1000);
        }
        else if(diff>=0 && diff<1500)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x01);
            SysCtlDelay(1000);
        }
        else if(diff>=1500 && diff<3000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x03);
            SysCtlDelay(1000);
        }
        else if(diff>=3000 && diff<4500)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x07);
            SysCtlDelay(1000);
        }
        else if(diff>=4500 && diff<6000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x1F);
            SysCtlDelay(1000);
        }
        else if(diff>=6000 && diff<7500)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x3F);
            SysCtlDelay(1000);
        }
        else if(diff>=7500 && diff<9000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x7F);
            SysCtlDelay(1000);
        }
        else if(diff>=9000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                         | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0xFF);
            SysCtlDelay(1000);
        }
    }

    i_sample++;

    if (i_sample==BUFFER_SIZE)
    {
        i_sample=0;
        //until the first 1000 Array completely registered
        Babun=1;
    }
}

