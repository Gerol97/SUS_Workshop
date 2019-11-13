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

int main(void){

    //Enable GPIO-Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //Set GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 as Digital Output
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    while(1){

        //GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 High
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0xFF);
        delay();

        //GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 Low
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
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
    uint32_t i=50000;
    while(i) {i--;}
}

void delay2(void)
{
    uint32_t k=50000;
    while(k) {k--;}
}
int main(void){

    //Enable GPIO-Port B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //Set GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 as Digital Output
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    while(1){

        //GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 High
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0xFF);
        delay2();

        //GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 Low
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x00);
        delay1();
    }

}
*/


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
    uint32_t waitPeriod=(waitTime*SysCtlClockGet()/1000);
    while (waitPeriod)
    {
        waitPeriod--;
    }
}

int main(void)
{
    // /...initialisiere Ports und Pins.../
    systemTime_ms = 0;
    clockSetup();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    while(1)
    {
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0xFF);
        delay_ms(1000);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x00);
        delay_ms(1000);
    }
}

