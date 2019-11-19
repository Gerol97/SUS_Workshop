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
        //GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 High
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0xFF);
        delay2();

        //GPIO Pin 1, GPIO Pin 2, GPIO Pin 3 Low
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
        delay1();                                                                                                GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0x00);
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
    uint32_t Time=systemTime_ms;
    while(1){
        if(Time==(systemTime_ms-waitTime)){
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
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0xFF);
        delay_ms(500);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);
        delay_ms(500);
    }
}
*/
/*

//AUFGABE 6.1==========================================================================================================================

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
    //Frequency(1/ms) times waitTime(ms) is equal to wait
    uint32_t wait=(waitTime*SysCtlClockGet()/1000);
    while(wait)
    {
        wait--;
    }
}

void delay_us(uint32_t waitTime)
{
    //Frequency(1/us) times waitTime(us) is equal to wait
    uint32_t wait=(waitTime*SysCtlClockGet()/1000000);
    uint32_t test=4290000000;
    while(test)
    {
        wait--;
    }
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

    while(1)
    {
        //Taster gedrueckt
        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,0x00);

        if (GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_7)==0)
        {
            //auf Tastendruck reagieren
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,0xFF);
        }
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


void delay_ms(uint32_t waitTime)
{
    //Frequency(1/ms) times waitTime(ms) is equal to wait
    uint32_t wait=(waitTime*SysCtlClockGet()/1000);
    while(wait)
    {
        wait--;
    }
}

void delay_us(uint32_t waitTime)
{
    //Frequency(1/us) times waitTime(us) is equal to wait
    uint32_t wait=(waitTime*SysCtlClockGet()/1000000);
    uint32_t test=4290000000;
    while(test)
    {
        wait--;
    }
}

int main(void)
{
    //Enable GPIO PORT B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //Enable GPIO PORT C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    //Enable GPIO PORT F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //Set Port C Pin 7 as input with pull-up resistance.
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE,GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_7,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
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
    //Frequency(1/ms) times waitTime(ms) is equal to wait
    uint32_t wait=(waitTime*SysCtlClockGet()/1000);
    while(wait)
    {
        wait--;
    }
}

volatile uint32_t Wert=0x01;

// Interruptroutine
void ex_int_handler(void) {
    // delay, um Taster zu stabilisieren
    delay_ms(13);
    // Auf Tastendruck reagieren
    Wert=Wert*2;
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
        if (Wert==0x100)
        {
            Wert=0x01;
        }
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, Wert);
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

uint32_t systemTime_ms;
void InterruptHandlerTimer0A (void)
{
    //delete timer interrupt flag
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //add 1 to ms counter
    systemTime_ms++;
}

void clockSetup(void)
{
    uint32_t timerPeriod;
    //clock configure

    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    //activate Peripherie for timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    //configure timer as 32 bit in periodic mode
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //set timerPeriod ....
    timerPeriod = (SysCtlClockGet()/1000);
    //
    TimerLoadSet(TIMER0_BASE, TIMER_A, timerPeriod-1);
    //
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
    uint32_t systemTime_ms1=systemTime_ms;
    while(1)
    {
        if (systemTime_ms1==(systemTime_ms-waitTime))
        {
            break;
        }
    }
}

void main(void)
{
    //benutzen Fliesskommazahl
    FPUEnable();
    FPUStackingEnable();

    clockSetup();
    systemTime_ms=0;


    // Peripherie ADC aktivieren
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2
                          | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    // PIN PE2 ADC Funktion zuweisen
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_2);

    // ADC konfigurieren
    ADCSequenceConfigure(ADC0_BASE,1,ADC_TRIGGER_PROCESSOR,0);  // Prozessor als Trigger Quelle
    ADCSequenceStepConfigure(ADC0_BASE,1,0,ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END); // AI1 abtasten/Interrupt erzeugen bei Ende/letzter Schritt
    ADCSequenceEnable(ADC0_BASE,1); // ADC Sequenz 1 aktivieren

    uint32_t ui32ADC0Value;

    float Widerstand;
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

        Widerstand =(1000.0/(4095.0/ui32ADC0Value-1));//Mit ADC Wert Widerstand ausrechnen

        GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x00);
        delay_ms(20);
        if (Widerstand>100&&Widerstand<=300)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x01);
        }
        else if(Widerstand>300&&Widerstand<=500)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x02);
        }
        else if(Widerstand>500&&Widerstand<=1000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x04);
        }
        else if(Widerstand>1000&&Widerstand<=3000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x08);
        }
        else if(Widerstand>3000&&Widerstand<=5000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x10);
        }
        else if(Widerstand>5000&&Widerstand<=9000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x20);
        }
        else if(Widerstand>9000&&Widerstand<=13000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x40);
        }
        else if(Widerstand>13000&&Widerstand<=20000)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,0x80);
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
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"

// Makros
#define FSAMPLE  44000
#define BUFFER_SIZE 1000

// globale Variable
int32_t buffer_sample[BUFFER_SIZE];     //Quadratische Signale
uint32_t i_sample = 0;
int32_t buffer_sample_sum = 0;          // momentaner Pegel

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
uint32_t test=0;
// Interrupt handler
void ADC_int_handler(void)
{
    ADCIntClear(ADC0_BASE, 3);  // delete interrupt flag
    ADCProcessorTrigger(ADC0_BASE,3);   // Konvertierung beginnen
    while(!ADCIntStatus(ADC0_BASE,3,false));    // warten bis Konvertierung abgeschlossen
    ADCSequenceDataGet(ADC0_BASE,3,&Value); // Wert auslesen

    //Die Summe von jeden 1000 Werten addieren
    buffer_sample_sum=buffer_sample_sum+Value*Value-buffer_sample[i_sample];
    buffer_sample[i_sample]=Value*Value;
    //uint32_t Z1ahl=buffer_sample_sum-buffer_sample_sum/100000*100000;

    //Nominierung der Summe, damit man mit 8 LEDs sehen kann
    int i =0;
    for (;i<256;i++)
    {
        if ((buffer_sample_sum-buffer_sample_sum/100000*100000)>=i*392)
        {
            GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,i);
        }
    }

    i_sample++;
    if (i_sample==BUFFER_SIZE)
    {
        i_sample=0;
    }
}
