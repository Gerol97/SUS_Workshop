/*
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

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    while(1){

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0xFF);
        delay();

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
        delay();
    }

}
