/*
 * File:   newmain.c
 * Author: joeld
 *
 * Created on February 23, 2024, 4:23 PM
 */

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)



#include <xc.h>
#include "enc28j60.h"



void main(void) {
    
    
    
    USART_Init(9600); // Initialize USART with baud rate 9600
    CS_PIN = 1;  //Desactiar esclavo
    configurar_SPI();
    TRISCbits.TRISC2=0;
    uint8_t current_pktcnt = 0;
    uint8_t ss_pktcnt = 0;
    usart_newline();
            USART_Transmit('E');
            USART_Transmit('T');
            USART_Transmit('H');
            USART_Transmit(' ');
            USART_Transmit('s');
            USART_Transmit('t');
            USART_Transmit('a');
            USART_Transmit('r');
            USART_Transmit('t');
    usart_newline();
    
    //soft_reset();
    
    // Esperar ESTAT.CLKRDY a 1
    wait_ost();
    
    //Configurar puntero de ambos buffer ethernet
    eth_buffer_init();
   
   
    // Configurar filtrado de paquetes
        //Cambiar al banco 1
        move_bank(BANK1); 
        // Configurar el filtro Unicast
        //write_register(ERXFCON, 0x80); // 0x80 es 10000000 en binario, lo que establece el bit UCEN a 1 y todos los demás bits a 0
        write_register(ERXFCON, 0x00); // modo promiscuo
        //write_register(ERXFCON, 0xC1); // new 11000001
    
        
    
        
    // Configurar registros MAC
        mac_init();
        
    // Configurar registros PHY
        phy_init();
    //write_buffer_memory();   
    dhcp_discover();
    
    
    
   move_bank(BANK1);
   
   write_register(ECON1, read_register(ECON1) | 0x04);   //Activate RXEN to enable reception of paquets

   move_bank(BANK0); 
   
   
   
   

    
    while(1){

        // Move to bank 1
        move_bank(BANK1);
        // Save value of epktcnt
        ss_pktcnt = read_register(EPKTCNT);
        // if pktcnt mayor a 0 then - print session_current_pkt , increment it, decrement pktcnt
        if(ss_pktcnt > 0 ){
            current_pktcnt++;
            move_bank(BANK1);
            
            usart_newline();
            usart_tx_byte_to_hex(ss_pktcnt);
            USART_Transmit(' ');
            usart_tx_byte_to_hex(current_pktcnt);          //Comprobe MACON1 value     packet_number
            move_bank(BANK0); 
            usart_newline();
            usart_tx_byte_to_hex(read_register(ERDPTH));           // Where is pointed the user writing pointer 
            usart_tx_byte_to_hex(read_register(ERDPTL));     
            usart_newline();
            usart_tx_byte_to_hex(read_register(ERXWRPTH));         // Where is pointed enc28j60 writing pinter
            usart_tx_byte_to_hex(read_register(ERXWRPTL));
            usart_newline();
            usart_tx_byte_to_hex(read_register(ERXRDPTH));         // where is the final of queue
            usart_tx_byte_to_hex(read_register(ERXRDPTL));

            read_receive_buffer_pkt();
            
            
            // return to BANK0
            move_bank(BANK0);
            __delay_ms(2000);
        }
        
        
        
        
        
        // if EIR-PKTIF mayor a 0 then - print session_current_pkt , increment it, decrement pktcnt
                //if EIR.RXERIF   must be cleared by yourself
        // Encender el LED (RC2 = 1) 
                
        

        
        /*        
                PORTCbits.RC2 = 1;
         *      // Esperar un tiempo
                __delay_ms(2000);

                // Apagar el LED (RA0 = 0)
                PORTCbits.RC2 = 0;

                // Esperar un tiempo
                __delay_ms(2000);*/

    }
    return;
}
