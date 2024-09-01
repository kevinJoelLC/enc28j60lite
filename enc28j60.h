/* 
 * File: enc28j60 driver for pic
 * Author: Kevin López
 * Comments: 
 * Revision history: 17/05/2024
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef ENC28J60_H
#define	ENC28J60_H

#include <xc.h> // include processor files - each processor file is guarded.  

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */




#define _XTAL_FREQ 20000000 // Frecuencia del cristal (4 MHz)
#define CS_PIN PORTDbits.RD3 // Alias global prar pin 3 puerto D
#define RESET_ENC28J60_PIN PORTDbits.RD2  //To identifie which pin is to reset the enc28j60






/*Conf 1  enc28j60: Pointers to receive buffer*/
#define ERXSTL 0x08  // Dirección del registro ERXSTL
#define ERXSTH 0x09  // Dirección del registro ERXSTH
#define ERXNDL 0x0A  // Dirección del registro ERXNDL
#define ERXNDH 0x0B  // Dirección del registro ERXNDH
#define ERDPTL 0x00  // Pointer to current read register
#define ERDPTH 0x01  


/*Conf 2 enc28j60 no es necesaria*/
#define ETXSTL 0x04
#define ETXSTH 0x05
#define ETXNDL 0x06
#define ETXNDH 0x07
#define EWRPTL 0x02  //Pointer to current write register
#define EWRPTH 0x03

/*Conf 3 enc28j60: Configurar registro ERXFCON en banco 1*/
#define ECON1 0x1F     // Bank 0-3
#define ECON2 0x1E     // Bank 0-3
#define EIE 0x1B       // Bank 0-3
#define EIR 0x1C       // Bank 0-3
#define ESTAT 0x1D     // Bank 0-3
#define ERXFCON 0x18   //Bank 1

/*Conf 4 enc28j60: Esperar al OST */
#define ESTAT 0x1D    // Bank 0-3

                uint8_t delete = 0; //unnecesary


/*Conf 5 enc28j60: configurar MAC registers*/
enum banks{
    BANK0 = 0x00,
    BANK1 = 0x01,
    BANK2 = 0x02,
    BANK3 = 0x03,
    
};

#define MACON1 0x00  //bank 2
#define MACON3 0x02  //bank 2
#define MACON4 0x03  //bank 2

#define MAMXFLL 0x0A //bank 2
#define MAMXFLH 0x0B //bank 2

#define MABBIPG 0x04 //bank 2

#define MAIPGL 0x06 //bank2
#define MAIPGH 0x07 //bank2

#define MACLCON1 0x08 //bank2
#define MACLCON2 0x09 //bank2


#define MAADR1 0x04  //bank 3
#define MAADR2 0x05  //bank 3
#define MAADR3 0x02  //bank 3
#define MAADR4 0x03  //bank 3
#define MAADR5 0x00  //bank 3
#define MAADR6 0x01  //bank 3

/* Conf 6 enc28j60: PHY registers*/
typedef struct{
    uint8_t low_byte;
    uint8_t high_byte;
}register_16bits;

#define MICMD 0x12  //bank 2
#define MIREGADR 0x14  //bank 2
#define MIWRL 0x16  //bank 2
#define MIWRH 0x17  //bank 2
#define MIRDL 0x18  //bank 2
#define MIRDH 0x19  //bank 2

#define MISTAT 0x0A  //bank 3

#define PHCON1 0x00  // registros phy
#define PHCON2 0x10
#define PHLCON 0x14

//Registros buffer ethernet
#define EPKTCNT 0x19   // bank 1
#define ERXWRPTH 0x0F  // bank 0
#define ERXWRPTL 0x0E  // bank 0
#define ERXRDPTH 0x0D  // bank 0
#define ERXRDPTL 0x0C  // bank 0




void USART_Init(long baud_rate);

void USART_Transmit(char out);

void usart_tx_byte(uint8_t byte);

void usart_tx_byte_to_hex(uint8_t byte);

void usart_newline();

void configurar_SPI();

uint8_t spi_send(uint8_t byte);

void soft_reset();

void write_register(unsigned char address, unsigned char data);

uint8_t read_register(unsigned char address);

uint8_t read_register_mac(unsigned char address);

void move_bank(enum banks bank);

void wait_ost();

void mac_init();

void write_register_phy(unsigned char phy_address, uint8_t byte_h, uint8_t byte_l);

void read_register_phy(uint8_t address, register_16bits *data);

void phy_init();

void eth_buffer_init();

uint8_t write_buffer_memory();

uint8_t dhcp_discover();

uint8_t read_receive_buffer_pkt();

#endif

