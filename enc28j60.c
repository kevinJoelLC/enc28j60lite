#include <xc.h>
#include "enc28j60.h"


void USART_Init(long baud_rate){
    float temp;
    TRISCbits.TRISC6 = 0; // TX Pin set as output
    TRISCbits.TRISC7 = 1; // RX Pin set as input
    temp = ((_XTAL_FREQ/(float)baud_rate)/64) - 1;
    SPBRG = (int) temp; // Baud rate
    TXSTA = 0x20; // Transmit enabled
    RCSTA = 0x90; // Serial Port enabled
}

void USART_Transmit(char out){
    while(!TXSTAbits.TRMT); // Espera hasta que el registro de desplazamiento de transmisión esté vacío
    TXREG = out; // Carga el buffer del transmisor con el valor recibido
}
void usart_tx_byte(uint8_t byte){
    for (int i = 7; i >= 0; i--) {  // Recorre desde el bit 7 (MSB) al bit 0 (LSB)
        if (byte & (1 << i)) {     // Comprueba si el i-ésimo bit es 1
            //printf("Bit %d es 1\n", i);
            USART_Transmit('1');
        } else {
            //printf("Bit %d es 0\n", i);
            USART_Transmit('0');
        }
    }
}
void usart_newline(){
    USART_Transmit('\r');
    USART_Transmit('\n');
}
void usart_tx_byte_to_hex(uint8_t byte){
    USART_Transmit( "0123456789ABCDEF"[(byte >> 4) & 0x0F]); // Parte alta del byte
    USART_Transmit( "0123456789ABCDEF"[byte & 0x0F]);        // Parte baja del byte
}
void soft_reset_en28j60(){
    // Genera un pulso de soft - reset para el enc28j60
    RESET_ENC28J60_PIN = 0; // Lleva el pin de RESET a bajo
    __delay_ms(10);           // Espera 10 ms
    RESET_ENC28J60_PIN = 1;  // Lleva el pin de RESET a alto
    __delay_ms(80); 
}
void configurar_SPI() {
    SSPCONbits.SSPEN=0;  //Desactivo el modulo MSSP
    // Configurar los pines SCK, SDI y SDO como salida  0
    TRISCbits.TRISC3 = 0;   // SCK
    TRISCbits.TRISC4 = 1;   // SDI
    TRISCbits.TRISC5 = 0;   // SDO
    TRISDbits.TRISD3  = 0;  // CS
    TRISDbits.TRISD2 = 0;   // Reset
    
    PORTCbits.RC3 = 0;
    PORTCbits.RC5 = 0;
    PORTDbits.RD2 = 1;

    // Genera un pulso de soft - reset para el enc28j60
    soft_reset_en28j60();
    
    
    // Configurar el módulo MSSP para SPI con:
    // - Modo Maestro
    // - Reloj FOSC/4
    // - Modo de datos 0,0
    SSPSTAT = 0b01000000;  // SMP=0, CKE=1  Transmit occurs on transition from Idle TO ACTIVE clock state 
    SSPCON  = 0b00100001;  // SSPEN=1, CKP=0, SSPM=0000
}

uint8_t spi_send(uint8_t byte) {
    SSPBUF = byte;              
    while (!SSPSTATbits.BF);    // Wait for prevent collisions
    return SSPBUF;              // To put SSPSTATbits.BF to 0 again
}

void soft_reset(){
    CS_PIN = 0; 
    
    spi_send(0xFF);   // Advice to ENC28J60 incoming SOFT RESET command
    
    CS_PIN = 1;
}

void write_register(unsigned char address, unsigned char data) {
    CS_PIN = 0; 
    
    spi_send(0x40 | (address & 0x1F));   // Advice to ENC28J60 incoming write comand on  address register
    
    spi_send(data);
    
    CS_PIN = 1;
}

uint8_t read_register(unsigned char address){
    uint8_t receivedData1 = 0x22;
    CS_PIN = 0;
    
    receivedData1 = spi_send(0x00 | (address & 0x1F));  // Advice ENC28J60 that read comand on address is comming.    
    
    receivedData1 = spi_send(0x00 | (address & 0x1F));    //Dummy byte to receive the value of register after read command
    
    CS_PIN = 1;
    return receivedData1;
}
uint8_t read_register_mac(unsigned char address){
    uint8_t byte = 0x22;
    CS_PIN = 0;
    
    byte = spi_send(0x00 | (address & 0x1F));  // Advice ENC28J60 that read comand on address is comming.    
    
    byte = spi_send(0x00 | (address & 0x1F));    //Dummy byte to receive the dummy response
    
    byte = spi_send(0x00 | (address & 0x1F));    //Dummy byte to receive the value of register after read command
    
    CS_PIN = 1;
    return byte;
}

void move_bank(enum banks bank){
    
    uint8_t econ1 = read_register(ECON1);
    uint8_t current_ECON1_value = read_register(ECON1) & 0x03;
    uint8_t new_ECON1_value;
    
    if(current_ECON1_value >= bank){
        
        write_register(ECON1, econ1 - (current_ECON1_value - bank) );
    }else{

        write_register(ECON1, econ1 + (bank - current_ECON1_value));
    }
}
void wait_ost() {
    // Espera hasta que el bit CLKRDY esté establecido en el registro ESTAT
    usart_newline();
    USART_Transmit('O');
    USART_Transmit('S');
    USART_Transmit('T');
    USART_Transmit(':');
    if ( (read_register(ESTAT) & 0x01) == 0x01) {
        USART_Transmit('1');
    } else {
        USART_Transmit('0');
    }

    while ((read_register(ESTAT) & 0x01) != 0x01);
    usart_newline();
    USART_Transmit('R');
    USART_Transmit('e');
    USART_Transmit('a');
    USART_Transmit('d');
    USART_Transmit('y');
    USART_Transmit(':');
    USART_Transmit('1');

}

void mac_init(){
    // 6.5 Configuraciones MAC
    move_bank(BANK2);
    
    // 6.5.1
    write_register(MACON1, 0x0D); //0b00001101
    // 6.5.2
    write_register(MACON3, 0x37); //0b00110111
    // 6.5.3
    write_register(MACON4, 0x40);
    //6.5.4
    write_register(MAMXFLL, 0xEE);
    write_register(MAMXFLH, 0x05);
    //6.5.5
    write_register(MABBIPG, 0x15);
    //6.5.6
    write_register(MAIPGL, 0x12);
    //6.5.7
    write_register(MAIPGH, 0x0C);
    //6.5.8
    
    //6.5.9
    move_bank(BANK3);
    write_register(MAADR1, 0x02);  // First byte - Most significative
    write_register(MAADR2, 0x00);
    write_register(MAADR3, 0x00);
    write_register(MAADR4, 0x00);
    write_register(MAADR5, 0x00);
    write_register(MAADR6, 0xFE);  // Menos Significativo
    
    
    move_bank(BANK0);
      
}

void write_register_phy(unsigned char phy_address, uint8_t byte_h, uint8_t byte_l){
    move_bank(BANK2);
    
    CS_PIN = 0;   
    spi_send(0x40 | (MIREGADR & 0x1F));   // Advice to ENC28J60 incoming write comand on  address register
    spi_send(phy_address);
    CS_PIN = 1;
    
    CS_PIN = 0; 
    spi_send(0x40 | (MIWRL & 0x1F));  
    spi_send(byte_l);
    CS_PIN = 1;
    
    CS_PIN = 0;
    spi_send(0x40 | (MIWRH & 0x1F));
    spi_send(byte_h);
    CS_PIN = 1;
    
    while(read_register_mac(MISTAT) & 0x01)
    
    move_bank(BANK0);
}

void read_register_phy(uint8_t address, register_16bits *data){
    move_bank(BANK2);
    //3.3.1.1
    write_register(MIREGADR, address);
    uint8_t test = read_register_mac(MIREGADR);//delete
    //usart_tx_byte(test); //delete
    
    //3.3.1.2
    write_register(MICMD, 0x01);
    
    //3.3.1.3
    while(read_register_mac(MISTAT) & 0x01)
    
    //3.3.1.4
    write_register(MICMD, 0x00);
    data->high_byte = read_register_mac(MIRDH);
    data->low_byte = read_register_mac(MIRDL);
    
    
    move_bank(BANK0);
}

void phy_init(){
    //move_bank(BANK2);
    move_bank(BANK0);
    write_register_phy(PHCON1, 0x01, 0x00);
    write_register_phy(PHLCON, 0x31, 0x22);
    
    register_16bits my_data;
        read_register_phy(PHLCON, &my_data);
            usart_newline();
            USART_Transmit('P');
            USART_Transmit('H');
            USART_Transmit('L');
            USART_Transmit('C');
            USART_Transmit('O');
            USART_Transmit('N');
            USART_Transmit(':');
            USART_Transmit(' ');
            usart_tx_byte_to_hex(my_data.high_byte);
            USART_Transmit(' ');
            usart_tx_byte_to_hex(my_data.low_byte);
            
    //move_bank(BANK0);
}

uint8_t write_buffer_memory(){
    move_bank(BANK0);
    
    
    
    
    CS_PIN = 0;
    spi_send(0x60 | (0x1A & 0x1F));
    spi_send(0x0E);
    
    // Encabezado Ethernet
    spi_send(0xFF); // Destino MAC (Broadcast)
    spi_send(0xFF);
    spi_send(0xFF);
    spi_send(0xFF);
    spi_send(0xFF);
    spi_send(0xFF);
    spi_send(0x02); // Origen MAC
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0xFE);
    spi_send(0x08); // Tipo (ARP)
    spi_send(0x06);

    // Encabezado ARP
    spi_send(0x00); // Tipo de Hardware (Ethernet)
    spi_send(0x01);
    spi_send(0x08); // Tipo de Protocolo (IPv4)
    spi_send(0x00);
    spi_send(0x06); // Longitud de Hardware
    spi_send(0x04); // Longitud de Protocolo
    spi_send(0x00); // Operación (Solicitud)
    spi_send(0x01);
    spi_send(0x02); // MAC de Remitente
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0xFE);
    spi_send(0xC0); // IP de Remitente
    spi_send(0xA8);
    spi_send(0x01);
    spi_send(0x0F); // 192.168.1.15
    spi_send(0x00); // MAC de Destinatario
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0xC0); // IP de Destinatario
    spi_send(0xA8);
    spi_send(0x01);
    spi_send(0x02); // 192.168.1.2
    
    
    

    CS_PIN = 1;
    
    /* debuggin, print value of pointer to the current write address available to write next packet
    usart_newline(); 
    usart_tx_byte(read_register(EWRPTH));
    USART_Transmit('|');
    usart_tx_byte(read_register(EWRPTL));
    USART_Transmit(' ');
    */
    
                // Hay que poner el pointer de fin al final del payload.
                    /*todo*/
                write_register(ETXNDL, read_register(EWRPTL));
                write_register(ETXNDH, read_register(EWRPTH));

    // Configure before transmission 
    write_register(EIR, read_register(EIR) & 0xF5);    //Clear EIR.TXIF and EIR.TXERIF
    write_register(EIE, read_register(EIE) | 0x08);    //set EIE.TXIE
    
    
    // Start transmission
    write_register(ECON1, read_register(ECON1) | 0x08);  //Start transmission
    
    
    //write_register(ECON1, read_register(ECON1) & 0xF7);  //Deactivate transmission
    

    while((read_register(EIR) & 0x08) >> 3) //Wait for end transmission by EIR,TXIF set to 1
        

    
    if(  (read_register(EIR) & 0x02) == 0x02 ){
        //an error ocurrs, clean ESTAT.TXABRT and ESTAT.LATECOL           && (read_register(ESTAT) & 0x02)>>1
        write_register(ESTAT, read_register(ESTAT) | 0xED); //ASUMIMOS QUE SE PUSIERON EN 1 Y LIMPIAMOS ESTAT.LATECOL and ESTAT.TXABRT
        USART_Transmit('E');
        USART_Transmit('r');
        USART_Transmit('r');
        USART_Transmit(' ');
        USART_Transmit('A');
        USART_Transmit('R');
        USART_Transmit('P');
        return 0x48; //'H'
    }
    
    
    
    usart_newline();
    
    USART_Transmit('O');
    USART_Transmit('K');
    USART_Transmit(' ');
    USART_Transmit('A');
    USART_Transmit('R');
    USART_Transmit('P');
    usart_newline();
    usart_newline();
    usart_newline();
    
    return 0x47; //'G'
    
    
    
}

uint8_t dhcp_discover(){
   move_bank(BANK0);
    
    
    
    
    CS_PIN = 0;
    spi_send(0x60 | (0x1A & 0x1F));
    spi_send(0x0E);
    
    // Encabezado Ethernet
    spi_send(0xFF); // Destino MAC (Broadcast)
    spi_send(0xFF);
    spi_send(0xFF);
    spi_send(0xFF);
    spi_send(0xFF);
    spi_send(0xFF);
    spi_send(0x02); // Origen MAC
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0x00);
    spi_send(0xFE);
    spi_send(0x08); // Tipo (ARP)
    spi_send(0x00);


    // Encabezado IPv4
spi_send(0x45); // Versión (4) + Longitud del encabezado (5 palabras de 32 bits = 20 bytes)
spi_send(0x00); // Tipo de servicio (no se especifica en DHCP)
spi_send(0x00); // Longitud total (rellenar según la longitud total del paquete)
spi_send(0x1D);
spi_send(0x51); // Identificación
spi_send(0xC9);
spi_send(0x00); // Flags + Desplazamiento del fragmento
spi_send(0x00);
spi_send(0x80); // Tiempo de vida (64 en segundos)
spi_send(0x11); // Protocolo (UDP)
spi_send(0x07); // Suma de verificación del encabezado (rellenar)
spi_send(0x77);
spi_send(0xC0); // Dirección IP de origen (0.0.0.0)
spi_send(0xA8);
spi_send(0x01);
spi_send(0x09);
spi_send(0xFF); // Dirección IP de destino (255.255.255.255)
spi_send(0xFF);
spi_send(0xFF);
spi_send(0xFF);
   
    // Encabezado UDP
spi_send(0xC0); // Puerto de origen (49320 dinamic ports)
spi_send(0xA8);
spi_send(0xC0); // Puerto de destino (49320 dinamic ports)
spi_send(0xA8);
spi_send(0x00); // Longitud del datagrama UDP (rellenar)
spi_send(0x09);
spi_send(0x52); // checksum unverified
spi_send(0x9C);
    
spi_send(0x08); 
/**/
    CS_PIN = 1;
    
    /* debuggin, print value of pointer to the current write address available to write next packet
    usart_newline(); 
    usart_tx_byte(read_register(EWRPTH));
    USART_Transmit('|');
    usart_tx_byte(read_register(EWRPTL));
    USART_Transmit(' ');
    */
    
                // Hay que poner el pointer de fin al final del payload.
                    /*todo*/
                write_register(ETXNDL, read_register(EWRPTL));
                write_register(ETXNDH, read_register(EWRPTH));

    // Configure before transmission 
    write_register(EIR, read_register(EIR) & 0xF5);    //Clear EIR.TXIF and EIR.TXERIF
    write_register(EIE, read_register(EIE) | 0x08);    //set EIE.TXIE
    
    
    // Start transmission
    write_register(ECON1, read_register(ECON1) | 0x08);  //Start transmission
    
    
    //write_register(ECON1, read_register(ECON1) & 0xF7);  //Deactivate transmission
    

    while((read_register(EIR) & 0x08) >> 3) //Wait for end transmission by EIR,TXIF set to 1
        

    
    if(  (read_register(EIR) & 0x02) == 0x02 ){
        //an error ocurrs, clean ESTAT.TXABRT and ESTAT.LATECOL           && (read_register(ESTAT) & 0x02)>>1
        write_register(ESTAT, read_register(ESTAT) | 0xED); //ASUMIMOS QUE SE PUSIERON EN 1 Y LIMPIAMOS ESTAT.LATECOL and ESTAT.TXABRT
        USART_Transmit('E');
        USART_Transmit('r');
        USART_Transmit('r');
        USART_Transmit(' ');
        USART_Transmit('A');
        USART_Transmit('R');
        USART_Transmit('P');
        return 0x48; //'H'
    }
    
    
    
    usart_newline();
    
    USART_Transmit('O');
    USART_Transmit('K');
    USART_Transmit(' ');
    USART_Transmit('A');
    USART_Transmit('R');
    USART_Transmit('P');
    usart_newline();
    usart_newline();
    usart_newline();
    
    return 0x47; //'G'
    
    
    
}

uint8_t read_receive_buffer_pkt(){
    register_16bits pkt_address, nxt_pkt_address, ether_type, length_payload;
    uint8_t type_destiny, ipv4_header[20], protocol, udp_packet[10], dhcp_packet[40];
    uint8_t status;
    uint8_t mac_dst_6;
    uint8_t mac_dst_5;
    uint8_t mac_dst_4;
    uint8_t mac_dst_3;
    uint8_t mac_dst_2;
    uint8_t mac_dst_1;
    uint8_t mac_src_6;
    uint8_t mac_src_5;
    uint8_t mac_src_4;
    uint8_t mac_src_3;
    uint8_t mac_src_2;
    uint8_t mac_src_1;
    move_bank(BANK0);  // es necesario? delete 
    
    //pkt_address.low_byte = read_register(ERDPTL); 
    //pkt_address.high_byte = read_register(ERDPTH);
    
    
    CS_PIN = 0;
    
    spi_send(0x30 | 0x1A);   // Advice to ENC28J60 incoming read comand on where ERDPT pointed
    
    //START Preamble
    nxt_pkt_address.low_byte = spi_send(0xFF);  
    nxt_pkt_address.high_byte = spi_send(0xFF);
    length_payload.low_byte =spi_send(0xFF);   //rsv1
    length_payload.high_byte =spi_send(0xFF);  //rsv2
    status =(spi_send(0xFF)>> 7) & 0x01;       //rsv3
    type_destiny = spi_send(0xFF) & 0x03;  //rsv4       
        
        //START Ethernet packet
        mac_dst_6 = spi_send(0xFF); //Destination MAC
        mac_dst_5 = spi_send(0xFF); 
        mac_dst_4 = spi_send(0xFF); 
        mac_dst_3 = spi_send(0xFF); 
        mac_dst_2 = spi_send(0xFF); 
        mac_dst_1 = spi_send(0xFF); 

        mac_src_6 = spi_send(0xFF); //Source MAC
        mac_src_5 = spi_send(0xFF); 
        mac_src_4 = spi_send(0xFF); 
        mac_src_3 = spi_send(0xFF); 
        mac_src_2 = spi_send(0xFF); 
        mac_src_1 = spi_send(0xFF); 
        ether_type.high_byte = spi_send(0xFF); 
        ether_type.low_byte = spi_send(0xFF);     
    
            //START Type specific frame
            
            switch(ether_type.high_byte){
                case 0x08:
                    if(ether_type.low_byte == 0x00){
                        // Is an IPV4 packet
                        ipv4_header[0] = spi_send(0xFF);  //Version  4 for ipv4  and HLEN: IP header length (4 bits), which is the number of 32 bit words in the header.
                        ipv4_header[1] = spi_send(0xFF);  //Type of service 
                        ipv4_header[2] = spi_send(0xFF);  //Total Length: Length of header + Data (16 bits), which has a minimum value 20 bytes and the maximum is 65,535 bytes. 
                        ipv4_header[3] = spi_send(0xFF);   
                        ipv4_header[4] = spi_send(0xFF);  //Identification: Unique Packet Id for identifying the group of fragments of a single IP datagram (16 bits)
                        ipv4_header[5] = spi_send(0xFF);
                        ipv4_header[6] = spi_send(0xFF);  // 3 flags + Fragment offset
                        ipv4_header[7] = spi_send(0xFF);
                        ipv4_header[8] = spi_send(0xFF);  //Time to live
                        protocol = ipv4_header[9] = spi_send(0xFF);  //Protocolo
                        ipv4_header[10] = spi_send(0xFF);  //Header checksum
                        ipv4_header[11] = spi_send(0xFF);
                        ipv4_header[12] = spi_send(0xFF);  //Source IP
                        ipv4_header[13] = spi_send(0xFF);
                        ipv4_header[14] = spi_send(0xFF);
                        ipv4_header[15] = spi_send(0xFF);
                        ipv4_header[16] = spi_send(0xFF);  //Destination IP
                        ipv4_header[17] = spi_send(0xFF);
                        ipv4_header[18] = spi_send(0xFF);
                        ipv4_header[19] = spi_send(0xFF);
                            
                        

                    } else if(ether_type.low_byte == 0x06){
                        // Is an ARP packet

                    }
                    break;
                default:

                    break;
            }
        
                // START Protocol frame
                switch (protocol){
                    case 0x11:
                        // 17 is UDP
                            udp_packet[0] = spi_send(0xFF);  //Source Port
                            udp_packet[1] = spi_send(0xFF);
                            udp_packet[2] = spi_send(0xFF);  //Destination Port
                            udp_packet[3] = spi_send(0xFF);
                            udp_packet[4] = spi_send(0xFF);  //Length
                            udp_packet[5] = spi_send(0xFF);
                            udp_packet[6] = spi_send(0xFF);  //Checksum
                            udp_packet[7] = spi_send(0xFF);
                                
                                //start DHCP packet
                                dhcp_packet[0] = spi_send(0xFF);  //Operation Code
                                dhcp_packet[1] = spi_send(0xFF);  //Hardware Type
                                dhcp_packet[2] = spi_send(0xFF);  //Hlens
                                dhcp_packet[3] = spi_send(0xFF);  //Hops
                                dhcp_packet[4] = spi_send(0xFF);  //XID
                                dhcp_packet[5] = spi_send(0xFF);  //
                                dhcp_packet[6] = spi_send(0xFF);  //
                                dhcp_packet[7] = spi_send(0xFF);  //
                                dhcp_packet[8] = spi_send(0xFF);  //Seconds
                                dhcp_packet[9] = spi_send(0xFF);  //
                                dhcp_packet[10] = spi_send(0xFF);  //Flags
                                dhcp_packet[11] = spi_send(0xFF);  //
                                dhcp_packet[12] = spi_send(0xFF);  //Client IP Address
                                dhcp_packet[13] = spi_send(0xFF);  //
                                dhcp_packet[14] = spi_send(0xFF);  //
                                dhcp_packet[15] = spi_send(0xFF);  //
                                dhcp_packet[16] = spi_send(0xFF);  //Your IP Address
                                dhcp_packet[17] = spi_send(0xFF);  //
                                dhcp_packet[18] = spi_send(0xFF);  //
                                dhcp_packet[19] = spi_send(0xFF);  //
                                dhcp_packet[20] = spi_send(0xFF);  //Server IP Address
                                dhcp_packet[21] = spi_send(0xFF);  //
                                dhcp_packet[22] = spi_send(0xFF);  //
                                dhcp_packet[23] = spi_send(0xFF);  //
                                dhcp_packet[24] = spi_send(0xFF);  //Gateway IP Address
                                dhcp_packet[25] = spi_send(0xFF);  //
                                dhcp_packet[26] = spi_send(0xFF);  //
                                dhcp_packet[27] = spi_send(0xFF);  //

                                //faltan oros campos dhcp pero son no necesarios hasa el momento

                                //END DHCP packet
                        break;
                    case 0x06:
                        // 6 is TCP

                        break;
                    case 0x01:
                        // 1 is ICMP

                        break;
                    default:
                        
                        break;
                }
                
            //END Type specific frame
        // END Ethernet Packet
    //END Preamble  
    CS_PIN = 1;
    
    
    //point ERXRDPT to last byte of this packet = next pkt adddress less 1 address
    if(nxt_pkt_address.high_byte == 0x01 && nxt_pkt_address.low_byte == 0x00){
        write_register( ERXRDPTL, 0xFF);
        write_register( ERXRDPTH, 0x1F);
    }else if(nxt_pkt_address.low_byte > 0x00 ){
        write_register( ERXRDPTL, nxt_pkt_address.low_byte - 1 );
        write_register( ERXRDPTH, nxt_pkt_address.high_byte );      
    }else{
        write_register( ERXRDPTL, 0xFF );
        write_register( ERXRDPTH, nxt_pkt_address.high_byte - 1 );
    }
    
    write_register(ERDPTL, nxt_pkt_address.low_byte); //move to first addres of next packet 
    write_register(ERDPTH, nxt_pkt_address.high_byte);
    
    write_register( ECON2, read_register(ECON2) | 0x40 );    // less epktcnt

    delete++;
    usart_newline();
    USART_Transmit('-');
    USART_Transmit('-');
    USART_Transmit('-');
    USART_Transmit('-');
    USART_Transmit('-');
    USART_Transmit('-');
    USART_Transmit('-');
    USART_Transmit('-');
    
    usart_newline();
    
    USART_Transmit('N');
    USART_Transmit('.');
    USART_Transmit(' ');
    usart_tx_byte_to_hex(delete);
    
    usart_newline();
    
    USART_Transmit('M');
    USART_Transmit('A');
    USART_Transmit('C');
    USART_Transmit(':');
    USART_Transmit(' ');
    USART_Transmit(' ');
    move_bank(BANK3);
    usart_tx_byte_to_hex(read_register(MAADR1));
    USART_Transmit(':');
    usart_tx_byte_to_hex(read_register(MAADR2));
    USART_Transmit(':');
    usart_tx_byte_to_hex(read_register(MAADR3));
    USART_Transmit(':');
    usart_tx_byte_to_hex(read_register(MAADR4));
    USART_Transmit(':');
    usart_tx_byte_to_hex(read_register(MAADR5));
    USART_Transmit(':');
    usart_tx_byte_to_hex(read_register(MAADR6));
    move_bank(BANK0);
    
    usart_newline();
    
    USART_Transmit('M');
    USART_Transmit('A');
    USART_Transmit('C');
    USART_Transmit('d');
    USART_Transmit(' ');
    USART_Transmit(' ');
    usart_tx_byte_to_hex(mac_dst_6);
    USART_Transmit(':');
    usart_tx_byte_to_hex(mac_dst_5);
    USART_Transmit(':');
    usart_tx_byte_to_hex(mac_dst_4);
    USART_Transmit(':');
    usart_tx_byte_to_hex(mac_dst_3);
    USART_Transmit(':');
    usart_tx_byte_to_hex(mac_dst_2);
    USART_Transmit(':');
    usart_tx_byte_to_hex(mac_dst_1);
        
    usart_newline();
    
    USART_Transmit('M');
    USART_Transmit('A');
    USART_Transmit('C');
    USART_Transmit('s');
    USART_Transmit(' ');
    USART_Transmit(' ');
    usart_tx_byte_to_hex(mac_src_6);
    USART_Transmit(':');
    usart_tx_byte_to_hex(mac_src_5);
    USART_Transmit(':');
    usart_tx_byte_to_hex(mac_src_4);
    USART_Transmit(':');
    usart_tx_byte_to_hex(mac_src_3);
    USART_Transmit(':');
    usart_tx_byte_to_hex(mac_src_2);
    USART_Transmit(':');
    usart_tx_byte_to_hex(mac_src_1);
    
    
    usart_newline();
    
    USART_Transmit('T');
    USART_Transmit('y');
    USART_Transmit('p');
    USART_Transmit('e');
    USART_Transmit(':');
    USART_Transmit(' ');
    usart_tx_byte_to_hex(ether_type.high_byte);
    usart_tx_byte_to_hex(ether_type.low_byte);
    
    if(ether_type.high_byte == 0x08 && ether_type.low_byte == 0x00){
        usart_newline();
        USART_Transmit('\t');
        USART_Transmit('O');
        USART_Transmit('P');
        USART_Transmit(':');
        USART_Transmit(' ');
        usart_tx_byte_to_hex(dhcp_packet[0]);
        usart_newline();
        USART_Transmit('\t');
        USART_Transmit('I');
        USART_Transmit('P');
        USART_Transmit(':');
        USART_Transmit(' ');
        usart_tx_byte_to_hex(dhcp_packet[16]);
        USART_Transmit('.');
        usart_tx_byte_to_hex(dhcp_packet[17]);
        USART_Transmit('.');
        usart_tx_byte_to_hex(dhcp_packet[18]);
        USART_Transmit('.');
        usart_tx_byte_to_hex(dhcp_packet[19]);
    } 
    
    usart_newline();
    USART_Transmit('-');
    USART_Transmit('-');
    USART_Transmit('-');
    USART_Transmit('-');
    USART_Transmit('-');
    USART_Transmit('-');
    USART_Transmit('-');
    USART_Transmit('-');
    usart_newline();
    return 0x00;
}

void eth_buffer_init(){
   move_bank(BANK1); 
   write_register(ECON1, read_register(ECON1) & 0xFB);   //Desactiva 
   
   
   
   //write_register(ECON2, 0xC0);                          //Activa pktcnt autoincrement
   
   //write_register(ECON1, read_register(ECON1) | 0x04);   //Activa RXEN

   move_bank(BANK0); 
   
   // Configurar inicio, final y puntero del buffer transmisión.
   write_register(ETXSTL, 0x00);  // Inicio del buffer TX (bajo)
   write_register(ETXSTH, 0x00);  // Inicio del buffer TX (alto)
   
   write_register(ETXNDL, 0xFF);  // Fin del buffer de TX (bajo)
   write_register(ETXNDH, 0x00);  // Fin del buffer de TX (alto)
    
   write_register(EWRPTL, 0x00);  // Puntero
   write_register(EWRPTH, 0x00);
   
   //Configurar inicio, final y puntero del buffer recepción.
   write_register(ERXSTL, 0x00);
   write_register(ERXSTH, 0x01);
   
   write_register(ERXNDL, 0xFF);
   write_register(ERXNDH, 0x1F);
   
   write_register(ERDPTL, 0x00);     // User read pointer
   write_register(ERDPTH, 0x01);
  
   
   write_register(ERXRDPTL, 0xFF);   // Puntero ERXRDPTH
   write_register(ERXRDPTH, 0x1F);
   
   
   
    usart_newline();
    USART_Transmit('E');
    USART_Transmit('W');
    USART_Transmit('R');
    USART_Transmit('P');
    USART_Transmit('T');
    USART_Transmit(':');
    USART_Transmit(' ');
    usart_tx_byte_to_hex(read_register(EWRPTH));
    USART_Transmit(' ');
    usart_tx_byte_to_hex(read_register(EWRPTL));
    USART_Transmit(' ');
    

    usart_newline();
    USART_Transmit('E');
    USART_Transmit('R');
    USART_Transmit('D');
    USART_Transmit('P');
    USART_Transmit('T');
    USART_Transmit(':');
    USART_Transmit(' ');
    usart_tx_byte_to_hex(read_register(ERDPTH));
    USART_Transmit(' ');
    usart_tx_byte_to_hex(read_register(ERDPTL));
    USART_Transmit(' ');
}