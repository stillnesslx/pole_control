//Ruixiaoliang
//20200731 14:17
#include "w5500_init.h"
#include "spi_port.h"
#include "wizchip_conf.h"

#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include "socket.h"
#include "dhcp.h"
#include "dns.h"

#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define HTTP_SOCKET     2

//uint8_t dhcp_buffer[256];
uint8_t dhcp_buffer[512];
// 1K seems to be enough for this buffer as well
//uint8_t dns_buffer[1024];
uint8_t dns_buffer[64];

volatile bool ip_assigned = false;

void Callback_IPAssigned(void) {
    //UART_Printf("Callback: IP assigned! Leased time: %d sec\r\n", getDHCPLeasetime());
    ip_assigned = true;
}

void Callback_IPConflict(void) {
    //UART_Printf("Callback: IP conflict!\r\n");
}

uint8_t w5500_read_byte(void)
{
    return spi1_read_write_byte(0xff);
}
void w5500_write_byte(uint8_t byte)
{
    spi1_read_write_byte(byte);
}
void w5500_read_buff(uint8_t* buff, uint16_t len)
{
    uint16_t i;
    for(i=0; i<len; i++)
    {
        buff[i] = spi1_read_write_byte(0xff);
    }
}
void w5500_write_buff(uint8_t* buff, uint16_t len)
{
    uint16_t i;
    for(i=0; i<len; i++)
    {
         spi1_read_write_byte(buff[i]);
    }
}
void w5500_init(void)
{
#if 1
    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
    wiz_NetInfo net_info = {
        { 0xEA, 0x11, 0x22, 0x33, 0x44, 0xEA },
        {192,168,29,199},
        {255,255,255,0},
        {192,168,29,1},
        {8,8,8,8},
        NETINFO_STATIC
    };
    //wiz_NetInfo net_info = {.mac  = { 0xEA, 0x11, 0x22, 0x33, 0x44, 0xEA },.dhcp = NETINFO_DHCP};
    //wiz_NetInfo net_info;
    //uint32_t ctr = 10000;
    //uint8_t addr[4];
    //uint8_t dns[4];

    //uint8_t http_socket;
    //uint8_t code;
    //uint8_t mac[6];
    //net_info.mac[0] = 0xEA;
    //net_info.mac[1] = 0x11;
    //net_info.mac[2] = 0x22;
    //net_info.mac[3] = 0x33;
    //net_info.mac[4] = 0x44;
    //net_info.mac[5] = 0xEA;
    //net_info.dhcp = NETINFO_DHCP;
    spi_init();
    reg_wizchip_cs_cbfunc(spi1_cs_low, spi1_cs_high);
    reg_wizchip_spi_cbfunc(w5500_read_byte, w5500_write_byte);
    reg_wizchip_spiburst_cbfunc(w5500_read_buff, w5500_write_buff);
    
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
    // set MAC address before using DHCP
    //setSHAR(net_info.mac);
    //getSHAR(mac);
    //rxl 20200803 11:25
    //getIPfromDHCP(net_info.ip);
    //getGWfromDHCP(net_info.gw);
    //getSNfromDHCP(net_info.sn);
    //net_info.ip
    //setSIPR(net_info.ip);
    //setGAR(net_info.gw);
    //setSUBR(net_info.sn);
    //getDNSfromDHCP(dns);
    
    wizchip_setnetinfo(&net_info);

#if 0
    DHCP_init(DHCP_SOCKET, dhcp_buffer);

    reg_dhcp_cbfunc(
        Callback_IPAssigned,
        Callback_IPAssigned,
        Callback_IPConflict
    );
    
    while((!ip_assigned) && (ctr > 0)) {
        DHCP_run();
        ctr--;
    }
    if(!ip_assigned) {
        //UART_Printf("\r\nIP was not assigned :(\r\n");
        return;
    }

    getIPfromDHCP(net_info.ip);
    getGWfromDHCP(net_info.gw);
    getSNfromDHCP(net_info.sn);

    getDNSfromDHCP(dns);
    
    wizchip_setnetinfo(&net_info);

    DNS_init(DNS_SOCKET, dns_buffer);
    
    {
        //char domain_name[] = "192.168.137.1";
        char domain_name[] = "eax.me";
        //UART_Printf("Resolving domain name \"%s\"...\r\n", domain_name);
        int8_t res = DNS_run(dns, (uint8_t*)&domain_name, addr);
        if(res != 1) {
            //UART_Printf("DNS_run() failed, res = %d", res);
            return;
        }
        //UART_Printf("Result: %d.%d.%d.%d\r\n", addr[0], addr[1], addr[2], addr[3]);
    }
    
    http_socket = HTTP_SOCKET;
    code = socket(http_socket, Sn_MR_TCP, 10888, 0);
    
    if(code != http_socket) {
        //UART_Printf("socket() failed, code = %d\r\n", code);
        return;
    }

    code = connect(http_socket, addr, 80);
    if(code != SOCK_OK) {
        //UART_Printf("connect() failed, code = %d\r\n", code);
        close(http_socket);
        return;
    }

    //UART_Printf("Connected, sending HTTP request...\r\n");
    {
        char req[] = "GET / HTTP/1.0\r\nHost: eax.me\r\n\r\n";
        //char req[] = "GET / HTTP/1.0\r\nHost: 192.168.137.1\r\n\r\n";
        uint16_t len = sizeof(req) - 1;
        uint8_t* buff = (uint8_t*)&req;
        while(len > 0) {
            //UART_Printf("Sending %d bytes...\r\n", len);
            int32_t nbytes = send(http_socket, buff, len);
            if(nbytes <= 0) {
                //UART_Printf("send() failed, %d returned\r\n", nbytes);
                close(http_socket);
                return;
            }
            //UART_Printf("%d bytes sent!\r\n", nbytes);
            len -= nbytes;
        }
    }

    //UART_Printf("Request sent. Reading response...\r\n");
    {
        char buff[32];
        for(;;) {
            int32_t nbytes = recv(http_socket, (uint8_t*)&buff, sizeof(buff)-1);
            if(nbytes == SOCKERR_SOCKSTATUS) {
                //UART_Printf("\r\nConnection closed.\r\n");
                break;
            }

            if(nbytes <= 0) {
                //UART_Printf("\r\nrecv() failed, %d returned\r\n", nbytes);
                break;
            }

            buff[nbytes] = '\0';
            //UART_Printf("%s", buff);
        }
    }

    //UART_Printf("Closing socket.\r\n");
    close(http_socket);
    #endif
#endif
    
}


//end of file

