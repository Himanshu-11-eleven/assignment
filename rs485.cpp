#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include "stdint.h"


#define TERMINAL    "/dev/ttyUSB0"      // usb port ttyUSB0, as mentioned in the assignment
#define CRC16 0x8005                    // CRC-16 poly
#define BAUDRATE B115200                // desigred baudrate



struct DATA_FRAM_T
{
    uint8_t HEADER_FRAME;
    uint8_t ID;
    uint8_t LENGHT;
    uint8_t DATA[8];
    uint8_t CRC_L;
    uint8_t CRC_H;
};

uint16_t Cal_CRC_16(uint8_t *data, uint16_t size)
{
    uint16_t out = 0;
    int bits_read = 0, bit_flag;

    if(data == NULL)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        out <<= 1;
        out |= (*data >> bits_read) & 1; 

        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        if(bit_flag)
            out ^= CRC16;
    }

    int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= CRC16;
    }

    uint16_t crc = 0;
    i = 0x8000;
    int j = 0x0001;
    for (; i != 0; i >>=1, j <<= 1) {
        if (i & out) crc |= j;
    }

    return crc;
}

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = 100;       // get max 100 reads
    tty.c_cc[VTIME] = 10;       // wait for 10 decisecond

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}

uint8_t count_digits(int num)
{
	
	int count=0;
	do {
    num /= 10;
    ++count;
  } while (num != 0);

	return count;
}

int main()
{
    char *portname = TERMINAL;
    int serialPort;
    int wlen;
    int xlen;

    serialPort = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (serialPort < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    // baudrate 115200, 8 bits, no parity, one stop bit, according to the assignment
    set_interface_attribs(serialPort, BAUDRATE);
/* 
    preparing the data frame according to the RMD-X Servo Motor Control Protocol v3.6 Documentation
*/

    DATA_FRAM_T data_buffer;

    data_buffer.HEADER_FRAME =  0x3E;           // header frame
    data_buffer.ID =            0x01;           // ID
    data_buffer.LENGHT =        0x08;           // LENGTH
    data_buffer.DATA[0] =       0x9A;           // D0
    data_buffer.DATA[1] =       0x00;           // D1
    data_buffer.DATA[2] =       0x00;           // D2
    data_buffer.DATA[3] =       0x00;           // D3
    data_buffer.DATA[4] =       0x00;           // D4
    data_buffer.DATA[5] =       0x00;           // D5
    data_buffer.DATA[6] =       0x00;           // D6
    data_buffer.DATA[7] =       0x00;           // D7
    data_buffer.CRC_L =         0x00;           // CRC Lower order  , I'll update later
    data_buffer.CRC_H =         0x00;           // CRC Higher order , I'll update later

    xlen = sizeof(data_buffer);
    int crc, crc_l, crc_h;
    char buffer_for_str[10];
    sprintf(buffer_for_str,"%d",data_buffer.DATA[0]);


    crc = Cal_CRC_16((uint8_t*)buffer_for_str,count_digits(data_buffer.DATA[0]));
    crc_l = crc & 0x00ff;
    crc_h = crc >> 8;

    data_buffer.CRC_L = crc_l;
    data_buffer.CRC_H = crc_h;

    wlen = write(serialPort, &data_buffer, sizeof(data_buffer));
    if (wlen != xlen) {
        printf("Error from write:- %d, %d\n", wlen, errno);
    }
    tcdrain(serialPort);    /* delay for output */

    
    /* Reading input */
    while(1)
    {
        unsigned char buf[80];
        int read_len;

        read_len = read(serialPort, buf, sizeof(buf) - 1);
        if (read_len > 0) {

            buf[read_len] = 0;
            printf("Read %d:- \"%s\"\n", read_len, buf);

        } else if (read_len < 0) {
            printf("Error from read- %d- %s\n", read_len, strerror(errno));
        } else { 
            printf("Timeout from read\n");
        }               
    }
}