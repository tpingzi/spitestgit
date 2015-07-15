/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.0";
//static const char *io_dev ="/dev/io_control";
static uint8_t mode = 1;
static uint8_t bits = 8;
static uint32_t speed = 8000000;
static uint16_t delay = 0;
int ret = 0;
//int write_on = 6;
//int write_off = 5;
int fd;

void _delay_ms( unsigned int x)
{
    unsigned int  i,j;
    for(i=0;i<x;i++)
       for(j=0;j<100;j++);
}

static void transfer(int fd)
{
	int ret;
	uint8_t tx[] = {
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x40, 0x00, 0x00, 0x00, 0x00, 0x95,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xAD,
		0xF0, 0x0D,
	};
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
	  //    .speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);   //返回为READ／WRITE（全双工）的字节数 spidev.c
//	if (ret < 1)
//		pabort("can't send spi message!!!!");
        printf("%d\n",ret);
//        printf("%x",SPI_IOC_MESSAGE(5));

	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
		if (!(ret % 6))
			puts("");
		printf("%.2X ", rx[ret]);
	}
	puts("");
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev0.0)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

void e_write(uint8_t data)
{

       uint8_t txb[1];                            
        txb[0] = data;
        ret = write(fd,txb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't write");

}

void ee_write(uint16_t data)
{

       uint8_t txb[2];                            
        txb[0] = (data>>8) & 0xFF;
        txb[1] = (uint8_t)data;
        ret = write(fd,txb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't write");

}

void eee_write(int data)
{

       uint8_t txb[4];                            
        txb[0] = (uint8_t)(data>>24) & 0xff;
        txb[1] = (uint8_t)(data>>16) & 0xff;
        txb[2] = (uint8_t)(data>>8) & 0xff;
        txb[3] = (uint8_t)data & 0xff;
        ret = write(fd,txb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't write");

}

void _read()
{
       
        uint8_t rxb[64] = {0,
        };
       
        //spi_transmit_byte(0xff);
        ret = read(fd,rxb,64);
        if (ret <1)
                pabort("can't read");

  //temp=SSPBUF;        
  //return rxb;
}

unsigned char e_read(unsigned char BufferOffset)
{
        uint8_t txb[1] = {
                0,
        };
        uint8_t rxb[ARRAY_SIZE(txb)] = {0,
        };
       
      //spi_transmit_byte(bufferset);      
        txb[0] = BufferOffset;
//       printf("txb :%x\n",txb[0]);
        ret = write(fd,txb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't write");

        //spi_transmit_byte(0xff);
        ret = read(fd,rxb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't read");

  //temp=SSPBUF;        
  return rxb[0];
}

unsigned char ee_read(uint16_t BufferOffset)
{
        uint8_t txb[1] = {
                0,
        };
        uint8_t rxb[2] = {0,0};
       
      //spi_transmit_byte(bufferset);      
        txb[0] = (BufferOffset>>8) | 0xff;

        printf("txb :%x\n",txb[0]);
        ret = write(fd,txb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't write");

        //spi_transmit_byte(0xff);
        ret = read(fd,rxb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't read");

  //temp=SSPBUF;        
  return rxb[0];
}

unsigned char eee_read(unsigned int BufferOffset)
{
        uint8_t txb[1] = {
                0,
        };
        uint8_t rxb[ARRAY_SIZE(txb)] = {0,
        };


        //spi_transmit_byte(0xD4);      
        txb[0] = 0xD4;
        ret = write(fd,txb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't write");

        //spi_transmit_byte(0xff);
        txb[0] = 0xff;
        ret = write(fd,txb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't write");

  //spi_transmit_byte((unsigned char)(BufferOffset>>8));   
        txb[0] = (unsigned char)(BufferOffset>>8);
        ret = write(fd,txb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't write");

        //spi_transmit_byte((unsigned char)BufferOffset);       
        txb[0] = (unsigned char)BufferOffset;
        ret = write(fd,txb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't write");

        //spi_transmit_byte(0xff);      
        txb[0] = 0xff;
        ret = write(fd,txb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't write");

        //spi_transmit_byte(0xff);
        ret = read(fd,rxb,ARRAY_SIZE(txb));
        if (ret <1)
                pabort("can't read");

  //temp=SSPBUF;        
  return rxb[0];
}

int main(int argc, char *argv[])
{
	int ret = 0;
        unsigned int i, num;

	parse_opts(argc, argv);

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device!!!!!!");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
        printf("%x\n",SPI_IOC_WR_MODE);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
 
 /* Write and  read */
 printf("Start to write data\n");

  unsigned char read;
  while(1)
  {
//     eee_write(0xABABABAB);
//     ee_write(0xAB79);
//       e_write(0xAB);
     read = e_read(0xAB);
//       _read();
//       printf("rxb: %x \n",read);  gsgsdfsfs
//     for(i=0;i<40;i++);     
  }
  //printf("\n");

  //   while(1)
  //     {	
  //      transfer(fd);
  //     }
	close(fd); 

	return ret;
}
