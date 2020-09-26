/*************************************************************************
 * 
 * Bac Son Technologies  
 * __________________
 * 
 *  [2019] Bac Son Technologies LLC 
 *  All Rights Reserved.
 * 
 * NOTICE:  All information contained herein is, and remains
 * the property of Bac Son Technologies LLC and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to Bac Son Technologies LLC 
 * and its suppliers and may be covered by U.S. and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Bac Son Technologies LLC.
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>
#include <string.h>

#include "cmd.h"
#include "error.h"

static char hexchars[] = "0123456789ABCDEF";
#define READ_BUF_SIZE   4096    
#define HEADER_SIZE     7 
#define READER_SERIAL_DEV "/dev/ttyS0"

int unit_test(int fd);
unsigned char CheckSum(unsigned char *uBuff, unsigned char uBuffLen);
int hex_to_char(unsigned char *bytes, char *hex, int size);
int reader_delay_sleep();

int reader_set_serial(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
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

int host_cmd_reset(int fd)
{
	write(fd,&cmd_reset[0],cmd_reset[1]+2);
	sleep(2); // Make sure device reset before next call
	return 0;
}

int host_cmd_version(int fd, char *version)
{
	unsigned char buf[64];

	write(fd,&cmd_version[0],cmd_version[1]+2);
    reader_delay_sleep();
	read(fd,&buf,64);

	*version = buf[4] + 0x30;
	*(version+1) = '.';
	*(version+2) = buf[5] + 0x30;
	*(version+3) = '\0';
	
	return 0;
}

int host_cmd_baudrate_38400(int fd)
{
	unsigned char buf[64];

	write(fd,&cmd_baud_38400[0],cmd_baud_38400[1]+2);
    reader_delay_sleep();
	read(fd,&buf,64);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_baudrate_115200(int fd)
{
	unsigned char buf[64];

	write(fd,&cmd_baud_115200[0],cmd_baud_115200[1]+2);
	reader_delay_sleep();
	read(fd,&buf,64);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;

}

int host_cmd_set_ant(int fd, int antid)
{
	unsigned char buf[64];
	int resp_len = 6;
	switch(antid) {
		case 1:
			cmd_antid_set[4] = 0x00;
			cmd_antid_set[5] = 0xE7;
			break;
		case 2:
			cmd_antid_set[4] = 0x01;
			cmd_antid_set[5] = 0xE6;
			break;
		case 3:
			cmd_antid_set[4] = 0x02;
			cmd_antid_set[5] = 0xE5;
			break;
		case 4:
			cmd_antid_set[4] = 0x03;
			cmd_antid_set[5] = 0xE4;
			break;
		default:
			return CMD_ANT_ID_OOR;
	}
	write(fd,&cmd_antid_set[0],cmd_antid_set[1]+2);
	reader_delay_sleep();
	read(fd,&buf, resp_len);
	
	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_ant(int fd, int *antid)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_antid_get[0],cmd_antid_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];
	else
		*antid = buf[4] + 1;
	return 0;
}

int host_cmd_set_power(int fd, int power)
{
	unsigned char buf[64];
	int resp_len = 6;

	cmd_power_set[4] = power;
	cmd_power_set[5] = 0xE5 - power;

	write(fd,&cmd_power_set[0],cmd_power_set[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_power(int fd, int *power)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_power_get[0],cmd_power_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	*power = buf[4];

	return 0;
}

int host_cmd_set_region_NA(int fd)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_region_set[0],cmd_region_set[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_region(int fd, int *region)
{
	unsigned char buf[64];
	int resp_len = 8;

	write(fd,&cmd_region_get[0],cmd_region_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	*region = buf[4];

	return 0;
}

int host_cmd_get_temperature(int fd, int *temperature)
{
	unsigned char buf[64];
	int resp_len = 7;

	write(fd,&cmd_temp_get[0],cmd_temp_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);
	
	if(buf[4] == 0)
		*temperature = -buf[5];
	else if(buf[4] == 1)
		*temperature = buf[5];

	return 0;
}

int host_cmd_get_gpio(int fd, int *gpio1, int *gpio2)
{
	unsigned char buf[64];
	int resp_len = 7;

	write(fd,&cmd_gpio_get[0],cmd_gpio_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);
	
	*gpio1 = buf[4];
	*gpio2 = buf[5];

	return 0;
}

int host_cmd_set_gpio(int fd, int pin, int value)
{
	unsigned char buf[64];
	int resp_len = 6;

	if(pin == 0x03)
	{
		cmd_gpio_set[4] = 0x03;
		if(value == 0)
		{
			cmd_gpio_set[5] = 0x00;
			cmd_gpio_set[6] = 0xF6;
		}
		else	
		{
			cmd_gpio_set[5] = 0x01;
			cmd_gpio_set[6] = 0xF5;
		}
	}
	else if(pin == 0x04)
	{
		cmd_gpio_set[4] = 0x04;
		if(value == 0)
		{
			cmd_gpio_set[5] = 0x00;
			cmd_gpio_set[6] = 0xF5;
		}
		else	
		{
			cmd_gpio_set[5] = 0x01;
			cmd_gpio_set[6] = 0xF4;
		}
	}

	write(fd,&cmd_gpio_set[0],cmd_gpio_set[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);
	
	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}


int host_cmd_set_ant_detect(int fd)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_ant_det_set[0],cmd_ant_det_set[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_ant_detect(int fd, int *sensity)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_ant_det_get[0],cmd_ant_det_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	*sensity = buf[4];

	return 0;
}

int host_cmd_get_reader_id(int fd, char *readerID)
{
	unsigned char buf[64];
	int resp_len = 17;

	write(fd,&cmd_readerid_get[0],cmd_readerid_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	memcpy(readerID, (char *) &buf[4], 12);

	return 0;
}

int host_cmd_set_reader_id(int fd)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_readerid_set[0],cmd_readerid_set[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_set_rflink_profile(int fd, unsigned char profileID)
{
	unsigned char buf[64];
	int resp_len = 6;

	switch(profileID) {
		case 0xD0:
			cmd_rflink_prof_set[4] = 0xD0;
			cmd_rflink_prof_set[5] = 0x22;
			break;
		case 0xD1:
			cmd_rflink_prof_set[4] = 0xD1;
			cmd_rflink_prof_set[5] = 0x21;
			break;
		case 0xD2:
			cmd_rflink_prof_set[4] = 0xD2;
			cmd_rflink_prof_set[5] = 0x20;
			break;
		case 0xD3:
			cmd_rflink_prof_set[4] = 0xD3;
			cmd_rflink_prof_set[5] = 0x1F;
			break;
		default:
			break;
	}

	write(fd,&cmd_rflink_prof_set[0],cmd_rflink_prof_set[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	if(buf[4] != CMD_SUCCESS)
		return buf[4];

	return 0;
}

int host_cmd_get_rflink_profile(int fd, unsigned char *profileID)
{
	unsigned char buf[64];
	int resp_len = 6;

	write(fd,&cmd_rflink_prof_get[0],cmd_rflink_prof_get[1]+2);
	reader_delay_sleep();
	
	read(fd,&buf, resp_len);

	*profileID = buf[4];

	return 0;
}

int host_cmd_read_time_inventory(int fd)
{
	write(fd,&cmd_read_time_inv[0],cmd_read_time_inv[1]+2);
	reader_delay_sleep();
	
	return 0;
}

int reader_delay_sleep()
{
    usleep(150000);
}

int host_cmd_read_all_ants(int fd)
{
	write(fd,&cmd_read_all_ants[0],cmd_read_all_ants[1]+2);
	reader_delay_sleep();

	return 0;
}

int host_process_read_all_ants(int fd)
{
	unsigned char buf[READ_BUF_SIZE];
	unsigned char header[HEADER_SIZE];
	unsigned char epc[64];
	int location = 0, rbyte = 0, i;
	char temptag[128];

	bzero(&buf[0],READ_BUF_SIZE); 
	rbyte = read(fd,&buf[0], READ_BUF_SIZE);

    for(i = 0; i < rbyte; i++)
    {
        if(buf[i] == 0xA0 && buf[i+3] == 0x8A)
        {
            if(buf[i+1] < 16) //Less than EPC length + PC + CSUM + CMD + FREQ_ANT
            {
                i += buf[i+1] + 1;
            }
            else
            {
                location = i+buf[i+1]-12; // cur_pos + len + header + EPC + rssi + csum
                hex_to_char(&buf[location],&epc[0],EPC_12BYTE);
				// Print tag infor to temp string compare with existed scan list before insert
				snprintf((char *) &temptag[0], TAG_SIZE, "%s %d %d %d %d\n", //NO READ COUNT
					&epc[0], EPC_12BYTE, (buf[i+4]&0x03)+1, buf[location+EPC_12BYTE],buf[i+4]>>2 );

				if(buf[location+EPC_12BYTE] == 0)
				{
					continue;
				}
				// Print out tag
				printf("Tag read: %s", (char *) &temptag[0]);
            }
        }
    }
	return 0;
}

int host_cmd_read_single_port(int fd)
{
        write(fd,&cmd_read_single_port[0],cmd_read_single_port[1]+2);
        reader_delay_sleep();

        return 0;
}

int hex_print(unsigned char *cmd, int len)
{
	int i;
	for(int i=0; i < len; i++)
		printf("%02x ", *cmd++);

	printf("\n");
}

int hex_to_char(unsigned char *bytes, char *hex, int size)
{
	while (size--)
    {
		*hex++ = hexchars[*bytes >> 4];
        *hex++ = hexchars[*bytes & 15];
        bytes++;
    }
    *hex = '\0';
}


unsigned char CheckSum(unsigned char *uBuff, unsigned char uBuffLen)
{
	unsigned char i,uSum=0;
	for(i=0;i<uBuffLen;i++)
	{
		uSum = uSum + uBuff[i];
	}
	uSum = (~uSum) + 1;
	return uSum;
}

int	unit_test(int fd)
{
	char version[16];
	int error;
	// reset
	//host_cmd_reset(fd);

	// get version
	//host_cmd_version(fd,&version[0]);
	//printf("%s\n", &version[0]);

	// ant ID
	//host_cmd_set_ant(fd, 3);

	//int getantid;
	//host_cmd_get_ant(fd, &getantid);
	//printf("Get antid: %d\n", getantid+1);

	// set power
	//host_cmd_set_power(fd, 33);

	// get power
	//int power;
	//host_cmd_get_power(fd,&power);
	//printf("Get ant power: %d\n", power);

	// Freq region NA set
	//host_cmd_set_region_NA(fd);

	// region get
	//int region;
	//host_cmd_get_region(fd, &region);
	//printf("Get region: %d\n", region);

	// temperature get
	//int temperature;
	//host_cmd_get_temperature(fd, &temperature);
	//printf("Get temperature: %d\n", temperature);

	// gpio get
	//int gpio1, gpio2;
	//host_cmd_get_gpio(fd, &gpio1, &gpio2);
	//printf("Get gpio1: %d gpio2: %d\n", gpio1, gpio2);

	// gpio set
	//host_cmd_set_gpio(fd, 0x03, 1);

	// antenna detect set
	//host_cmd_set_ant_detect(fd);

	// set reader ID
	//host_cmd_set_reader_id(fd);
	//printf("Done reader id set\n");

	// get reader ID
	//unsigned char readerID[16];
	//host_cmd_get_reader_id(fd, &readerID[0]);
	//for(int i=0; i < 12; i++)
	//	printf("0x%02x ", readerID[i]);

	// set rflink profile
	//host_cmd_set_rflink_profile(fd, 0x0D);

	// get rflink profile
	//unsigned char profileID;
	//host_cmd_get_rflink_profile(fd, &profileID);
	//printf("Get profileID: 0x%02x\n", profileID);

	// To read set antenna then read
	//error = host_cmd_set_ant(fd, 0x00);
	//if(error > 0)
	//	printf("Set ant failed error: %d\n", error);
	
	// To read set antenna then read
int i;
	for(i=0 ; i < 10; i++)
	{
		host_cmd_read_all_ants(fd);
		host_process_read_all_ants(fd);
		sleep(1);
	}
}

int main()
{
	int fd = -1, wbyte,rbyte;
	unsigned char cmd[64];
	unsigned char rsp[64];
	char version[16];

	fd = open(READER_SERIAL_DEV, O_RDWR);
	if( fd < 0)
	{
	    printf("Failed to open serial port: %s\n", READER_SERIAL_DEV);
	    return -1;
	}

	if( 0 > reader_set_serial(fd, B115200))
	    printf("Failed to setup serial port\n");

	unit_test(fd);
}

