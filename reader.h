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

#ifndef _RDB_READER_H
#define _RDB_READER_H

#ifdef  __cplusplus
extern "C" {
#endif

// function prototype
int host_cmd_reset(int fd);
int host_cmd_version(int fd,char *version);
int host_cmd_get_ant(int fd, int *getantid);
int host_cmd_set_power(int fd, int power);
int host_cmd_get_power(int fd,int *power);
int host_cmd_set_region_NA(int fd);
int host_cmd_get_region(int fd, int *region);
int host_cmd_get_temperature(int fd, int *temperature);
int host_cmd_get_gpio(int fd, int *gpio1, int *gpio2);
int host_cmd_set_gpio(int fd, int gpio, int level);
int host_cmd_set_ant(int fd, int antid);
int host_cmd_set_ant_detect(int fd);
int host_cmd_set_reader_id(int fd);
int host_cmd_get_reader_id(int fd, char *readerID);
int host_cmd_set_rflink_profile(int fd, unsigned char profileID);
int host_cmd_get_rflink_profile(int fd, unsigned char *profileID);
int host_cmd_read_time_inventory(int fd);
int host_process_tag_read(int fd);
int host_cmd_read_all_ants(int fd);
int host_process_read_all_ants(int fd, RASPI_TAG_T *raspi_tag);
int host_cmd_read_single_port(int fd);
int host_process_read_single_port(int fd, RASPI_TAG_T *raspi_tag, int *tagcnt);
int host_process_read_single_ant(int fd, RASPI_TAG_T *raspi_tag);

// Remember to change this between 1 and 4 port reader
#define SINGLE_ANTENNA 1

#endif




