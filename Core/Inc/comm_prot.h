/**
  *********************************************************************************************************************************************************
  @file     :comm_prot.h
  @brief    :Definition of the communication protocol library
  *********************************************************************************************************************************************************
  ESPlot allows real-time communication between an embedded system and a computer offering signal processing and plotting capabilities and it relies on
  hardware graphics acceleration for systems disposing of OpenGL-compatible graphic units. More info at www.uni-saarland.de/lehrstuhl/nienhaus/esplot.
 
  Copyright (C) Universität des Saarlandes 2020. Authors: Emanuele Grasso and Niklas König.
 
  The Software and the associated materials have been developed at the Universität des Saarlandes (hereinafter "UdS").
  Any copyright or patent right is owned by and proprietary material of the UdS hereinafter the “Licensor”.
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Affero General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU Affero General Public License for more details.
 
  You should have received a copy of the GNU Affero General Public License
  along with this program. If not, see <https://www.gnu.org/licenses/>.
 
  This Agreement shall be governed by the laws of the Federal Republic of Germany except for the UN Sales Convention and the German rules of conflict of law.
 
  Commercial licensing opportunities
  For commercial uses of the Software beyond the conditions applied by AGPL 3.0 please contact the Licensor sending an email to patentverwertungsagentur@uni-saarland.de
  *********************************************************************************************************************************************************
  */

#ifndef comm_prot_H
#define	comm_prot_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "color_def.h"
#include "comm_prot_config.h"

/**
 * Version of the used protocol
 */
#define COMM_PROT_VERSION 90
    
/**
 * Defines for assignments to plots
 */
#define PLOT_NONE		0
#define PLOT_NR_1       (1 << 0)
#define PLOT_NR_2       (1 << 1)
#define PLOT_NR_3       (1 << 2)
#define PLOT_NR_4       (1 << 3)
#define PLOT_NR_5       (1 << 4)
#define PLOT_NR_6       (1 << 5)
#define PLOT_NR_7       (1 << 6)
#define PLOT_NR_8       (1 << 7)
#define PLOT_NR_9       (1 << 8)
#define PLOT_NR_10      (1 << 9)
#define PLOT_NR_11      (1 << 10)
#define PLOT_NR_12      (1 << 11)
#define PLOT_NR_13      (1 << 12)
#define PLOT_NR_14      (1 << 13)
#define PLOT_NR_15      (1 << 14)
#define PLOT_NR_16      (1 << 15)

/**
 * Defines for signals types
 */
#define TYPE_FLOAT		6
#define TYPE_INT32		5
#define TYPE_UINT32		4
#define TYPE_INT16		3
#define	TYPE_UINT16		2
#define TYPE_UINT8		1
#define TYPE_INT8		0

/**
 * Defines if conversion factor is to be applied
 */
#define SCALING_FACTOR_APPLIED		1
#define SCALING_FACTOR_NOT_APPLIED 	0

/**
 * Defines for the record functionality
 */
#define NO_CMD		0
#define RECORD_CMD	1

/**
 * Calculation of the Info Frame Size (20 bytes for init and end frame and parameters, 30 bytes per each comm_data_descriptor
 *  (uint32_t) initial sequence 0xFFFFFFFF
 *  (uint8_t)  initiation byte for confirmation
 *  (uint8_t)  n_tx_data
 *  (uint8_t)  n_rx_data
 *  (uint8_t)  add a initiation byte for the used microcontroller version
 *  (uint32_t) buffer dimension that will be used for communication
 *  (uint32_t) information about the process frequency
 *  (uint8_t)  (uc->n_tx_data + uc->n_rx_data) * 30;
 *  (uint32_t) adds end sequence 0xEEEEEEEE (not used anymore)
 */
#define INFO_FRAME_SIZE (20 + (30*(N_RX_SIGNALS+N_TX_SIGNALS)))

/**
 * Calculation of the Data Frame for Tx Signal (7 bytes for init frame, 2 for command, 5 bytes per each signal (plus time stamp) assuming that its is 32 bit)
 */
#define MAX_DATA_FRAME_SIZE_TX	(7 + 2 + (5*(N_TX_SIGNALS+1)))

/**
 * Calculation of the Data Frame for Rx Signal (7 bytes for init frame, 16 bytes for serial console, 5 bytes per each 32-bit signal )
 */
#define MAX_DATA_FRAME_SIZE_RX	(7 + 16 + (5*N_RX_SIGNALS))

/**
 * Define maximum data frame size depending which data frame is longer
 */
#if (MAX_DATA_FRAME_SIZE_TX > MAX_DATA_FRAME_SIZE_RX)
	#define MAX_DATA_FRAME_SIZE MAX_DATA_FRAME_SIZE_TX
#else
	#define MAX_DATA_FRAME_SIZE MAX_DATA_FRAME_SIZE_RX
#endif

/**
 * Number of data frames per defined maximum communication buffer
 */
#define MAX_N_FRAMES_PER_BUFFER	(MAX_COMM_BUFFER_DIM / MAX_DATA_FRAME_SIZE)

/**
 * Calculation of the allocated data buffer size
 */
#define MAX_ALLOCATED_DATA_BUFFER_SIZE (MAX_N_FRAMES_PER_BUFFER * MAX_DATA_FRAME_SIZE)

/**
 * Check for errors during compiling
 */
#if (PROCESS_FREQUENCY < 5)
	#error "Minimum frequency for the exchange of the info frame is 5 Hz"
#endif

#if ((MAX_DATA_FRAME_SIZE_TX * PROCESS_FREQUENCY) > COMM_DEV_BAUDRATE)
	#error "Amount of data exceeds the maximum communication speed, reduce number of signals or process frequency or increase the baudrate of your communication interface"
#endif

typedef struct	//the data descriptor will be exchanged only during initialization phase
{
	char descriptor[16];  								//32 characters description of the data
	float scaling_factor;								//optional scaling factor
	uint16_t representation;							//assignment to plot if desired

    uint8_t idx;  										//index of the data
    uint8_t scaling_factor_applied;  					//scaling factor applied or not applied
    uint8_t type;  										//data type

    uint8_t line_width;									//line width of the plot
    uint8_t alpha;										//color of the plot
    uint8_t r;
    uint8_t g;
    uint8_t b;
} comm_data_descriptor_t;

typedef enum {BOTH_BUFFERS_READY, BUFFER_A_ACTIVE, BUFFER_B_ACTIVE} buffer_status_t;

typedef enum {UNINITIALIZED, INITIALIZED, ESTABLISHED} comm_prot_status_t;

typedef enum {BUFFER_INFO, BUFFER_A, BUFFER_B} buffer_t;

typedef struct _comm_prot
{
    uint8_t rx_i_buff[INFO_FRAME_SIZE];  //receiving buffer for info data transfer
    uint8_t tx_i_buff[INFO_FRAME_SIZE];  //transmitting buffer for info data transfer
    uint8_t rx_d_buff[MAX_ALLOCATED_DATA_BUFFER_SIZE];  //receiving buffer for data transfer
    uint8_t tx_d_buff_A[MAX_ALLOCATED_DATA_BUFFER_SIZE];  //transmission buffer A for data transfer
    uint8_t tx_d_buff_B[MAX_ALLOCATED_DATA_BUFFER_SIZE];  //transmission buffer A for data transfer
    comm_data_descriptor_t rx_data[N_RX_SIGNALS];  //pointer to a usb_data array
    comm_data_descriptor_t tx_data[N_TX_SIGNALS];  //pointer to a usb_data array
    float actual_tx_values[N_TX_SIGNALS];  //buffer for the most actual values produced by the process
    int actual_rx_values[N_RX_SIGNALS];  //buffer for the most actual values produced by the process

    char terminal_cmd[16];
    uint8_t tx_cmd;

    uint32_t process_frequency;
    uint32_t buff_dimension_info;  //size of the buffer for initialization data
    uint32_t buff_dimension_data;  //dimension of RX and TX buffers expressed in bytes
    uint32_t allocated_tx_buffer_size;  //size of the actually allocated buffer
    uint32_t number_dataframes_per_buffer;  //number of data frames in a buffer
    uint32_t buffer_pointer;  //indicates in which location of the active buffer the next date will be written
    uint32_t actual_time_stamp;  //indicates the actual time stamp of the process and is updated automatically by the add signals function

    int32_t info_delay_counter;
    int32_t info_delay_max;
    int32_t ack_missing_counter;
    int32_t ack_missing_counter_max;

    comm_prot_status_t comm_state;
    comm_prot_status_t prev_comm_state;

    uint32_t error_counter;
    buffer_status_t active_buffer;

} comm_prot;

void comm_prot_init_struct(volatile comm_prot* cp);
void  comm_prot_init_comm(volatile comm_prot* cp);  //prepare the info signal buffer and initialize the DMA transfer

uint8_t comm_prot_set_tx_data_info(volatile comm_prot* cp, uint8_t idx, uint8_t scaling_factor_applied, uint8_t type, const char* desc, uint16_t representation, float scaling_factor, uint8_t line_width, uint8_t alpha, uint8_t r, uint8_t g, uint8_t b);
uint8_t comm_prot_set_rx_data_info(volatile comm_prot* cp, uint8_t idx, const char* desc);

void comm_prot_write_tx_value(volatile comm_prot* cp, uint8_t idx, float value);
uint8_t  comm_prot_update_tx_values(volatile comm_prot* cp);

int comm_prot_get_rx_value(volatile comm_prot* cp, uint8_t idx);
uint8_t comm_prot_update_rx_values(volatile comm_prot* cp);

uint8_t  comm_prot_manager(volatile comm_prot* cp);
void comm_prot_init_transfer(volatile comm_prot* cp, buffer_t buffer);

void comm_prot_set_cmd(volatile comm_prot* cp, uint8_t cmd);

void    start_data_transfer(uint32_t buffer_size, uint8_t* tx_data_buff, uint8_t* rx_data_buff);
uint8_t get_transfer_status();
    
#ifdef	__cplusplus
}
#endif

#endif	/* comm_prot_H */

