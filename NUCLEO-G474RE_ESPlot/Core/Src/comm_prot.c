/**
  *********************************************************************************************************************************************************
  @file     :comm_prot.c
  @brief    :Communication protocol library functions
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
#include "comm_prot.h"

#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

void comm_prot_init_struct(volatile comm_prot* cp)
{
    //Initializes the communication structure
    cp->comm_state = UNINITIALIZED;
    cp->prev_comm_state = UNINITIALIZED;
    cp->process_frequency = PROCESS_FREQUENCY;
    cp->buff_dimension_info = INFO_FRAME_SIZE;
    cp->buff_dimension_data = 0;
    cp->allocated_tx_buffer_size = 0;
    cp->number_dataframes_per_buffer = 0;
    cp->active_buffer = BOTH_BUFFERS_READY;
    cp->buffer_pointer = 0;
    cp->actual_time_stamp = 0;
    cp->info_delay_counter = 0;
    cp->info_delay_max = 0;
    cp->ack_missing_counter = 0;
    cp->ack_missing_counter_max = 0;
    cp->error_counter = 0;

    // Initalizes the buffers
    for (unsigned int i = 0; i < N_RX_SIGNALS; ++i)
    	cp->actual_rx_values[i] = 0;
    for (unsigned int i = 0; i < N_TX_SIGNALS; ++i)
    	cp->actual_tx_values[i] = 0.0f;

    memset((uint8_t*) cp->rx_i_buff, 0, sizeof(cp->rx_i_buff));
    memset((uint8_t*) cp->tx_i_buff, 0, sizeof(cp->tx_i_buff));
    memset((uint8_t*) cp->rx_d_buff, 0, sizeof(cp->rx_d_buff));
    memset((uint8_t*) cp->tx_d_buff_A, 0, sizeof(cp->tx_d_buff_A));
    memset((uint8_t*) cp->tx_d_buff_B, 0, sizeof(cp->tx_d_buff_B));

    // Initializes the comm_descriptor_structs
    for (unsigned int i = 0; i < N_RX_SIGNALS; ++i)
    {
    	cp->rx_data[i].idx = i;
    	cp->rx_data[i].scaling_factor_applied = SCALING_FACTOR_NOT_APPLIED;
    	cp->rx_data[i].type = TYPE_FLOAT;
    	cp->rx_data[i].representation = PLOT_NONE;
    	cp->rx_data[i].scaling_factor = 1;
    	cp->rx_data[i].line_width = 4.0f;
    	strncpy((char*)cp->rx_data[i].descriptor, "NO NAME", 16);
    	cp->rx_data[i].alpha = 0; //random color
    	cp->rx_data[i].r = 0;
    	cp->rx_data[i].g = 0;
    	cp->rx_data[i].b = 0;
    }
    for (unsigned int i = 0; i < N_TX_SIGNALS; ++i)
    {
    	cp->tx_data[i].idx = i;
    	cp->tx_data[i].scaling_factor_applied = SCALING_FACTOR_NOT_APPLIED;
    	cp->tx_data[i].type = TYPE_FLOAT;
    	cp->tx_data[i].representation = PLOT_NONE;
    	cp->tx_data[i].scaling_factor = 1;
    	cp->tx_data[i].line_width = 4;
    	strncpy((char*)cp->tx_data[i].descriptor, "NO NAME", 16);

    	cp->tx_data[i].alpha = 0; //random color
    	cp->tx_data[i].r = 0;
    	cp->tx_data[i].g = 0;
    	cp->tx_data[i].b = 0;
    }
}

uint8_t comm_prot_set_tx_data_info(volatile comm_prot* cp, uint8_t idx, uint8_t scaling_factor_applied, uint8_t type, const char* desc, uint16_t representation, float scaling_factor, uint8_t line_width, uint8_t alpha, uint8_t r, uint8_t g, uint8_t b)
{
    if (idx >= N_TX_SIGNALS)
       return EXIT_FAILURE;

    cp->tx_data[idx].idx = idx;
    cp->tx_data[idx].scaling_factor_applied = scaling_factor_applied;
    cp->tx_data[idx].type = type;
    strncpy((char*)cp->tx_data[idx].descriptor, desc, 16);  //copies maximum 16 characters
    cp->tx_data[idx].representation = representation;
    cp->tx_data[idx].scaling_factor = scaling_factor;
    cp->tx_data[idx].line_width = line_width;
    cp->tx_data[idx].alpha = alpha;
    cp->tx_data[idx].r = r;
    cp->tx_data[idx].g = g;
    cp->tx_data[idx].b = b;

    return EXIT_SUCCESS;
}

uint8_t comm_prot_set_rx_data_info(volatile comm_prot* cp, uint8_t idx, const char* desc)
{
    if (idx >= N_RX_SIGNALS)
       return EXIT_FAILURE;

    cp->rx_data[idx].idx = idx;
    strncpy((char*)cp->rx_data[idx].descriptor, desc, 16);  //copies maximum 16 characters

    return EXIT_SUCCESS;
}

void comm_prot_init_comm(volatile comm_prot* cp)
{
    //Calculate timings based on the given process frequency
    cp->info_delay_max = (cp->process_frequency / 20) - 1;  //Descriptor frame usually gets transmitted every 50 ms (= 20 Hz), maximum time is 200 ms ( = 5 Hz)
    if (cp->info_delay_max < 0)
    	 cp->info_delay_max = 0;
    cp->ack_missing_counter_max = (cp->process_frequency * 5) - 1; //Wait 5s until switching from init phase to data phase
    if (cp->ack_missing_counter_max < 0)
    	 cp->ack_missing_counter_max = 0;

    //buffer of the data frame was allocated with maximum size (ALLOCATED_DATA_BUFFER_SIZE) assuming that you always have 32 bit signals
    //now calculate the real dimension of this buffer and fill the rest with zeros (which will be not sent)
    cp->buff_dimension_data = 0;
    for (unsigned int i = 0; i < N_TX_SIGNALS; ++i)
    {
        if ((cp->tx_data[i].type == TYPE_INT32)||(cp->tx_data[i].type == TYPE_UINT32)||(cp->tx_data[i].type == TYPE_FLOAT))
        	cp->buff_dimension_data += 4 * sizeof(uint8_t);  	//4 bytes
        else if ((cp->tx_data[i].type == TYPE_INT16)||(cp->tx_data[i].type == TYPE_UINT16))
        	cp->buff_dimension_data += 2 * sizeof(uint8_t);  	//2 bytes
        else if ((cp->tx_data[i].type == TYPE_INT8)||(cp->tx_data[i].type == TYPE_UINT8))
        	cp->buff_dimension_data += sizeof(uint8_t);  		//2 bytes
    }
    cp->buff_dimension_data += (sizeof(uint8_t) * N_TX_SIGNALS);  //adds one byte per each tx signal (0xEE)
    cp->buff_dimension_data += 5 * sizeof(uint8_t);  //adds 4 bytes for the time stamp + 1 byte for the 0xEE
    cp->buff_dimension_data += 7 * sizeof(uint8_t);	//adds 7 bytes for initial sequence
    cp->buff_dimension_data += 2 * sizeof(uint8_t);	//adds 7 bytes for the command sequence
    cp->number_dataframes_per_buffer = MAX_ALLOCATED_DATA_BUFFER_SIZE / cp->buff_dimension_data;  //adds 6 bytes for the initial sequence of 0xFF FF FF FF FF FF
    cp->allocated_tx_buffer_size = cp->number_dataframes_per_buffer * cp->buff_dimension_data;
    
    //Move necessary data into the info buffer
    uint32_t idx = 0;
    cp->tx_i_buff[0] = 0xFF;  //initialization byte
    cp->tx_i_buff[1] = 0xFF;
    cp->tx_i_buff[2] = 0xFF;
    cp->tx_i_buff[3] = 0xFF;
    cp->tx_i_buff[4] = 0x0F;  //initialization byte
    cp->tx_i_buff[5] = N_TX_SIGNALS;
    cp->tx_i_buff[6] = N_RX_SIGNALS;
    cp->tx_i_buff[7] = COMM_PROT_VERSION;
    cp->tx_i_buff[8] = (cp->buff_dimension_data & 0xFF000000) >> 24; // (Most Significant 8 bits)
    cp->tx_i_buff[9] = (cp->buff_dimension_data & 0x00FF0000) >> 16;
    cp->tx_i_buff[10] = (cp->buff_dimension_data & 0x0000FF00) >> 8;
    cp->tx_i_buff[11] = (cp->buff_dimension_data & 0x000000FF);  // (Least Significant 8 bits)
    cp->tx_i_buff[12] = (cp->process_frequency & 0xFF000000) >> 24; // (Most Significant 8 bits)
    cp->tx_i_buff[13] = (cp->process_frequency & 0x00FF0000) >> 16;
    cp->tx_i_buff[14] = (cp->process_frequency & 0x0000FF00) >> 8;
    cp->tx_i_buff[15] = (cp->process_frequency & 0x000000FF);  // (Least Significant 8 bits)
    idx = 16;
    for (unsigned int i = 0; i < N_TX_SIGNALS; ++i) {
        cp->tx_i_buff[idx] = cp->tx_data[i].idx;
        ++idx;
        cp->tx_i_buff[idx] = cp->tx_data[i].scaling_factor_applied;
        ++idx;
        cp->tx_i_buff[idx] = cp->tx_data[i].type;
        ++idx;

        for (unsigned int j = 0; j < 16; j++) {
            cp->tx_i_buff[idx] = cp->tx_data[i].descriptor[j];
            ++idx;
        }        
        cp->tx_i_buff[idx] = (cp->tx_data[i].representation & 0x0000FF00) >> 8;
        ++idx;
        cp->tx_i_buff[idx] = (cp->tx_data[i].representation & 0x000000FF);
        ++idx;
        int32_t temp;
        memcpy((uint32_t*) &temp, (float*) &cp->tx_data[i].scaling_factor, 4); //float to 4 bytes
        cp->tx_i_buff[idx]   = (temp & 0xFF000000) >> 24; // (Most Significant 8 bits)
        cp->tx_i_buff[idx+1] = (temp & 0x00FF0000) >> 16;
        cp->tx_i_buff[idx+2] = (temp & 0x0000FF00) >> 8;
        cp->tx_i_buff[idx+3] = (temp & 0x000000FF);  // (Least Significant 8 bits)
        idx+=4;
        cp->tx_i_buff[idx] = cp->tx_data[i].line_width;
        ++idx;
        cp->tx_i_buff[idx]   = cp->tx_data[i].alpha;
        cp->tx_i_buff[idx+1] = cp->tx_data[i].r;
        cp->tx_i_buff[idx+2] = cp->tx_data[i].g;
        cp->tx_i_buff[idx+3] = cp->tx_data[i].b;
        idx+=4;
    }
    for (unsigned int i = 0; i < N_RX_SIGNALS; ++i) {
        cp->tx_i_buff[idx] = cp->rx_data[i].idx;
        ++idx;
        cp->tx_i_buff[idx] = cp->rx_data[i].scaling_factor_applied;
        ++idx;
        cp->tx_i_buff[idx] = cp->rx_data[i].type;
        ++idx;

        for (unsigned int j = 0; j < 16; j++) {
            cp->tx_i_buff[idx] = cp->rx_data[i].descriptor[j];
            ++idx;
        }
        cp->tx_i_buff[idx] = (cp->rx_data[i].representation & 0x0000FF00) >> 8;
        ++idx;
        cp->tx_i_buff[idx] = (cp->rx_data[i].representation & 0x000000FF);
        ++idx;
        int32_t temp;
        memcpy((uint32_t*) &temp, (float*) &cp->rx_data[i].scaling_factor, 4); //float to 4 bytes
        cp->tx_i_buff[idx]   = (temp & 0xFF000000) >> 24; // (Most Significant 8 bits)
        cp->tx_i_buff[idx+1] = (temp & 0x00FF0000) >> 16;
        cp->tx_i_buff[idx+2] = (temp & 0x0000FF00) >> 8;
        cp->tx_i_buff[idx+3] = (temp & 0x000000FF);  // (Least Significant 8 bits)
        idx+=4;
        cp->tx_i_buff[idx] = cp->rx_data[i].line_width;
        ++idx;
        cp->tx_i_buff[idx]   = cp->rx_data[i].alpha;
        cp->tx_i_buff[idx+1] = cp->rx_data[i].r;
        cp->tx_i_buff[idx+2] = cp->rx_data[i].g;
        cp->tx_i_buff[idx+3] = cp->rx_data[i].b;
        idx+=4;
    }
    cp->tx_i_buff[idx] = 0xEE;  //last 4 termination bytes
    ++idx;
    cp->tx_i_buff[idx] = 0xEE;
    ++idx;
    cp->tx_i_buff[idx] = 0xEE;
    ++idx;
    cp->tx_i_buff[idx] = 0xEE;

    //Init finished
    cp->comm_state = INITIALIZED;
    cp->prev_comm_state = INITIALIZED;
   
    //Call the init of the data transfer
    comm_prot_init_transfer(cp, 0);
}

inline void comm_prot_init_transfer(volatile comm_prot* cp, buffer_t buffer)
{
    if (buffer == BUFFER_INFO)  	 //enables device for info transfer
    	start_data_transfer(INFO_FRAME_SIZE, (uint8_t*)cp->tx_i_buff, (uint8_t*)cp->rx_d_buff);
    else if (buffer == BUFFER_A)   //enables device for data transfer of first buffer
    	start_data_transfer(cp->allocated_tx_buffer_size, (uint8_t*)cp->tx_d_buff_A, (uint8_t*)cp->rx_d_buff);
    else if (buffer == BUFFER_B)  //enables device for data transfer of second buffer
    	start_data_transfer(cp->allocated_tx_buffer_size, (uint8_t*)cp->tx_d_buff_B, (uint8_t*)cp->rx_d_buff);
}

int comm_prot_get_rx_value(volatile comm_prot* cp, uint8_t idx)
{
    if (idx < N_TX_SIGNALS)
    {
        return cp->actual_rx_values[idx];
    }

    return 0;
}

uint32_t search_for_start(volatile uint8_t* rx_d_buff, volatile uint32_t allocated_buffer_size)
{
    uint32_t idx = 0;

    //Check if real rx data are available or if just the flags from the previous call of the comm_prot_manager are present
    if (((rx_d_buff[0] == 0xEE) && (rx_d_buff[1] == 0xEE) && (rx_d_buff[2] == 0xEE) && (rx_d_buff[3] == 0xEE) && (rx_d_buff[4] == 0xEE) && (rx_d_buff[5] == 0xEE)))
    	return allocated_buffer_size;

    uint8_t* buff_ptr = rx_d_buff;
    uint32_t limit = allocated_buffer_size - ((allocated_buffer_size - 1) % 7) - 1;
    uint8_t* limit_ptr = &rx_d_buff[limit];

    //if the last byte in the array is zero, modify it to a value which is not zero, but set a flag to undo this later
    uint8_t save = 0;
    if (rx_d_buff[limit] == 0)
    {
    	rx_d_buff[limit] = 0xFF;
    	save = 1;
    }

    //Fast search algorithm, will definitely end at the last entry of the array because this has been set to a nonzero value before, iterator is increased by a stepsize of 6, which makes the search even faster because the start frame is 6 bytes long
    for (; *buff_ptr == 0; buff_ptr+=6);

    //Redefined search, where we really look for the start of the start frame
    if (buff_ptr >= &rx_d_buff[5]) //Can we go 5 entries back?
    {
    	buff_ptr -= 5; //Then we go 5 entries back and make a fine search
		for (; buff_ptr < limit_ptr; ++buff_ptr)
		{
			if (*buff_ptr == 0xFF)
				break;
		}
    }
    //Start frame begins directly at zero
    else if (((rx_d_buff[0] == 0xFF) && (rx_d_buff[1] == 0xFF) && (rx_d_buff[2] == 0xFF) && (rx_d_buff[3] == 0xFF) && (rx_d_buff[4] == 0xFF) && (rx_d_buff[5] == 0xFF) && (rx_d_buff[6] != 0xFF) ))
    {
    	buff_ptr = &rx_d_buff[0];
    }
    //Start frame at the beginning is corrupted, therefore we need to do another fast and fine search
    else
    {
    	buff_ptr += 6;
        for (; *buff_ptr == 0; buff_ptr+=6);

    	buff_ptr -= 5;
		for (; buff_ptr < limit_ptr; ++buff_ptr)
		{
			if (*buff_ptr == 0xFF)
				break;
		}
    }
    
    //Undo the changes if the last entry has been modified before
    if (save == 1)
    	rx_d_buff[limit] = 0;

    //Calculate index from pointer and return it
    idx = (buff_ptr - rx_d_buff);
    if (idx > limit)
    	idx = limit;
    return idx;
}

uint8_t comm_prot_update_rx_values(volatile comm_prot* cp)
{
	//Search for the beginning of the start frame
	uint32_t idx = search_for_start(cp->rx_d_buff, cp->allocated_tx_buffer_size);

    //Rx Buff is full of zeros
    if (idx >= cp->allocated_tx_buffer_size - 6)
    {
    	cp->ack_missing_counter++;
    	if (cp->ack_missing_counter > cp->ack_missing_counter_max)
    	{
    		cp->ack_missing_counter = cp->ack_missing_counter_max;
    		cp->comm_state = INITIALIZED;
    	}
        return EXIT_FAILURE;
    }

    //No Ack Frame detected
    if (!((cp->rx_d_buff[idx] == 0xFF) && (cp->rx_d_buff[idx+1] == 0xFF) && (cp->rx_d_buff[idx+2] == 0xFF) && (cp->rx_d_buff[idx+3] == 0xFF) && (cp->rx_d_buff[idx+4] == 0xFF) && (cp->rx_d_buff[idx+5] == 0xFF) && (cp->rx_d_buff[idx+6] == 0xEE) ))
    {
    	cp->ack_missing_counter++;
    	if (cp->ack_missing_counter > cp->ack_missing_counter_max)
    	{
    		cp->ack_missing_counter = cp->ack_missing_counter_max;
    		cp->comm_state = INITIALIZED;
    	}
        return EXIT_FAILURE;
    }

    //An ack frame was detected, datas are being sent back to the microcontroller
	cp->ack_missing_counter = 0;
	cp->comm_state = ESTABLISHED;
    
	//Process the RX Signals
    idx += 7;
    for (unsigned int i = 0; i < N_RX_SIGNALS; ++i)
    {
        uint32_t temp = (((uint8_t)cp->rx_d_buff[idx] << 24) | ((uint8_t)cp->rx_d_buff[idx+1] << 16) | ((uint8_t)cp->rx_d_buff[idx+2] << 8) | (uint8_t)cp->rx_d_buff[idx+3]);
        cp->actual_rx_values[i] = (int)temp;
        idx += 5;
    }
    
    //Is terminal command available?
    if(cp->rx_d_buff[idx] == 0)
    	return EXIT_SUCCESS;

    //Get the terminal command
    for (unsigned int i = 0; i < 16; ++i)
    {
        cp->terminal_cmd[i] = (char)cp->rx_d_buff[idx];
        ++idx;
    }

    return EXIT_SUCCESS;
}

void comm_prot_write_tx_value(volatile comm_prot* cp, uint8_t idx, float value)
{
    if (idx < N_TX_SIGNALS)
    {
        cp->actual_tx_values[idx] = value;
    }
}

void comm_prot_set_cmd(volatile comm_prot* cp, uint8_t cmd)
{
	cp->tx_cmd = cmd;
}

uint8_t comm_prot_update_tx_values(volatile comm_prot* cp)
{
    if (cp->buffer_pointer >= cp->allocated_tx_buffer_size)  //allocated_tx_buffer_size is multiple of duff_dimension
        return EXIT_FAILURE;
    
    volatile uint8_t *buff;
    if ((cp->active_buffer == BOTH_BUFFERS_READY) || (cp->active_buffer == BUFFER_A_ACTIVE))
        buff = cp->tx_d_buff_A; //we are writing into Buffer A
    else
        buff = cp->tx_d_buff_B;  //we are writing into Buffer B
        
    uint32_t idx = cp->buffer_pointer;
    
    //Asign init frame and timestamp
    buff[idx] = 0xFF; buff[idx + 1] = 0xFF; buff[idx + 2] = 0xFF;
    buff[idx + 3] = 0xFF; buff[idx + 4] = 0xFF; buff[idx + 5] = 0xFF; buff[idx + 6] = 0xEE;

    buff[idx + 7] = cp->tx_cmd;
    buff[idx + 8] = 0xEE;

    buff[idx + 9] = (cp->actual_time_stamp & 0xFF000000) >> 24; // (Most Significant 8 bits)
    buff[idx + 10] = (cp->actual_time_stamp & 0x00FF0000) >> 16;
    buff[idx + 11] = (cp->actual_time_stamp & 0x0000FF00) >> 8;
    buff[idx + 12] = (cp->actual_time_stamp & 0x000000FF);  // (Least Significant 8 bits)
    buff[idx + 13] = 0xEE;
    idx = idx + 14;

    for (unsigned int i = 0; i < N_TX_SIGNALS; ++i)
    {
    	switch(cp->tx_data[i].type)
    	{

    	case TYPE_UINT8:
    	{
           	uint8_t temp = (uint8_t)cp->actual_tx_values[i];
           	buff[idx] = (temp & 0x000000FF); ++idx; // (Least Significant 8 bits)
    	}
           	break;

    	case TYPE_INT8:
    	{
           	int8_t temp = (int8_t)cp->actual_tx_values[i];
           	buff[idx] = (temp & 0x000000FF); ++idx; // (Least Significant 8 bits)
    	}
           	break;

    	case TYPE_UINT16:
    	{
        	uint16_t temp = (uint16_t)cp->actual_tx_values[i];
            buff[idx] = (temp & 0x0000FF00) >> 8; ++idx;
            buff[idx] = (temp & 0x000000FF); ++idx; // (Least Significant 8 bits)
    	}
            break;

    	case TYPE_INT16:
    	{
        	int16_t temp = (int16_t)cp->actual_tx_values[i];
            buff[idx] = (temp & 0x0000FF00) >> 8; ++idx;
            buff[idx] = (temp & 0x000000FF); ++idx; // (Least Significant 8 bits)
    	}
            break;

    	case TYPE_UINT32:
    	{
        	uint32_t temp = (uint32_t)cp->actual_tx_values[i];
            buff[idx] = (temp & 0xFF000000) >> 24; ++idx;// (Most Significant 8 bits)
            buff[idx] = (temp & 0x00FF0000) >> 16; ++idx;
            buff[idx] = (temp & 0x0000FF00) >> 8; ++idx;
            buff[idx] = (temp & 0x000000FF); ++idx; // (Least Significant 8 bits)
    	}
            break;

    	case TYPE_INT32:
    	{
        	int32_t temp = (int32_t)cp->actual_tx_values[i];
            buff[idx] = (temp & 0xFF000000) >> 24; ++idx;// (Most Significant 8 bits)
            buff[idx] = (temp & 0x00FF0000) >> 16; ++idx;
            buff[idx] = (temp & 0x0000FF00) >> 8; ++idx;
            buff[idx] = (temp & 0x000000FF); ++idx; // (Least Significant 8 bits)
    	}
            break;

    	case TYPE_FLOAT:
    	{
            int32_t temp;
            memcpy((uint32_t*) &temp, (float*) &cp->actual_tx_values[i], 4); //float to 4 bytes
            buff[idx] = (temp & 0xFF000000) >> 24; ++idx;// (Most Significant 8 bits)
            buff[idx] = (temp & 0x00FF0000) >> 16; ++idx;
            buff[idx] = (temp & 0x0000FF00) >> 8; ++idx;
            buff[idx] = (temp & 0x000000FF); ++idx; // (Least Significant 8 bits)
    	}
            break;

    	}

        buff[idx] = 0xEE; ++idx;
    }
    cp->buffer_pointer = idx;

    return EXIT_SUCCESS;
}

void reset_buffers(volatile comm_prot* cp)
{
    //first we reset the pointer, buffer and actual time stamp
    cp->buffer_pointer = 0;  //resets the pointers
    cp->active_buffer = BOTH_BUFFERS_READY;  //the active buffer is 0 (that means first A will be filled, then A will be sent while B gets filled)
    cp->actual_time_stamp = 0;  //the time stamp gets reset
    cp->info_delay_counter = 0;  //resets the delay counter
    cp->error_counter = 0;
}

uint8_t comm_prot_manager(volatile comm_prot* cp)
{
    //Check comm status
    cp->prev_comm_state  = cp->comm_state;

    //Return if unconnected
    if(cp->comm_state == UNINITIALIZED)
    	return EXIT_FAILURE;

    //Check new data (from previous cycle), comm status gets updated in here
    comm_prot_update_rx_values(cp);

    //Set the rx_buff as "read", receiving new data will overwrite this
    for (volatile uint8_t* ptr = cp->rx_d_buff; ptr < &cp->rx_d_buff[6]; ++ptr)
    	*ptr = 0xEE;

    //If connection is not established, send info frame until acknowledgment is received
    if (cp->comm_state == INITIALIZED)
    {
        if (cp->prev_comm_state == INITIALIZED)
        {
        	//Transfer is finished --> prepare for new transfer
            if (get_transfer_status() == 1)
            {
            	++cp->info_delay_counter;  //increments the info delay counter

                if (cp->info_delay_counter > cp->info_delay_max)
                {
                	comm_prot_init_transfer(cp, BUFFER_INFO);  //sends the info buffer again
                    cp->info_delay_counter = 0;  //resets the delay counter
                }
            }
        }

        else if (cp->prev_comm_state == ESTABLISHED)
          	reset_buffers(cp);

        return EXIT_SUCCESS;
    }
    
    //Connection is established --> send data
    else if (cp->comm_state == ESTABLISHED)
    {
        if (cp->prev_comm_state == INITIALIZED)
        	reset_buffers(cp);

        //Increment time stamp
        ++cp->actual_time_stamp;
        
        if (cp->active_buffer == BOTH_BUFFERS_READY)  //Nothing is sending
        {
            if (comm_prot_update_tx_values(cp) == EXIT_FAILURE)
            {
                //Buffer A has been completed for the first time, Buffer A is ready to be sent
                cp->active_buffer = BUFFER_B_ACTIVE;  //active buffer passes to B
                cp->buffer_pointer = 0;
                comm_prot_init_transfer(cp, BUFFER_A);  //starts sending Buffer A
                comm_prot_update_tx_values(cp);  //writes the last value in Buffer B
                return EXIT_SUCCESS;
            }
        }
        else
        {
            if (cp->active_buffer == BUFFER_A_ACTIVE)  //Buffer A is writing <===> Buffer B is sending
            {
                if (get_transfer_status() == 1)  //Buffer B has finished sending => we can send Buffer A IF Buffer A has finished writing
                {
                    if (comm_prot_update_tx_values(cp) == EXIT_FAILURE)  //Buffer A has finished and ready for sending
                    {
                    	comm_prot_init_transfer(cp, BUFFER_A);  //starts sending Buffer A
                        cp->active_buffer = BUFFER_B_ACTIVE;  //active buffer passes to B
                        cp->buffer_pointer = 0;
                        comm_prot_update_tx_values(cp);  //writes the last value in Buffer B
                        return EXIT_SUCCESS;
                    }
                }
                else
                {
                    if (comm_prot_update_tx_values(cp) == EXIT_FAILURE)
                    {
                        //WE LOSE VALUES => WE DO NOT WANT TO ENTER HERE
                    	++cp->error_counter;
                        //Buffer A has been completed, Buffer A is ready to be sent
                        cp->active_buffer = BUFFER_A_ACTIVE;  //active buffer stays to 1
                        return EXIT_FAILURE;
                    }
                }
            }
            else  //Buffer B is writing <===> Buffer A is sending
            {
                if (get_transfer_status() == 1)  //Buffer A has finished sending => we can send Buffer B IF Buffer B has finished writing
                {
                    if (comm_prot_update_tx_values(cp) == EXIT_FAILURE)  //Buffer B has finished and ready for sending
                    {
                    	comm_prot_init_transfer(cp, BUFFER_B);  //starts sending Buffer B
                        cp->active_buffer = BUFFER_A_ACTIVE;  //active buffer passes to A
                        cp->buffer_pointer = 0;
                        comm_prot_update_tx_values(cp);  //writes the last value in Buffer A
                        return EXIT_SUCCESS;
                    }
                }
                else
                {
                    if (comm_prot_update_tx_values(cp) == EXIT_FAILURE)
                    {
                        //WE LOSE VALUES => WE DO NOT WANT TO ENTER HERE
                    	++cp->error_counter;
                        //Buffer A has been completed, Buffer A is ready to be sent
                        cp->active_buffer = BUFFER_B_ACTIVE;  //active buffer stays to 2
                        return EXIT_FAILURE;
                    }
                }
            }
        }
    }
    
    return EXIT_SUCCESS;
}

__attribute__((weak)) void start_data_transfer(uint32_t buffer_size, uint8_t* tx_data_buff, uint8_t* rx_data_buff)
{
	(void) buffer_size;
	(void) tx_data_buff;
	(void) rx_data_buff;
}

__attribute__((weak)) uint8_t  get_transfer_status()
{
	return 0;
}
