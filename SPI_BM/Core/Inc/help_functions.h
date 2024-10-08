/*
 * help_functions.h
 *
 *  Created on: Sep 25, 2024
 *      Author: user
 */

#ifndef INC_HELP_FUNCTIONS_H_
#define INC_HELP_FUNCTIONS_H_
extern uint16_t init_PEC;
extern uint16_t converted_PEC_15[16];
extern uint16_t converted_PEC_10[16];
extern uint16_t in[16];
extern uint16_t converted_Cmd[16];
extern uint8_t data_read[64];
extern SPI_HandleTypeDef hspi2;
extern uint8_t spi_frame[16];
extern uint8_t dummy_standby[1];
extern uint8_t dummy_wakeup[120];
extern uint8_t binary_array[48];
extern uint8_t expanded_array[54];
#define CRC10_POLY 0x08F  // Polynomial: x^10 + x^7 + x^3 + x^2 + x + 1
#define CRC15_POLY 0x4599 // Polynomial: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
#define PEC10_WRITE   1
#define PEC10_READ    2
void word_to_binary_array10_data_PEC(uint16_t word, uint16_t binary_array[16]);
void byte_array_to_bit_array10_data_PEC(uint8_t *data, int data_len, uint8_t *bit_array, uint8_t *expanded_array);


void word_to_binary_array(uint16_t word, uint16_t binary_array[16]) {
    // Loop through each bit in the 16-bit word, starting from LSB
    for (int i = 0; i < 16; i++) {
        // Extract the i-th bit and store it in the array
        binary_array[i] = (word >> i) & 1;
    }
}


uint16_t binary_array_to_word(uint16_t binary_array[16]) {
    uint16_t word = 0;

    // Iterate through each bit in the binary array
    for (int i = 0; i < 16; i++) {
        // Shift the current word to the left and add the current bit
        word |= (binary_array[i] << i);
    }

    return word;
}

// Function to convert an array of bytes into an array of bits
void byte_array_to_bit_array10(uint8_t *data, int data_len, uint8_t *bit_array) {
    // Iterate through each byte in the data array
    for (int i = 0; i < data_len; i++) {
        // Extract each bit from the current byte and store it in the bit array
        for (int bit = 0; bit < 8; bit++) {
            bit_array[i * 8 + bit] = (data[i] >> (7 - bit)) & 1;  // Extract MSB first
        }
    }
}

void word_to_binary_array10(uint16_t word, uint16_t binary_array[10]) {
    // Loop through each bit in the 16-bit word, starting from LSB
    for (int i = 0; i < 10; i++) {
        // Extract the i-th bit and store it in the array
        binary_array[i] = (word >> i) & 1;
    }
}

uint16_t binary_array_to_10bit_word(uint16_t binary_array[10]) {
    uint16_t word = 0;

    // Iterate through each bit in the binary array (only 10 bits)
    for (int i = 0; i < 10; i++) {
        // Shift the current word to the left and add the current bit
        word |= (binary_array[i] << i);
    }

    return word;  // Return the 10-bit word (stored in a 16-bit variable)
}


uint16_t cmdPec(uint16_t command) {

    word_to_binary_array(init_PEC, converted_PEC_15);
    word_to_binary_array(command, converted_Cmd);

	 for(int i = 15;i > -1;i--) {

	    in[0] = converted_Cmd[i] ^ converted_PEC_15[14];
	    in[3] = in[0] ^ converted_PEC_15[2];
	    in[4] = in[0] ^ converted_PEC_15[3];
	    in[7] = in[0] ^ converted_PEC_15[6];
	    in[8] = in[0] ^ converted_PEC_15[7];
	    in[10] = in[0] ^ converted_PEC_15[9];
	    in[14] = in[0] ^ converted_PEC_15[13];

	    converted_PEC_15[14] = in[14];
	    converted_PEC_15[13] = converted_PEC_15[12];
	    converted_PEC_15[12] = converted_PEC_15[11];
	    converted_PEC_15[11] = converted_PEC_15[10];
	    converted_PEC_15[10] = in[10];
	    converted_PEC_15[9] = converted_PEC_15[8];
	    converted_PEC_15[8] = in[8];
	    converted_PEC_15[7] = in[7];
	    converted_PEC_15[6] = converted_PEC_15[5];
	    converted_PEC_15[5] = converted_PEC_15[4];
	    converted_PEC_15[4] = in[4];
	    converted_PEC_15[3] = in[3];
	    converted_PEC_15[2] = converted_PEC_15[1];
	    converted_PEC_15[1] = converted_PEC_15[0];
	    converted_PEC_15[0] = in[0];

	    }

	 return binary_array_to_word(converted_PEC_15);

}


uint16_t dataPec(uint8_t bytes, uint8_t *data) {




    byte_array_to_bit_array10_data_PEC(data, 6, binary_array,expanded_array);
    word_to_binary_array10_data_PEC(init_PEC,converted_PEC_10);


    for (int i = 53; i > -1; i--) {

        in[0] = expanded_array[i] ^ converted_PEC_10[9];
        in[1] = in[0] ^ converted_PEC_10[0];
        in[2] = in[0] ^ converted_PEC_10[1];
        in[3] = in[0] ^ converted_PEC_10[2];
        in[7] = in[0] ^ converted_PEC_10[6];

        converted_PEC_10[9] = converted_PEC_10[8];
        converted_PEC_10[8] = converted_PEC_10[7];
        converted_PEC_10[7] = in[7];
        converted_PEC_10[6] = converted_PEC_10[5];
        converted_PEC_10[5] = converted_PEC_10[4];
        converted_PEC_10[4] = converted_PEC_10[3];
        converted_PEC_10[3] = in[3];
        converted_PEC_10[2] = in[2];
        converted_PEC_10[1] = in[1];
        converted_PEC_10[0] = in[0];


    }


    return binary_array_to_word(converted_PEC_10);

}


/*
 * instruction - 0 => COMMAND SPI FRAME
 * instruction - 1 => WRITE DATA SPI FRAME
 * instruction - 2 => READ DATA SPI FRAME
 * */
void construct_spi_write_frame(uint16_t command, uint8_t *spi_frame, uint16_t instruction, uint8_t *data, uint8_t data_size) {
    // Construct the 32-bit frame
    uint32_t frame = 0;
    uint16_t pec = cmdPec(command);  // Calculate the 15-bit PEC
    uint8_t frame_counter = 0;  // Start counting from 0 for the frame

    // Compose the frame:
    // 5 bits of 00000 (shifted left 27 bits)
    // 11 bits of command (shifted left 16 bits)
    // 15 bits of PEC (shifted left 1 bit)
    // 1 bit of 0
    frame |= (0x00 << 27);               // 5-bit prefix 00000
    frame |= ((command & 0x7FF) << 16);  // 11-bit command
    frame |= ((pec & 0x7FFF) << 1);      // 15-bit PEC
    frame |= 0x00;                       // 1-bit suffix 0

    // Break the 32-bit frame into 4 bytes to send via SPI
    spi_frame[frame_counter++] = (frame >> 24) & 0xFF;
    spi_frame[frame_counter++] = (frame >> 16) & 0xFF;
    spi_frame[frame_counter++] = (frame >> 8) & 0xFF;
    spi_frame[frame_counter++] = (frame) & 0xFF;

    // If it's just a command, return here
    if (instruction == 0) {
        return;
    }

    // Otherwise, add the data for a write instruction
    if (instruction == 1 && data != NULL && data_size > 0) {
        // Add the data bytes
        for (int i = 0; i < data_size; i++) {
            spi_frame[frame_counter++] = data[i];
        }

        // Calculate data PEC (implement your PEC calculation for data)
        //uint8_t data_PEC = pec10_calc(data,1);  // Replace with actual data PEC calculation
        uint16_t dPEC = dataPec(data_size, data);
        // Add the data PEC to the frame (10-bit PEC in 2 bytes)
        spi_frame[frame_counter++] = (dPEC >> 8) & 0xFF;  // High byte of PEC
        spi_frame[frame_counter++] = dPEC & 0xFF;         // Low byte of PEC
    }

    else if (instruction == 2 && data == NULL && data_size == 0) {
    	return;



    }
}

	void BMS_read_SPI(uint16_t command, uint8_t *spi_frame, uint8_t *read_frame) {
		construct_spi_write_frame(command, spi_frame, 2, NULL, 0);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, spi_frame, 4, 10);
		HAL_SPI_Receive(&hspi2, read_frame, 8, 10);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
	}

	void BMS_command_SPI(uint16_t command) {
		construct_spi_write_frame(command, spi_frame, 0, NULL, 0);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, spi_frame, 4, 10);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);

	}

	void BMS_write_SPI(uint16_t command, uint8_t *spi_frame, uint8_t *data, uint8_t data_size) {
		construct_spi_write_frame(command, spi_frame, 1, data, data_size);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, spi_frame, data_size + 4 + 2, 10);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
	}




	void wakeup_dummy() {
	    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
	    HAL_SPI_Transmit(&hspi2, dummy_wakeup, 120, 10);
	    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET); //Send a dummy byte to trigger IC
	    HAL_Delay(1.5);
	}

	void send_dummy_byte() {
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, dummy_standby, 1, 10);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET); //Send a dummy byte to trigger IC
		HAL_Delay(0.1);

	}


	// Function to convert an array of bytes into an array of bits
	void byte_array_to_bit_array10_data_PEC(uint8_t *data, int data_len, uint8_t *bit_array, uint8_t *expanded_array) {
	    int j = -1;
	    // Iterate through each byte in the data array
	    for (int i = data_len; i > 0; i--) {
	        // Extract each bit from the current byte and store it in the bit array
	        j++;
	        for ( int bit = 0; bit < 8; bit++) {
	            bit_array[j*8+bit] = (data[i-1] & 1);
	            data[i-1] = data[i-1] >> 1;

	        }


	    }
	    for (int i = 6; i < 54; i++) {
	        expanded_array[i] = bit_array[i-6];
	    }

	}

	void word_to_binary_array10_data_PEC(uint16_t word, uint16_t binary_array[16]) {
	    // Loop through each bit in the 16-bit word, starting from LSB
	    for (int i = 0; i < 10; i++) {
	        // Extract the i-th bit and store it in the array
	        binary_array[i] = (word >> i) & 1;
	    }
	}


#endif /* INC_HELP_FUNCTIONS_H_ */
