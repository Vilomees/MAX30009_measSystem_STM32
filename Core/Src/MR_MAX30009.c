#include "main.h"
#include "MR_MAX30009_DEF.h"
#include "MR_MAX30009.h"
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern SPI_HandleTypeDef hspi1;
volatile bool   	max30009_int 					= false; // set this high when MAX30009 INT pin fires
volatile bool   	max30009_fifo_new_data_readt 	= false; // set this high in spi rx complete callback when new fifo data is available

const double MAX30009_current_magnitudes[16] = { 	1.6e-8L,  3.2e-8L,  8.0e-8L,  1.6e-7L,  3.2e-7L,  6.4e-7L,  1.6e-6L,  3.2e-6L,
													6.4e-6L, 1.28e-5L,  3.2e-5L,  6.4e-5L,  1.28e-4L, 2.56e-4L,  6.4e-4L, 1.28e-3L };

const double MAX30009_voltage_gains[4] = { 1.0L,  2.0L,  5.0L,  10.0L };

uint8_t     imp_data_index=0;

uint8_t     imp_fifo_data[1+ (MAX30009_SAMPLE_COUNT_RAW * 3)] = {0};   // byte0 - status, rest is fifo data
uint8_t     imp_fifo_data_size                      = MAX30009_SAMPLE_COUNT_RAW * 3;

struct MAX30009_status impedance_status;


struct impedance       impedance_data_array[MAX30009_FIFO_ARRAY_SIZE];

uint8_t     imp_fifo_data_rx[MAX30009_FIFO_ARRAY_SIZE] = {0};   // byte0 - status, rest is fifo data
uint8_t     imp_fifo_data_tx[MAX30009_FIFO_ARRAY_SIZE] = {0};
uint8_t     imp_fifo_array_size               = MAX30009_FIFO_ARRAY_SIZE;

uint8_t     max30009_last_decoded_sample_count   = 0;


// This array contains the configuration and calibration data of 20 frequencies
struct imp_profile imp_cal_data_gen[20] = {
											{0,	14,3,true,288,160,0.840446,0.840991,81.868617,81.870263},
											{2,	13,3,true,268,-188,0.935264,0.937805,64.493055,64.488115},
											{4,	13,3,true,213,160,1.002791,1.004605,49.773317,49.816890},
											{9,	12,3,true,-7,-188,1.073347,1.074970,28.924560,29.011976},
											{14,12,3,true,-179,-188,1.089706,1.090210,17.639019,17.712231},
											{20,12,3,true,-269,-256,1.063391,1.062665,5.733175,5.773091},
											{24,12,3,true,-187,-291,1.040809,1.040129,2.876540,2.921329},
											{28,12,3,true,-301,-291,1.008551,1.007825,0.597135,0.668758},
											{37,12,3,true,-245,-230,0.968299,0.966537,-0.412015,-0.428513},
											{45,12,3,true,-241,-234,0.960318,0.959086,-0.287524,-0.303761},
											{48,12,3,true,-209,-234,0.959749,0.958496,-0.295623,-0.283533},
											{50,12,3,true,-241,-233,0.958604,0.957541,-0.201853,-0.192995},
											{52,12,3,true,-202,-193,0.958087,0.956891,-0.086231,-0.052257},
											{53,12,3,true,-202,-195,0.957991,0.956663,-0.039715,-0.023862},
											{54,11,3,true,-199,160,1.100116,1.098560,0.037549,0.039581},
											{55,11,3,true,-201,-188,1.099964,1.098446,0.017789,0.025730},
											{56,11,3,true,-202,-188,1.100002,1.098446,0.013835,0.013855},
											{57,11,3,true,-199,160,1.100002,1.098484,0.007906,0.007917},
											{58,11,3,true,-197,-188,1.100040,1.098484,-0.001976,0.001979},
											{59,11,3,true,-264,-188,1.099053,1.097611,-0.013847,-0.003962},
										};

// This array contains the configuration and calibration data of 20 frequencies
struct imp_profile imp_cal_data_spec[10] = {
											{4,	12,3,true,213,160,1.002791,1.004605,49.773317,49.816890},
											{14,12,3,true,-179,-188,1.089706,1.090210,17.639019,17.712231},
											{23,12,3,true,-187,-291,1.040809,1.040129,2.876540,2.921329},
											{33,12,3,true,-245,-230,0.968299,0.966537,-0.412015,-0.428513},
											{42,12,3,true,-241,-234,0.960318,0.959086,-0.287524,-0.303761},
											{50,12,3,true,-241,-233,0.958604,0.957541,-0.201853,-0.192995},
											{52,12,3,true,-202,-193,0.958087,0.956891,-0.086231,-0.052257},
											{53,10,3,true,-202,-195,0.957991,0.956663,-0.039715,-0.023862},
											{55,9,3,true,-201,-188,1.099964,1.098446,0.017789,0.025730},
											{56,8,3,true,-202,-188,1.100002,1.098446,0.013835,0.013855},
										};

struct imp_profile imp_cal_data_spec2[10] = {
											{53,13,3,true,213,160,1.002791,1.004605,49.773317,49.816890},
											{53,12,3,true,-179,-188,1.089706,1.090210,17.639019,17.712231},
											{53,11,3,true,-187,-291,1.040809,1.040129,2.876540,2.921329},
											{53,10,3,true,-245,-230,0.968299,0.966537,-0.412015,-0.428513},
											{53,9,3,true,-241,-234,0.960318,0.959086,-0.287524,-0.303761},
											{53,8,3,true,-241,-233,0.958604,0.957541,-0.201853,-0.192995},
											{53,7,3,true,-202,-193,0.958087,0.956891,-0.086231,-0.052257},
											{53,6,3,true,-202,-195,0.957991,0.956663,-0.039715,-0.023862},
											{53,5,3,true,-201,-188,1.099964,1.098446,0.017789,0.025730},
											{53,4,3,true,-202,-188,1.100002,1.098446,0.013835,0.013855},
										};


void print_bin8(uint8_t value)
{
    printf("0b");
    for (int i = 7; i >= 0; i--)
    {
        printf("%d", (value >> i) & 1);
        if (i % 4 == 0 && i != 0)
            printf(" ");
    }
}

void print_bin16(uint16_t value)
{
    printf("0b");
    for (int i = 15; i >= 0; i--)
    {
        printf("%d", (value >> i) & 1);
        if (i % 4 == 0 && i != 0)
            printf(" ");
    }
}




int8_t MAX30009_GetInfo(void)
{
	return MAX30009_regRead(0xFF);
}


int32_t convertTwosComplement20ToSigned(int32_t value)
{
    value &= 0xFFFFF;
    if (value & 0x80000) {value |= ~0xFFFFF; }
    return value;
}

uint8_t MAX30009_regRead(uint8_t reg)
{
	int err;
	uint8_t rx_buffer[3];
	uint8_t tx_buffer[3] = {reg, 0x80, 0};

    HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_RESET);
    //HAL_Delay(0.1);
    err = HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer , 3, 10);
    //HAL_Delay(0.1);
    HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_SET);

	if (err < 0) {
		printf("spi_transceive failed, err: %d", err);
	}

    if(err==0)
	{
    	 return ( rx_buffer[2]);
    }
    else return err;
}

void MAX30009_regWrite(uint8_t reg, uint8_t val)
{
	int err;
	uint8_t            tx_buffer[3]   = {reg,0,val};
    HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_RESET);
    err = HAL_SPI_Transmit(&hspi1, tx_buffer, 3, 10);
    HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_SET);
	if (err < 0) {
		printf("spi_transceive failed, err: %d", err);
	}
}

uint8_t MAX30009_changeReg(uint8_t reg, uint8_t val,uint8_t bit1, uint8_t numBits)
{
    /*This is a function to change specific bits of the byte*/
	int i=0;
	uint8_t x1=0;
	for(i=7;i>bit1;i--)
	{
		x1 = x1 + (uint8_t)(1u << i);
	}
	uint8_t x2=0;
	for(i=0;i<(bit1-numBits+1);i++)
	{
		x2 = x2 + (uint8_t)(1u << i);
	}
	uint8_t newBits = x1+x2;
	uint8_t reg1 = MAX30009_regRead(reg);
	newBits = reg1&newBits;
	val = val << (bit1 - numBits+1);
	newBits = newBits+val;
	MAX30009_regWrite(reg,newBits);
	return newBits;
}

int MAX30009_regReadMany(uint8_t reg, uint8_t no_of_bytes, uint8_t* buffer)
{
	int     err = 0;
	uint8_t tx_len = 2 + no_of_bytes;
	uint8_t rx_len = 2 + no_of_bytes;

	uint8_t rx_buffer[rx_len];
	uint8_t tx_buffer[tx_len];
	memset(tx_buffer, 0, sizeof(tx_buffer));
	memset(rx_buffer, 0, sizeof(rx_buffer));
	tx_buffer[0] = reg;
	tx_buffer[1] = 0x80;

	HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_RESET);
	err = HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, rx_len, 10);
	HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_SET);

	if (err != HAL_OK) {
		printf("spi burst read failed, err: %d", err);
		return err;
	}

	if (buffer != NULL) {
		memcpy(buffer, rx_buffer, rx_len);
	}
	return err;
}

/*
 * Function to read the FIFO content to global variable with DMA
 */
uint8_t MAX30009_FifoRead_DMA(uint8_t *imp_fifo_array, uint8_t imp_fifo_array_size, bool debug)
{
    uint8_t      				status              = 0;
	uint8_t 					tx_buffer[2] 		= { 0x0C , 0x80 };
	HAL_StatusTypeDef  		    err;

	//gpio_pin_set(gpio_dev, LED_BLU_PIN, 0);

							MAX30009_regRead(MAX30009_STATUS_1_REGISTER);
	uint8_t		status2 = 	MAX30009_regRead(MAX30009_STATUS_2_REGISTER);

	if(debug)	printf("status2: %X\r\n", status2);

	uint8_t         fifoCounter[2];
	fifoCounter[1]= MAX30009_regRead(MAX30009_FIFO_COUNTER_1_REGISTER);
	fifoCounter[0]= MAX30009_regRead(MAX30009_FIFO_COUNTER_2_REGISTER);

	if(status2 & 0b01000000) {status = status | 0b00000010;} else {status = status & ~0b00000010;}//code_undr
	if(status2 & 0b00100000) {status = status | 0b00000100;} else {status = status & ~0b00000100;}//code_ovr
	if(status2 & 0b00010000) {status = status | 0b00001000;} else {status = status & ~0b00001000;}//idrv_ovr
    if(status2 & 0b01000000) {status = status | 0b00010000;} else {status = status & ~0b00010000;}//leadsoff

    uint16_t fifoCount = (( (fifoCounter[1] & 0x80) << 1 ) | fifoCounter[0]); // 1 fifo sample is 3 bytes long
    uint16_t fifoOvf   =     fifoCounter[1] & 0x7F;

	if(fifoOvf){ if(debug)printf("fifo ovf: %d\r\n", fifoCount);} else if(debug) printf("fifo: %d\r\n", fifoCount);

	HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_RESET);

	err = HAL_SPI_Transmit(&hspi1, tx_buffer, 2, 10);
	if (err != HAL_OK) {
		if(debug) printf("SPI transmit error: %d\r\n", err);
		HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_SET);
		return (status | 0b00000001);
	}

	err = HAL_SPI_Receive_DMA(&hspi1, imp_fifo_array+1, imp_fifo_array_size);
	if (err != HAL_OK) {
		if(debug) printf("SPI DMA receive start error: %d\r\n", err);
		HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_SET);
		return (status | 0b00000001);
	}

	//err = HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer , rx_len, 10);
	//HAL_GPIO_WritePin(IMP_CS_GPIO_Port, IMP_CS_Pin, GPIO_PIN_SET); do not raise it when using DMA

	status = status & ~0b00000001;
	return status;
}



struct MAX30009_status MAX30009_FifoReadRaw(bool debug)
{
    struct MAX30009_status      status = {0};
    struct raw_impedance        RawImpedance[MAX30009_SAMPLE_COUNT_RAW];
	static uint32_t             imp_sample_count;
	uint8_t                     i=0,j=0;
	uint16_t                    status2 			= 0;
	uint8_t 	                regAddr				= 0x0C;
	uint8_t                     fifoCounter[2];
    uint16_t  	                fifoCount			=0;
	uint16_t  	                fifoOvf				=0;
	uint32_t  	                temp_data			=0;
    int64_t                     real_adc_raw_avg 	=0;
    int64_t                     imag_adc_raw_avg 	=0;
    int 		                err					=0;
    imp_data_index = 0;
    max30009_last_decoded_sample_count = 0;
    memset(impedance_data_array, 0, sizeof(struct impedance) * MAX30009_SAMPLE_COUNT_RAW);
	//gpio_pin_set(gpio_dev, LED_BLU_PIN, 0);

    			MAX30009_regRead(MAX30009_STATUS_1_REGISTER);
	status2 = 	MAX30009_regRead(MAX30009_STATUS_2_REGISTER);
	//if(debug)	printf("status2: %X\r\n", status2);

	fifoCounter[1]= MAX30009_regRead(MAX30009_FIFO_COUNTER_1_REGISTER);
	fifoCounter[0]= MAX30009_regRead(MAX30009_FIFO_COUNTER_2_REGISTER);


	if(status2 & 0b10000000)                           if(debug)    printf("S:LON\r\n");
	/*
	 	LON: DC Leads On Detected: LON is set to 1 when a BioZ lead-on condition is detected.
	 	This is a read-only bit and is cleared by reading the Status 2 register.
	 	If the BioZ lead-on condition persists, the LON bit is set again.
		For the LON status bit to work when PLL is not enabled, set REF_CLK_SEL to 0 to enable the on-chip oscillator.
	 */
	if(status2 & 0b01000000) {status.code_ovr 	= true;if(debug)	printf("S:OVR\r\n");} else {status.code_ovr 	= false;}
	/*
		BIOZ_OVER: BIOZ Over Range:	BIOZ_OVER is set to 1 when the absolute value of the BioZ ADC reading exceeds the
		BioZ high threshold set by register BIOZ_HI_THRESH[7:0](0x27) for more than 128ms if CLK_FREQ_SEL = 0, or 125ms if CLK_FREQ_SEL = 1.
		This bit is cleared when the Status 2 register is read.
		If the BIOZ_OVER condition persists at the end of next BioZ sample, the bit is set to 1 again.
		This status bit is recommended for use in two-electrode and four-electrode BioZ Lead-Off detection.
	 */
	if(status2 & 0b00100000) {status.code_und 	= true;if(debug)	printf("S:UND\r\n");} else {status.code_und 	= false;}
	/*
		BIOZ_UNDR: BIOZ Under Range: BIOZ_UNDR is set to 1 when the absolute value of the BioZ ADC reading is below the
		BIOZ Low Threshold set by register BIOZ_LO_THRESH[7:0](0x26) for more than 128ms if CLK_FREQ_SEL = 0, or 125ms if CLK_FREQ_SEL = 1.
		This bit is cleared when the Status 2 register is read.
		If the BIOZ_UNDR condition persists at the end of the next BioZ sample, the bit is set to 1 again.
		This status bit is recommended for use in four-electrode BioZ Lead-Off detection.
	*/
	if(status2 & 0b00010000) {status.idrv_ovr 	= true;if(debug)	printf("S:OOR\r\n");} else {status.idrv_ovr 	= false;}
	/*
		DRV_OOR: BIOZ Current Generator Indicates Leads Off:
		DRV_OOR is set to 1 when the BioZ DRVN voltage peaks are out of range (< 0.2V or > (VAVDD - 0.2V)) for more than
		128ms if CLK_FREQ_SEL = 0, or 125ms if CLK_FREQ_SEL = 1.This bit is cleared when the Status 2 register is read.
		If the BioZ drive out-of-range condition persists, this bit continues to remain asserted.
	*/
	if(status2 & 0b00001000) {status.leadsoff	= true;if(debug)	printf("S:FPH\r\n");}
	/*
		DC_LOFF_PH: BIOZP is above High Threshold: DC_LOFF_PH is set to 1 when the BIP voltage is greater than VBIOZ_TH_H
		for more than 128ms if CLK_FREQ_SEL = 0, or 125ms if CLK_FREQ_SEL = 1.
		VBIOZ_TH_H is set by LOFF_THRESH[3:0](0x51).This is a read-only bit and it is cleared by reading the Status 2 register.
		If the lead-off condition for DC_LOFF_PH persists, this bit is set again.
	*/
	if(status2 & 0b00000100) {status.leadsoff	= true;if(debug)	printf("S:FPL\r\n");}
	/*
		DC_LOFF_PL: BIOZP is below LowThreshold	DC_LOFF_PL is set to 1 when the BIP voltage is less than VBIOZ_TH_L for more than
		128ms if CLK_FREQ_SEL = 0, or125ms if CLK_FREQ_SEL = 1.	VBIOZ_TH_L is set by LOFF_THRESH[3:0](0x51).
		This is a read-only bit and it is cleared by reading the Status 2 register. If the lead-off condition for DC_LOFF_PL
		persists, this bit is set again.
	*/
	if(status2 & 0b00000010) {status.leadsoff	= true;if(debug)	printf("S:FNH\r\n");}
	/*
		DC_LOFF_NH: BIOZN is above High Threshold: DC_LOFF_NH is set to 1 when the BIN voltage is greater than VBIOZ_TH_H for more than
		128ms if CLK_FREQ_SEL = 0, or 125ms if CLK_FREQ_SEL = 1. VBIOZ_TH_H is set by LOFF_THRESH[3:0](0x51).
		This is a read-only bit and it is cleared by reading the Status 2 register.
		If the lead-off condition for DC_LOFF_NH persists, this bit is set again
	*/

	if(status2 & 0b00000001) {status.leadsoff	= true;if(debug)	printf("S:FNL\r\n");}
	/*
		DC_LOFF_NL: BIOZN is below Low Threshold: DC_LOFF_NL is set to 1 when the BIN voltage is less than VBIOZ_TH_L for more than
		128ms if CLK_FREQ_SEL = 0, or 125ms if CLK_FREQ_SEL = 1. VBIOZ_TH_L is set by LOFF_THRESH[3:0](0x51).
		This is a read-only bit and it is cleared by reading the Status 2 register.
		If the lead-off condition for DC_LOFF_NL persists, this bit is set again.
	*/


    status.real_adc_span =  0;
    status.imag_adc_span =  0;

	fifoCount = (( (fifoCounter[1]&0x80) << 1 ) | fifoCounter[0]); // 1 fifo sample is 3 bytes long
	fifoOvf   =    fifoCounter[1]&0x7F;

	if(fifoOvf)
	 {
		//gpio_pin_toggle(gpio_dev, LED_RED_PIN);
		    if(debug)printf("fifo ovf: %d\r\n", fifoCount);
	 } else if(debug)printf("fifo: %d\r\n", fifoCount);
	err=0;

	//if(fifoCount > 6 )
	{ //confirm there are enough values stored
		uint16_t ix     = 0;
		uint16_t rx_len = 2 + (fifoCount * 3);
		uint8_t  rx_buffer[2 + MAX30009_SAMPLE_COUNT_RAW * 6];
		uint8_t  tx_buffer[2 + MAX30009_SAMPLE_COUNT_RAW * 6];
		if (rx_len > sizeof(rx_buffer)) {
			printf("FIFO count too large: %u\r\n", fifoCount);
			status.spi_fault = true;
			return status;
		}
				 tx_buffer[ix++] = regAddr;
				 tx_buffer[ix++] = 0x80;

		HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_RESET);
		err = HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer , rx_len, 10);
		HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_SET);

		if (err != HAL_OK) {
			printf("spi burst read failed, err: %d \r\n", err);
			status.spi_fault = true;
            return status;
		}

        /* Parse FIFO words into I/Q pairs.
         *
         * FIFO sõnad tulevad järjekorras: tag 0x1x (I/real), siis 0x2x (Q/imag).
         * Ühe paari kohta kasvab j ainult ÜKS kord — pärast Q-sõna vastuvõttu.
         * 0xFFFFFF = invalid data tag, 0xFFFFFE = marker tag.
         */
        uint16_t sorting_data = fifoCount;
        i = 2;
        j = 0;
        while (sorting_data > 0 && j < (MAX30009_SAMPLE_COUNT_RAW))
        {
            if ((i + 2) >= rx_len) break;  /* kaitse mittetäieliku paketi korral */

            uint8_t tag = rx_buffer[i] & 0xF0;

            if (tag == 0x10) {
                /* I (real) kanal — salvesta, oota Q-d */
                temp_data = ((uint32_t)(rx_buffer[i] & 0x0F) << 16)
                          | ((uint32_t) rx_buffer[i+1]        <<  8)
                          |             rx_buffer[i+2];
                RawImpedance[j].realbits = convertTwosComplement20ToSigned((int32_t)temp_data);
                /* EI inkrementeeri j — ootame Q-sõna paari lõpetamiseks */
            }
            else if (tag == 0x20) {
                /* Q (imag) kanal — lõpetab paari, inkrementeeri j */
                temp_data = ((uint32_t)(rx_buffer[i] & 0x0F) << 16)
                          | ((uint32_t) rx_buffer[i+1]        <<  8)
                          |             rx_buffer[i+2];
                RawImpedance[j].imagbits = convertTwosComplement20ToSigned((int32_t)temp_data);
                RawImpedance[j].index = imp_sample_count;
                RawImpedance[j].flags = status2;
                imp_sample_count++;
                j++;
            }
            else if (rx_buffer[i] == 0xFF && rx_buffer[i+1] == 0xFF && rx_buffer[i+2] == 0xFF) {
                RawImpedance[j].error = true;
                /* invalid data tag ei loo tervet paari — ära inkrementeeri j */
            }
            else if (rx_buffer[i] == 0xFF && rx_buffer[i+1] == 0xFF && rx_buffer[i+2] == 0xFE) {
                RawImpedance[j].marker = true;
                /* marker tag ei kuulu I/Q paari — ära inkrementeeri j */
            }

            i += 3;
            sorting_data--;
        }

        /* j on nüüd täielike I/Q paaride arv (mitte FIFO-sõnade arv!) */
        uint8_t pair_count = j;
        if (pair_count > 0) {
            for (uint8_t k = 0; k < pair_count; k++) {
                real_adc_raw_avg += RawImpedance[k].realbits;
                imag_adc_raw_avg += RawImpedance[k].imagbits;
            }
            real_adc_raw_avg = real_adc_raw_avg / pair_count;
            imag_adc_raw_avg = imag_adc_raw_avg / pair_count;
        }
        max30009_last_decoded_sample_count = pair_count;
	}

    status.real_adc_span =  (uint8_t)(labs(real_adc_raw_avg) >> 11) ;
    status.imag_adc_span =  (uint8_t)(labs(imag_adc_raw_avg) >> 11) ;
    status.real_adc_avg  =  (int32_t)      real_adc_raw_avg;
    status.imag_adc_avg  =  (int32_t)      imag_adc_raw_avg;

    //ble_sensor_data.packet_type = 2;
    //ble_sensor_data.packet_index++;
    //ble_sensor_data.span_real                = (uint8_t)(labs(real_adc_raw_avg) >> 11) ;
    //ble_sensor_data.span_imag                = (uint8_t)(labs(imag_adc_raw_avg) >> 11) ;
    //ble_sensor_data.imp_flags                = status2;

    if(debug) printf("RAW R:%ld, I:%ld -> %u, %u\r\n",
           (int32_t)real_adc_raw_avg, (int32_t)imag_adc_raw_avg,
           (uint8_t)status.real_adc_span, (uint8_t)status.imag_adc_span);

		//gpio_pin_set(gpio_dev, LED_BLU_PIN, 1);
	return status;
}


/*
 * MAX30009_FifoReadSamples — reads the FIFO and returns individual I/Q pairs.
 *
 * Each complete pair (one I-word followed by one Q-word) is written as a single
 * struct raw_impedance entry in samples[]. No averaging is performed.
 *
 * Returns: count of complete I/Q pairs written to samples[].
 *
 * The FIFO delivers words tagged in the upper nibble of byte[0]:
 *   0x1x = I (real) channel
 *   0x2x = Q (imaginary) channel
 * Pairs are expected to arrive in I,Q order. j advances after each Q-word.
 */
uint8_t MAX30009_FifoReadSamples(struct raw_impedance *samples, uint8_t max_samples,
                                  struct MAX30009_status *status_out, bool debug)
{
    uint8_t  fifoCounter[2];
    uint16_t fifoCount  = 0;
    uint32_t temp_data  = 0;
    int      err        = 0;

    memset(status_out, 0, sizeof(*status_out));

    /* Read and clear status registers */
    MAX30009_regRead(MAX30009_STATUS_1_REGISTER);
    uint16_t status2 = MAX30009_regRead(MAX30009_STATUS_2_REGISTER);

    if (status2 & 0b01000000) { status_out->code_ovr  = true; if(debug) printf("S:OVR\r\n"); }
    if (status2 & 0b00100000) { status_out->code_und  = true; if(debug) printf("S:UND\r\n"); }
    if (status2 & 0b00010000) { status_out->idrv_ovr  = true; if(debug) printf("S:OOR\r\n"); }
    if (status2 & 0b00001000) { status_out->leadsoff  = true; if(debug) printf("S:FPH\r\n"); }
    if (status2 & 0b00000100) { status_out->leadsoff  = true; if(debug) printf("S:FPL\r\n"); }
    if (status2 & 0b00000010) { status_out->leadsoff  = true; if(debug) printf("S:FNH\r\n"); }
    if (status2 & 0b00000001) { status_out->leadsoff  = true; if(debug) printf("S:FNL\r\n"); }

    /* Read FIFO counter */
    fifoCounter[1] = MAX30009_regRead(MAX30009_FIFO_COUNTER_1_REGISTER);
    fifoCounter[0] = MAX30009_regRead(MAX30009_FIFO_COUNTER_2_REGISTER);
    fifoCount = (uint16_t)(((fifoCounter[1] & 0x80) << 1) | fifoCounter[0]);

    if (fifoCount == 0) return 0;

    /* Fixed-size SPI buffers — no VLA */
    uint16_t rx_len = (uint16_t)(2 + fifoCount * 3);
    uint8_t  rx_buffer[2 + MAX30009_SAMPLE_COUNT_RAW * 6];
    uint8_t  tx_buffer[2 + MAX30009_SAMPLE_COUNT_RAW * 6];

    if (rx_len > sizeof(rx_buffer)) {
        printf("FifoReadSamples: count too large: %u\r\n", fifoCount);
        status_out->spi_fault = true;
        return 0;
    }

    memset(tx_buffer, 0, sizeof(tx_buffer));
    tx_buffer[0] = 0x0C;  /* FIFO data register address */
    tx_buffer[1] = 0x80;  /* read flag */

    HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_RESET);
    err = HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, rx_len, 10);
    HAL_GPIO_WritePin(Z_CS_GPIO_Port, Z_CS_Pin, GPIO_PIN_SET);

    if (err != HAL_OK) {
        printf("FifoReadSamples: SPI error %d\r\n", err);
        status_out->spi_fault = true;
        return 0;
    }

    /* Parse FIFO words into I/Q pairs */
    memset(samples, 0, sizeof(struct raw_impedance) * max_samples);

    uint8_t  j          = 0;   /* complete pair index */
    uint16_t i          = 2;   /* byte index in rx_buffer (skip 2-byte SPI header) */
    uint16_t words_left = fifoCount;

    while (words_left > 0 && j < max_samples) {
        if ((i + 2) >= rx_len) break;  /* guard against malformed data */

        uint8_t tag = rx_buffer[i] & 0xF0;

        if (tag == 0x10) {
            /* I (real) channel — store and wait for matching Q */
            temp_data = ((uint32_t)(rx_buffer[i] & 0x0F) << 16)
                      | ((uint32_t) rx_buffer[i+1]        <<  8)
                      |             rx_buffer[i+2];
            samples[j].realbits = convertTwosComplement20ToSigned((int32_t)temp_data);
        }
        else if (tag == 0x20) {
            /* Q (imaginary) channel — complete the pair, advance j */
            temp_data = ((uint32_t)(rx_buffer[i] & 0x0F) << 16)
                      | ((uint32_t) rx_buffer[i+1]        <<  8)
                      |             rx_buffer[i+2];
            samples[j].imagbits = convertTwosComplement20ToSigned((int32_t)temp_data);
            j++;
        }
        else if (rx_buffer[i] == 0xFF && rx_buffer[i+1] == 0xFF && rx_buffer[i+2] == 0xFF) {
            samples[j].error = true;
            j++;
        }
        else if (rx_buffer[i] == 0xFF && rx_buffer[i+1] == 0xFF && rx_buffer[i+2] == 0xFE) {
            samples[j].marker = true;
            j++;
        }

        i          += 3;
        words_left--;
    }

    if (debug) printf("FifoReadSamples: %u words -> %u pairs\r\n", fifoCount, j);

    return j;
}


void MAX30009_SortFifoCode(uint8_t *rx_buffer, struct impedance *impedance_data_array, struct MAX30009_status *status ,bool debug)
{
	static uint32_t             imp_sample_count;
	uint8_t                     i=0,j=0;
	uint16_t                    status2 			= 0;
    //uint16_t  	                fifoCount			= MAX30009_SAMPLE_COUNT_RAW;
	uint32_t  	                temp_data			= 0;
    int64_t                     real_adc_raw_avg 	= 0;
    int64_t                     imag_adc_raw_avg 	= 0;
    imp_data_index = 0;
    max30009_last_decoded_sample_count = 0;
    memset(impedance_data_array, 0, sizeof(struct impedance) * (MAX30009_SAMPLE_COUNT_RAW / 2));

	uint16_t sorting_data = MAX30009_SAMPLE_COUNT_RAW;
	i = 2;
	j = 0;

	while(sorting_data)
	{
		if( (rx_buffer[i] & 0xF0) == 0x10 )
		{
			temp_data = ((rx_buffer[i]& 0x0F)<<16) | (rx_buffer[i+1]<<8) | rx_buffer[i+2];
			impedance_data_array[j].realbits = convertTwosComplement20ToSigned(temp_data);
			i = i+3;
			sorting_data--;
		}
		else if( (rx_buffer[i] & 0xF0) == 0x20 )
		{
			temp_data = ((rx_buffer[i]& 0x0F)<<16) | (rx_buffer[i+1]<<8) | rx_buffer[i+2];
			impedance_data_array[j].imagbits = convertTwosComplement20ToSigned(temp_data);
			i = i+3;
			sorting_data--;
		}
		else if( (rx_buffer[i] == 0xFF )&&(rx_buffer[i+1] == 0xFF )&&(rx_buffer[i+2] == 0xFF ))
		{
			impedance_data_array[j].error = true;
			i = i+3;
			sorting_data--;
		}
		else if( (rx_buffer[i] == 0xFF )&&(rx_buffer[i+1] == 0xFF )&&(rx_buffer[i+2] == 0xFE ))
		{
			impedance_data_array[j].marker = true;
			i = i+3;
			sorting_data--;
		}
		impedance_data_array[j].index = imp_sample_count;
		impedance_data_array[j].flags = status2;
		imp_sample_count++;
		j++;
		if( j > MAX30009_SAMPLE_COUNT_RAW ) sorting_data = 0;
	}



    max30009_last_decoded_sample_count = j;
    status->index = j;

	for(uint8_t i=0; i < j; i++)
	{
		real_adc_raw_avg += impedance_data_array[i].realbits;
		imag_adc_raw_avg += impedance_data_array[i].imagbits;

	    if(ble_spectrum_mode == true) // Dynamic mode
	    {
	    	//ble_icg_data.imp[i].real   = (float) real_adc_raw_avg;
	    	//ble_icg_data.imp[i].imag   = (float) imag_adc_raw_avg;
	    }
	    else // Spectral mode
	    {

	    }

		//if(debug)printf("%u:%ld,%ld\r\n", i, impedance_data_array[i].realbits, impedance_data_array[i].imagbits);
	}




    if (j > 0)
    {
        real_adc_raw_avg = real_adc_raw_avg / j;
        imag_adc_raw_avg = imag_adc_raw_avg / j;
    }

    status->real_adc_span 			=  (uint8_t)(labs(real_adc_raw_avg) >> 11) ;
    status->imag_adc_span 			=  (uint8_t)(labs(imag_adc_raw_avg) >> 11) ;
    status->real_adc_avg  			=  (int32_t)      real_adc_raw_avg;
    status->imag_adc_avg  			=  (int32_t)      imag_adc_raw_avg;

    //ble_sensor_data.packet_type 	= 2;
    //ble_sensor_data.packet_index++;
    //ble_sensor_data.span_real       = (uint8_t)(labs(real_adc_raw_avg) >> 11) ;
    //ble_sensor_data.span_imag       = (uint8_t)(labs(imag_adc_raw_avg) >> 11) ;
    //ble_sensor_data.imp_flags       = status2;

    if(debug)printf("RAW R:%ld, I:%ld -> %u, %u\r\n", (int32_t)real_adc_raw_avg, (int32_t)imag_adc_raw_avg, (uint8_t)status->real_adc_span, (uint8_t)status->imag_adc_span);
}

void MAX30009_CalculateImpedanceBasic(struct impedance *impedance_data_array, struct MAX30009_status *status ,bool debug)
{
	#define VREF_TRUE           1.0L

	//if(debug)printf("i:%u, I:%f, g:%u,  G:%f \r\n", status->current_index, max30009_current_magnitudes[status->current_index], status->gain_index, max30009_voltage_gains[status->gain_index] );
	for(uint8_t i=0; i < 25; i++ ) // the number of re & im pairs in fifo is currently hardcoded to 25
	{
		//impedance_data_array[i].real = ((double)impedance_data_array[i].realbits * VREF_TRUE) / (max30009_current_magnitudes[status->current_index]*max30009_voltage_gains[status->gain_index]*411774.832291321379L);
		//impedance_data_array[i].imag = ((double)impedance_data_array[i].imagbits * VREF_TRUE) / (max30009_current_magnitudes[status->current_index]*max30009_voltage_gains[status->gain_index]*411774.832291321379L);
		impedance_data_array[i].real = ((double)impedance_data_array[i].realbits * VREF_TRUE) / (MAX30009_current_magnitudes[status->current_index]*MAX30009_voltage_gains[status->gain_index]*62881.6L);
		impedance_data_array[i].imag = ((double)impedance_data_array[i].imagbits * VREF_TRUE) / (MAX30009_current_magnitudes[status->current_index]*MAX30009_voltage_gains[status->gain_index]*62881.6L);



		//impedance_data_array[i].real = ((double)impedance_data_array[i].realbits) / (6.4e-4L*5.0L*411774.832291321379L);

		//impedance_data_array[i].imag = ((double)impedance_data_array[i].imagbits) / (6.4e-4L*5.0L*411774.832291321379L);

		impedance_data_array[i].mag = sqrt(impedance_data_array[i].real*impedance_data_array[i].real+impedance_data_array[i].imag*impedance_data_array[i].imag);

		impedance_data_array[i].pha = atan(impedance_data_array[i].imag/impedance_data_array[i].real)*57.29577951308232;


		//if(debug)printf("IMP:%u: I:%f, Q:%f, M:%f, P:%f \r\n", i, impedance_data_array[i].real, impedance_data_array[i].imag, impedance_data_array[i].mag, impedance_data_array[i].pha );

	}
}


uint8_t MAX30009_GetDecodedSampleCount(void)
{
    return max30009_last_decoded_sample_count;
}

int MAX30009_detect(bool debug)
{
    //if(debug) printf("max30009_detect BEGIN\r\n");
    uint8_t tempc = MAX30009_regRead(0xFF);
    if(debug) printf("RD:0x%02X-0x%02X\r\n", 0xFF, tempc);
    //if(debug) printf("max30009_detect END\r\n");
    if(tempc == 0x42){return 1;}return 0;
}



void MAX30009_start_measurement(bool debug)
{
    uint8_t bioz1;
    uint8_t st1;
    uint32_t t0;

    if (debug) printf("MAX30009_start_measurement BEGIN\r\n");

    /* 1) tühjenda vanad status bitid */
    (void)MAX30009_regRead(MAX30009_STATUS_1_REGISTER);
    (void)MAX30009_regRead(MAX30009_STATUS_2_REGISTER);
    if (debug) {
        printf("RD:0x00\r\n");
        printf("RD:0x01\r\n");
    }

    /* 2) too kiip shutdownist välja */
    MAX30009_regWrite(MAX30009_SYSTEM_CONFIGURATION_1_REGISTER, 0x00);
    if (debug) printf("WR:0x%02X-0x%02X\r\n",
                      MAX30009_SYSTEM_CONFIGURATION_1_REGISTER, 0x00);

    /* 3) hoia BG sees, aga ära lõhu OSR bitte */
    bioz1 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_1_REGISTER);
    bioz1 |= 0x04;                 // BG on
    bioz1 &= (uint8_t)~0x03;       // I/Q enne starti maha
    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_1_REGISTER, bioz1);
    if (debug) printf("WR:0x%02X-0x%02X\r\n",
                      MAX30009_BIOZ_CONFIGURATION_1_REGISTER, bioz1);

    /* 4) oota FREQ_LOCK + PHASE_LOCK */
    t0 = HAL_GetTick();
    do {
        st1 = MAX30009_regRead(MAX30009_STATUS_1_REGISTER);
        if ((st1 & (MAX30009_FREQ_LOCK_MASK | MAX30009_PHASE_LOCK_MASK)) ==
            (MAX30009_FREQ_LOCK_MASK | MAX30009_PHASE_LOCK_MASK)) {
            break;
        }
    } while ((HAL_GetTick() - t0) < 20U);

    if (debug) printf("STATUS1 lock check: 0x%02X\r\n", st1);

    /* 5) FIFO interrupt 20 sample'i juures */
    MAX30009_regWrite(MAX30009_FIFO_CONFIGURATION_1_REGISTER, 236);
    if (debug) printf("WR:0x%02X-0x%02X\r\n",
                      MAX30009_FIFO_CONFIGURATION_1_REGISTER, 236);

    /* 6) flush + FIFO_STAT_CLR + FIFO_RO */
    MAX30009_regWrite(MAX30009_FIFO_CONFIGURATION_2_REGISTER, 0x1A);
    if (debug) printf("WR:0x%02X-0x%02X\r\n",
                      MAX30009_FIFO_CONFIGURATION_2_REGISTER, 0x1A);

    /* 7) clear status pärast flushi */
    (void)MAX30009_regRead(MAX30009_STATUS_1_REGISTER);
    if (debug) printf("RD:0x00\r\n");

    /* 8) enable BG + Q + I */
    bioz1 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_1_REGISTER);
    bioz1 |= 0x07;
    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_1_REGISTER, bioz1);
    if (debug) printf("WR:0x%02X-0x%02X\r\n",
                      MAX30009_BIOZ_CONFIGURATION_1_REGISTER, bioz1);

    HAL_Delay(2);

    if (debug) printf("MAX30009_start_measurement END\r\n");
}

void MAX30009_stop_measurement(bool debug)
{
    uint8_t tempc;
    if(debug) printf("MAX30009_stop_measurement BEGIN\r\n");
    tempc = MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x00, 1, 2);  // disable Q & I
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, tempc);
    if(debug) printf("MAX30009_stop_measurement END\r\n");
}

void MAX30009_init(bool debug)	// call this on power up
{
    if(debug) printf("MAX30009_init BEGIN\r\n");

    // the register sequence in this section is required for RESET
    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_1_REGISTER    , 0b00000100);     // BIOZ_BG_EN
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, 0b00000100);

    MAX30009_regWrite(MAX30009_SYSTEM_CONFIGURATION_1_REGISTER  , 0b00000000);     // clear SHDN
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_SYSTEM_CONFIGURATION_1_REGISTER, 0b00000000);

    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_1_REGISTER     , 0b00000000);     // clear PLL_EN
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_PLL_CONFIGURATION_1_REGISTER, 0b00000000);

    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_4_REGISTER     , 0b00000000);     // clear REF_CLK_SEL
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_PLL_CONFIGURATION_4_REGISTER, 0b00000000);

    HAL_Delay(1);
    MAX30009_regWrite(MAX30009_SYSTEM_CONFIGURATION_1_REGISTER  , 0b00000001);     // RESET
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_SYSTEM_CONFIGURATION_1_REGISTER, 0b00000001);

    HAL_Delay(10);
    MAX30009_regRead(MAX30009_STATUS_1_REGISTER);
    MAX30009_regRead(MAX30009_STATUS_2_REGISTER);
    if(debug) printf("RD:0x00\r\n");
    if(debug) printf("RD:0x01\r\n");

    MAX30009_regWrite(MAX30009_SYSTEM_CONFIGURATION_1_REGISTER  , 0b00000010);     // SHUTDOWN
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_SYSTEM_CONFIGURATION_1_REGISTER, 0b00000010);
    MAX30009_regWrite(MAX30009_INTERRUPT_ENABLE_1_REGISTER, 0b10000000);    // A_FULL_EN; enable interrupt pin on A_FULL assertion
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_INTERRUPT_ENABLE_1_REGISTER, 0b10000000);
    MAX30009_regWrite(MAX30009_FIFO_CONFIGURATION_1_REGISTER    , 246);            // FIFO_A_FULL; assert A_FULL on NUM_SAMPLES_PER_INT samples
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_FIFO_CONFIGURATION_1_REGISTER, 246);

    MAX30009_regWrite(MAX30009_PIN_FUNCTIONAL_CONFIG_REGISTER   , 0b00000101);  // INT is enabled and is cleared upon reading of any status register or FIFO.
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_PIN_FUNCTIONAL_CONFIG_REGISTER, 0b00001100);// int enabled, self clear 240us,

    MAX30009_regWrite(MAX30009_OUTPUT_PIN_CONFIGURATION_REGISTER, 0b00000100);// push-pull active high
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_OUTPUT_PIN_CONFIGURATION_REGISTER, 0b00000100);

    MAX30009_regWrite(MAX30009_BIOZ_MUX_CONFIGURATION_1_REGISTER, 0b00000110);      // MUX_EN
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_MUX_CONFIGURATION_1_REGISTER, 0b00000110);
    MAX30009_regWrite(MAX30009_BIOZ_MUX_CONFIGURATION_3_REGISTER, 0b10100000);      // ASSIGN PINS
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_MUX_CONFIGURATION_3_REGISTER, 0b10100000);

    //MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_2_REGISTER, 0b00000001);      // enable threshold sense
    //if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_MUX_CONFIGURATION_2_REGISTER, 0b10100000);
    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_2_REGISTER, 0b00000001);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_2_REGISTER, 0b00000001);

    MAX30009_regWrite(MAX30009_DC_LEADS_CONFIGURATION_REGISTER, 0b00010000);      // ASSIGN PINS
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_DC_LEADS_CONFIGURATION_REGISTER, 0b10100000);

    /* AHPF bypass — oluline! Reset väärtus on 0x0 = 100Hz HPF.
       Spektroskoopia jaoks peab AHPF olema bypass (0x0F).
       Kasutaja saab hiljem H-käsuga soovitud filtri seadistada. */
    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_5_REGISTER, 0b11110000);
    if(debug) printf("WR:0x%02X-0x%02X (AHPF bypass)\r\n",
                     MAX30009_BIOZ_CONFIGURATION_5_REGISTER, 0b11110000);

    /* Sisemine parasiitkompensatsioon BIP ja BIN sisenditel.
	   EN_INT_INLOAD[0] = 1 → negatiivne mahtuvus tühistab IC sisemise parasiitmahtuvuse.
	   Parandab faasitäpsust kõrgematel sagedustel. */
	MAX30009_regWrite(MAX30009_BIOZ_MUX_CONFIGURATION_2_REGISTER, 0b00000001);
	if(debug) printf("WR:0x%02X-0x%02X (INT parasitic comp ON)\r\n",
					 MAX30009_BIOZ_MUX_CONFIGURATION_2_REGISTER, 0b00000001);

    if(debug) printf("MAX30009_init END\r\n");
}

void MAX30009_setMdiv(int val){
    uint32_t V = val;
    uint8_t  a = V >> 8;
    uint8_t  b = V &  0xFF;
    MAX30009_changeReg(MAX30009_PLL_CONFIGURATION_1_REGISTER,a,7,2);//step 3:
    MAX30009_changeReg(MAX30009_PLL_CONFIGURATION_2_REGISTER,b,7,8);//MDIV
}

void MAX30009_reset(bool debug)	// call this on power up
{
    // the register sequence in this section is required for RESET
    MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 2, 1);    // Set BIOZ_BG_EN = 1.
    MAX30009_changeReg(MAX30009_SYSTEM_CONFIGURATION_1_REGISTER , 0x00, 1, 1);    // Set SHDN = 0.
    MAX30009_changeReg(MAX30009_PLL_CONFIGURATION_4_REGISTER    , 0x00, 5, 1);    // Set REF_CLK_SEL = 0
    MAX30009_changeReg(MAX30009_PLL_CONFIGURATION_1_REGISTER    , 0x00, 0, 1);    // Set PLL_EN = 0.
    HAL_Delay(1);
    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_1_REGISTER,     0x04);          // BioZ bandgap enable
    HAL_Delay(250);                                              //

    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_1_REGISTER    ,  0x01);          // Enable PLL by setting PLL_EN to 1.

    MAX30009_changeReg(MAX30009_SYSTEM_CONFIGURATION_1_REGISTER , 0x01, 0, 1);    // Set RESET = 1 to reset all registers.
    HAL_Delay(1);
    MAX30009_regRead(0x00);  MAX30009_regRead(0x01);                              // Clear status register
    MAX30009_regWrite(MAX30009_SYSTEM_CONFIGURATION_1_REGISTER, 0x02);            // Write 0b02 to register 0x11

    MAX30009_regWrite(MAX30009_INTERRUPT_ENABLE_1_REGISTER    , 0x80);            // Write 0x80 to register 0x80
    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_4_REGISTER   , 0x20);            // Write 0x20 to register 0x1A

}

void MAX30009_general_init(bool debug)	// call this on power up
{

    MAX30009_regWrite(MAX30009_FIFO_CONFIGURATION_1_REGISTER    , 255 - 5   );
    MAX30009_regWrite(MAX30009_INTERRUPT_ENABLE_1_REGISTER      , 0b10000000);      // A_FULL_EN; enable interrupt pin on A_FULL assertion
    MAX30009_regWrite(MAX30009_BIOZ_MUX_CONFIGURATION_1_REGISTER, 0b00000110);      // MUX_EN
    MAX30009_regWrite(MAX30009_BIOZ_MUX_CONFIGURATION_3_REGISTER, 0xA0);
}


/*
void MAX30009_bioz_startup(bool debug)	// call this on power up
{
    uint8_t tempc;
    if(debug) printf("MAX30009_bioz_startup BEGIN\r\n");
    tempc = MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 2, 1);  // Set t BIOZ_BG_EN = 1.
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, tempc);
    HAL_Delay(200);
    tempc = MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 1, 1);  // Set BIOZ_Q_EN = 1.
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, tempc);
    tempc = MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 0, 1);  // Set BIOZ_I_EN  = 1.
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, tempc);
    if(debug) printf("MAX30009_bioz_startup BEGIN\r\n");
}

void MAX30009_bioz_shutdown(bool debug)	// call this on power up
{
    MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 2, 0);  // Set t BIOZ_BG_EN = 0.
    MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 1, 0);  // Set BIOZ_Q_EN = 0.
    MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 0, 0);  // Set BIOZ_I_EN  = 0.
}

void MAX30009_SetFreq(uint8_t frequency,bool debug)
{
    if(debug) printf("MAX30009_SetFreq BEGIN\r\n");

    uint8_t     bioz_state      =0;
    uint16_t    max_mdiv        =0;
    uint16_t    max_ndiv        =0;
    uint16_t    max_kdiv        =0;
    uint16_t    max_dac_osr     =0;
    uint16_t    max_adc_osr     =0;

    uint8_t     dat_pll_cnf_1   =0;
    uint8_t     dat_pll_cnf_2   =0;
    uint8_t     dat_pll_cnf_3   =0;
    uint8_t     dat_pll_cnf_4   =0;
    uint8_t     max_bioz_cnf_1  =0;
    uint8_t     max_ndiv_bits   =0;
    uint8_t     max_kdiv_bits   =0;

    uint8_t tempdata            = MAX30009_regRead( MAX30009_BIOZ_CONFIGURATION_1_REGISTER);
    bioz_state                  = tempdata & 0b00000011;
    if(debug) printf("RD:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, tempdata);

    if(bioz_state)
    {
         MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_1_REGISTER, (tempdata & 0b11111100) );
         if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, tempdata & 0b11111100);
    }
    max_bioz_cnf_1 = tempdata & 0b00001111;

    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_1_REGISTER     , dat_pll_cnf_1 | 0b00000001 );
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_PLL_CONFIGURATION_1_REGISTER, dat_pll_cnf_1 | 0b00000001);

    switch(frequency)
    {
        case 0: // 808960Hz, 197.5 SPS
            max_mdiv            = 789;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 32;
            max_adc_osr         = 128;
            break;

        case 1: // 722944Hz, 176.5 SPS
            max_mdiv            = 705;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 32;
            max_adc_osr         = 128;
            break;

        case 2: // 649216Hz, 158.5 SPS
            max_mdiv            = 633;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 32;
            max_adc_osr         = 128;
            break;

        case 3: // 581632Hz, 284.0 SPS
            max_mdiv            = 567;
            max_kdiv            = 1;
            max_ndiv            = 512;
            max_dac_osr         = 32;
            max_adc_osr         = 128;
            break;

        case 4: // 499712Hz, 244.0 SPS
            max_mdiv            = 487;
            max_kdiv            = 1;
            max_ndiv            = 512;
            max_dac_osr         = 32;
            max_adc_osr         = 128;
            break;

        case 5: // 468992Hz, 229.0 SPS
            max_mdiv            = 457;
            max_kdiv            = 1;
            max_ndiv            = 512;
            max_dac_osr         = 32;
            max_adc_osr         = 128;
            break;

        case 6: // 420864Hz, 205.5 SPS
            max_mdiv            = 821;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 64;
            max_adc_osr         = 128;
            break;

        case 7: // 377856Hz, 184.5 SPS
            max_mdiv            = 737;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 64;
            max_adc_osr         = 128;
            break;

        case 8: // 338944Hz, 165.5 SPS
            max_mdiv            = 661;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 64;
            max_adc_osr         = 128;
            break;

        case 9: // 304128Hz, 148.5 SPS
            max_mdiv            = 593;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 64;
            max_adc_osr         = 128;
            break;

        case 10: // 272896Hz, 266.5 SPS
            max_mdiv            = 532;
            max_kdiv            = 1;
            max_ndiv            = 512;
            max_dac_osr         = 64;
            max_adc_osr         = 128;
            break;

        case 11: // 249856Hz, 244.0 SPS
            max_mdiv            = 487;
            max_kdiv            = 1;
            max_ndiv            = 512;
            max_dac_osr         = 64;
            max_adc_osr         = 128;
            break;

        case 12: // 245248Hz, 239.5 SPS
            max_mdiv            = 478;
            max_kdiv            = 1;
            max_ndiv            = 512;
            max_dac_osr         = 64;
            max_adc_osr         = 128;
            break;

        case 13: // 220160Hz, 215.0 SPS
            max_mdiv            = 429;
            max_kdiv            = 1;
            max_ndiv            = 512;
            max_dac_osr         = 64;
            max_adc_osr         = 128;
            break;

        case 14: // 199936Hz, 195.25 SPS
            max_mdiv            = 780;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 128;
            max_adc_osr         = 128;
            break;

        case 15: // 176896Hz, 172.75 SPS
            max_mdiv            = 690;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 128;
            max_adc_osr         = 128;
            break;

        case 16: // 158976Hz, 155.25 SPS
            max_mdiv            = 620;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 128;
            max_adc_osr         = 128;
            break;

        case 17: // 143104Hz, 279.5 SPS
            max_mdiv            = 558;
            max_kdiv            = 1;
            max_ndiv            = 512;
            max_dac_osr         = 128;
            max_adc_osr         = 128;
            break;

        case 18: // 131072Hz, 256.0 SPS
            max_mdiv            = 511;
            max_kdiv            = 1;
            max_ndiv            = 512;
            max_dac_osr         = 128;
            max_adc_osr         = 128;
            break;

        case 19: // 114944Hz, 224.5 SPS
            max_mdiv            = 448;
            max_kdiv            = 1;
            max_ndiv            = 512;
            max_dac_osr         = 128;
            max_adc_osr         = 128;
            break;

        case 20: // 99968Hz, 195.25 SPS
            max_mdiv            = 780;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 128;
            break;

        case 21: // 93056Hz, 181.75 SPS
            max_mdiv            = 726;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 128;
            break;

        case 22: // 82944Hz, 162.0 SPS
            max_mdiv            = 647;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 128;
            break;

        case 23: // 82048Hz, 160.25 SPS
            max_mdiv            = 640;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 128;
            break;

        case 24: // 75008Hz, 146.5 SPS
            max_mdiv            = 585;
            max_kdiv            = 1;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 128;
            break;

        case 25: // 66944Hz, 261.5 SPS
            max_mdiv            = 522;
            max_kdiv            = 1;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 128;
            break;

        case 26: // 60032Hz, 234.5 SPS
            max_mdiv            = 468;
            max_kdiv            = 1;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 128;
            break;

        case 27: // 54016Hz, 211.0 SPS
            max_mdiv            = 843;
            max_kdiv            = 2;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 128;
            break;

        case 28: // 49984Hz, 195.25 SPS
            max_mdiv            = 780;
            max_kdiv            = 2;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 128;
            break;

        case 29: // 43008Hz, 168.0 SPS
            max_mdiv            = 671;
            max_kdiv            = 2;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 128;
            break;

        case 30: // 41024Hz, 160.25 SPS
            max_mdiv            = 640;
            max_kdiv            = 2;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 128;
            break;

        case 31: // 38976Hz, 76.13 SPS
            max_mdiv            = 608;
            max_kdiv            = 2;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 32: // 35008Hz, 136.75 SPS
            max_mdiv            = 546;
            max_kdiv            = 2;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 33: // 30976Hz, 121.0 SPS
            max_mdiv            = 483;
            max_kdiv            = 2;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 34: // 28032Hz, 109.5 SPS
            max_mdiv            = 437;
            max_kdiv            = 2;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 35: // 24992Hz, 97.63 SPS
            max_mdiv            = 780;
            max_kdiv            = 4;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 36: // 23008Hz, 89.88 SPS
            max_mdiv            = 718;
            max_kdiv            = 4;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 37: // 20000Hz, 78.13 SPS
            max_mdiv            = 624;
            max_kdiv            = 4;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 38: // 18016Hz, 140.75 SPS
            max_mdiv            = 562;
            max_kdiv            = 4;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 39: // 16000Hz, 125.0 SPS
            max_mdiv            = 499;
            max_kdiv            = 4;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 40: // 15008Hz, 117.25 SPS
            max_mdiv            = 468;
            max_kdiv            = 4;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 41: // 14016Hz, 109.5 SPS
            max_mdiv            = 437;
            max_kdiv            = 4;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 42: // 13008Hz, 101.63 SPS
            max_mdiv            = 812;
            max_kdiv            = 8;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 43: // 12000Hz, 93.75 SPS
            max_mdiv            = 749;
            max_kdiv            = 8;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 44: // 11008Hz, 86.0 SPS
            max_mdiv            = 687;
            max_kdiv            = 8;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 45: // 10000Hz, 78.13 SPS
            max_mdiv            = 624;
            max_kdiv            = 8;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 46: // 9008Hz, 70.38 SPS
            max_mdiv            = 562;
            max_kdiv            = 8;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 47: // 8000Hz, 125.0 SPS
            max_mdiv            = 499;
            max_kdiv            = 8;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 48: // 7008Hz, 109.5 SPS
            max_mdiv            = 437;
            max_kdiv            = 8;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 49: // 6000Hz, 93.75 SPS
            max_mdiv            = 749;
            max_kdiv            = 16;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 50: // 5000Hz, 78.13 SPS
            max_mdiv            = 624;
            max_kdiv            = 16;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 51: // 4000Hz, 125.0 SPS
            max_mdiv            = 499;
            max_kdiv            = 16;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 256;
            break;

        case 52: // 2000Hz, 62.5 SPS
            max_mdiv            = 499;
            max_kdiv            = 32;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 512;
            break;

        case 53: // 1000Hz, 31.25 SPS
            max_mdiv            = 499;
            max_kdiv            = 64;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 1024;
            break;

        case 54: // 500Hz, 31.25 SPS
            max_mdiv            = 499;
            max_kdiv            = 128;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 1024;
            break;

        case 55: // 250Hz, 31.25 SPS
            max_mdiv            = 499;
            max_kdiv            = 256;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 1024;
            break;

        case 56: // 125Hz, 31.25 SPS
            max_mdiv            = 499;
            max_kdiv            = 512;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 1024;
            break;

        case 57: // 64Hz, 32 SPS
            max_mdiv            = 511;
            max_kdiv            = 1024;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 1024;
            break;

        case 58: // 32Hz, 32 SPS
            max_mdiv            = 511;
            max_kdiv            = 2048;
            max_ndiv            = 512;
            max_dac_osr         = 256;
            max_adc_osr         = 1024;
            break;

        case 59: // 16Hz, 16 SPS
            max_mdiv            = 511;
            max_kdiv            = 4096;
            max_ndiv            = 1024;
            max_dac_osr         = 256;
            max_adc_osr         = 1024;
            break;
    }


    switch (max_dac_osr)
    {
        case  32: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b00111111) | 0b00000000; break;
        case  64: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b00111111) | 0b01000000; break;
        case 128: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b00111111) | 0b10000000; break;
        case 256: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b00111111) | 0b11000000; break;
        default : max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b00111111) | 0b00000000; break;
    }
    switch (max_adc_osr)
    {
        case    8: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00000000; break;
        case   16: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00001000; break;
        case   32: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00010000; break;
        case   64: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00011000; break;
        case  128: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00100000; break;
        case  256: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00101000; break;
        case  512: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00110000; break;
        case 1024: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00111000; break;
        default  : max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00000000; break;
    }
    switch (max_kdiv)
    {
        case    1: max_kdiv_bits  = 0b00000000; break;
        case    2: max_kdiv_bits  = 0b00000010; break;
        case    4: max_kdiv_bits  = 0b00000100; break;
        case    8: max_kdiv_bits  = 0b00000110; break;
        case   16: max_kdiv_bits  = 0b00001000; break;
        case   32: max_kdiv_bits  = 0b00001010; break;
        case   64: max_kdiv_bits  = 0b00001100; break;
        case  128: max_kdiv_bits  = 0b00001110; break;
        case  256: max_kdiv_bits  = 0b00010000; break;
        case  512: max_kdiv_bits  = 0b00010010; break;
        case 1024: max_kdiv_bits  = 0b00010100; break;
        case 2048: max_kdiv_bits  = 0b00010110; break;
        case 4096: max_kdiv_bits  = 0b00011000; break;
        case 8192: max_kdiv_bits  = 0b00011010; break;
        default  : max_kdiv_bits  = 0b00000000; break;
    }
    switch (max_ndiv)
    {
        case  512: max_ndiv_bits  = 0b00000000; break;
        case 1024: max_ndiv_bits  = 0b00100000; break;
        default  : max_ndiv_bits  = 0b00000000; break;
    }

    dat_pll_cnf_1 =     ((max_mdiv >> 2) & 0b11000000) | max_ndiv_bits | max_kdiv_bits;   // MDIV[7:0]
    dat_pll_cnf_2 =       max_mdiv & 0b11111111;
    dat_pll_cnf_3 =       0b00000001;                                               // PLL_LOCK_WNDW = 1 : 2 periods
    dat_pll_cnf_4 =       0b00100000;                                               // INT CLOCK, 32.768kHz



    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_1_REGISTER     , dat_pll_cnf_1);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_PLL_CONFIGURATION_1_REGISTER, dat_pll_cnf_1);
    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_2_REGISTER     , dat_pll_cnf_2);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_PLL_CONFIGURATION_2_REGISTER, dat_pll_cnf_2);
    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_3_REGISTER     , dat_pll_cnf_3);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_PLL_CONFIGURATION_3_REGISTER, dat_pll_cnf_3);
    //regWrite(MAX30009_PLL_CONFIGURATION_4_REGISTER     , dat_pll_cnf_4);
    //if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_PLL_CONFIGURATION_4_REGISTER, dat_pll_cnf_4);
    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_1_REGISTER     , max_bioz_cnf_1);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, max_bioz_cnf_1);

    if(debug) printf("MAX30009_SetFreq END\r\n");
}

double MAX30009_SetStimulusIndex(uint8_t stimulus_index,bool debug) // sets stimulus current based on index
{
    const uint8_t current_codes[16] = { 0b00000000,0b00010000,0b00100000,0b00110000,0b00000100,0b00010100,0b00100100,0b00110100,
                                        0b00001000,0b00011000,0b00101000,0b00111000,0b00001100,0b00011100,0b00101100,0b00111100  };
    const double current_magnitudes[16] = {
                                        1.6e-8L,  3.2e-8L,  8.0e-8L,  1.6e-7L,  3.2e-7L,  6.4e-7L,  1.6e-6L,  3.2e-6L,
                                        6.4e-6L, 1.28e-5L,  3.2e-5L,  6.4e-5L,  1.28e-4L, 2.56e-4L,  6.4e-4L, 1.28e-3L };

    if(debug) printf("MAX30009_SetStimulus BEGIN\r\n");

    uint8_t bioz_conf_3 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_3_REGISTER);
    if(debug) printf("RD:0x%02X-%02X\r\n", MAX30009_BIOZ_CONFIGURATION_3_REGISTER, bioz_conf_3);

    bioz_conf_3 = (bioz_conf_3 & 0b11000000) | current_codes[stimulus_index];

    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_3_REGISTER,bioz_conf_3);
    if(debug) printf("WR:0x%02X-%02X\r\n", MAX30009_BIOZ_CONFIGURATION_3_REGISTER, bioz_conf_3);
    if(debug) printf("MAX30009_SetStimulus END\r\n");
    return current_magnitudes[stimulus_index];
}

double MAX30009_SetStimulusCode(uint8_t stimulus_code,bool debug) // sets stimulus current based on index
{
    const double current_magnitudes[16] = {
                                        1.6e-8L,  3.2e-8L,  8.0e-8L,  1.6e-7L,  3.2e-7L,  6.4e-7L,  1.6e-6L,  3.2e-6L,
                                        6.4e-6L, 1.28e-5L,  3.2e-5L,  6.4e-5L,  1.28e-4L, 2.56e-4L,  6.4e-4L, 1.28e-3L };

    if(debug) printf("MAX30009_SetStimulus BEGIN\r\n");

    uint8_t bioz_conf_3 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_3_REGISTER);
    if(debug) printf("RD:0x%02X-%02X\r\n", MAX30009_BIOZ_CONFIGURATION_3_REGISTER, bioz_conf_3);

    bioz_conf_3 = (bioz_conf_3 & 0b11000000) | stimulus_code;

    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_3_REGISTER,bioz_conf_3);
    if(debug) printf("WR:0x%02X-%02X\r\n", MAX30009_BIOZ_CONFIGURATION_3_REGISTER, bioz_conf_3);
    if(debug) printf("MAX30009_SetStimulus END\r\n");
    return current_magnitudes[stimulus_code];
}

uint8_t MAX30009_SetStimulus2(uint8_t stimulus, uint8_t voltag_mag, uint8_t current_mag,bool debug)
{
    if(debug) printf("MAX30009_SetStimulus BEGIN\r\n");
    uint8_t bioz_conf_3 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_3_REGISTER);
    if(debug) printf("RD:0x%02X-%02X\r\n", MAX30009_BIOZ_CONFIGURATION_3_REGISTER, bioz_conf_3);

	if(     stimulus == 0)
    {
        switch(current_mag)
        {
            case 0: // 16nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00000000;
            break;
            case 1: // 32nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00000100;
            break;
            case 2: // 80nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00001000;
            break;
            case 3: // 160nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00001100;
            break;
            case 4: // 320nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00010000;
            break;
            case 5: // 640nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00010100;
            break;
            case 6: // 1.6uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00011000;
            break;
            case 7: // 3.2uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00011100;
            break;
            case 8: // 6.4uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00100000;
            break;
            case 9: // 12.8uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00100100;
            break;
            case 10: // 32uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00101000;
            break;
            case 11: // 64uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00101100;
            break;
            case 12: // 128uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00110000;
            break;
            case 13: // 256uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00110100;
            break;
            case 14: // 640uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00111000;
            break;
            case 15: // 1.28mA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00111100;
            break;
            default: // 16nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00000000;
            break;
        }
    }
	else if(stimulus == 1)
    {
        switch(voltag_mag)
        {
            case 0: // 35.4mV
                bioz_conf_3 = (bioz_conf_3 & 0b11000001) | 0b00000000;
            break;
            case 1: // 70.7mV
                bioz_conf_3 = (bioz_conf_3 & 0b11000001) | 0b00010000;
            break;
            case 2: // 177mV
                bioz_conf_3 = (bioz_conf_3 & 0b11000001) | 0b00100000;
            break;
            case 3: // 354mV
                bioz_conf_3 = (bioz_conf_3 & 0b11000001) | 0b00110000;
            break;
            default: // 35.4mV
                bioz_conf_3 = (bioz_conf_3 & 0b11000001) | 0b00000000;
            break;
        }
    }
	else if(stimulus == 2)
	{
			bioz_conf_3 = (bioz_conf_3 & 0b11000011);
	}

	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_3_REGISTER,bioz_conf_3);

    if(debug) printf("WR:0x%02X-%02X\r\n", MAX30009_BIOZ_CONFIGURATION_3_REGISTER, bioz_conf_3);
    if(debug) printf("MAX30009_SetStimulus END\r\n");
    return current_mag;
}

void MAX30009_SetAnalogHPF(uint8_t filter,bool debug)
{
    if(debug) printf("MAX30009_SetAnalogHPF BEGIN\r\n");

    uint8_t bioz_conf_5 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_5_REGISTER);
    if(debug) printf("RD:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_5_REGISTER, bioz_conf_5);
    bioz_conf_5 = bioz_conf_5 & 0b00001111;

	switch(filter)
	{
		case 0: // 100Hz
			bioz_conf_5 = bioz_conf_5 | 0b00000000;
		break;
		case 1: // 200Hz
			bioz_conf_5 = bioz_conf_5 | 0b00010000;
		break;
		case 2: // 500Hz
			bioz_conf_5 = bioz_conf_5 | 0b00100000;
		break;
		case 3: // 1000Hz
			bioz_conf_5 = bioz_conf_5 | 0b00110000;
		break;
		case 4: // 2000Hz
			bioz_conf_5 = bioz_conf_5 | 0b01000000;
		break;
		case 5: // 5000Hz
			bioz_conf_5 = bioz_conf_5 | 0b01010000;
		break;
		case 6: // 10000Hz
			bioz_conf_5 = bioz_conf_5 | 0b01100000;
		break;
		case 7: // resistor opened AHPF bypassed
			bioz_conf_5 = bioz_conf_5 | 0b01110000;
		break;
		case 8: // 42.4MΩ, internal capacitors shorted
			bioz_conf_5 = bioz_conf_5 | 0b10000000;
		break;
		case 9: // 21.2MΩ, internal capacitors shorted
			bioz_conf_5 = bioz_conf_5 | 0b10010000;
		break;
		case 10: // 8.4MΩ, internal capacitors shorted
			bioz_conf_5 = bioz_conf_5 | 0b10100000;
		break;
		case 11: // 4.2MΩ, internal capacitors shorted
			bioz_conf_5 = bioz_conf_5 | 0b10110000;
		break;
		case 12: // 2.2MΩ, internal capacitors shorted
			bioz_conf_5 = bioz_conf_5 | 0b11000000;
		break;
		case 13: // 848kΩ, internal capacitors shorted
			bioz_conf_5 = bioz_conf_5 | 0b11010000;
		break;
		case 14: // 848kΩ, internal capacitors shorted
			bioz_conf_5 = bioz_conf_5 | 0b11100000;
		break;
		case 15: // Resistor opened, internal capacitor shorted AHPF bypassed)
			bioz_conf_5 = bioz_conf_5 | 0b11110000;
		break;
		default: //
			bioz_conf_5 = bioz_conf_5 | 0b11110000;
		break;
	}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_5_REGISTER,bioz_conf_5);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_5_REGISTER, bioz_conf_5);
    if(debug) printf("MAX30009_SetAnalogHPF END\r\n");
}

void MAX30009_SetLeadBias(uint8_t lead_bias, bool lb_bip, bool lb_bin ,bool debug)
{
    if(debug) printf("max30009_SetLeadBias BEGIN\r\n");

    uint8_t lead_bias_conf = MAX30009_regRead(MAX30009_LEAD_BIAS_CONFIGURATION_1_REGISTER);
    if(debug) printf("RD:0x%02X-0x%02X\r\n", MAX30009_LEAD_BIAS_CONFIGURATION_1_REGISTER, lead_bias_conf);

    lead_bias_conf = (lead_bias<<2) | (lb_bip<<1) | lb_bin;
    MAX30009_regWrite(MAX30009_LEAD_BIAS_CONFIGURATION_1_REGISTER,lead_bias_conf);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_LEAD_BIAS_CONFIGURATION_1_REGISTER, lead_bias_conf);
    if(debug) printf("max30009_SetLeadBias END\r\n");
}


void MAX30009_SetDigitalFilter(uint8_t dhpf, uint8_t dlpf,bool debug)
{
    if(debug) printf("MAX30009_SetDigitalFilter BEGIN\r\n");

    uint8_t bioz_conf_2 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_2_REGISTER);
    if(debug) printf("RD:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_2_REGISTER, bioz_conf_2);
    bioz_conf_2 = bioz_conf_2 & 0b00000111;
    bioz_conf_2 = bioz_conf_2 | ((dhpf << 6) & 0b11000000) | ((dlpf << 3) & 0b00111000);
    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_2_REGISTER,bioz_conf_2);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_2_REGISTER, bioz_conf_2);
    if(debug) printf("MAX30009_SetDigitalFilter END\r\n");
}

uint8_t MAX30009_SetBiozGain(uint8_t gain,bool debug)
{
    uint8_t real_gain=0;
    if(debug) printf("MAX30009_SetBiozGain BEGIN\r\n");

    uint8_t bioz_conf_5 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_5_REGISTER);
	if(debug) printf("RD:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_5_REGISTER, bioz_conf_5);
    bioz_conf_5 = bioz_conf_5 & 0b11111100;

	switch(gain)
	{
		case 0: // 1X
			bioz_conf_5 = (bioz_conf_5 & 0b11111100) | 0b00000000;
            real_gain=1;
		break;
		case 1: // 2X
			bioz_conf_5 = (bioz_conf_5 & 0b11111100) | 0b00000001;
            real_gain=2;
		break;
		case 2: // 5X
			bioz_conf_5 = (bioz_conf_5 & 0b11111100) | 0b00000010;
            real_gain=5;
		break;
		case 3: //10X
			bioz_conf_5 = (bioz_conf_5 & 0b11111100) | 0b00000011;
            real_gain=10;
		break;
		default:// 1X
			bioz_conf_5 = (bioz_conf_5 & 0b11111100) | 0b00000000;
            real_gain=1;
		break;
	}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_5_REGISTER,bioz_conf_5);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_5_REGISTER, bioz_conf_5);
    if(debug) printf("MAX30009_SetBiozGain END\r\n");
    return real_gain;

}

void MAX30009_Clock(uint8_t clock_source, uint8_t clock_frequency, uint8_t int_clk_tune ,bool debug)
{
    if(debug) printf("MAX30009_Clock BEGIN\r\n");

    uint8_t pll_conf_4 = MAX30009_regRead(MAX30009_PLL_CONFIGURATION_4_REGISTER);
	if(debug) printf("RD:0x%02X-0x%02X\r\n", MAX30009_PLL_CONFIGURATION_4_REGISTER, pll_conf_4);
    pll_conf_4 = clock_source | clock_frequency | int_clk_tune;
    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_4_REGISTER,pll_conf_4);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_PLL_CONFIGURATION_4_REGISTER, pll_conf_4);
    if(debug) printf("MAX30009_Clock END\r\n");
}

void MAX30009_SetBiozDriveState(uint8_t reset,bool debug) // reset state or normal state
{
    if(debug) printf("MAX30009_SetBiozDriveState BEGIN\r\n");
    uint8_t bioz_conf_6 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_6_REGISTER);
 	if(debug) printf("RD:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
    bioz_conf_6 = bioz_conf_6 & 0b11011111;

	if(reset)
		{bioz_conf_6 = (bioz_conf_6 & 0b11011111) | 0b00100000;}
	else{bioz_conf_6 = (bioz_conf_6 & 0b11011111) | 0b00000000;}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_6_REGISTER,bioz_conf_6);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
    if(debug) printf("MAX30009_SetBiozDriveState END\r\n");
}

void MAX30009_SetBiozDACState(uint8_t reset,bool debug) // reset state or normal state
{
    if(debug) printf("MAX30009_SetBiozDACState BEGIN\r\n");
    uint8_t bioz_conf_6 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_6_REGISTER);
 	if(debug) printf("RD:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
    bioz_conf_6 = bioz_conf_6 & 0b11101111;
	if(reset)
		{bioz_conf_6 = (bioz_conf_6 & 0b11101111) | 0b00010000;}
	else{bioz_conf_6 = (bioz_conf_6 & 0b11101111) | 0b00000000;}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_6_REGISTER,bioz_conf_6);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
    if(debug) printf("MAX30009_SetBiozDACState END\r\n");
}

void MAX30009_SetBiozDCrestore(uint8_t state,bool debug) // 10MΩ feedback resistance is applied to the current-drive amplifier.
{
    if(debug) printf("MAX30009_SetBiozDCrestore BEGIN\r\n");
    uint8_t bioz_conf_6 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_6_REGISTER);
 	if(debug)
 		{
 			printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
 			print_bin8(bioz_conf_6);
 			printf("\r\n");
 		}

    bioz_conf_6 = bioz_conf_6 & 0b10111111;
	if(state)
		{bioz_conf_6 = (bioz_conf_6 & 0b10111111) | 0b01000000;}
	else{bioz_conf_6 = (bioz_conf_6 & 0b10111111) | 0b00000000;}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_6_REGISTER,bioz_conf_6);
    if(debug)
    {
		printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
		print_bin8(bioz_conf_6);
    	printf("MAX30009_SetBiozDCrestore END\r\n");
    }
}

void MAX30009_SetBiozExtCap(uint8_t state,bool debug) // selects the external capacitor CEXT connected between DRVXC and DRVSJ
{
    if(debug) printf("MAX30009_SetBiozExtCap BEGIN\r\n");
	uint8_t bioz_conf_6 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_6_REGISTER);
 	if(debug) printf("RD:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
    bioz_conf_6 = bioz_conf_6 & 0b01111111;
	if(state)
		{bioz_conf_6 = (bioz_conf_6 & 0b01111111) | 0b10000000;}
	else{bioz_conf_6 = (bioz_conf_6 & 0b01111111) | 0b00000000;}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_6_REGISTER,bioz_conf_6);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
    if(debug) printf("MAX30009_SetBiozExtCap END\r\n");
}

void MAX30009_SetBiozAmpRange(uint8_t range,bool debug)
{
    if(debug) printf("MAX30009_SetBiozAmpRange BEGIN\r\n");
    uint8_t bioz_conf_6 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_6_REGISTER);
 	if(debug) printf("RD:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
    bioz_conf_6 = bioz_conf_6 & 0b11110011;
	switch(range)
	{
		case 0: // Low
			bioz_conf_6 = (bioz_conf_6 & 0b11110011) | 0b00000000;
		break;
		case 1: // Medium-Low
			bioz_conf_6 = (bioz_conf_6 & 0b11110011) | 0b00000100;
		break;
		case 2: // Medium-High
			bioz_conf_6 = (bioz_conf_6 & 0b11110011) | 0b00001000;
		break;
		case 3: //High
			bioz_conf_6 = (bioz_conf_6 & 0b11110011) | 0b00001100;
		break;
		default://High
			bioz_conf_6 = (bioz_conf_6 & 0b11110011) | 0b00001100;
		break;
	}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_6_REGISTER,bioz_conf_6);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
    if(debug) printf("MAX30009_SetBiozAmpRange END\r\n");
}

void MAX30009_SetBiozAmpBW(uint8_t bandwidth,bool debug)
{
    if(debug) printf("MAX30009_SetBiozAmpBW BEGIN\r\n");
	uint8_t bioz_conf_6 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_6_REGISTER);
 	if(debug) printf("RD:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
    bioz_conf_6 = bioz_conf_6 & 0b11111100;

	switch(bandwidth)
	{
		case 0: // Low
			bioz_conf_6 = (bioz_conf_6 & 0b11111100) | 0b00000000;
		break;
		case 1: // Medium-Low
			bioz_conf_6 = (bioz_conf_6 & 0b11111100) | 0b00000001;
		break;
		case 2: // Medium-High
			bioz_conf_6 = (bioz_conf_6 & 0b11111100) | 0b00000010;
		break;
		case 3: //High
			bioz_conf_6 = (bioz_conf_6 & 0b11111100) | 0b00000011;
		break;
		default://High
			bioz_conf_6 = (bioz_conf_6 & 0b11111100) | 0b00000000;
		break;
	}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_6_REGISTER,bioz_conf_6);
    if(debug) printf("WR:0x%02X-0x%02X\r\n", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
    if(debug) printf("MAX30009_SetBiozAmpBW BEGIN\r\n");
}

*/


void MAX30009_bioz_startup(bool debug)	// call this on power up
{
    uint8_t tempc;
    if(debug) printf("MAX30009_bioz_startup BEGIN\r\n");
    tempc = MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 2, 1);  // Set t BIOZ_BG_EN = 1.
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, tempc); print_bin8(tempc); printf("\r\n"); }
    HAL_Delay(200);
    tempc = MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 1, 1);  // Set BIOZ_Q_EN = 1.
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, tempc); print_bin8(tempc); printf("\r\n"); }
    tempc = MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 0, 1);  // Set BIOZ_I_EN  = 1.
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, tempc); print_bin8(tempc); printf("\r\n"); }
    if(debug) printf("MAX30009_bioz_startup END\r\n");
}

void MAX30009_bioz_shutdown(bool debug)	// call this on power up
{
    MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 2, 0);  // Set t BIOZ_BG_EN = 0.
    MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 1, 0);  // Set BIOZ_Q_EN = 0.
    MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x01, 0, 0);  // Set BIOZ_I_EN  = 0.
}

void MAX30009_SetFreq(uint8_t frequency,bool debug)
{
    if(debug) printf("MAX30009_SetFreq BEGIN\r\n");

    uint8_t     bioz_state      =0;
    uint16_t    max_mdiv        =0;
    uint16_t    max_ndiv        =0;
    uint16_t    max_kdiv        =0;
    uint16_t    max_dac_osr     =0;
    uint16_t    max_adc_osr     =0;

    uint8_t     dat_pll_cnf_1   =0;
    uint8_t     dat_pll_cnf_2   =0;
    uint8_t     dat_pll_cnf_3   =0;
    uint8_t     dat_pll_cnf_4   =0;
    uint8_t     max_bioz_cnf_1  =0;
    uint8_t     max_ndiv_bits   =0;
    uint8_t     max_kdiv_bits   =0;

    uint8_t tempdata            = MAX30009_regRead( MAX30009_BIOZ_CONFIGURATION_1_REGISTER);
    bioz_state                  = tempdata & 0b00000011;
    if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, tempdata); print_bin8(tempdata); printf("\r\n"); }

    if(bioz_state)
    {
         MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_1_REGISTER, (tempdata & 0b11111100) );
         if(debug) { uint8_t val = tempdata & 0b11111100; printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, val); print_bin8(val); printf("\r\n"); }
    }
    max_bioz_cnf_1 = tempdata & 0b00000111;

    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_1_REGISTER     , dat_pll_cnf_1 | 0b00000001 );
    if(debug) { uint8_t val = dat_pll_cnf_1 | 0b00000001; printf("WR:0x%02X-0x%02X, ", MAX30009_PLL_CONFIGURATION_1_REGISTER, val); print_bin8(val); printf("\r\n"); }

    switch(frequency)
    {
        /* (unchanged switch cases) */
        case 0: max_mdiv=789; max_kdiv=1; max_ndiv=1024; max_dac_osr=32; max_adc_osr=128; break;
        case 1: max_mdiv=705; max_kdiv=1; max_ndiv=1024; max_dac_osr=32; max_adc_osr=128; break;
        case 2: max_mdiv=633; max_kdiv=1; max_ndiv=1024; max_dac_osr=32; max_adc_osr=128; break;
        case 3: max_mdiv=567; max_kdiv=1; max_ndiv=512;  max_dac_osr=32; max_adc_osr=128; break;
        case 4: max_mdiv=487; max_kdiv=1; max_ndiv=512;  max_dac_osr=32; max_adc_osr=128; break;
        case 5: max_mdiv=457; max_kdiv=1; max_ndiv=512;  max_dac_osr=32; max_adc_osr=128; break;
        case 6: max_mdiv=821; max_kdiv=1; max_ndiv=1024; max_dac_osr=64; max_adc_osr=128; break;
        case 7: max_mdiv=737; max_kdiv=1; max_ndiv=1024; max_dac_osr=64; max_adc_osr=128; break;
        case 8: max_mdiv=661; max_kdiv=1; max_ndiv=1024; max_dac_osr=64; max_adc_osr=128; break;
        case 9: max_mdiv=593; max_kdiv=1; max_ndiv=1024; max_dac_osr=64; max_adc_osr=128; break;
        case 10: max_mdiv=532; max_kdiv=1; max_ndiv=512; max_dac_osr=64; max_adc_osr=128; break;
        case 11: max_mdiv=487; max_kdiv=1; max_ndiv=512; max_dac_osr=64; max_adc_osr=128; break;
        case 12: max_mdiv=478; max_kdiv=1; max_ndiv=512; max_dac_osr=64; max_adc_osr=128; break;
        case 13: max_mdiv=429; max_kdiv=1; max_ndiv=512; max_dac_osr=64; max_adc_osr=128; break;
        case 14: max_mdiv=780; max_kdiv=1; max_ndiv=1024; max_dac_osr=128; max_adc_osr=128; break;
        case 15: max_mdiv=690; max_kdiv=1; max_ndiv=1024; max_dac_osr=128; max_adc_osr=128; break;
        case 16: max_mdiv=620; max_kdiv=1; max_ndiv=1024; max_dac_osr=128; max_adc_osr=128; break;
        case 17: max_mdiv=558; max_kdiv=1; max_ndiv=512; max_dac_osr=128; max_adc_osr=128; break;
        case 18: max_mdiv=511; max_kdiv=1; max_ndiv=512; max_dac_osr=128; max_adc_osr=128; break;
        case 19: max_mdiv=448; max_kdiv=1; max_ndiv=512; max_dac_osr=128; max_adc_osr=128; break;
        case 20: max_mdiv=780; max_kdiv=1; max_ndiv=1024; max_dac_osr=256; max_adc_osr=128; break;
        case 21: max_mdiv=726; max_kdiv=1; max_ndiv=1024; max_dac_osr=256; max_adc_osr=128; break;
        case 22: max_mdiv=647; max_kdiv=1; max_ndiv=1024; max_dac_osr=256; max_adc_osr=128; break;
        case 23: max_mdiv=640; max_kdiv=1; max_ndiv=1024; max_dac_osr=256; max_adc_osr=128; break;
        case 24: max_mdiv=585; max_kdiv=1; max_ndiv=1024; max_dac_osr=256; max_adc_osr=128; break;
        case 25: max_mdiv=522; max_kdiv=1; max_ndiv=512;  max_dac_osr=256; max_adc_osr=128; break;
        case 26: max_mdiv=468; max_kdiv=1; max_ndiv=512;  max_dac_osr=256; max_adc_osr=128; break;
        case 27: max_mdiv=843; max_kdiv=2; max_ndiv=1024; max_dac_osr=256; max_adc_osr=128; break;
        case 28: max_mdiv=780; max_kdiv=2; max_ndiv=1024; max_dac_osr=256; max_adc_osr=128; break;
        case 29: max_mdiv=671; max_kdiv=2; max_ndiv=1024; max_dac_osr=256; max_adc_osr=128; break;
        case 30: max_mdiv=640; max_kdiv=2; max_ndiv=1024; max_dac_osr=256; max_adc_osr=128; break;
        case 31: max_mdiv=608; max_kdiv=2; max_ndiv=1024; max_dac_osr=256; max_adc_osr=256; break;
        case 32: max_mdiv=546; max_kdiv=2; max_ndiv=512;  max_dac_osr=256; max_adc_osr=256; break;
        case 33: max_mdiv=483; max_kdiv=2; max_ndiv=512;  max_dac_osr=256; max_adc_osr=256; break;
        case 34: max_mdiv=437; max_kdiv=2; max_ndiv=512;  max_dac_osr=256; max_adc_osr=256; break;
        case 35: max_mdiv=780; max_kdiv=4; max_ndiv=1024; max_dac_osr=256; max_adc_osr=256; break;
        case 36: max_mdiv=718; max_kdiv=4; max_ndiv=1024; max_dac_osr=256; max_adc_osr=256; break;
        case 37: max_mdiv=624; max_kdiv=4; max_ndiv=1024; max_dac_osr=256; max_adc_osr=256; break;
        case 38: max_mdiv=562; max_kdiv=4; max_ndiv=512;  max_dac_osr=256; max_adc_osr=256; break;
        case 39: max_mdiv=499; max_kdiv=4; max_ndiv=512;  max_dac_osr=256; max_adc_osr=256; break;
        case 40: max_mdiv=468; max_kdiv=4; max_ndiv=512;  max_dac_osr=256; max_adc_osr=256; break;
        case 41: max_mdiv=437; max_kdiv=4; max_ndiv=512;  max_dac_osr=256; max_adc_osr=256; break;
        case 42: max_mdiv=812; max_kdiv=8; max_ndiv=1024; max_dac_osr=256; max_adc_osr=256; break;
        case 43: max_mdiv=749; max_kdiv=8; max_ndiv=1024; max_dac_osr=256; max_adc_osr=256; break;
        case 44: max_mdiv=687; max_kdiv=8; max_ndiv=1024; max_dac_osr=256; max_adc_osr=256; break;
        case 45: max_mdiv=624; max_kdiv=8; max_ndiv=1024; max_dac_osr=256; max_adc_osr=256; break;
        case 46: max_mdiv=562; max_kdiv=8; max_ndiv=1024; max_dac_osr=256; max_adc_osr=256; break;
        case 47: max_mdiv=499; max_kdiv=8; max_ndiv=512;  max_dac_osr=256; max_adc_osr=256; break;
        case 48: max_mdiv=437; max_kdiv=8; max_ndiv=512;  max_dac_osr=256; max_adc_osr=256; break;
        case 49: max_mdiv=749; max_kdiv=16;max_ndiv=1024; max_dac_osr=256; max_adc_osr=256; break;
        case 50: max_mdiv=624; max_kdiv=16;max_ndiv=1024; max_dac_osr=256; max_adc_osr=256; break;
        case 51: max_mdiv=499; max_kdiv=16;max_ndiv=512;  max_dac_osr=256; max_adc_osr=256; break;
        case 52: max_mdiv=499; max_kdiv=32;max_ndiv=512;  max_dac_osr=256; max_adc_osr=512; break;
        case 53: max_mdiv=499; max_kdiv=64;max_ndiv=512;  max_dac_osr=256; max_adc_osr=1024; break;
        case 54: max_mdiv=499; max_kdiv=128;max_ndiv=512; max_dac_osr=256; max_adc_osr=1024; break;
        case 55: max_mdiv=499; max_kdiv=256;max_ndiv=512;  max_dac_osr=256; max_adc_osr=1024; break;
        case 56: max_mdiv=499; max_kdiv=512;max_ndiv=512;  max_dac_osr=256; max_adc_osr=1024; break;
        case 57: max_mdiv=511; max_kdiv=1024;max_ndiv=512; max_dac_osr=256; max_adc_osr=1024; break;
        case 58: max_mdiv=511; max_kdiv=2048;max_ndiv=512; max_dac_osr=256; max_adc_osr=1024; break;
        case 59: max_mdiv=511; max_kdiv=4096;max_ndiv=1024; max_dac_osr=256; max_adc_osr=1024; break;
    }

    /* (unchanged bit field encoding for dat_pll_cnf_1 / dat_pll_cnf_2 / dat_pll_cnf_3 / dat_pll_cnf_4 and max_bioz_cnf_1 adjustments) */

    switch (max_dac_osr)
    {
        case  32: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b00111111) | 0b00000000; break;
        case  64: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b00111111) | 0b01000000; break;
        case 128: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b00111111) | 0b10000000; break;
        case 256: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b00111111) | 0b11000000; break;
        default : max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b00111111) | 0b00000000; break;
    }
    switch (max_adc_osr)
    {
        case    8: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00000000; break;
        case   16: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00001000; break;
        case   32: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00010000; break;
        case   64: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00011000; break;
        case  128: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00100000; break;
        case  256: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00101000; break;
        case  512: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00110000; break;
        case 1024: max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00111000; break;
        default  : max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11000111) | 0b00000000; break;
    }
    switch (max_kdiv)
    {
        case    1: max_kdiv_bits  = 0b00000000; break;
        case    2: max_kdiv_bits  = 0b00000010; break;
        case    4: max_kdiv_bits  = 0b00000100; break;
        case    8: max_kdiv_bits  = 0b00000110; break;
        case   16: max_kdiv_bits  = 0b00001000; break;
        case   32: max_kdiv_bits  = 0b00001010; break;
        case   64: max_kdiv_bits  = 0b00001100; break;
        case  128: max_kdiv_bits  = 0b00001110; break;
        case  256: max_kdiv_bits  = 0b00010000; break;
        case  512: max_kdiv_bits  = 0b00010010; break;
        case 1024: max_kdiv_bits  = 0b00010100; break;
        case 2048: max_kdiv_bits  = 0b00010110; break;
        case 4096: max_kdiv_bits  = 0b00011000; break;
        case 8192: max_kdiv_bits  = 0b00011010; break;
        default  : max_kdiv_bits  = 0b00000000; break;
    }
    switch (max_ndiv)
    {
        case  512: max_ndiv_bits  = 0b00000000; break;
        case 1024: max_ndiv_bits  = 0b00100000; break;
        default  : max_ndiv_bits  = 0b00000000; break;
    }

    dat_pll_cnf_1 =     ((max_mdiv >> 2) & 0b11000000) | max_ndiv_bits | max_kdiv_bits;   // MDIV[7:0]
    dat_pll_cnf_2 =       max_mdiv & 0b11111111;
    dat_pll_cnf_3 =       0b00000001;                                               // PLL_LOCK_WNDW = 1 : 2 periods
    dat_pll_cnf_4 =       0b00100000;                                               // INT CLOCK, 32.768kHz

    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_1_REGISTER     , dat_pll_cnf_1);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_PLL_CONFIGURATION_1_REGISTER, dat_pll_cnf_1); print_bin8(dat_pll_cnf_1); printf("\r\n"); }
    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_2_REGISTER     , dat_pll_cnf_2);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_PLL_CONFIGURATION_2_REGISTER, dat_pll_cnf_2); print_bin8(dat_pll_cnf_2); printf("\r\n"); }
    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_3_REGISTER     , dat_pll_cnf_3);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_PLL_CONFIGURATION_3_REGISTER, dat_pll_cnf_3); print_bin8(dat_pll_cnf_3); printf("\r\n"); }
    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_4_REGISTER, dat_pll_cnf_4);
    if(debug) {
        printf("WR:0x%02X-0x%02X, ", MAX30009_PLL_CONFIGURATION_4_REGISTER, dat_pll_cnf_4);
        print_bin8(dat_pll_cnf_4);
        printf("\r\n");
    }
    uint8_t current_bioz1 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_1_REGISTER);
    max_bioz_cnf_1 = (max_bioz_cnf_1 & 0b11111000) | (current_bioz1 & 0b00000111);
    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_1_REGISTER, max_bioz_cnf_1);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_1_REGISTER, max_bioz_cnf_1); print_bin8(max_bioz_cnf_1); printf("\r\n"); }

    if(debug) printf("MAX30009_SetFreq END\r\n");
}

double MAX30009_SetStimulusIndex(uint8_t stimulus_index,bool debug) // sets stimulus current based on index
{
    const uint8_t current_codes[16] = { 0b00000000,0b00010000,0b00100000,0b00110000,0b00000100,0b00010100,0b00100100,0b00110100,
                                        0b00001000,0b00011000,0b00101000,0b00111000,0b00001100,0b00011100,0b00101100,0b00111100  };
    const double current_magnitudes[16] = {
                                        1.6e-8L,  3.2e-8L,  8.0e-8L,  1.6e-7L,  3.2e-7L,  6.4e-7L,  1.6e-6L,  3.2e-6L,
                                        6.4e-6L, 1.28e-5L,  3.2e-5L,  6.4e-5L,  1.28e-4L, 2.56e-4L,  6.4e-4L, 1.28e-3L };

    if(debug) printf("MAX30009_SetStimulus BEGIN\r\n");

    uint8_t bioz_conf_3 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_3_REGISTER);
    if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_3_REGISTER, bioz_conf_3); print_bin8(bioz_conf_3); printf("\r\n"); }

    bioz_conf_3 = (bioz_conf_3 & 0b11000000) | current_codes[stimulus_index];

    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_3_REGISTER,bioz_conf_3);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_3_REGISTER, bioz_conf_3); print_bin8(bioz_conf_3); printf("\r\n"); }
    if(debug) printf("MAX30009_SetStimulus END\r\n");
    return current_magnitudes[stimulus_index];
}

double MAX30009_SetStimulusCode(uint8_t stimulus_code,bool debug) // sets stimulus current based on index
{
    const double current_magnitudes[16] = {
                                        1.6e-8L,  3.2e-8L,  8.0e-8L,  1.6e-7L,  3.2e-7L,  6.4e-7L,  1.6e-6L,  3.2e-6L,
                                        6.4e-6L, 1.28e-5L,  3.2e-5L,  6.4e-5L,  1.28e-4L, 2.56e-4L,  6.4e-4L, 1.28e-3L };

    if(debug) printf("MAX30009_SetStimulus BEGIN\r\n");

    uint8_t bioz_conf_3 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_3_REGISTER);
    if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_3_REGISTER, bioz_conf_3); print_bin8(bioz_conf_3); printf("\r\n"); }

    bioz_conf_3 = (bioz_conf_3 & 0b11000000) | stimulus_code;

    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_3_REGISTER,bioz_conf_3);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_3_REGISTER, bioz_conf_3); print_bin8(bioz_conf_3); printf("\r\n"); }
    if(debug) printf("MAX30009_SetStimulus END\r\n");
    return current_magnitudes[stimulus_code];
}

uint8_t MAX30009_SetStimulus2(uint8_t stimulus, uint8_t voltag_mag, uint8_t current_mag,bool debug)
{
    if(debug) printf("MAX30009_SetStimulus BEGIN\r\n");
    uint8_t bioz_conf_3 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_3_REGISTER);
    if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_3_REGISTER, bioz_conf_3); print_bin8(bioz_conf_3); printf("\r\n"); }

	if(     stimulus == 0)
    {
        switch(current_mag)
        {
            case 0: // 16nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00000000;
            break;
            case 1: // 32nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00000100;
            break;
            case 2: // 80nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00001000;
            break;
            case 3: // 160nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00001100;
            break;
            case 4: // 320nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00010000;
            break;
            case 5: // 640nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00010100;
            break;
            case 6: // 1.6uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00011000;
            break;
            case 7: // 3.2uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00011100;
            break;
            case 8: // 6.4uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00100000;
            break;
            case 9: // 12.8uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00100100;
            break;
            case 10: // 32uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00101000;
            break;
            case 11: // 64uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00101100;
            break;
            case 12: // 128uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00110000;
            break;
            case 13: // 256uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00110100;
            break;
            case 14: // 640uA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00111000;
            break;
            case 15: // 1.28mA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00111100;
            break;
            default: // 16nA
                bioz_conf_3 = (bioz_conf_3 & 0b11000000) | 0b00000000;
            break;
        }
    }
	else if(stimulus == 1)
    {
        switch(voltag_mag)
        {
            case 0: // 35.4mV
                bioz_conf_3 = (bioz_conf_3 & 0b11000001) | 0b00000000;
            break;
            case 1: // 70.7mV
                bioz_conf_3 = (bioz_conf_3 & 0b11000001) | 0b00010000;
            break;
            case 2: // 177mV
                bioz_conf_3 = (bioz_conf_3 & 0b11000001) | 0b00100000;
            break;
            case 3: // 354mV
                bioz_conf_3 = (bioz_conf_3 & 0b11000001) | 0b00110000;
            break;
            default: // 35.4mV
                bioz_conf_3 = (bioz_conf_3 & 0b11000001) | 0b00000000;
            break;
        }
    }
	else if(stimulus == 2)
	{
			bioz_conf_3 = (bioz_conf_3 & 0b11000011);
	}

	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_3_REGISTER,bioz_conf_3);

    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_3_REGISTER, bioz_conf_3); print_bin8(bioz_conf_3); printf("\r\n"); }
    if(debug) printf("MAX30009_SetStimulus END\r\n");
    return current_mag;
}

void MAX30009_SetAnalogHPF(uint8_t filter,bool debug)
{
    if(debug) printf("MAX30009_SetAnalogHPF BEGIN\r\n");

    uint8_t bioz_conf_5 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_5_REGISTER);
    if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_5_REGISTER, bioz_conf_5); print_bin8(bioz_conf_5); printf("\r\n"); }
    bioz_conf_5 = bioz_conf_5 & 0b00001111;

	switch(filter)
	{
		case 0: bioz_conf_5 = bioz_conf_5 | 0b00000000; break;
		case 1: bioz_conf_5 = bioz_conf_5 | 0b00010000; break;
		case 2: bioz_conf_5 = bioz_conf_5 | 0b00100000; break;
		case 3: bioz_conf_5 = bioz_conf_5 | 0b00110000; break;
		case 4: bioz_conf_5 = bioz_conf_5 | 0b01000000; break;
		case 5: bioz_conf_5 = bioz_conf_5 | 0b01010000; break;
		case 6: bioz_conf_5 = bioz_conf_5 | 0b01100000; break;
		case 7: bioz_conf_5 = bioz_conf_5 | 0b01110000; break;
		case 8: bioz_conf_5 = bioz_conf_5 | 0b10000000; break;
		case 9: bioz_conf_5 = bioz_conf_5 | 0b10010000; break;
		case 10: bioz_conf_5 = bioz_conf_5 | 0b10100000; break;
		case 11: bioz_conf_5 = bioz_conf_5 | 0b10110000; break;
		case 12: bioz_conf_5 = bioz_conf_5 | 0b11000000; break;
		case 13: bioz_conf_5 = bioz_conf_5 | 0b11010000; break;
		case 14: bioz_conf_5 = bioz_conf_5 | 0b11100000; break;
		case 15: bioz_conf_5 = bioz_conf_5 | 0b11110000; break;
		default: bioz_conf_5 = bioz_conf_5 | 0b11110000; break;
	}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_5_REGISTER,bioz_conf_5);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_5_REGISTER, bioz_conf_5); print_bin8(bioz_conf_5); printf("\r\n"); }
    if(debug) printf("MAX30009_SetAnalogHPF END\r\n");
}

void MAX30009_SetLeadBias(uint8_t lead_bias, bool lb_bip, bool lb_bin ,bool debug)
{
    if(debug) printf("max30009_SetLeadBias BEGIN\r\n");

    uint8_t lead_bias_conf = MAX30009_regRead(MAX30009_LEAD_BIAS_CONFIGURATION_1_REGISTER);
    if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_LEAD_BIAS_CONFIGURATION_1_REGISTER, lead_bias_conf); print_bin8(lead_bias_conf); printf("\r\n"); }

    lead_bias_conf = (lead_bias<<2) | (lb_bip<<1) | lb_bin;
    MAX30009_regWrite(MAX30009_LEAD_BIAS_CONFIGURATION_1_REGISTER,lead_bias_conf);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_LEAD_BIAS_CONFIGURATION_1_REGISTER, lead_bias_conf); print_bin8(lead_bias_conf); printf("\r\n"); }
    if(debug) printf("max30009_SetLeadBias END\r\n");
}


void MAX30009_SetDigitalFilter(uint8_t dhpf, uint8_t dlpf,bool debug)
{
    if(debug) printf("MAX30009_SetDigitalFilter BEGIN\r\n");

    uint8_t bioz_conf_2 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_2_REGISTER);
    if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_2_REGISTER, bioz_conf_2); print_bin8(bioz_conf_2); printf("\r\n"); }
    /* BIOZ_CFG_2 (0x21) bit-paigutus:
     *   [7:6] DHPF[1:0]
     *   [5:3] DLPF[2:0]   <-- NB! dlpf << 3 (mitte << 4)
     *   [2:1] CMP[1:0]
     *   [0]   EN_BIOZ_THRESH
     */
    bioz_conf_2 = bioz_conf_2 & 0b00000111;
    bioz_conf_2 = bioz_conf_2 | ((dhpf << 6) & 0b11000000) | ((dlpf << 3) & 0b00111000);
    MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_2_REGISTER,bioz_conf_2);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_2_REGISTER, bioz_conf_2); print_bin8(bioz_conf_2); printf("\r\n"); }
    if(debug) printf("MAX30009_SetDigitalFilter END\r\n");
}

uint8_t MAX30009_SetBiozGain(uint8_t gain,bool debug)
{
    uint8_t real_gain=0;
    if(debug) printf("MAX30009_SetBiozGain BEGIN\r\n");

    uint8_t bioz_conf_5 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_5_REGISTER);
	if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_5_REGISTER, bioz_conf_5); print_bin8(bioz_conf_5); printf("\r\n"); }
    bioz_conf_5 = bioz_conf_5 & 0b11111100;

	switch(gain)
	{
		case 0: bioz_conf_5 = (bioz_conf_5 & 0b11111100) | 0b00000000; real_gain=1; break;
		case 1: bioz_conf_5 = (bioz_conf_5 & 0b11111100) | 0b00000001; real_gain=2; break;
		case 2: bioz_conf_5 = (bioz_conf_5 & 0b11111100) | 0b00000010; real_gain=5; break;
		case 3: bioz_conf_5 = (bioz_conf_5 & 0b11111100) | 0b00000011; real_gain=10; break;
		default: bioz_conf_5 = (bioz_conf_5 & 0b11111100) | 0b00000000; real_gain=1; break;
	}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_5_REGISTER,bioz_conf_5);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_5_REGISTER, bioz_conf_5); print_bin8(bioz_conf_5); printf("\r\n"); }
    if(debug) printf("MAX30009_SetBiozGain END\r\n");
    return real_gain;

}

void MAX30009_Clock(uint8_t clock_source, uint8_t clock_frequency, uint8_t int_clk_tune ,bool debug)
{
    if(debug) printf("MAX30009_Clock BEGIN\r\n");

    uint8_t pll_conf_4 = MAX30009_regRead(MAX30009_PLL_CONFIGURATION_4_REGISTER);
	if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_PLL_CONFIGURATION_4_REGISTER, pll_conf_4); print_bin8(pll_conf_4); printf("\r\n"); }
    pll_conf_4 = clock_source | clock_frequency | int_clk_tune;
    MAX30009_regWrite(MAX30009_PLL_CONFIGURATION_4_REGISTER,pll_conf_4);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_PLL_CONFIGURATION_4_REGISTER, pll_conf_4); print_bin8(pll_conf_4); printf("\r\n"); }
    if(debug) printf("MAX30009_Clock END\r\n");
}

void MAX30009_SetBiozDCrestore(uint8_t state,bool debug) // 10MΩ feedback resistance is applied to the current-drive amplifier.
{
	if(debug) printf("MAX30009_SetBiozDCrestore BEGIN\r\n");
	uint8_t bioz_conf_6 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_6_REGISTER);
	if(debug)
	{
		printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
		print_bin8(bioz_conf_6);
		printf("\r\n");
	}
	bioz_conf_6 = bioz_conf_6 & 0b10111111;
	if(state) 	{bioz_conf_6 = (bioz_conf_6 & 0b10111111) | 0b01000000;}
	else		{bioz_conf_6 = (bioz_conf_6 & 0b10111111) | 0b00000000;}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_6_REGISTER,bioz_conf_6);
	if(debug)
	{
		printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6);
		print_bin8(bioz_conf_6);
		printf("\r\n");
		printf("MAX30009_SetBiozDCrestore END\r\n");
	}
}

void MAX30009_SetBiozDriveState(uint8_t reset,bool debug) // reset state or normal state
{
    if(debug) printf("MAX30009_SetBiozDriveState BEGIN\r\n");
    uint8_t bioz_conf_6 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_6_REGISTER);
 	if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6); print_bin8(bioz_conf_6); printf("\r\n"); }
    bioz_conf_6 = bioz_conf_6 & 0b11011111;

	if(reset)
		{bioz_conf_6 = (bioz_conf_6 & 0b11011111) | 0b00100000;}
	else{bioz_conf_6 = (bioz_conf_6 & 0b11011111) | 0b00000000;}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_6_REGISTER,bioz_conf_6);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6); print_bin8(bioz_conf_6); printf("\r\n"); }
    if(debug) printf("MAX30009_SetBiozDriveState END\r\n");
}

void MAX30009_SetBiozDACState(uint8_t reset,bool debug) // reset state or normal state
{
    if(debug) printf("MAX30009_SetBiozDACState BEGIN\r\n");
    uint8_t bioz_conf_6 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_6_REGISTER);
 	if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6); print_bin8(bioz_conf_6); printf("\r\n"); }
    bioz_conf_6 = bioz_conf_6 & 0b11101111;
	if(reset)
		{bioz_conf_6 = (bioz_conf_6 & 0b11101111) | 0b00010000;}
	else{bioz_conf_6 = (bioz_conf_6 & 0b11101111) | 0b00000000;}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_6_REGISTER,bioz_conf_6);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6); print_bin8(bioz_conf_6); printf("\r\n"); }
    if(debug) printf("MAX30009_SetBiozDACState END\r\n");
}

void MAX30009_SetBiozExtCap(uint8_t state,bool debug) // selects the external capacitor CEXT connected between DRVXC and DRVSJ
{
    if(debug) printf("MAX30009_SetBiozExtCap BEGIN\r\n");
	uint8_t bioz_conf_6 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_6_REGISTER);
 	if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6); print_bin8(bioz_conf_6); printf("\r\n"); }
    bioz_conf_6 = bioz_conf_6 & 0b01111111;
	if(state)
		{bioz_conf_6 = (bioz_conf_6 & 0b01111111) | 0b10000000;}
	else{bioz_conf_6 = (bioz_conf_6 & 0b01111111) | 0b00000000;}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_6_REGISTER,bioz_conf_6);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6); print_bin8(bioz_conf_6); printf("\r\n"); }
    if(debug) printf("MAX30009_SetBiozExtCap END\r\n");
}

void MAX30009_SetBiozAmpRange(uint8_t range,bool debug)
{
    if(debug) printf("MAX30009_SetBiozAmpRange BEGIN\r\n");
    uint8_t bioz_conf_6 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_6_REGISTER);
 	if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6); print_bin8(bioz_conf_6); printf("\r\n"); }
    bioz_conf_6 = bioz_conf_6 & 0b11110011;
	switch(range)
	{
		case 0: bioz_conf_6 = (bioz_conf_6 & 0b11110011) | 0b00000000; break;
		case 1: bioz_conf_6 = (bioz_conf_6 & 0b11110011) | 0b00000100; break;
		case 2: bioz_conf_6 = (bioz_conf_6 & 0b11110011) | 0b00001000; break;
		case 3: bioz_conf_6 = (bioz_conf_6 & 0b11110011) | 0b00001100; break;
		default: bioz_conf_6 = (bioz_conf_6 & 0b11110011) | 0b00001100; break;
	}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_6_REGISTER,bioz_conf_6);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6); print_bin8(bioz_conf_6); printf("\r\n"); }
    if(debug) printf("MAX30009_SetBiozAmpRange END\r\n");
}

void MAX30009_SetBiozAmpBW(uint8_t bandwidth,bool debug)
{
    if(debug) printf("MAX30009_SetBiozAmpBW BEGIN\r\n");
	uint8_t bioz_conf_6 = MAX30009_regRead(MAX30009_BIOZ_CONFIGURATION_6_REGISTER);
 	if(debug) { printf("RD:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6); print_bin8(bioz_conf_6); printf("\r\n"); }
    bioz_conf_6 = bioz_conf_6 & 0b11111100;

	switch(bandwidth)
	{
		case 0: bioz_conf_6 = (bioz_conf_6 & 0b11111100) | 0b00000000; break;
		case 1: bioz_conf_6 = (bioz_conf_6 & 0b11111100) | 0b00000001; break;
		case 2: bioz_conf_6 = (bioz_conf_6 & 0b11111100) | 0b00000010; break;
		case 3: bioz_conf_6 = (bioz_conf_6 & 0b11111100) | 0b00000011; break;
		default: bioz_conf_6 = (bioz_conf_6 & 0b11111100) | 0b00000000; break;
	}
	MAX30009_regWrite(MAX30009_BIOZ_CONFIGURATION_6_REGISTER,bioz_conf_6);
    if(debug) { printf("WR:0x%02X-0x%02X, ", MAX30009_BIOZ_CONFIGURATION_6_REGISTER, bioz_conf_6); print_bin8(bioz_conf_6); printf("\r\n"); }
    if(debug) printf("MAX30009_SetBiozAmpBW END\r\n");
}




void MAX30009_SetBiozLow(uint8_t bioz_low_thres,bool debug)
{
    if(debug) printf("MAX30009_SetBiozLow BEGIN\r\n");
    MAX30009_regWrite(MAX30009_BIOZ_LOW_THRESHOLD_REGISTER ,bioz_low_thres);
    if(debug) printf("MAX30009_SetBiozLow BEGIN\r\n");
}

void MAX30009_SetBiozHigh(uint8_t bioz_high_thres,bool debug)
{
    if(debug) printf("MAX30009_SetBiozLow BEGIN\r\n");
    MAX30009_regWrite(MAX30009_BIOZ_HIGH_THRESHOLD_REGISTER ,bioz_high_thres);
    if(debug) printf("MAX30009_SetBiozLow BEGIN\r\n");
}

void MAX30009_DetermineMagRange(struct imp_profile *cal_data, bool debug) // scans through the impedance profile and sets new exc currents and amp gains. Must be calibrated afterward.
{
    #define SETTLING_CYCLES         4
    struct MAX30009_status          status;
	uint8_t j=0, settling_counter   = 0, frequency = 0;
    uint8_t maximum_safe_current    = 0;
    uint8_t maximum_current         = 0;
    uint8_t maximum_gain            = 0;

    frequency                       = cal_data->frequency;        // set excitation frequency
    status.real_adc_span            = 0;
    status.imag_adc_span            = 0;
    status.idrv_ovr                 = false;

    // chip does not allow higher currents at lower frequencies, determine maximum current:
    //     if(max_frequencies[frequency] >= 16384){ maximum_safe_current = 15;}
    //else if(max_frequencies[frequency] >=  8192){ maximum_safe_current = 14;}
    //else if(max_frequencies[frequency] >=  2048){ maximum_safe_current = 13;}
    //else if(max_frequencies[frequency] >=   512){ maximum_safe_current = 12;}
    //else                                        { maximum_safe_current = 11;}


    MAX30009_SetFreq(frequency, false);            // using index, not actual frequency in hz
    MAX30009_SetBiozGain( 0 , false);// set the lowest gain
    MAX30009_SetStimulusIndex(0, false);
    MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x03, 1, 2);  //Set Bioz I and Q enable


    for(j = 0; j < maximum_safe_current+1; j++)
    {
        maximum_current = j;
        MAX30009_SetStimulusIndex(maximum_current, false);
        settling_counter = SETTLING_CYCLES;
        while(settling_counter)              //
        {
            if(max30009_int)
            {
            	max30009_int    = 0;
                status          = MAX30009_FifoReadRaw(false);
                settling_counter--;
                //if(debug)printf("F:%u Hz I=%lf, G=%u, I:%u, Re:%u, Im:%u \r\n ", max_frequencies[cal_data->frequency], 1000000 * max_current_magnitudes[maximum_current], max_gains[maximum_gain ],status.idrv_ovr, status.real_adc,status.imag_adc);
                //if(debug)printf("1 I:%u, Re:%u, Im:%u \r\n ", status.idrv_ovr, status.real_adc, status.imag_adc);
            }
            HAL_Delay(1);
        }
        //if(debug)printf("F:%u Hz I=%lf, G=%u, I:%u, Re:%u, Im:%u \r\n ", max_frequencies[cal_data->frequency], 1000000 * max_current_magnitudes[maximum_current], max_gains[maximum_gain ],status.idrv_ovr, status.real_adc,status.imag_adc);
        if(status.idrv_ovr)
        {
             //printf("BREAK I \r\n");
             break;
        }
        if(status.real_adc_span > 220)
        {
            //printf("BREAK Re %u \r\n",status.real_adc_span);
            break;
        }
        if(status.imag_adc_span > 220)
        {
            //printf("BREAK Im %u\r\n",status.imag_adc_span);
            break;
        }
    }


    if( status.idrv_ovr && (maximum_current > 0))
    {
        maximum_current -= 1;
    }
    else if (((status.real_adc_span > 240)||(status.imag_adc_span > 240) ) && (maximum_current > 0))
    {
        maximum_current -= 1;
    }

    MAX30009_SetStimulusIndex(maximum_current, false);
    settling_counter = SETTLING_CYCLES;

    while(settling_counter)
    {
        if(max30009_int)
        {
        	max30009_int    = 0;
            status          = MAX30009_FifoReadRaw(false);
            settling_counter--;
            //if(debug)printf("2 I:%u, Re:%u, Im:%u \r\n ", status.idrv_ovr, status.real_adc, status.imag_adc);
        }
        HAL_Delay(1);
    }

    for(j = 0; j < 4; j++)
    {
        maximum_gain = j;
        MAX30009_SetBiozGain(j, false);
        settling_counter = SETTLING_CYCLES;
        while(settling_counter)              //
        {
            if(max30009_int)
            {
            	max30009_int    = 0;
                status          = MAX30009_FifoReadRaw(false);
                settling_counter--;
                //if(debug)printf("3 I:%u, Re:%u, Im:%u \r\n ", status.idrv_ovr, status.real_adc, status.imag_adc);
            }
            HAL_Delay(1);
        }
        if(status.real_adc_span > 220)
        {
            //printf("BREAK2 Re %u \r\n",status.real_adc_span);
            break;
        }
        if(status.imag_adc_span > 220)
        {
            //printf("BREAK2 Im %u\r\n",status.imag_adc_span);
            break;
        }
    }

    if((maximum_gain > 0 ) && ((status.real_adc_span > 240) || (status.imag_adc_span > 240))) // adc overrange condition on the lowest gain - exitation current must be decreased
    {
            maximum_gain     -= 1;
    }

    MAX30009_SetBiozGain(maximum_gain, false);// set the lowest gain
    settling_counter = SETTLING_CYCLES;
    while(settling_counter)              //
    {
        if(max30009_int)
        {
        	max30009_int    = 0;
            status          = MAX30009_FifoReadRaw(false);
            settling_counter--;
            //if(debug)printf("3 I:%u, Re:%u, Im:%u \r\n ", status.idrv_ovr, status.real_adc, status.imag_adc);
        }
        HAL_Delay(1);
    }

    MAX30009_changeReg(MAX30009_BIOZ_CONFIGURATION_1_REGISTER   , 0x00, 1, 2);


    cal_data->calibrated            = false;
    cal_data->gain                  = maximum_gain;
    cal_data->stimulus_current      = maximum_current;

    //if(debug)printf("OPT: F:%u Hz: Isaf=%lf, I=%lf, G=%u, I:%u, Re:%u, Im:%u \r\n ", max_frequencies[cal_data->frequency], 1000000 * max_current_magnitudes[maximum_safe_current], 1000000 * max_current_magnitudes[maximum_current], max_gains[maximum_gain ],status.idrv_ovr, status.real_adc_span,status.imag_adc_span);

    //if(debug)printf("OPT: F:%u Hz: Isaf=%lf, Imax=%lf, Gmax=%u \r\n ", max_frequencies[cal_data->frequency],max_current_magnitudes[maximum_safe_current],max_current_magnitudes[cal_data->stimulus_current], max_gains[cal_data->gain ]);

}
