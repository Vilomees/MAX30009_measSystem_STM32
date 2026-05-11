#ifndef MR_MAX30009_DEF_H_
#define MR_MAX30009_DEF_H_

/* CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


// DEFINES
	#define MAX30009_ID				    	0x42
	#define MAX30009_SAMPLE_COUNT_RAW    	40
    #define MAX30009_FIFO_ARRAY_SIZE        (2 + (MAX30009_SAMPLE_COUNT_RAW * 3))
	
	#define MAX30009_NUM_SAMPLES_PER_INT	2	/* number of samples in FIFO that generates a FIFO_A_FULL interrupt */
	#define MAX30009_NUM_BYTES_PER_SAMPLE 	3

	#define MAX30009_STATUS_1_REGISTER		0x00
	#define MAX30009_A_FULL_MASK			0x80
	#define MAX30009_A_FULL_SHIFT			7
	#define MAX30009_FIFO_DATA_RDY_MASK		0x20
	#define MAX30009_FIFO_DATA_RDY_SHIFT	5
	#define MAX30009_FREQ_UNLOCK_MASK		0x10
	#define MAX30009_FREQ_UNLOCK_SHIFT		4
	#define MAX30009_FREQ_LOCK_MASK			0x08
	#define MAX30009_FREQ_LOCK_SHIFT		3
	#define MAX30009_PHASE_UNLOCK_MASK		0x04
	#define MAX30009_PHASE_UNLOCK_SHIFT		2
	#define MAX30009_PHASE_LOCK_MASK		0x02
	#define MAX30009_PHASE_LOCK_SHIFT		1
	#define MAX30009_PWR_RDY_MASK			0x01
	#define MAX30009_PWR_RDY_SHIFT			0

	#define MAX30009_STATUS_2_REGISTER		0x01
	#define MAX30009_LON_MASK				0x80
	#define MAX30009_LON_SHIFT				7
	#define MAX30009__OVER_MASK				0x40
	#define MAX30009__OVER_SHIFT			6
	#define MAX30009__UNDR_MASK				0x20
	#define MAX30009__UNDR_SHIFT			5
	#define MAX30009_DRV_OOR_MASK			0x10
	#define MAX30009_DRV_OOR_SHIFT			4
	#define MAX30009_DC_LOFF_PH_MASK		0x08
	#define MAX30009_DC_LOFF_PH_SHIFT		3
	#define MAX30009_DC_LOFF_PL_MASK		0x04
	#define MAX30009_DC_LOFF_PL_SHIFT		2
	#define MAX30009_DC_LOFF_NH_MASK		0x02
	#define MAX30009_DC_LOFF_NH_SHIFT		1
	#define MAX30009_DC_LOFF_NL_MASK		0x01
	#define MAX30009_DC_LOFF_NL_SHIFT		0

	// Section - FIFO
	#define MAX30009_FIFO_WRITE_POINTER_REGISTER	0x08
	#define MAX30009_FIFO_WR_PTR_MASK				0xFF
	#define MAX30009_FIFO_WR_PTR_SHIFT				0

	#define MAX30009_FIFO_READ_POINTER_REGISTER		0x09
	#define MAX30009_FIFO_RD_PTR_MASK				0xFF
	#define MAX30009_FIFO_RD_PTR_SHIFT				0

	#define MAX30009_FIFO_COUNTER_1_REGISTER		0x0A
	#define MAX30009_FIFO_DATA_COUNT_MSB_MASK		0x80
	#define MAX30009_FIFO_DATA_COUNT_MSB_SHIFT		7
	#define MAX30009_OVF_COUNTER_MASK				0x7F
	#define MAX30009_OVF_COUNTER_SHIFT				0

	#define MAX30009_FIFO_COUNTER_2_REGISTER		0x0B
	#define MAX30009_FIFO_DATA_COUNT_LSB_MASK		0xFF
	#define MAX30009_FIFO_DATA_COUNT_LSB_SHIFT		0

	#define MAX30009_FIFO_DATA_REGISTER_REGISTER	0x0C
	#define MAX30009_FIFO_DATA_MASK					0xFF
	#define MAX30009_FIFO_DATA_SHIFT				0

	#define MAX30009_FIFO_CONFIGURATION_1_REGISTER	0x0D
	#define MAX30009_FIFO_A_FULL_MASK				0xFF
	#define MAX30009_FIFO_A_FULL_SHIFT				0

	#define MAX30009_FIFO_CONFIGURATION_2_REGISTER	0x0E
	#define MAX30009_FIFO_MARK_MASK					0x20
	#define MAX30009_FIFO_MARK_SHIFT				5
	#define MAX30009_FLUSH_FIFO_MASK				0x10
	#define MAX30009_FLUSH_FIFO_SHIFT				4
	#define MAX30009_FIFO_STAT_CLR_MASK				0x08
	#define MAX30009_FIFO_STAT_CLR_SHIFT			3
	#define MAX30009_A_FULL_TYPE_MASK				0x04
	#define MAX30009_A_FULL_TYPE_SHIFT				2
	#define MAX30009_FIFO_RO_MASK					0x02
	#define MAX30009_FIFO_RO_SHIFT					1

	// Section - System Control
	#define MAX30009_SYSTEM_SYNC_REGISTER			0x10
	#define MAX30009_TIMING_SYS_RESET_MASK			0x80
	#define MAX30009_TIMING_SYS_RESET_SHIFT			7

	#define MAX30009_SYSTEM_CONFIGURATION_1_REGISTER	0x11
	#define MAX30009_MASTER_MASK						0x80
	#define MAX30009_MASTER_SHIFT						7
	#define MAX30009_DISABLE_I2C_MASK					0x40
	#define MAX30009_DISABLE_I2C_SHIFT					6
	#define MAX30009_SHDN_MASK							0x02
	#define MAX30009_SHDN_SHIFT							1
	#define MAX30009_RESET_MASK							0x01
	#define MAX30009_RESET_SHIFT						0

	#define MAX30009_PIN_FUNCTIONAL_CONFIG_REGISTER		0x12
	#define MAX30009_INT_FCFG_MASK						0x0C
	#define MAX30009_INT_FCFG_SHIFT						2
	#define MAX30009_TRIG_ICFG_MASK						0x01
	#define MAX30009_TRIG_ICFG_SHIFT					0

	#define MAX30009_OUTPUT_PIN_CONFIGURATION_REGISTER	0x13
	#define MAX30009_INT_OCFG_MASK						0x0C
	#define MAX30009_INT_OCFG_SHIFT						2
	#define MAX30009_TRIG_OCFG_MASK						0x03
	#define MAX30009_TRIG_OCFG_SHIFT					0

	#define MAX30009_I2C_BROADCAST_ADDRESS_REGISTER		0x14
	#define MAX30009_I2C_BCAST_ADDR_MASK				0xFE
	#define MAX30009_I2C_BCAST_ADDR_SHIFT				1
	#define MAX30009_I2C_BCAST_EN_MASK					0x01
	#define MAX30009_I2C_BCAST_EN_SHIFT					0

	// Section - PLL
	#define MAX30009_PLL_CONFIGURATION_1_REGISTER	0x17
	#define MAX30009_MDIV_MSB_MASK					0xC0
	#define MAX30009_MDIV_MSB_SHIFT					6
	#define MAX30009_NDIV_MASK						0x20
	#define MAX30009_NDIV_SHIFT						5
	#define MAX30009_KDIV_MASK						0x1E
	#define MAX30009_KDIV_SHIFT						1
	#define MAX30009_PLL_EN_MASK					0x01
	#define MAX30009_PLL_EN_SHIFT					0

	#define MAX30009_PLL_CONFIGURATION_2_REGISTER	0x18
	#define MAX30009_MDIV_LSB_MASK					0xFF
	#define MAX30009_MDIV_LSB_SHIFT					0

	#define MAX30009_PLL_CONFIGURATION_3_REGISTER	0x19
	#define MAX30009_PLL_LOCK_WNDW_MASK				0x01
	#define MAX30009_PLL_LOCK_WNDW_SHIFT			0

	#define MAX30009_PLL_CONFIGURATION_4_REGISTER	0x1A
	#define MAX30009_REF_CLK_SEL_MASK				0x40
	#define MAX30009_REF_CLK_SEL_SHIFT				6
	#define MAX30009_CLK_FREQ_SEL_MASK				0x20
	#define MAX30009_CLK_FREQ_SEL_SHIFT				5
	#define MAX30009_CLK_FINE_TUNE_MASK				0x1F
	#define MAX30009_CLK_FINE_TUNE_SHIFT			0

	// Section - BioZ Setup
	#define MAX30009_BIOZ_CONFIGURATION_1_REGISTER	0x20
	#define MAX30009_BIOZ_DAC_OSR_MASK				0xC0
	#define MAX30009_BIOZ_DAC_OSR_SHIFT				6
	#define MAX30009_BIOZ_ADC_OSR_MASK				0x38
	#define MAX30009_BIOZ_ADC_OSR_SHIFT				3
	#define MAX30009_BIOZ_BG_EN_MASK				0x04
	#define MAX30009_BIOZ_BG_EN_SHIFT				2
	#define MAX30009_BIOZ_Q_EN_MASK					0x02
	#define MAX30009_BIOZ_Q_EN_SHIFT				1
	#define MAX30009_BIOZ_I_EN_MASK					0x01
	#define MAX30009_BIOZ_I_EN_SHIFT				0

	#define MAX30009_BIOZ_CONFIGURATION_2_REGISTER	0x21
	#define MAX30009_BIOZ_DHPF_MASK					0xC0
	#define MAX30009_BIOZ_DHPF_SHIFT				6
	#define MAX30009_BIOZ_DLPF_MASK					0x38
	#define MAX30009_BIOZ_DLPF_SHIFT				3
	#define MAX30009_BIOZ_CMP_MASK					0x06
	#define MAX30009_BIOZ_CMP_SHIFT					1
	#define MAX30009_EN_BIOZ_THRESH_MASK			0x01
	#define MAX30009_EN_BIOZ_THRESH_SHIFT			0

	#define MAX30009_BIOZ_CONFIGURATION_3_REGISTER	0x22
	#define MAX30009_BIOZ_EXT_RES_MASK				0x80
	#define MAX30009_BIOZ_EXT_RES_SHIFT				7
	#define MAX30009_LOFF_RAPID_MASK				0x40
	#define MAX30009_LOFF_RAPID_SHIFT				6
	#define MAX30009_BIOZ_VDRV_MAG_MASK				0x30
	#define MAX30009_BIOZ_VDRV_MAG_SHIFT			4
	#define MAX30009_BIOZ_IDRV_RGE_MASK				0x0C
	#define MAX30009_BIOZ_IDRV_RGE_SHIFT			2
	#define MAX30009_BIOZ_DRV_MODE_MASK				0x03
	#define MAX30009_BIOZ_DRV_MODE_SHIFT			0

	#define MAX30009_BIOZ_CONFIGURATION_4_REGISTER	0x23
	#define MAX30009_BIOZ_FAST_MANUAL_MASK			0x02
	#define MAX30009_BIOZ_FAST_MANUAL_SHIFT			1
	#define MAX30009_BIOZ_FAST_START_EN_MASK		0x01
	#define MAX30009_BIOZ_FAST_START_EN_SHIFT		0

	#define MAX30009_BIOZ_CONFIGURATION_5_REGISTER	0x24
	#define MAX30009_BIOZ_AHPF_MASK					0xF0
	#define MAX30009_BIOZ_AHPF_SHIFT				4
	#define MAX30009_BIOZ_INA_MODE_MASK				0x08
	#define MAX30009_BIOZ_INA_MODE_SHIFT			3
	#define MAX30009_BIOZ_DM_DIS_MASK				0x04
	#define MAX30009_BIOZ_DM_DIS_SHIFT				2
	#define MAX30009_BIOZ_GAIN_MASK					0x03
	#define MAX30009_BIOZ_GAIN_SHIFT				0

	#define MAX30009_BIOZ_CONFIGURATION_6_REGISTER	0x25
	#define MAX30009_BIOZ_EXT_CAP_MASK				0x80
	#define MAX30009_BIOZ_EXT_CAP_SHIFT				7
	#define MAX30009_BIOZ_DC_RESTORE_MASK			0x40
	#define MAX30009_BIOZ_DC_RESTORE_SHIFT			6
	#define MAX30009_BIOZ_DRV_RESET_MASK			0x20
	#define MAX30009_BIOZ_DRV_RESET_SHIFT			5
	#define MAX30009_BIOZ_DAC_RESET_MASK			0x10
	#define MAX30009_BIOZ_DAC_RESET_SHIFT			4
	#define MAX30009_BIOZ_AMP_RGE_MASK				0x0C
	#define MAX30009_BIOZ_AMP_RGE_SHIFT				2
	#define MAX30009_BIOZ_AMP_BW_MASK				0x03
	#define MAX30009_BIOZ_AMP_BW_SHIFT				0

	#define MAX30009_BIOZ_LOW_THRESHOLD_REGISTER	0x26
	#define MAX30009_BIOZ_LO_THRESH_MASK			0xFF
	#define MAX30009_BIOZ_LO_THRESH_SHIFT			0

	#define MAX30009_BIOZ_HIGH_THRESHOLD_REGISTER	0x27
	#define MAX30009_BIOZ_HI_THRESH_MASK			0xFF
	#define MAX30009_BIOZ_HI_THRESH_SHIFT			0

	#define MAX30009_BIOZ_CONFIGURATION_7_REGISTER	0x28
	#define MAX30009_BIOZ_STBYON_MASK				0x10
	#define MAX30009_BIOZ_STBYON_SHIFT				4
	#define MAX30009_BIOZ_Q_CLK_PHASE_MASK			0x08
	#define MAX30009_BIOZ_Q_CLK_PHASE_SHIFT			3
	#define MAX30009_BIOZ_I_CLK_PHASE_MASK			0x04
	#define MAX30009_BIOZ_I_CLK_PHASE_SHIFT			2
	#define MAX30009_BIOZ_INA_CHOP_EN_MASK			0x02
	#define MAX30009_BIOZ_INA_CHOP_EN_SHIFT			1
	#define MAX30009_BIOZ_CH_FSEL_MASK				0x01
	#define MAX30009_BIOZ_CH_FSEL_SHIFT				0

	// Section - BioZ Calibration
	#define MAX30009_BIOZ_MUX_CONFIGURATION_1_REGISTER	0x41
	#define MAX30009_BMUX_RSEL_MASK						0xC0
	#define MAX30009_BMUX_RSEL_SHIFT					6
	#define MAX30009_BMUX_BIST_EN_MASK					0x20
	#define MAX30009_BMUX_BIST_EN_SHIFT					5
	#define MAX30009_CONNECT_CAL_ONLY_MASK				0x04
	#define MAX30009_CONNECT_CAL_ONLY_SHIFT				2
	#define MAX30009_MUX_EN_MASK						0x02
	#define MAX30009_MUX_EN_SHIFT						1
	#define MAX30009_CAL_EN_MASK						0x01
	#define MAX30009_CAL_EN_SHIFT						0

	#define MAX30009_BIOZ_MUX_CONFIGURATION_2_REGISTER	0x42
	#define MAX30009_BMUX_GSR_RSEL_MASK					0xC0
	#define MAX30009_BMUX_GSR_RSEL_SHIFT				6
	#define MAX30009_GSR_LOAD_EN_MASK					0x20
	#define MAX30009_GSR_LOAD_EN_SHIFT					5
	#define MAX30009_EN_EXT_INLOAD_MASK					0x02
	#define MAX30009_EN_EXT_INLOAD_SHIFT				1
	#define MAX30009_EN_INT_INLOAD_MASK					0x01
	#define MAX30009_EN_INT_INLOAD_SHIFT				0

	#define MAX30009_BIOZ_MUX_CONFIGURATION_3_REGISTER	0x43
	#define MAX30009_BIP_ASSIGN_MASK					0xC0
	#define MAX30009_BIP_ASSIGN_SHIFT					6
	#define MAX30009_BIN_ASSIGN_MASK					0x30
	#define MAX30009_BIN_ASSIGN_SHIFT					4
	#define MAX30009_DRVP_ASSIGN_MASK					0x0C
	#define MAX30009_DRVP_ASSIGN_SHIFT					2
	#define MAX30009_DRVN_ASSIGN_MASK					0x03
	#define MAX30009_DRVN_ASSIGN_SHIFT					0

	#define MAX30009_BIOZ_MUX_CONFIGURATION_4_REGISTER	0x44
	#define MAX30009_BIST_R_ERR_MASK					0xFF
	#define MAX30009_BIST_R_ERR_SHIFT					0

	// Section - DC Leads Setup
	#define MAX30009_DC_LEADS_CONFIGURATION_REGISTER	0x50
	#define MAX30009_EN_LON_DET_MASK					0x80
	#define MAX30009_EN_LON_DET_SHIFT					7
	#define MAX30009_EN_LOFF_DET_MASK					0x40
	#define MAX30009_EN_LOFF_DET_SHIFT					6
	#define MAX30009_EN_EXT_LOFF_MASK					0x20
	#define MAX30009_EN_EXT_LOFF_SHIFT					5
	#define MAX30009_EN_DRV_OOR_MASK					0x10
	#define MAX30009_EN_DRV_OOR_SHIFT					4
	#define MAX30009_LOFF_IPOL_MASK						0x08
	#define MAX30009_LOFF_IPOL_SHIFT					3
	#define MAX30009_LOFF_IMAG_MASK						0x07
	#define MAX30009_LOFF_IMAG_SHIFT					0

	#define MAX30009_DC_LEAD_DETECT_THRESHOLD_REGISTER	0x51
	#define MAX30009_LOFF_THRESH_MASK					0x0F
	#define MAX30009_LOFF_THRESH_SHIFT					0

	// Section - Lead Bias
	#define MAX30009_LEAD_BIAS_CONFIGURATION_1_REGISTER	0x58
	#define MAX30009_RBIAS_VALUE_MASK					0x0C
	#define MAX30009_RBIAS_VALUE_SHIFT					2
	#define MAX30009_EN_RBIAS_BIP_MASK					0x02
	#define MAX30009_EN_RBIAS_BIP_SHIFT					1
	#define MAX30009_EN_RBIAS_BIN_MASK					0x01
	#define MAX30009_EN_RBIAS_BIN_SHIFT					0
	#define MAX30009_LEAD_BIAS_50MEG            		0x0
	#define MAX30009_LEAD_BIAS_100MEG           		0x1
	#define MAX30009_LEAD_BIAS_200MEG           		0x2
	#define MAX30009_LEAD_BIAS_OFF              		0x3

	// Section - Interrupt Enables
	#define MAX30009_INTERRUPT_ENABLE_1_REGISTER		0x80
	#define MAX30009_A_FULL_EN_MASK						0x80
	#define MAX30009_A_FULL_EN_SHIFT					7
	#define MAX30009_FIFO_DATA_RDY_EN_MASK				0x20
	#define MAX30009_FIFO_DATA_RDY_EN_SHIFT				5
	#define MAX30009_FREQ_UNLOCK_EN_MASK				0x10
	#define MAX30009_FREQ_UNLOCK_EN_SHIFT				4
	#define MAX30009_FREQ_LOCK_EN_MASK					0x08
	#define MAX30009_FREQ_LOCK_EN_SHIFT					3
	#define MAX30009_PHASE_UNLOCK_EN_MASK				0x04
	#define MAX30009_PHASE_UNLOCK_EN_SHIFT				2
	#define MAX30009_PHASE_LOCK_EN_MASK					0x02
	#define MAX30009_PHASE_LOCK_EN_SHIFT				1

	#define MAX30009_INTERRUPT_ENABLE_2_REGISTER		0x81
	#define MAX30009_LON_EN_MASK						0x80
	#define MAX30009_LON_EN_SHIFT						7
	#define MAX30009_BIOZ_OVER_EN_MASK					0x40
	#define MAX30009_BIOZ_OVER_EN_SHIFT					6
	#define MAX30009_BIOZ_UNDR_EN_MASK					0x20
	#define MAX30009_BIOZ_UNDR_EN_SHIFT					5
	#define MAX30009_DRVP_OFF_EN_MASK					0x10
	#define MAX30009_DRVP_OFF_EN_SHIFT					4
	#define MAX30009_DC_LOFF_PH_EN_MASK					0x08
	#define MAX30009_DC_LOFF_PH_EN_SHIFT				3
	#define MAX30009_DC_LOFF_PL_EN_MASK					0x04
	#define MAX30009_DC_LOFF_PL_EN_SHIFT				2
	#define MAX30009_DC_LOFF_NH_EN_MASK					0x02
	#define MAX30009_DC_LOFF_NH_EN_SHIFT				1
	#define MAX30009_DC_LOFF_NL_EN_MASK					0x01
	#define MAX30009_DC_LOFF_NL_EN_SHIFT				0

	// Section - Part ID
	#define MAX30009_PART_ID_REGISTER					0xFF
	#define MAX30009_PART_ID_MASK						0xFF
	#define MAX30009_PART_ID_SHIFT						0
	
	
	// for max30009_SetFreq(uint8_t frequency,bool debug)
	#define MAX30009_BIOZ_FREQ_808960Hz   0
	#define MAX30009_BIOZ_FREQ_722944Hz   1
	#define MAX30009_BIOZ_FREQ_649216Hz   2
	#define MAX30009_BIOZ_FREQ_581632Hz   3
	#define MAX30009_BIOZ_FREQ_499712Hz   4
	#define MAX30009_BIOZ_FREQ_468992Hz   5
	#define MAX30009_BIOZ_FREQ_420864Hz   6
	#define MAX30009_BIOZ_FREQ_377856Hz   7
	#define MAX30009_BIOZ_FREQ_338944Hz   8
	#define MAX30009_BIOZ_FREQ_304128Hz   9
	#define MAX30009_BIOZ_FREQ_272896Hz  10
	#define MAX30009_BIOZ_FREQ_249856Hz  11
	#define MAX30009_BIOZ_FREQ_245248Hz  12
	#define MAX30009_BIOZ_FREQ_220160Hz  13
	#define MAX30009_BIOZ_FREQ_199936Hz  14
	#define MAX30009_BIOZ_FREQ_176896Hz  15
	#define MAX30009_BIOZ_FREQ_158976Hz  16
	#define MAX30009_BIOZ_FREQ_143104Hz  17
	#define MAX30009_BIOZ_FREQ_131072Hz  18
	#define MAX30009_BIOZ_FREQ_114944Hz  19
	#define MAX30009_BIOZ_FREQ_99968Hz   20
	#define MAX30009_BIOZ_FREQ_93056Hz   21
	#define MAX30009_BIOZ_FREQ_82944Hz   22
	#define MAX30009_BIOZ_FREQ_82048Hz   23
	#define MAX30009_BIOZ_FREQ_75008Hz   24
	#define MAX30009_BIOZ_FREQ_66944Hz   25
	#define MAX30009_BIOZ_FREQ_60032Hz   26
	#define MAX30009_BIOZ_FREQ_54016Hz   27
	#define MAX30009_BIOZ_FREQ_49984Hz   28
	#define MAX30009_BIOZ_FREQ_43008Hz   29
	#define MAX30009_BIOZ_FREQ_41024Hz   30
	#define MAX30009_BIOZ_FREQ_38976Hz   31
	#define MAX30009_BIOZ_FREQ_35008Hz   32
	#define MAX30009_BIOZ_FREQ_30976Hz   33
	#define MAX30009_BIOZ_FREQ_28032Hz   34
	#define MAX30009_BIOZ_FREQ_24992Hz   35
	#define MAX30009_BIOZ_FREQ_23008Hz   36
	#define MAX30009_BIOZ_FREQ_20000Hz   37
	#define MAX30009_BIOZ_FREQ_18016Hz   38
	#define MAX30009_BIOZ_FREQ_16000Hz   39
	#define MAX30009_BIOZ_FREQ_15008Hz   40
	#define MAX30009_BIOZ_FREQ_14016Hz   41
	#define MAX30009_BIOZ_FREQ_13008Hz   42
	#define MAX30009_BIOZ_FREQ_12000Hz   43
	#define MAX30009_BIOZ_FREQ_11008Hz   44
	#define MAX30009_BIOZ_FREQ_10000Hz   45
	#define MAX30009_BIOZ_FREQ_9008Hz    46
	#define MAX30009_BIOZ_FREQ_8000Hz    47
	#define MAX30009_BIOZ_FREQ_7008Hz    48
	#define MAX30009_BIOZ_FREQ_6000Hz    49
	#define MAX30009_BIOZ_FREQ_5000Hz    50
	#define MAX30009_BIOZ_FREQ_4000Hz    51
	#define MAX30009_BIOZ_FREQ_2000Hz    52
	#define MAX30009_BIOZ_FREQ_1000Hz    53
	#define MAX30009_BIOZ_FREQ_500Hz     54
	#define MAX30009_BIOZ_FREQ_250Hz     55
	#define MAX30009_BIOZ_FREQ_125Hz     56
	#define MAX30009_BIOZ_FREQ_64Hz      57
	#define MAX30009_BIOZ_FREQ_32Hz      58
	#define MAX30009_BIOZ_FREQ_16Hz      59	
	
	
	//max30009_SetStimulusIndex(uint8_t stimulus_index,bool debug)
									    //    xxvviimm
	#define MAX30009_STIM_H      	    	0b00000010

	#define MAX30009_STIM_V_35mV      		0b00000001
	#define MAX30009_STIM_V_71mV      		0b00010001
	#define MAX30009_STIM_V_177mV     		0b00100001
	#define MAX30009_STIM_V_354mV     		0b00110001

	#define MAX30009_STIM_I_ind_16nA      	0b00000000
	#define MAX30009_STIM_I_ind_32nA      	0b00010000
	#define MAX30009_STIM_I_ind_80nA      	0b00100000
	#define MAX30009_STIM_I_ind_160nA     	0b00110000
	#define MAX30009_STIM_I_ind_320nA     	0b00000100
	#define MAX30009_STIM_I_ind_640nA     	0b00010100
	#define MAX30009_STIM_I_ind_1600nA    	0b00100100
	#define MAX30009_STIM_I_ind_3200nA    	0b00110100
	#define MAX30009_STIM_I_ind_6400nA    	0b00001000
	#define MAX30009_STIM_I_ind_12800nA   	0b00011000
	#define MAX30009_STIM_I_ind_32uA      	0b00101000
	#define MAX30009_STIM_I_ind_64uA      	0b00111000
	#define MAX30009_STIM_I_ind_128uA     	0b00001100
	#define MAX30009_STIM_I_ind_256uA     	0b00011100
	#define MAX30009_STIM_I_ind_640uA     	0b00101100
	#define MAX30009_STIM_I_ind_1280uA   	0b00111100	

	#define MAX30009_STIM_I_code_16nA      	0b00000000
	#define MAX30009_STIM_I_code_32nA      	0b00010000
	#define MAX30009_STIM_I_code_80nA      	0b00100000
	#define MAX30009_STIM_I_code_160nA     	0b00110000
	#define MAX30009_STIM_I_code_320nA     	0b00000100
	#define MAX30009_STIM_I_code_640nA     	0b00010100
	#define MAX30009_STIM_I_code_1600nA    	0b00100100
	#define MAX30009_STIM_I_code_3200nA    	0b00110100
	#define MAX30009_STIM_I_code_6400nA    	0b00001000
	#define MAX30009_STIM_I_code_12800nA   	0b00011000
	#define MAX30009_STIM_I_code_32uA      	0b00101000
	#define MAX30009_STIM_I_code_64uA      	0b00111000
	#define MAX30009_STIM_I_code_128uA     	0b00001100
	#define MAX30009_STIM_I_code_256uA     	0b00011100
	#define MAX30009_STIM_I_code_640uA     	0b00101100
	#define MAX30009_STIM_I_code_1280uA   	0b00111100

	//max30009_SetStimulusCode(uint8_t stimulus_code,bool debug)
	#define MAX30009_BIOZ_STIMULUS_I      	0
	#define MAX30009_BIOZ_STIMULUS_V      	1
	#define MAX30009_BIOZ_STIMULUS_H      	2

	#define MAX30009_BIOZ_V_MAG_35mV      	0
	#define MAX30009_BIOZ_V_MAG_71mV      	1
	#define MAX30009_BIOZ_V_MAG_177mV     	2
	#define MAX30009_BIOZ_V_MAG_354mV     	3

	#define MAX30009_BIOZ_I_MAG_16nA      	0
	#define MAX30009_BIOZ_I_MAG_32nA      	1
	#define MAX30009_BIOZ_I_MAG_80nA      	2
	#define MAX30009_BIOZ_I_MAG_160nA     	3
	#define MAX30009_BIOZ_I_MAG_320nA     	4
	#define MAX30009_BIOZ_I_MAG_640nA     	5
	#define MAX30009_BIOZ_I_MAG_1600nA    	6
	#define MAX30009_BIOZ_I_MAG_3200nA    	7
	#define MAX30009_BIOZ_I_MAG_6400nA    	8
	#define MAX30009_BIOZ_I_MAG_12800nA   	9
	#define MAX30009_BIOZ_I_MAG_32uA      	10
	#define MAX30009_BIOZ_I_MAG_64uA      	11
	#define MAX30009_BIOZ_I_MAG_128uA     	12
	#define MAX30009_BIOZ_I_MAG_256uA     	13
	#define MAX30009_BIOZ_I_MAG_640uA     	14
	#define MAX30009_BIOZ_I_MAG_12800uA   	15

	//max30009_SetStimulus2(uint8_t stimulus, uint8_t voltag_mag, uint8_t current_mag,bool debug)
	#define MAX30009_BIOZ_AN_HPF_BYPASS    	15
	#define MAX30009_BIOZ_AN_HPF_100Hz      0
	#define MAX30009_BIOZ_AN_HPF_200Hz      1
	#define MAX30009_BIOZ_AN_HPF_500Hz      2
	#define MAX30009_BIOZ_AN_HPF_1kHz       3
	#define MAX30009_BIOZ_AN_HPF_2kHz       4
	#define MAX30009_BIOZ_AN_HPF_5kHz       5
	#define MAX30009_BIOZ_AN_HPF_10kHz      6
	#define MAX30009_BIOZ_AN_HPF_42MOhm     7
	#define MAX30009_BIOZ_AN_HPF_21MOhm     8
	#define MAX30009_BIOZ_AN_HPF_8MOhm      9
	#define MAX30009_BIOZ_AN_HPF_4MOhm     	10
	#define MAX30009_BIOZ_AN_HPF_2MOhm     	11
	#define MAX30009_BIOZ_AN_HPF_1MOhm     	12
	
	//max30009_SetAnalogHPF(uint8_t filter ,bool debug)
	#define MAX30009_DHPF_BYP               0x0
	#define MAX30009_DHPF_000025_SR_BIOZ    0x1
	#define MAX30009_DHPF_0002_SR_BIOZ      0x2
	#define MAX30009_DLPF_BYP               0x0
	#define MAX30009_DLPF_0005_SR_BIOZ      0x1
	#define MAX30009_DLPF_002_SR_BIOZ       0x2
	#define MAX30009_DLPF_008_SR_BIOZ       0x3
	#define MAX30009_DLPF_025_SR_BIOZ       0x3	
	
	//max30009_SetDigitalFilter(uint8_t dhpf, uint8_t dlpf,bool debug);
	#define MAX30009_BIOZ_GAIN_1X           0
	#define MAX30009_BIOZ_GAIN_2X           1
	#define MAX30009_BIOZ_GAIN_5X           2
	#define MAX30009_BIOZ_GAIN_10X          3
		
	//max30009_SetBiozGain(uint8_t gain,bool debug)
	#define MAX30009_BIOZ_DRV_RESET_SET     1
	#define MAX30009_BIOZ_DRV_RESET_CLR     0

	//max30009_SetBiozDriveState(uint8_t reset,bool debug)
	#define MAX30009_BIOZ_DAC_RESET_SET     1
	#define MAX30009_BIOZ_DAC_RESET_CLR     0

	//max30009_SetBiozDACState(uint8_t reset,bool debug)
	#define MAX30009_BIOZ_DC_RESTORE_ON    	1
	#define MAX30009_BIOZ_DC_RESTORE_OFF   	0

	//max30009_SetBiozDCrestore(uint8_t state,bool debug)
	#define MAX30009_BIOZ_EXTCAP_ON    		1
	#define MAX30009_BIOZ_EXTCAP_OFF   		0

	//max30009_SetBiozExtCap(uint8_t state,bool debug)
	#define MAX30009_BIOZ_RANGE_LOW      	0
	#define MAX30009_BIOZ_RANGE_MEDLOW   	1
	#define MAX30009_BIOZ_RANGE_MEDHIGH  	2
	#define MAX30009_BIOZ_RANGE_HIGH     	3

	//max30009_SetBiozAmpRange(uint8_t range,bool debug)
	#define MAX30009_BIOZ_BW_LOW      		0
	#define MAX30009_BIOZ_BW_MEDLOW   		1
	#define MAX30009_BIOZ_BW_MEDHIGH  		2
	#define MAX30009_BIOZ_BW_HIGH     		3

	//max30009_SetBiozAmpBW(uint8_t bandwidth,bool debug)
	#define MAX30009_CLOCK_INT     			0b00000000
	#define MAX30009_CLOCK_EXT     			0b01000000
	#define MAX30009_CLOCK_32768   		    0b00100000
	#define MAX30009_CLOCK_32000 			0b00000000
	#define MAX30009_CLOCK_TUNE_000 		0b00000000
	#define MAX30009_CLOCK_TUNE_P02 		0b00000001
	#define MAX30009_CLOCK_TUNE_P04 		0b00000010
	#define MAX30009_CLOCK_TUNE_P06 		0b00000011
	#define MAX30009_CLOCK_TUNE_P08 		0b00000100
	#define MAX30009_CLOCK_TUNE_P10 		0b00000101
	#define MAX30009_CLOCK_TUNE_P12 		0b00000110
	#define MAX30009_CLOCK_TUNE_P14 		0b00000111
	#define MAX30009_CLOCK_TUNE_016 		0b00001000
	#define MAX30009_CLOCK_TUNE_P18 		0b00001001
	#define MAX30009_CLOCK_TUNE_P20 		0b00001010
	#define MAX30009_CLOCK_TUNE_P22 		0b00001011
	#define MAX30009_CLOCK_TUNE_P24 		0b00001100
	#define MAX30009_CLOCK_TUNE_P26 		0b00001101
	#define MAX30009_CLOCK_TUNE_P28 		0b00001110
	#define MAX30009_CLOCK_TUNE_P30 		0b00001111
	#define MAX30009_CLOCK_TUNE_N32 		0b00010000
	#define MAX30009_CLOCK_TUNE_N30 		0b00010001
	#define MAX30009_CLOCK_TUNE_N28 		0b00010010
	#define MAX30009_CLOCK_TUNE_N26 		0b00010011
	#define MAX30009_CLOCK_TUNE_N24 		0b00010100
	#define MAX30009_CLOCK_TUNE_N22 		0b00010101
	#define MAX30009_CLOCK_TUNE_N20 		0b00010110
	#define MAX30009_CLOCK_TUNE_N18 		0b00010111
	#define MAX30009_CLOCK_TUNE_N16 		0b00011000
	#define MAX30009_CLOCK_TUNE_N14 		0b00011001
	#define MAX30009_CLOCK_TUNE_N12 		0b00011010
	#define MAX30009_CLOCK_TUNE_N10 		0b00011011
	#define MAX30009_CLOCK_TUNE_N08 		0b00011100
	#define MAX30009_CLOCK_TUNE_N06 		0b00011101
	#define MAX30009_CLOCK_TUNE_N04 		0b00011110
	#define MAX30009_CLOCK_TUNE_N02 		0b00011111	

// DEFINES END


// ENUM


// ENUM END


// STRUCTS

struct imp_IQ
{
	float real;
	float imag;
};


struct MAX30009_status
{
	uint32_t	index;
	int32_t 	real_adc_avg;
	int32_t 	imag_adc_avg;
	uint8_t 	real_adc_span;
	uint8_t 	imag_adc_span;
	uint8_t 	frequency_index;
	uint8_t 	current_index;
	uint8_t 	gain_index;
	bool    	idrv_ovr;
	bool    	code_ovr;
	bool    	code_und;
	bool    	leadsoff;
	bool    	spi_fault;
};

struct raw_impedance
{
	uint32_t index;
	int32_t  realbits;
	int32_t  imagbits;
	bool     marker;
	bool     error;
	uint8_t  flags;
	uint8_t  real_span;
	uint8_t  imag_span;
};

struct impedance
{
	uint32_t    index;
	int32_t    	realbits;
	int32_t    	imagbits;
	bool    	marker;
	bool    	error;
	uint8_t    	flags;

	double  	real;
	double  	imag;
	double  	mag;
	double  	pha;
};


struct imp_conf_registers
{
	uint8_t fifo_cnf1;
	uint8_t fifo_cnf2;

	uint8_t sys_cnf1;
	uint8_t sys_cnf2;
	uint8_t sys_cnf3;
	uint8_t sys_cnf4;

	uint8_t pll_cnf1;
	uint8_t pll_cnf2;
	uint8_t pll_cnf3;
	uint8_t pll_cnf4;

	uint8_t bioz_cnf1;
	uint8_t bioz_cnf2;
	uint8_t bioz_cnf3;
	uint8_t bioz_cnf4;
	uint8_t bioz_cnf5;
	uint8_t bioz_cnf6;
	uint8_t bioz_thres_hi;
	uint8_t bioz_thres_lo;
	uint8_t bioz_cnf7;

	uint8_t mux_cnf1;
	uint8_t mux_cnf2;
	uint8_t mux_cnf3;
	uint8_t mux_cnf4;	

	uint8_t dc_lead_cnf;
	uint8_t dc_lead_thres;
	uint8_t lead_bias_cnf;

	uint8_t int_en1;
	uint8_t int_en2;
};


struct imp_profile
{
	uint8_t 						frequency;
	uint8_t 						stimulus_current;
	uint8_t 						gain;
	bool							calibrated;	
	int32_t							I_offset_bits;	
	int32_t							Q_offset_bits;	
	double      					I_mag_coef;
	double      					Q_mag_coef;	
	double      					I_pha_coef;
	double      					Q_pha_coef;
};

struct imp_profile2 //96 bytes, extended configuration data included
{
	uint8_t 						frequency;
	uint8_t 						stimulus_current;
	uint8_t 						gain;
	struct  imp_conf_registers 		general_conf_regs;
	bool							calibrated;
	int32_t							I_offset_bits;	
	int32_t							Q_offset_bits;	
	double      					I_mag_coef;
	double      					Q_mag_coef;	
	double      					I_pha_coef;
	double      					Q_pha_coef;
};



// STRUCTS END








#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif // MR_MAX30009_DEF_H_
