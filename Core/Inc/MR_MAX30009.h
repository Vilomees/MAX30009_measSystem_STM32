#ifndef MR_MAX30009_H_
#define MR_MAX30009_H_

/* CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include "MR_MAX30009_DEF.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

extern struct   	ble_ICG_profile  ble_icg_data;
extern struct   	ble_SPEC_profile le_spec_data;
extern bool 		ble_spectrum_mode;


extern volatile 	bool   			max30009_int; 	// set this high when MAX30009 INT pin fires
extern volatile  	bool   			max30009_fifo_new_data_readt;
extern        		uint8_t     	imp_data_index;
extern 				uint8_t     	imp_fifo_data[];
extern 				uint8_t     	imp_fifo_data_size;
extern struct 		imp_profile 	imp_cal_data_gen[20];
extern struct 		imp_profile 	imp_cal_data_spec[10];
extern struct 		MAX30009_status impedance_status;
extern struct 		impedance       impedance_data_array[];


int 	MAX30009_detect(bool debug);
int8_t 	MAX30009_GetInfo(void);
void 	MAX30009_reset(bool debug);
void 	MAX30009_general_init(bool debug);
void 	MAX30009_bioz_startup(bool debug);
void 	MAX30009_bioz_shutdown(bool debug);
void 	MAX30009_start_measurement(bool debug);
void 	MAX30009_stop_measurement(bool debug);
void 	MAX30009_SetFreq(uint8_t frequency,bool debug);
double 	MAX30009_SetStimulusIndex(uint8_t stimulus_index,bool debug);
double 	MAX30009_SetStimulusCode(uint8_t stimulus_code,bool debug);
uint8_t MAX30009_SetStimulus2(uint8_t stimulus, uint8_t voltag_mag, uint8_t current_mag,bool debug);
void 	MAX30009_SetAnalogHPF(uint8_t filter ,bool debug);
void 	MAX30009_SetLeadBias(uint8_t lead_bias, bool lb_bip, bool lb_bin ,bool debug);
void 	MAX30009_SetDigitalFilter(uint8_t dhpf, uint8_t dlpf,bool debug);
uint8_t MAX30009_SetBiozGain(uint8_t gain,bool debug);
void 	MAX30009_SetBiozDriveState(uint8_t reset,bool debug);
void 	MAX30009_SetBiozDACState(uint8_t reset,bool debug);
void 	MAX30009_SetBiozDCrestore(uint8_t state,bool debug);
void 	MAX30009_SetBiozExtCap(uint8_t state,bool debug);
void 	MAX30009_SetBiozAmpRange(uint8_t range,bool debug);
void 	MAX30009_SetBiozAmpBW(uint8_t bandwidth,bool debug);
void 	MAX30009_Clock(uint8_t clock_source, uint8_t clock_frequency, uint8_t int_clk_tune ,bool debug);
void 	MAX30009_SetBiozLow(uint8_t bioz_low_thres,bool debug);
void 	MAX30009_SetBiozHigh(uint8_t bioz_high_thres,bool debug);
void 	MAX30009_init(bool debug);
void 	MAX30009_setMdiv(int val);
void 	MAX30009_DetermineMagRange(struct imp_profile *cal_data, bool debug);

uint8_t	MAX30009_regRead(uint8_t reg);
void	MAX30009_regWrite(uint8_t reg, uint8_t val);
uint8_t MAX30009_changeReg(uint8_t reg, uint8_t val, uint8_t bit1, uint8_t numBits);
int     MAX30009_regReadMany(uint8_t reg, uint8_t no_of_bytes, uint8_t* buffer);
uint8_t MAX30009_FifoRead_DMA(uint8_t *imp_fifo_array, uint8_t imp_fifo_array_size, bool debug);

struct MAX30009_status MAX30009_FifoReadRaw(bool debug);
uint8_t MAX30009_FifoReadSamples(struct raw_impedance *samples, uint8_t max_samples,
                                  struct MAX30009_status *status_out, bool debug);

void 	MAX30009_SortFifoCode(uint8_t *rx_buffer, struct impedance *impedance_data_array, struct MAX30009_status *status, bool debug);
void 	MAX30009_CalculateImpedanceBasic(struct impedance *impedance_data_array, struct MAX30009_status *status, bool debug);
uint8_t MAX30009_GetDecodedSampleCount(void);


#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif // MR_MAX30009_H_
