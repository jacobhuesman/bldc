#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout

#include "commands.h"
#include "comm_can.h"
#include "app.h"
#include "hw.h"
 
// TODO: figure out how much stack space is actually needed
static THD_FUNCTION(can_status1, arg);
static THD_WORKING_AREA(can_status1_wa, 2048); // 2kb stack for this thread
 
static THD_FUNCTION(can_status2, arg);
static THD_WORKING_AREA(can_status2_wa, 2048); // 2kb stack for this thread

static THD_FUNCTION(can_status3, arg);
static THD_WORKING_AREA(can_status3_wa, 2048); // 2kb stack for this thread

static THD_FUNCTION(can_status4, arg);
static THD_WORKING_AREA(can_status4_wa, 2048); // 2kb stack for this thread

static THD_FUNCTION(custom_control, arg);
static THD_WORKING_AREA(custom_control_wa, 2048); // 2kb stack for this thread

#ifdef LIMIT_SWITCH
static THD_FUNCTION(limit_switcher, arg);
static THD_WORKING_AREA(limit_switcher_wa, 1024); // 1kb stack for this thread
#endif

static volatile mc_configuration *m_conf;

void app_can_status_init(void) {
	// Set the UART TX pin as an input with pulldown
	#ifdef LIMIT_SWITCH
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);
	#endif
 
	// Start the example thread
	chThdCreateStatic(can_status1_wa, sizeof(can_status1_wa),
		NORMALPRIO, can_status1, NULL);
	chThdCreateStatic(can_status2_wa, sizeof(can_status2_wa),
		NORMALPRIO, can_status2, NULL);
	chThdCreateStatic(can_status3_wa, sizeof(can_status3_wa),
		NORMALPRIO, can_status3, NULL);
	chThdCreateStatic(can_status4_wa, sizeof(can_status4_wa),
		NORMALPRIO, can_status4, NULL);

	chThdCreateStatic(custom_control_wa, sizeof(custom_control_wa),
		NORMALPRIO, custom_control, NULL);

	m_conf = mc_interface_get_configuration();

	
#ifdef LIMIT_SWITCH
	chThdCreateStatic(limit_switcher_wa, sizeof(limit_switcher_wa),
		NORMALPRIO, limit_switcher, NULL);
#endif
}
 
// most updated 100hz
typedef struct VESC_status1 { // 64bits
	int rpm:32;
	int position:16;
	int motorCurrent:16;
} VESC_status1;

// slightly less updated 50hz
typedef struct VESC_status2 { // 32bits
	int tachometer:32;
	unsigned adc:12;
	unsigned flimit:1;
	unsigned rlimit:1;
}__attribute__((packed)) VESC_status2;

// even less updated 10hz
typedef struct VESC_status3 { // 60bits
	float wattHours;
	int inCurrent:16;
	unsigned voltage:12;
} VESC_status3;
typedef struct VESC_status4 { // 29bits
	unsigned tempMotor:12;
	unsigned tempPCB:12;
	unsigned faultCode:3;
	unsigned state:2;
	unsigned encoderIndex:1;
} VESC_status4;

static THD_FUNCTION(can_status1, arg) {
	(void)arg;
 
	chRegSetThreadName("CAN_STATUS1");
 
	for(;;) {
		VESC_status1 status1;
		status1.rpm = mc_interface_get_rpm();
		status1.position = (int) (mc_interface_get_pid_pos_now()*1000);
		status1.motorCurrent = (int) (mc_interface_get_tot_current()*10);
		comm_can_transmit(app_get_configuration()->controller_id |  (CAN_PACKET_STATUS1 << 8), (uint8_t*) &status1, sizeof(VESC_status1));

		//commands_printf("%f\t%f\t%f", 3.3* ADC_Value[11]/4095.0, NTC_TEMP_GND(ADC_IND_TEMP_MOTOR) , NTC_TEMP(ADC_IND_TEMP_PCB));
		// Run this loop at 100Hz
		chThdSleepMilliseconds(10);
 
		// Reset the timeout
		//timeout_reset();
	}
}
static THD_FUNCTION(can_status2, arg) {
	(void)arg;
	chRegSetThreadName("CAN_STATUS2");
	for(;;) {
		VESC_status2 status2;
		status2.tachometer = mc_interface_get_tachometer_value(false);
		status2.adc = ADC_Value[ADC_IND_EXT];
	#ifdef LIMIT_SWITCH
		status2.flimit = mc_interface_get_for_lim();
		status2.rlimit = mc_interface_get_rev_lim();
#endif
		comm_can_transmit(app_get_configuration()->controller_id |  (CAN_PACKET_STATUS2 << 8), (uint8_t*) &status2, sizeof(VESC_status2));
		chThdSleepMilliseconds(20); // Run this loop at 50Hz
	}
}
static THD_FUNCTION(can_status3, arg) {
	(void)arg;
	chRegSetThreadName("CAN_STATUS3");
	for(;;) {
		VESC_status3 status3;
		status3.wattHours = mc_interface_get_watt_hours(false);
		status3.inCurrent = mc_interface_get_tot_current_in() * 100;
		status3.voltage = ADC_Value[ADC_IND_VIN_SENS];
		comm_can_transmit(app_get_configuration()->controller_id |  (CAN_PACKET_STATUS3 << 8), (uint8_t*) &status3, sizeof(VESC_status3));
		chThdSleepMilliseconds(100); // Run this loop at 10Hz
	}
}
static THD_FUNCTION(can_status4, arg) {
	(void)arg;
	chRegSetThreadName("CAN_STATUS4");
	for(;;) {
		VESC_status4 status4;
		status4.tempMotor = ADC_Value[ADC_IND_TEMP_MOTOR];
		//commands_printf("%d\t%d", status4.tempMotor, ADC_Value[ADC_IND_TEMP_MOTOR]);
		status4.tempPCB = ADC_Value[ADC_IND_TEMP_PCB];
		status4.faultCode = mc_interface_get_fault();
		status4.state = mc_interface_get_state();
		status4.encoderIndex = encoder_index_found();
		comm_can_transmit(app_get_configuration()->controller_id |  (CAN_PACKET_STATUS4 << 8), (uint8_t*) &status4, sizeof(VESC_status4));
		chThdSleepMilliseconds(100); // Run this loop at 10Hz
	}
}

static THD_FUNCTION(custom_control, arg) { // this is where you put your own control loop
	(void)arg;
	chRegSetThreadName("CUSTOM_CONTROL");
	int error = 0;
	float output = 0;
	for(;;) {

		#ifdef LIMIT_SWITCH
		// green: rev
		// blue: for
		//if(mc_interface_get_for_lim()) commands_printf("tx pressed");
		//if(mc_interface_get_rev_lim()) {
		//	//commands_printf("rx pressed");
		//	mc_interface_get_tachometer_value(true);
		//}
		//error = custom_setpoint - mc_interface_get_tachometer_value(false);
		//if(abs(error) < m_conf.cus_ticks_close_enough) output = 0; //stop when close enough
		//else if (abs(error) < m_conf.cus_ticks_to_approach) { // slow down when close
		//	output = copysign(m_conf.cus_slow_output, error);;
		//} else { // full speed ahead
		//	output = copysign(1.0, error);
		//}
		//mc_interface_set_duty(output * m_conf.l_max_duty);
		
		#endif
		//commands_printf("%f", custom_setpoint);
		chThdSleepMilliseconds(100); // Run this loop at 10Hz
	}
}

#ifdef LIMIT_SWITCH
static THD_FUNCTION(limit_switcher, arg) {
	(void)arg;
	chRegSetThreadName("LIMIT_SWITCHER");
	int state = 0;
	for(;;) {
		switch(state) {
			case 0:
				if(mc_interface_get_for_lim() || mc_interface_get_rev_lim()) { // brake if one limit switch is pressed
					mc_interface_brake_now();
					state = 1;
				}
				break;

			case 1:
				if(!(mc_interface_get_for_lim() || mc_interface_get_rev_lim())) { // wait for limit switches to be released
					state = 0;
				}
				break;

			default:
				break;
		}
		chThdSleepMilliseconds(100);
	}
}
#endif

