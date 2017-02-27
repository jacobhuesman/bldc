#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout

#include "comm_can.h"
#include "app.h"
 
// Example thread
static THD_FUNCTION(example_thread, arg);
static THD_WORKING_AREA(example_thread_wa, 2048); // 2kb stack for this thread
 
void app_example_init(void) {
	// Set the UART TX pin as an input with pulldown
	//palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLDOWN);
 
	// Start the example thread
	chThdCreateStatic(example_thread_wa, sizeof(example_thread_wa),
		NORMALPRIO, example_thread, NULL);
}
 
typedef struct VESC_status1{
	int rpm:32;
	int position:16;
	int motorCurrent:16;
} VESC_status1;

static THD_FUNCTION(example_thread, arg) {
	(void)arg;
 
	chRegSetThreadName("APP_EXAMPLE");
 
	for(;;) {
		VESC_status1 status1;
		status1.rpm = (int) mc_interface_get_rpm();
		status1.position = (int) (mc_interface_get_pid_pos_now()*1000);
		status1.motorCurrent = (int) (mc_interface_get_tot_current()*10);
		comm_can_transmit(app_get_configuration()->controller_id |  (CAN_PACKET_STATUS1 << 8), (uint8_t*) &status1, sizeof(VESC_status1));
		// Run this loop at 100Hz
		chThdSleepMilliseconds(10);
 
		// Reset the timeout
		//timeout_reset();
	}
}
