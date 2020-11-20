#include "r_flash_rx_if.h"

flash_states_t g_flash_state = FLASH_UNINITIALIZED;
static int32_t g_flash_lock;

/* Internal functions. */
void (*flash_ready_isr_handler)(void *); /* Function pointer for Flash Ready ISR */
void (*flash_error_isr_handler)(void *); /* Function pointer for Flash Error ISR */

/*Structure that holds the parameters for current operations*/
current_param_t g_current_parameters = { 0, /* Source Address */
0, /* Destination Address */
0, /* Total Count */
0, /* Current Count */
FLASH_CUR_INVALID, /* Current Operation */
0, /* Minimum Program Size */
0, /* Wait Count for current operation */
false, /* DF BGO Disabled */
false, /* CF BGO Disabled */
};

flash_err_t R_FLASH_Open(void) {
	flash_err_t err;
	R_FlashCodeCopy();
	err = flash_api_open();
	if (err == FLASH_SUCCESS) {
		g_flash_state = FLASH_READY;
	}

	return (err);
}

void R_FlashCodeCopy(void) {
}

bool flash_softwareLock(int32_t * const plock) {
	bool ret = false;
	int32_t is_locked = true;
	xchg(&is_locked, plock);
	if (is_locked == false) {
		ret = true;
	} else {
	}

	return ret;
}

bool flash_softwareUnlock(int32_t * const plock) {
	*plock = false;

	return true;
}

flash_err_t R_FLASH_Write(uint32_t src_address, uint32_t dest_address,
		uint32_t num_bytes) {
	return (flash_api_write(src_address, dest_address, num_bytes));
}

flash_err_t R_FLASH_Erase(flash_block_address_t block_start_address,
		uint32_t num_blocks) {
	return (flash_api_erase(block_start_address, num_blocks)); /* Call FCU supported handler */
}

flash_err_t R_FLASH_BlankCheck(uint32_t address, uint32_t num_bytes,
		flash_res_t *result) {
	return (flash_api_blankcheck(address, num_bytes, result));
}

flash_err_t R_FLASH_Control(flash_cmd_t cmd, void *pcfg) {
	return (flash_api_control(cmd, pcfg));
}

#pragma inline(R_FLASH_GetVersion)
uint32_t R_FLASH_GetVersion(void) {
	return ((((uint32_t) FLASH_RX_VERSION_MAJOR) << 16)
			| (uint32_t) FLASH_RX_VERSION_MINOR);
}

flash_err_t flash_lock_state(flash_states_t new_state) {
	if (flash_softwareLock(&g_flash_lock) == true) {
		g_flash_state = new_state;
		return FLASH_SUCCESS;
	} else {
		return FLASH_ERR_BUSY;
	}
}

void flash_release_state(void) {
	g_flash_state = FLASH_READY;
	flash_softwareUnlock(&g_flash_lock);
}

flash_err_t flash_get_status(void) {
	if (g_flash_state == FLASH_READY) {
		return FLASH_SUCCESS;
	} else {
		return FLASH_ERR_BUSY;
	}
}

#pragma section
