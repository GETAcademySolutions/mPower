
#ifndef OUR_SERVICE_H__
#define OUR_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

// FROM_SERVICE_TUTORIAL: Defining 16-bit service and 128-bit base UUIDs
#define BLE_UUID_OUR_BASE_UUID              {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} // 128-bit base UUID
#define BLE_UUID_OUR_SERVICE_UUID                0xF00D // Just a random, but recognizable value

// ALREADY_DONE_FOR_YOU: Defining 16-bit characteristic UUID
#define BLE_UUID_OUR_CHARACTERISTC_UUID          0xBEEF // Just a random, but recognizable value

// This structure contains various status information for our service. 
// The name is based on the naming convention used in Nordics SDKs. 
// 'ble’ indicates that it is a Bluetooth Low Energy relevant structure and 
// ‘os’ is short for Our Service). 
// NB! Multiple Connections - THE CONNECTION HANDLE, CONN_HANDLE, IS THERE TO KEEP TRACK OF THE CURRENT CONNECTION
typedef struct
{
    uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    // OUR_JOB: Step 2.D, Add handles for the characteristic attributes to our struct
    ble_gatts_char_handles_t    char_handles;
}ble_os_t;

typedef enum {
    AVAILABLE = 0,
    FREE_CHARGE,              // Port busy but not yet paid for
    ACTIVE_CHARGE,            // Port active; port busy and paid for
    FREE_CHARGE_NOT_AVAILABLE // port busy; free charging time expired
} UsbPortStatus;

typedef struct {
    UsbPortStatus status;   
    int16_t  remainingChargeTicks;
    int16_t  notAvailableTicks;
} UsbPort;


//#define MAX_PORT 10
#define FIRST_PORT_NUMBER       1
#define MAX_USB_PORT_NUMBER     4
#define TURN_USB_POWER_OFF      0
#define TURN_USB_POWER_ON       1
#define CHOOSE_AVAILABLE_PORT   255 // = 0xFF
//TODO Test data
#define TEST_TIME               1000 * 20

#define MAX_CHARGE_TIME         60*60*1000 //1 time
#define MAX_FREE_TIME           5*60*1000  //5 min

#define ERROR_ILLEGAL_PORT      240 // = 0xF0
#define ERROR_NO_AVAILABLE_PORT 241 // = 0xF1


/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_our_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
void our_service_init(ble_os_t * p_our_service);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_our_service                     Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */

void our_temperature_characteristic_update(ble_os_t *p_our_service, int32_t *temperature_value);

void our_notification(ble_os_t *p_our_service, uint16_t conn_handle, uint32_t *p_data);

void initPortStatus(uint8_t port, UsbPortStatus status, uint16_t ticks);
UsbPortStatus getPortStatus(uint8_t port);
void setPortStatus(uint8_t port, UsbPortStatus status);
bool isPortFree(uint8_t port);
bool isPortTemporarilyInUSe(uint8_t port);
bool isPortActive(uint8_t port);
uint16_t getRamainingChargeTicks(uint8_t port);
void setRamainingChargeTicks(uint8_t port, uint16_t ticks);
bool decrementChargingTicks(uint8_t port);
bool isChargingTicksExpired(uint8_t port);
uint8_t allocateFreePort();

#endif  /* _ OUR_SERVICE_H__ */
