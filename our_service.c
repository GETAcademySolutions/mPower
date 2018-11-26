
#include <stdint.h>
#include <string.h>
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "our_service.h"
#include "ble_srv_common.h"
#include "app_error.h"


UsbPort* usbPorts[MAX_USB_PORT_NUMBER + 1];


void initPortStatus(uint8_t port, UsbPortStatus status, uint16_t ticks) {
  NRF_LOG_INFO("setting port %x %x", port, status);
  if (port < FIRST_PORT_NUMBER || port > MAX_USB_PORT_NUMBER) {
    return ERROR_ILLEGAL_PORT;
  }

  usbPorts[port]->status = status;
  usbPorts[port]->remainingChargeTicks = ticks;
}

UsbPortStatus getPortStatus(uint8_t port) {
  if (port < FIRST_PORT_NUMBER || port > MAX_USB_PORT_NUMBER) {
    return ERROR_ILLEGAL_PORT;
  }
  return usbPorts[port]->status;
}

void setPortStatus(uint8_t port, UsbPortStatus status) {
  if (port < FIRST_PORT_NUMBER || port > MAX_USB_PORT_NUMBER) {
    return ERROR_ILLEGAL_PORT;
  }
  usbPorts[port]->status = status;
}

bool isPortFree(uint8_t port) {
  if (port < FIRST_PORT_NUMBER || port > MAX_USB_PORT_NUMBER) {
    return ERROR_ILLEGAL_PORT;
  }
  return usbPorts[port]->status == AVAILABLE;
}

bool isPortTemporarilyInUSe(uint8_t port) {
  if (port < FIRST_PORT_NUMBER || port > MAX_USB_PORT_NUMBER) {
    return ERROR_ILLEGAL_PORT;
  }
  return usbPorts[port]->status == FREE_CHARGE;
}

bool isPortActive(uint8_t port) {
  if (port < FIRST_PORT_NUMBER || port > MAX_USB_PORT_NUMBER) {
    return ERROR_ILLEGAL_PORT;
  }
  return usbPorts[port]->status == ACTIVE_CHARGE;
}

uint16_t getRamainingChargeTicks(uint8_t port) {
  if (port < FIRST_PORT_NUMBER || port > MAX_USB_PORT_NUMBER) {
    return ERROR_ILLEGAL_PORT;
  }
  return usbPorts[port]->remainingChargeTicks;
} 

void setRamainingChargeTicks(uint8_t port, uint16_t ticks) {
  if (port < FIRST_PORT_NUMBER || port > MAX_USB_PORT_NUMBER) {
    return ERROR_ILLEGAL_PORT;
  }
  usbPorts[port]->remainingChargeTicks = ticks;
} 

bool decrementChargingTicks(uint8_t port) {
  if (port < FIRST_PORT_NUMBER || port > MAX_USB_PORT_NUMBER) {
    return ERROR_ILLEGAL_PORT;
  }
  return (--usbPorts[port]->remainingChargeTicks <= 0);
}

bool isChargingTicksExpired(uint8_t port) {
  if (port < FIRST_PORT_NUMBER || port > MAX_USB_PORT_NUMBER) {
    return ERROR_ILLEGAL_PORT;
  }
  return (usbPorts[port]->remainingChargeTicks <= 0);
}

/*
  allocateFreePort
  return: <port number> if a port is available and sets the port status to ACTIVE_CHARGE
          ERROR_NO_AVAILABLE_PORT if no ports are available
*/
uint8_t allocateFreePort() {

  for (int port = FIRST_PORT_NUMBER; port <= MAX_USB_PORT_NUMBER; port++) {
    if (usbPorts[port]->status == AVAILABLE) {
      return port;
    }
  }
  return ERROR_NO_AVAILABLE_PORT;
}


//  Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic
void ble_our_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_os_t * p_our_service =(ble_os_t *) p_context;  

    NRF_LOG_INFO("ble_our_service_on_ble_evt: event=%d", p_ble_evt->header.evt_id);

    // OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            // No implementation needed.
            break;
    }	
}

static void init_port_status() {
    for (int i = 0; i <= MAX_USB_PORT_NUMBER; i++) {
      usbPorts[i] = malloc(sizeof(UsbPort));
      memset(usbPorts[i], 0, sizeof(usbPorts));
    }
}

/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
static uint32_t our_char_add(ble_os_t * p_our_service)
{
    /*
    sd_ble_gatts_characteristic_add() takes four parameters:
    @param[in]  uint16_t                        service_handle.

    // The Attribute Medtadata
    // This is a structure holding permissions and authorization levels required by 
    // characteristic value attributes. 
    // It also holds information on whether or not the characteristic value is of variable length
    // and where in memory it is stored.
    ble_gatts_attr_md_t       attr_md

    // The Characteristic Metadata:
    // This is a structure holding the value properties of the characteristic value. 
    // It also holds metadata of the CCCD and possibly other kinds of descriptors.
    @param[in]  ble_gatts_char_md_t const *     p_char_md

    // The Characteristic Value Attribute:
    // This structure holds the actual value of the characteristic (like the temperature value).
    // It also holds the maximum length of the value (it might e.g. be four bytes long) and it's UUID.
    @param[in]  ble_gatts_attr_t const *        p_attr_char_value
    
    @param[out] ble_gatts_char_handles_t *      p_handles
    */

    // OUR_JOB: Step 2.A, Add a custom characteristic UUID
    uint32_t            err_code;
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_OUR_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_OUR_CHARACTERISTC_UUID;
    printf("char_uuid=%0x, uuid=%x, type=%x\n", char_uuid, char_uuid.uuid, char_uuid.type);
    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    printf("char_uuid=%0x, uuid=%x, type=%x\n", char_uuid, char_uuid.uuid, char_uuid.type);
    APP_ERROR_CHECK(err_code);  

    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = true;
    char_md.char_props.write = true;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;
	
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS()
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN()
    //BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM()
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;


    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    uint8_t value[5]            = {0x12,0x34,0x56,0x78};
    attr_char_value.max_len     = 5;
    //attr_char_value.init_len    = 4;
    attr_char_value.init_len    = sizeof(value);
    attr_char_value.p_value     = value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_our_service->char_handles);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}


/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
void our_service_init(ble_os_t * p_our_service)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

    // FROM_SERVICE_TUTORIAL: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    
    
    // OUR_JOB: Step 3.B, Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
	

    // FROM_SERVICE_TUTORIAL: Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                            &service_uuid,
                            &p_our_service->service_handle);
    
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Call the function our_char_add() to add our new characteristic to the service. 
    our_char_add(p_our_service);
    
    // Initialize portStatus array
    init_port_status();
}

// ALREADY_DONE_FOR_YOU: Function to be called when updating characteristic value
void our_temperature_characteristic_update(ble_os_t *p_our_service, int32_t *temperature_value)
{
    // OUR_JOB: Step 3.E, Update characteristic value
    // Here we must loop through the connecion collecion/vector - using another service 

    //if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID)
    if (false)
    {
        uint16_t               len = 4;
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_our_service->char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*)temperature_value;  

        sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
    }

}


void our_notification(ble_os_t *p_our_service, uint32_t *p_data)
{
   if (true)
    {
        uint16_t               len = 4;
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_our_service->char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = p_data;

        NRF_LOG_INFO("our_notification: conn_handle=%0x, len=%d, value=%0x", p_our_service->conn_handle, len, *p_data);

        sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
    }
}