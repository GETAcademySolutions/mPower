#include "app_error.h"
#include "ble_srv_common.h"
#include "bsp_btn_ble.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "sensorsim.h"
#include <stdint.h>
#include <string.h>

#include "ble_service.h"
#include "usb_port.h"

extern ble_mp_t m_ble_mp;

UsbPort *usbPorts[MP_MAX_USB_PORT_NUMBER + 1];

void initUsbPorts() {
  for (int i = MP_FIRST_USB_PORT_NUMBER; i <= MP_MAX_USB_PORT_NUMBER; i++) {
    usbPorts[i] = malloc(sizeof(UsbPort));
    memset(usbPorts[i], 0, sizeof(UsbPort));
    usbPorts[i]->connHandle = BLE_CONN_HANDLE_INVALID;
  }
}

uint8_t getPort(uint8_t port, UsbPort *data) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  uint32_t *p = usbPorts[port];
  return MP_SUCCESS;
}

uint8_t initPortStatus(uint8_t port, UsbPortStatus status, int16_t ticks, uint16_t handle) {
  NRF_LOG_INFO("setting port %x %x", port, status);
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }

  usbPorts[port]->status = status;
  usbPorts[port]->remainingChargeTicks = ticks;
  usbPorts[port]->connHandle = handle;
  return MP_SUCCESS;
}

uint8_t getPortStatus(uint8_t port, UsbPortStatus *status) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  *status = usbPorts[port]->status;
  return MP_SUCCESS;
}

uint8_t setPortStatus(uint8_t port, UsbPortStatus status) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  usbPorts[port]->status = status;
  return MP_SUCCESS;
}

uint8_t getConnHandle(uint8_t port, uint16_t *handle) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  *handle = usbPorts[port]->connHandle;
  return MP_SUCCESS;
}

uint8_t setConnHandle(uint8_t port, uint16_t handle) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  usbPorts[port]->connHandle = handle;
  return MP_SUCCESS;
}

uint8_t getRamainingChargeTicks(uint8_t port, int16_t *ticks) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  *ticks = usbPorts[port]->remainingChargeTicks;
  return MP_SUCCESS;
}

uint8_t setRamainingChargeTicks(uint8_t port, int16_t ticks) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  usbPorts[port]->remainingChargeTicks = ticks;
  return MP_SUCCESS;
}

uint8_t decrementChargingTicks(uint8_t port, int16_t *ticks) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  if (usbPorts[port]->status != MP_AVAILABLE) {
    --usbPorts[port]->remainingChargeTicks;
  }
  *ticks = usbPorts[port]->remainingChargeTicks;
  return MP_SUCCESS;
}

/*
  allocateFreePort
  return: <port number> if a port is available and sets the port status to FREE_CHARGE
          ERROR_NO_AVAILABLE_PORT if no ports are available
*/
uint8_t allocateFreePort(uint8_t *port) {

  for (int i = MP_FIRST_USB_PORT_NUMBER; i <= MP_MAX_USB_PORT_NUMBER; i++) {
    if (usbPorts[i]->status == MP_AVAILABLE) {
      usbPorts[i]->status = MP_FREE_CHARGE;
      *port = i;
      return MP_SUCCESS;
    }
  }
  return MP_ERROR_NO_AVAILABLE_PORT;
}

uint8_t turnOnOffPower(uint8_t port, uint8_t onOff) {
  if (port < MP_FIRST_USB_PORT_NUMBER || port > MP_MAX_USB_PORT_NUMBER) {
    return MP_ERROR_ILLEGAL_PORT_NUMBER;
  }
  // The _set function turns the LED off and _clear turns it ON
  //nrf_gpio_pin_write(port - 1 + LED_START, !onOff);
  if (onOff == MP_POWER_ON) {
    nrf_gpio_pin_clear(port - 1 + LED_START);
  } else {
    nrf_gpio_pin_set(port - 1 + LED_START);
  }

  return MP_SUCCESS;
}

uint8_t freeUsbPort(uint8_t port) {
  //turn off power of USB port and send BLE notification
  uint16_t connHandle;
  uint32_t alertMsg = 0;
  uint16_t charHandle = m_ble_mp.alert_char_handles.value_handle; // Should be alert charact4eristic
  uint8_t len = 4;
  uint8_t ret = MP_SUCCESS;

  NRF_LOG_INFO("timer: stop charging port=%d", port);
  ret = turnOnOffPower(port, MP_POWER_OFF);

  ret = getConnHandle(port, &connHandle);
  if (ret == MP_SUCCESS) {
    if (connHandle != BLE_CONN_HANDLE_INVALID && usbPorts[port]->status != MP_FREE_CHARGE_NOT_AVAILABLE) {
      // send USB port disconnect
      alertMsg = (port << 8) | MP_USB_PORT_CHARGING_STOPPED;
      sendNotification(charHandle, connHandle, &alertMsg, len);
    }
  } else {
    NRF_LOG_INFO("freeUsbPort: getConnHandle failed withe error code %d", ret);
  }
  NRF_LOG_INFO("port %x = %x", port, usbPorts[port]->status);
  if (usbPorts[port]->status == MP_FREE_CHARGE) {
    initPortStatus(port, MP_FREE_CHARGE_NOT_AVAILABLE, MP_FREE_CHARGE_NOT_AVAILABLE_TIME, BLE_CONN_HANDLE_INVALID);
  } else {
    initPortStatus(port, MP_AVAILABLE, 0, BLE_CONN_HANDLE_INVALID);
  }
}

void checkUsbPorts() {
  // Check if any USB ports has used up free chanrging time / charging time
  uint8_t ret;
  int16_t ticks;

  for (uint8_t port = MP_FIRST_USB_PORT_NUMBER; port <= MP_MAX_USB_PORT_NUMBER; port++) {
    ret = decrementChargingTicks(port, &ticks);
    if (ret == MP_SUCCESS && ticks < 0) {
      freeUsbPort(port);
    }
  }
}

// Max port number set to 4 in test
// Moved defines to our_service.h

void onNewCommand(ble_evt_t const *p_ble_evt) {
  // Edgar: Write event - decode the data set by client:
  // 1st byte: command - 0=off, 1=on
  // 2nd byte: port number
  //           legal port number is: 1 - MAX_USB_PORT_NUMBER

  //ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

  uint16_t connHandle = p_ble_evt->evt.gap_evt.conn_handle;
  uint16_t charHandle = m_ble_mp.alert_char_handles.value_handle;
  uint8_t *pData = (uint8_t *)p_ble_evt->evt.gatts_evt.params.write.data;
  uint16_t length = p_ble_evt->evt.gatts_evt.params.write.len;
  uint8_t command = 0xff;
  uint8_t port = MP_CHOOSE_AVAILABLE_PORT;
  uint8_t ackMsg[20];
  uint8_t byte_counter = 0;
  uint8_t len = 4;
  uint8_t ret = MP_SUCCESS;
  UsbPortStatus portStatus;

  NRF_LOG_INFO("onNewCommand() enter");

  ackMsg[byte_counter++] = m_ble_mp.location_id;

  if (length >= 2) {
    command = pData[0];
    port = pData[1];

    NRF_LOG_INFO("data received: length=%0x, command=%d, port=%d", length, command, port);
  } else if (length == 1) {
    command = pData[0];
    port = MP_CHOOSE_AVAILABLE_PORT;

    NRF_LOG_INFO("data received: length=%0x, command=%0x, port=allocate", length, command);
  } else {
    NRF_LOG_INFO("data received: length=%0x, illegal command=%0x", length, command);
    ackMsg[byte_counter++] = MP_ILLEGAL_COMMAND;
    ackMsg[byte_counter++] = MP_ERROR;
    sendNotification(charHandle, connHandle, &ackMsg, len);
    return;
  }

  if (port == MP_CHOOSE_AVAILABLE_PORT) {
    ret = allocateFreePort(&port);
  }

  if (ret != MP_SUCCESS) {
    NRF_LOG_INFO("No available port")
    ackMsg[byte_counter++] = MP_ERROR_NO_AVAILABLE_PORT;
    ackMsg[byte_counter++] = MP_ERROR;
    sendNotification(charHandle, connHandle, &ackMsg, len);
    return;
  }

  ret = getPortStatus(port, &portStatus);

  if (ret != MP_SUCCESS) {
    NRF_LOG_INFO("Get port status failed, %s", ret)
  } else {
    if (command == MP_TURN_USB_POWER_ON) {
      NRF_LOG_INFO("Turn power ON on port %x", port);

      if (portStatus != MP_AVAILABLE && portStatus != MP_FREE_CHARGE) {
        NRF_LOG_INFO("Illegal port status, portstatus=%d", portStatus)
        ret = MP_ERROR_ILLEGAL_PORT_STATUS;
      } else {
        ret = turnOnOffPower(port, MP_POWER_ON);

        // update port status
        // TODO CHANGE TIME TO ACTUAL CHARGE TIME
        ret = initPortStatus(port, MP_ACTIVE_CHARGE, MP_MAX_CHARGE_TIME, connHandle);
      }

    } else if (command == MP_TURN_USB_POWER_OFF) {
      NRF_LOG_INFO("Turn power OFF on port %x", port);

      /*
            if (portStatus != MP_ACTIVE_CHARGE) {
              NRF_LOG_INFO("Illegal port status, portstatus=%d", portStatus)
              ret = MP_ERROR_ILLEGAL_PORT_STATUS;
            } else {
              ret = turnOnOffPower(port, MP_POWER_OFF);

              // update port status
              // TODO CHANGE TIME TO ACTUAL CHARGE TIME
              ret = initPortStatus(port, MP_AVAILABLE, MP_TEST_TIME, connHandle);
            }
            */

      ret = turnOnOffPower(port, MP_POWER_OFF);

      // update port status
      // TODO CHANGE TIME TO ACTUAL CHARGE TIME
      ret = initPortStatus(port, MP_AVAILABLE, 0, connHandle);

    } else {
      NRF_LOG_INFO("Illegal command %d", port);
      ret = MP_ILLEGAL_COMMAND;
    }
  }

  // Send ack/nack message
  if (ret == MP_SUCCESS) {
    // send ack
    ackMsg[byte_counter++] = port,
    ackMsg[byte_counter++] = MP_SUCCESS;
  } else {
    // send nack
    ackMsg[byte_counter++] = ret;
    ackMsg[byte_counter++] = MP_ERROR;
  }
  sendNotification(charHandle, connHandle, &ackMsg, len);

  // TODO testStuff status
  ret = getPortStatus(port, &portStatus);
  NRF_LOG_INFO("Port status is %x", portStatus);

  NRF_LOG_INFO("onNewCommand() leave");
}

void onUsbChange(uint8_t port) {
  //TODO add USB status, ack message, ports, turn off power, turn on power, change USB status, get USB status
  uint16_t connHandle = NULL;
  uint16_t charHandle = m_ble_mp.alert_char_handles.value_handle;
  uint8_t ackMsg[20];
  uint8_t byte_counter = 0;
  uint8_t len = 4;
  uint8_t ret = MP_SUCCESS;
  UsbPortStatus portStatus;

  ackMsg[byte_counter++] = m_ble_mp.location_id;

  NRF_LOG_INFO("onUsbChange() enter");
  ret = getConnHandle(port, &connHandle);

  ret = getPortStatus(port, &portStatus);

  if (ret != MP_SUCCESS) {
    NRF_LOG_INFO("Get port status failed, %s", ret)
  } else {
    if (portStatus == MP_AVAILABLE) {
      NRF_LOG_INFO("Turn power ON on port %x", port);
      ret = turnOnOffPower(port, MP_POWER_ON);

      // update port status
      // TODO CHANGE TIME TO ACTUAL CHARGE TIME
      ret = initPortStatus(port, MP_FREE_CHARGE, MP_MAX_FREE_TIME, connHandle);
      NRF_LOG_INFO("Set portStatus to MP_FREE_CHARGE");

    } else if (portStatus == MP_ACTIVE_CHARGE) {
      NRF_LOG_INFO("Turn power OFF on port %x", port);

      ret = turnOnOffPower(port, MP_POWER_OFF);

      // update port status
      // TODO CHANGE TIME TO ACTUAL CHARGE TIME
      ret = initPortStatus(port, MP_AVAILABLE, 0, connHandle);
      NRF_LOG_INFO("Set portStatus to MP_AVAILABLE");

    } else if (portStatus == MP_FREE_CHARGE) {
      NRF_LOG_INFO("Turn power OFF on port %x", port);

      ret = turnOnOffPower(port, MP_POWER_OFF);

      // update port status
      // TODO CHANGE TIME TO ACTUAL NOT AVAILABLE TIME
      ret = initPortStatus(port, MP_FREE_CHARGE_NOT_AVAILABLE, MP_FREE_CHARGE_NOT_AVAILABLE_TIME, connHandle);
      NRF_LOG_INFO("Set portStatus to MP_FREE_CHARGE_NOT_AVAILABLE");
    }
  }
  // Send ack/nack message
  if (ret == MP_SUCCESS) {
    // send ack
    ackMsg[byte_counter++] = port,
    ackMsg[byte_counter++] = MP_SUCCESS;
  } else {
    // send nack
    ackMsg[byte_counter++] = ret;
    ackMsg[byte_counter++] = MP_ERROR;
  }
  if (connHandle != NULL)
    sendNotification(charHandle, connHandle, &ackMsg, len);

  // TODO testStuff status
  ret = getPortStatus(port, &portStatus);
  NRF_LOG_INFO("Port status is %x", portStatus);

  NRF_LOG_INFO("onUsbChange() leave");
}

void onBleDisconnect(ble_evt_t const *p_ble_evt) {
  // Remvove connection handle from UsbPort
  for (int i = MP_FIRST_USB_PORT_NUMBER; i <= MP_MAX_USB_PORT_NUMBER; i++) {
    if (usbPorts[i]->connHandle == p_ble_evt->evt.gap_evt.conn_handle) {
      usbPorts[i]->connHandle = BLE_CONN_HANDLE_INVALID;
    }
  }
}