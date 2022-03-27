/*
 * http_callbacks.h
 *
 *  Created on: Dec 17, 2021
 *      Author: gulziibayar
 */

#ifndef HTTP_CALLBACKS_H_
#define HTTP_CALLBACKS_H_

#include "board.h"
#include "cJSON.h"
#include "httpsrv.h"

/* wifi task */
#include "lwip/tcpip.h"
#include "wpl.h"
#include "http_server.h"
#include "webconfig.h"
#include "cred_flash_storage.h"

typedef enum board_wifi_states
{
    WIFI_STATE_CLIENT,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CLIENT_SCAN,
    WIFI_STATE_AP,
    WIFI_STATE_AP_SCAN,
	WIFI_STATE_OFF
} board_wifi_states;

struct board_state_variables
{
    board_wifi_states wifiState;
    char ssid[WPL_WIFI_SSID_LENGTH];
    char password[WPL_WIFI_PASSWORD_LENGTH];
    bool connected;
    TaskHandle_t wifiTask;
};
void change_page();
void wifi_task(void *arg);

#endif /* HTTP_CALLBACKS_H_ */
