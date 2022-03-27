/*
 * http_callbacks.c
 *
 *  Created on: Dec 17, 2021
 *      Author: gulziibayar
 */
#include "http_callbacks.h"
#include "stdio.h"
#include "mp3_player.h"
#include "main.h"
#include "multi_core_app.h"

struct _wsclient
{
	uint32_t handleId;
	struct _wsclient *next;
};

static struct _wsclient *wsClients = NULL;
struct board_state_variables g_BoardState;

/* Freertos mp3 command queue */
static QueueHandle_t *mp3_commandQ;
static QueueHandle_t *wifi_controlQ;
static queue_command_t voice_command = {.taskId=1};

static char ssid[WPL_WIFI_SSID_LENGTH];
static char password[WPL_WIFI_PASSWORD_LENGTH];
static uint8_t navigate_page = 0;

enum USER_LED
{
	RED = 0,
	GREEN = 1,
	BLUE = 2
};

typedef struct{
	GPIO_Type *base;
	uint32_t pin;
}led_struct;

led_struct led_map[3] =
{
	{BOARD_USER_LED_RED_GPIO, BOARD_USER_LED_RED_GPIO_PIN},
	{BOARD_USER_LED_GREEN_GPIO, BOARD_USER_LED_GREEN_GPIO_PIN},
	{BOARD_USER_LED_BLUE_GPIO, BOARD_USER_LED_BLUE_GPIO_PIN}
};

/* Prototypes */
static uint32_t SetBoardToClient();
static uint32_t SetBoardToAP();
static uint32_t CleanUpAP();
static uint32_t CleanUpClient();



/*!
 * @brief read GPIO_PIN connected led
 *
 * This function is used for reading gpio pin .
 *
 * @param _usr_led   	it is either R-0, G-1, B-2
 * @status           	returns gpio value.
 */
static uint32_t get_user_led(enum USER_LED _usr_led)
{
	return GPIO_PinRead(led_map[_usr_led].base, led_map[_usr_led].pin);
}

/*!
 * @brief write GPIO_PIN connected led
 *
 * This function is used for reading gpio pin .
 *
 * @param _usr_led   	it is either R-0, G-1, B-2
 * @param state   		1 or 0
 * @status void         returns nothing.
 */
static void set_user_led(enum USER_LED _usr_led, uint8_t state)
{
	GPIO_PinWrite(led_map[_usr_led].base, led_map[_usr_led].pin, state);
}

/*!
 * @brief compare string inside json on field "cmdType"
 *
 * This function is utility function to check json string.
 *
 * @param cJSON   		JSON object
 * @param char*   		string to compare
 * @status bool         true or false.
 */
static bool check_cmd_json(cJSON *json_obj, char *cmd)
{
	cJSON *jsonItem = NULL;
	if (json_obj != NULL)
	{
		jsonItem = cJSON_GetObjectItem(json_obj, "cmdType");
		if (jsonItem->type == cJSON_String && (jsonItem->valuestring != NULL))
		{
			if (strcmp(jsonItem->valuestring, cmd)==0)
			{
				return true;
			}
		}
	}
	return false;
}

/*!
 * @brief get integer value from JSON object with field
 *
 * @param cJSON   		JSON object
 * @param char*   		field string
 * @status int32_t      return integer value
 */
static int32_t get_intVal_json(cJSON *json_obj, char *field)
{
	cJSON *jsonItem = NULL;
	if (json_obj != NULL)
	{
		jsonItem = cJSON_GetObjectItem(json_obj, field);
		if (jsonItem->type == cJSON_Number)
		{
			return jsonItem->valueint;
		}
	}
	return -1;
}

#if HTTPSRV_CFG_WEBSOCKET_ENABLED

static void add_node(uint32_t handleId)
{
	struct _wsclient *currClient = wsClients;
	if (currClient==NULL)
	{
		currClient = pvPortMalloc(sizeof(struct _wsclient));
		currClient->handleId = handleId;
		currClient->next = NULL;
		wsClients = currClient;
	}
	else
	{
		while(currClient->next !=NULL)
		{
			currClient = currClient->next;
		}
		currClient->next = pvPortMalloc(sizeof(struct _wsclient));
		(currClient->next)->handleId = handleId;
		currClient->next->next = NULL;
	}
}

static int32_t delete_node(uint32_t handleId)
{
	struct _wsclient *currClient = wsClients;
	struct _wsclient *prevClient = NULL;
	if (currClient==NULL)
	{
		return -1;
	}
	else
	{
		while(currClient != NULL && currClient->handleId != handleId)
		{
			prevClient = currClient;
			currClient = currClient->next;
		}

		if (currClient == NULL)
		{
			return -1;
		}
		else if (prevClient==NULL) /* first element */
		{
			wsClients = currClient->next;
		}
		else if (currClient != NULL)
		{
			prevClient->next = currClient->next;
		}
		vPortFree(currClient);
		return 1;
	}
}
/*
 * Echo plugin code - simple plugin which echoes any message it receives back to
 * client.
 */
uint32_t ws_echo_connect(void *param, WS_USER_CONTEXT_STRUCT context)
{
	//add_node(context.handle);
#if DEBUG_WS
    PRINTF("WebSocket echo client connected.\r\n");
#endif
    return (0);
}

uint32_t ws_echo_disconnect(void *param, WS_USER_CONTEXT_STRUCT context)
{
	//delete_node(context.handle);
#if DEBUG_WS
    PRINTF("WebSocket echo client disconnected.\r\n");
#endif
    return (0);
}

uint32_t ws_echo_message(void *param, WS_USER_CONTEXT_STRUCT context)
{
    WS_send(&context); /* Send back what was received.*/
#if DEBUG_WS
    if (context.data.type == WS_DATA_TEXT)
    {
        /* Print received text message to console. */
        context.data.data_ptr[context.data.length] = 0;
        PRINTF("WebSocket message received:\r\n%s\r\n", context.data.data_ptr);
    }
    else
    {
        /* Inform user about binary message. */
        PRINTF("WebSocket binary data with length of %d bytes received.", context.data.length);
    }
#endif

    return (0);
}

uint32_t ws_echo_error(void *param, WS_USER_CONTEXT_STRUCT context)
{
#if DEBUG_WS
    PRINTF("WebSocket error: 0x%X.\r\n", context.error);
#endif
    return (0);
}

WS_PLUGIN_STRUCT ws_tbl[] = {{"/test", ws_echo_connect, ws_echo_message, ws_echo_error, ws_echo_disconnect, NULL},
                             {0, 0, 0, 0, 0, 0}};
#endif /* HTTPSRV_CFG_WEBSOCKET_ENABLED */

/*!
 * @brief LED Common Gateway Interface callback
 *
 * This controller processes HTTP post request for led control
 *
 * @param HTTPSRV_CGI_REQ_STRUCT   		http request parameter
 * @status int32_t      				returns count of http response string
 */
static int CGI_Led(HTTPSRV_CGI_REQ_STRUCT *param)
{
	HTTPSRV_CGI_RES_STRUCT response = {0};

	response.ses_handle  = param->ses_handle;
	response.status_code = HTTPSRV_CODE_OK;
	char str[7];
	int length = 0;
	char * json_serialized;
	if (param->request_method == HTTPSRV_REQ_GET)
	{
		cJSON *led = NULL;
		cJSON *cmd = NULL;
		cJSON *jsonObj = cJSON_CreateObject();
		for (int i=0; i<3; i++)
		{
			length += sprintf(str+length, "%d", get_user_led(i));
		}
		str[length] = 0;

		cmd = cJSON_CreateString("getLed");
		cJSON_AddItemToObject(jsonObj, "cmdType", cmd);

		led = cJSON_CreateString(str);
		cJSON_AddItemToObject(jsonObj, "led", led);

		json_serialized = cJSON_Print(jsonObj);
		length = strlen(json_serialized);

		response.content_type   = HTTPSRV_CONTENT_TYPE_PLAIN;
		response.data           = json_serialized;
		response.data_length    = length;
		response.content_length = response.data_length;
		HTTPSRV_cgi_write(&response);
		vPortFree(json_serialized);
		cJSON_Delete(jsonObj);
	}
	else if (param->request_method == HTTPSRV_REQ_POST)
	{
		uint32_t length = 0;
		uint32_t read;
		char buffer[100] = {0};
		int32_t led_num = -1;
		int32_t led_state = 0;
		length = param->content_length;
		read   = HTTPSRV_cgi_read(param->ses_handle, buffer, (length > sizeof(buffer)) ? sizeof(buffer) : length);

		if (read > 0)
		{
			cJSON *led_json = cJSON_Parse(buffer);

			if (check_cmd_json(led_json, "setLed"))
			{
				led_num = get_intVal_json(led_json, "ledNum");
				led_state = get_intVal_json(led_json, "ledState");
			}
			if (led_json != NULL)
			{
				cJSON_Delete(led_json);
			}
		}

		if (led_num >=0 && led_num < 3)
		{
			set_user_led((uint8_t)led_num, (uint8_t)led_state);
			length = snprintf(str, sizeof(str), "OK\n");
		}
		else
		{
			length = snprintf(str, sizeof(str), "FAIL\n");
		}

	    response.ses_handle   = param->ses_handle;
	    response.content_type = HTTPSRV_CONTENT_TYPE_PLAIN;
	    response.status_code  = HTTPSRV_CODE_OK;
	    /*
	    ** When the keep-alive is used we have to calculate a correct content length
	    ** so the receiver knows when to ACK the data and continue with a next request.
	    ** Please see RFC2616 section 4.4 for further details.
	    */

	    /* Calculate content length while saving it to buffer */

	    response.data           = str;
	    response.data_length    = length;
	    response.content_length = response.data_length;
	    /* Send response */
	    HTTPSRV_cgi_write(&response);
	}
	return (response.content_length);
}

/*!
 * @brief LED Common Gateway Interface callback
 *
 * These callbacks are called from the session tasks according to the Link struct above
 * The get.cgi request triggers a scan and responds with a list of the SSIDs
 *
 * @param HTTPSRV_CGI_REQ_STRUCT   		http request parameter
 * @status int32_t      				returns count of http response string
 */
static int CGI_HandleGet(HTTPSRV_CGI_REQ_STRUCT *param)
{
    HTTPSRV_CGI_RES_STRUCT response = {0};

    response.ses_handle  = param->ses_handle;
    response.status_code = HTTPSRV_CODE_OK;

    /* Buffer for hodling response JSON data */
    char buffer[CGI_BUFFER_LENGTH] = {0};
    char *ssids;

    if (g_BoardState.wifiState == WIFI_STATE_CLIENT || g_BoardState.wifiState == WIFI_STATE_AP)
    {
        /* Initiate Scan */
        int result = WPL_Scan();
        if (result != WPL_SUCCESS)
        {
            PRINTF("[WiFi!] Scan Error\r\n");
            ssids = "null"; // Interpreted as error by the website
        }
        else
        {
            /* Get JSON with scanned SSIDs */
            ssids = WPL_getSSIDs();
        }

        // Build the response JSON
        snprintf(buffer, sizeof(buffer), "{\"networks\":%s}", ssids);
    }
    else
    {
        // We can not start a scan if a previous scan is running or if we are connecting
        snprintf(buffer, sizeof(buffer), "{\"networks\":false}");
    }

    // Send the response back to browser
    response.content_type   = HTTPSRV_CONTENT_TYPE_PLAIN;
    response.data           = buffer;
    response.data_length    = strlen(buffer);
    response.content_length = response.data_length;
    HTTPSRV_cgi_write(&response);

    return (response.content_length);
}

/*!
 * @brief CGI callback for mp3 player
 *
 *fetches mp3 files in the root directory of USB disk
 * returns json frame containing list of files, current playing tracks
 * E.g
 * 		{
 *		  "isPlaying": 1,
 *		  "current_track" : 1,
 *		  "playlist": [
 *			"sample-3s.mp3",
 *			"sample-6s.mp3",
 *			"sample-9s.mp3",
 * 			"sample-15s.mp3",
 *			"sample-music.mp3"
 *		  ]
 *		}
 *
 * @param HTTPSRV_CGI_REQ_STRUCT   		http request parameter
 * @status int32_t      				returns count of http response string
 */
static int CGI_HandleGetMusic(HTTPSRV_CGI_REQ_STRUCT *param)
{
	HTTPSRV_CGI_RES_STRUCT response = {0};

	response.ses_handle  = param->ses_handle;
	response.status_code = HTTPSRV_CODE_OK;
    int length = 0;
    char * json_serialized;
    if (param->request_method == HTTPSRV_REQ_GET)
    {
    	cJSON *jsonObj = cJSON_CreateObject();
		char track_name[30];
		int status;
		cJSON *playlist = NULL;
		cJSON *track = NULL;
		cJSON *temp = NULL;

		playlist = cJSON_CreateArray();

		set_head_track();
		status = get_next_track(&track_name[0]);
		while(status)
		{
			track = cJSON_CreateString(&track_name[0]);
			cJSON_AddItemToArray(playlist, track);
			status = get_next_track(&track_name[0]);
		}
		cJSON_AddItemToObject(jsonObj, "tracks", playlist);

		temp = cJSON_CreateNumber(isPlaying());
		cJSON_AddItemToObject(jsonObj, "status", temp);

		temp = cJSON_CreateNumber(isUsbPlugged());
		cJSON_AddItemToObject(jsonObj, "usb", temp);

		temp = cJSON_CreateNumber(getCurrVolume());
		cJSON_AddItemToObject(jsonObj, "volume", temp);

		if (!isPlaying() || !get_playing_track(&track_name[0]))
		{
			track_name[0] = 0;
		}
		temp = cJSON_CreateString(&track_name[0]);
		cJSON_AddItemToObject(jsonObj, "currTrack", temp);
		json_serialized = cJSON_Print(jsonObj);

		length = strlen(json_serialized);

		response.content_type   = HTTPSRV_CONTENT_TYPE_PLAIN;
		response.data           = json_serialized;
		response.data_length    = length;
		response.content_length = response.data_length;
		HTTPSRV_cgi_write(&response);
		vPortFree(json_serialized);
		cJSON_Delete(jsonObj);
    }
    return (response.content_length);
}

/*!
 * @brief CGI callback for MP3 player control
 *
 * The POST request to change VOLUME, START, STOP, NEXT, PREVIOUS
 *
 * @param HTTPSRV_CGI_REQ_STRUCT   		http request parameter
 * @status int32_t      				returns count of http response string
 */
static int CGI_HandlePostMp3(HTTPSRV_CGI_REQ_STRUCT *param)
{
	HTTPSRV_CGI_RES_STRUCT response = {0};

	response.ses_handle  = param->ses_handle;
	response.status_code = HTTPSRV_CODE_OK;
	int32_t mp3_command = -1;
	char str[7];
	int length = 0;
	char * json_serialized;
	int32_t fail = 0;
    if (param->request_method == HTTPSRV_REQ_POST)
	{
		uint32_t length = 0;
		uint32_t read;
		char buffer[100] = {0};
		int32_t audio_volume = -1;
		length = param->content_length;
		read   = HTTPSRV_cgi_read(param->ses_handle, buffer, (length > sizeof(buffer)) ? sizeof(buffer) : length);

		if (read > 0)
		{
			cJSON *cmd_json = cJSON_Parse(buffer);

			mp3_command = get_intVal_json(cmd_json, "command");
			switch(mp3_command)
			{
				case CMD_START:
					voice_command.command_type = CMD_START;
					break;
				case CMD_STOP:
					voice_command.command_type = CMD_STOP;
					break;
				case CMD_NEXT:
					voice_command.command_type = CMD_NEXT;
					break;
				case CMD_PREVIOUS:
					voice_command.command_type = CMD_PREVIOUS;
					break;
				case CMD_PAUSE:
					voice_command.command_type = CMD_PAUSE;
					break;
				case CMD_VOL_VAL:
					voice_command.command_type = CMD_VOL_VAL;
					audio_volume = get_intVal_json(cmd_json, "volume");
					if (audio_volume >=0 && audio_volume <= 100)
					{
						voice_command.command_type = CMD_VOL_VAL;
						voice_command.buffer[0] = audio_volume & (0xFF);
					}
					else
					{
						fail = 1;
					}
					break;
				default:
					fail = 1;
					break;
			}
			if (cmd_json != NULL)
			{
				cJSON_Delete(cmd_json);
			}
		}

		if (!fail)
		{
			xQueueSend(*mp3_commandQ, (void *) &voice_command, 10);
			length = snprintf(str, sizeof(str), "OK\n");
		}
		else
		{
			length = snprintf(str, sizeof(str), "FAIL\n");
		}

		response.ses_handle   = param->ses_handle;
		response.content_type = HTTPSRV_CONTENT_TYPE_PLAIN;
		response.status_code  = HTTPSRV_CODE_OK;
		/*
		** When the keep-alive is used we have to calculate a correct content length
		** so the receiver knows when to ACK the data and continue with a next request.
		** Please see RFC2616 section 4.4 for further details.
		*/

		/* Calculate content length while saving it to buffer */

		response.data           = str;
		response.data_length    = length;
		response.content_length = response.data_length;
		/* Send response */
		HTTPSRV_cgi_write(&response);
	}
    return (response.content_length);
}

/*!
 * @brief CGI callback for AP connection
 *
 * The post.cgi request is used for triggering a connection to an external AP
 *
 * @param HTTPSRV_CGI_REQ_STRUCT   		http request parameter
 * @status int32_t      				returns count of http response string
 */
static int CGI_HandlePost(HTTPSRV_CGI_REQ_STRUCT *param)
{
    HTTPSRV_CGI_RES_STRUCT response = {0};

    response.ses_handle  = param->ses_handle;
    response.status_code = HTTPSRV_CODE_OK;

    char buffer[CGI_BUFFER_LENGTH] = {0};

    uint32_t length = 0;
    uint32_t read;
    char posted_passphrase[WPL_WIFI_PASSWORD_LENGTH + 1];
    char posted_ssid[WPL_WIFI_SSID_LENGTH + 1];

    /* We can not join another AP  if we are already connected to one */
    if (g_BoardState.wifiState == WIFI_STATE_CLIENT)
    {
        response.data           = "{\"status\":\"failed\"}";
        response.data_length    = strlen(response.data);
        response.content_length = response.data_length;
        HTTPSRV_cgi_write(&response);
        return 0;
    }

    length = param->content_length;
    read   = HTTPSRV_cgi_read(param->ses_handle, buffer, (length < sizeof(buffer)) ? length : sizeof(buffer));

    if (read > 0)
    {
        cgi_get_varval(buffer, "post_ssid", posted_ssid, sizeof(posted_ssid));
        cgi_get_varval(buffer, "post_passphrase", posted_passphrase, sizeof(posted_passphrase));
        cgi_urldecode(posted_ssid);
        cgi_urldecode(posted_passphrase);
    }

    /* Any post processing of the posted data (sanitation, validation) */
    format_post_data(posted_ssid);
    format_post_data(posted_passphrase);

    WC_DEBUG("[Http] Chosen ssid: %s\r\n[i] Chosen passphrase: \"%s\" \r\n", posted_ssid, posted_passphrase);

    response.content_type = HTTPSRV_CONTENT_TYPE_HTML;

    /* Initiate joining process */
    PRINTF("[Http] Joining: %s\r\n", posted_ssid);
    int32_t result = WPL_Join(posted_ssid, posted_passphrase);
    if (result != WPL_SUCCESS)
    {
        PRINTF("[Http] Cannot connect to wifi\r\n[!]ssid: %s\r\n[!]passphrase: %s\r\n", posted_ssid, posted_passphrase);
        /* Respond with a failure to the browser */
        response.data           = "{\"status\":\"failed\"}";
        response.data_length    = strlen(response.data);
        response.content_length = response.data_length;
        HTTPSRV_cgi_write(&response);
        response.data_length    = 0;
        response.content_length = response.data_length;
        HTTPSRV_cgi_write(&response);
    }
    else
    {
        /* We have successfully connected however the old AP is still running.
         * This session is still active and will try replying to the browser with a success message.
         * This message will also hold the new IP address under which the board will be reachable */
        PRINTF("[Http] Successfully joined: %s\r\n", posted_ssid);
        char ip[32];
        /* Get new client address to be sent back to the old browser session */
        WPL_GetIP(ip, 1);
        PRINTF("[Http] Now join that network on your device and connect to this IP: %s\r\n", ip);

        snprintf(buffer, sizeof(buffer), "{\"status\":\"success\",\"new_ip\":\"%s\"}", ip);

        response.data           = buffer;
        response.data_length    = strlen(response.data);
        response.content_length = response.data_length;
        HTTPSRV_cgi_write(&response);
        response.data_length    = 0;
        response.content_length = response.data_length;
        HTTPSRV_cgi_write(&response);

//        g_BoardState.wifiState = WIFI_STATE_CLIENT;
        g_BoardState.connected = true;
        /* Since the Joining was successful, we can save the credentials to the Flash */
        save_wifi_credentials(CONNECTION_INFO_FILENAME, posted_ssid, posted_passphrase);

        /* Resume the main task, this will make sure to clean up and shut down the AP*/
        /* Since g_BoardState.connected == true, the reconnection to AP will be skipped and
         * the main task will be put back to sleep waiting for a reset event */
//        vTaskResume(g_BoardState.wifiTask);
        uint8_t cmd_byte = 0;
        xQueueSend(*wifi_controlQ, (void *)&cmd_byte, 10);
    }
    return (response.content_length);
}

/*!
 * @brief CGI callback for AP reset
 *
 * The reset.cgi is used to clear the Flash memory and reset the board back to AP mode
 *
 * @param HTTPSRV_CGI_REQ_STRUCT   		http request parameter
 * @status int32_t      				returns count of http response string
 */
static int CGI_HandleReset(HTTPSRV_CGI_REQ_STRUCT *param)
{
    HTTPSRV_CGI_RES_STRUCT response;

    response.ses_handle   = param->ses_handle;
    response.status_code  = HTTPSRV_CODE_OK;
    response.content_type = HTTPSRV_CONTENT_TYPE_PLAIN;
    char str_buffer[128];

    /* Try to clear the flash memory */
    if (reset_saved_wifi_credentials(CONNECTION_INFO_FILENAME) != 0)
    {
        PRINTF("[Http] Error occured during resetting of saved credentials!\r\n");
        response.data        = "{\"status\":\"failed\"}";
        response.data_length = strlen(response.data);
    }
    else
    {
        /* The new ip will be the static ip configured for the local AP */
        snprintf(str_buffer, sizeof(str_buffer), "{\"status\":\"success\",\"new_ip\":\"%s\"}", WIFI_AP_IP_ADDR);

        response.data        = str_buffer;
        response.data_length = strlen(str_buffer);
    }

    response.content_length = response.data_length;
    HTTPSRV_cgi_write(&response);
    response.data_length    = 0;
    response.content_length = response.data_length;
    HTTPSRV_cgi_write(&response);

    // If we were client, disconnect from the external AP and start local AP
    if (g_BoardState.wifiState == WIFI_STATE_CLIENT)
    {
//        g_BoardState.wifiState = WIFI_STATE_AP;
        g_BoardState.connected = false;

//        vTaskResume(g_BoardState.wifiTask);
        uint8_t cmd_byte = 2;
        xQueueSend(*wifi_controlQ, (void *)&cmd_byte, 10);
    }
    return 0;
}

/*!
 * @brief CGI callback for WIFI status
 *
 * The status  status.cgi request returns status
 *
 * @param HTTPSRV_CGI_REQ_STRUCT   		http request parameter
 * @status int32_t      				returns count of http response string
 */
static int CGI_HandleStatus(HTTPSRV_CGI_REQ_STRUCT *param)
{
    HTTPSRV_CGI_RES_STRUCT response = {0};

    response.ses_handle  = param->ses_handle;
    response.status_code = HTTPSRV_CODE_OK;

    /* Buffer for hodling response JSON data */
    char buffer[CGI_BUFFER_LENGTH] = {0};
    char ip[16];
    char status_str[32] = {'\0'};

    // Get the Board IP address
    switch (g_BoardState.wifiState)
    {
        case WIFI_STATE_CONNECTING:
            strcpy(status_str, "connecting");
            WPL_GetIP(ip, 0);
            break;
        case WIFI_STATE_CLIENT_SCAN:
            strcpy(status_str, "scan_");
        case WIFI_STATE_CLIENT:
            strcat(status_str, "client");
            WPL_GetIP(ip, 1);
            break;
        case WIFI_STATE_AP_SCAN:
            strcpy(status_str, "scan_");
        case WIFI_STATE_AP:
        default:
            strcat(status_str, "ap");
            WPL_GetIP(ip, 0);
    }

    /* Build the response JSON */
    snprintf(buffer, sizeof(buffer), "{\"info\":{\"name\":\"%s\",\"ip\":\"%s\",\"ap\":\"%s\",\"status\":\"%s\"}}",
             BOARD_NAME, ip, g_BoardState.ssid, status_str);

    /* Send the response back to browser */
    response.content_type   = HTTPSRV_CONTENT_TYPE_PLAIN;
    response.data           = buffer;
    response.data_length    = strlen(buffer);
    response.content_length = response.data_length;
    HTTPSRV_cgi_write(&response);

    return (response.content_length);
}

/*CGI*/
/* Example Common Gateway Interface callback. */
/* These callbacks are called from the session tasks according to the Link struct above */
/* The get.cgi request triggers a scan and responds with a list of the SSIDs */
static int CGI_HandleGetSensor(HTTPSRV_CGI_REQ_STRUCT *param)
{
    HTTPSRV_CGI_RES_STRUCT response = {0};

    response.ses_handle  = param->ses_handle;
    response.status_code = HTTPSRV_CODE_OK;
    int length = 0;
    char * json_serialized;
    if (param->request_method == HTTPSRV_REQ_GET)
    {
    	cJSON *jsonObj = cJSON_CreateObject();
		int sensors[4] = {0,0,0};
		sensors[0] = fxos_pitch;
		sensors[1] = fxos_roll;
		sensors[2] = fxos_yaw;
		sensors[3] = light_distance;
		cJSON *sensor = NULL;
		cJSON *cmd = NULL;

		if (param->request_method == HTTPSRV_REQ_GET)
		{
			sensor = cJSON_CreateIntArray(sensors,4);
		}

		cmd = cJSON_CreateString("imu");
		cJSON_AddItemToObject(jsonObj, "cmdType", cmd);
		cJSON_AddItemToObject(jsonObj, "sensor", sensor);
		json_serialized = cJSON_Print(jsonObj);

		length = strlen(json_serialized);

		response.content_type   = HTTPSRV_CONTENT_TYPE_PLAIN;
		response.data           = json_serialized;
		response.data_length    = length;
		response.content_length = response.data_length;
		HTTPSRV_cgi_write(&response);
		vPortFree(json_serialized);
		cJSON_Delete(jsonObj);
    }
    return (response.content_length);
}

/*!
 * @brief CGI callback for Voice command status
 *
 * check whether voice command received to switch page
 *
 * @param HTTPSRV_CGI_REQ_STRUCT   		http request parameter
 * @status int32_t      				returns count of http response string
 */
static int CGI_HandleVoiceCommand(HTTPSRV_CGI_REQ_STRUCT *param)
{
	HTTPSRV_CGI_RES_STRUCT response = {0};

	response.ses_handle  = param->ses_handle;
	response.status_code = HTTPSRV_CODE_OK;
	int length = 0;
	char * json_serialized;
	cJSON *cmd = NULL;
	if (param->request_method == HTTPSRV_REQ_GET)
	{
		cJSON *jsonObj = cJSON_CreateObject();
		cmd = cJSON_CreateNumber(navigate_page);
		cJSON_AddItemToObject(jsonObj, "voiceCmd", cmd);
		json_serialized = cJSON_Print(jsonObj);
		/* clear after updated. */
		navigate_page = 0;
		length = strlen(json_serialized);

		response.content_type   = HTTPSRV_CONTENT_TYPE_PLAIN;
		response.data           = json_serialized;
		response.data_length    = length;
		response.content_length = response.data_length;
		HTTPSRV_cgi_write(&response);
		vPortFree(json_serialized);
		cJSON_Delete(jsonObj);
	}
	return (response.content_length);
}

void change_page()
{
	navigate_page = 1;
}

/* CGI link table */
const HTTPSRV_CGI_LINK_STRUCT cgi_lnk_tbl[] = {
	{"led", CGI_Led},				/* Led status, led control */
    {"reset", CGI_HandleReset},		/* Clear flash, start as AP mode */
	{"status", CGI_HandleStatus},	/* Get Wifi Status */
    {"get", CGI_HandleGet},			/* Get SSIDs from wifi scan */
	{"post", CGI_HandlePost},		/* Connect to given ssid, password */
	{"music", CGI_HandleGetMusic},	/* Mp3 player status: playlist, state */
	{"mp3", CGI_HandlePostMp3},		/* Mp3 player control: play, stop, pause */
	{"imu", CGI_HandleGetSensor},	/* Get IMU sensor, Light ranger sensor values */
	{"voice", CGI_HandleVoiceCommand},	/* Get IMU sensor, Light ranger sensor values */
    {0, 0} // DO NOT REMOVE - last item - end of table
};

static void wait_command()
{
	uint32_t result = 1;
	BaseType_t xResult;
	uint8_t wifiControl;
	xResult = xQueueReceive(*wifi_controlQ, &(wifiControl), portMAX_DELAY);
	if (xResult == pdTRUE)
	{
		switch(wifiControl) {
		case 0:
			g_BoardState.wifiState = WIFI_STATE_CLIENT;
			break;
		case 1:
			PRINTF("[WiFi] *** WIFI OFF ***\r\n");
			g_BoardState.wifiState = WIFI_STATE_OFF;
			break;
		case 2:
			g_BoardState.wifiState = WIFI_STATE_AP;
			break;
		case 3:
			PRINTF("[WiFi] *** WiFi ON ***\r\n");
			result = get_saved_wifi_credentials(CONNECTION_INFO_FILENAME, ssid, password);
			if (result == 0 && strcmp(ssid, "") != 0) {
				strcpy(g_BoardState.ssid, ssid);
				strcpy(g_BoardState.password, password);
				g_BoardState.wifiState = WIFI_STATE_CLIENT;
			} else
			{
				g_BoardState.wifiState = WIFI_STATE_AP;
			}
			break;
		default:
			break;
		}
	}
}


static void wifi_module_power_en(bool power_en){
	if (power_en)
	{
		GPIO_WritePinOutput(GPIO9, 13, 1);
	}else
	{
		GPIO_WritePinOutput(GPIO9, 13, 0);
	}
}

/*!
 * @brief FREERTOS task: "Wifi task"
 *
 * This task handles wifi connection and http server
 *
 * @param HTTPSRV_CGI_REQ_STRUCT   		http request parameter
 * @status int32_t      				returns count of http response string
 */
void wifi_task(void *arg)
{
    uint32_t result = 1;
    mp3_commandQ = ((wifi_task_param_t *)arg)->cmd_queue;
    wifi_controlQ = ((wifi_task_param_t *)arg)->cntrl_queue;

    /* When the App starts up, it will first read the mflash to check if any
     * credentials have been saved from previous runs.
     * If the mflash is empty, the board starts and AP allowing the user to configure
     * the desired Wi-Fi network.
     * Otherwise the stored credentials will be used to connect to the WiFi network.*/

    WC_DEBUG("Starting WiFi interface\r\n");
#if defined(SSID_AUTO) && (SSID_AUTO==1)
    init_flash_storage(CONNECTION_INFO_FILENAME);
    result = get_saved_wifi_credentials(CONNECTION_INFO_FILENAME, ssid, password);
#else
    /* temporary input the ssid, password */
    strcpy(ssid, SSID_DEF);
    strcpy(password, PASSWORD_DEF);
    result = 0;
#endif

    if (result == 0 && strcmp(ssid, "") != 0)
    {
        /* Credentials from last time have been found. The board will attempt to
         * connect to this network as a client */
        WC_DEBUG("[WiFi] Saved SSID: %s, Password: %s\r\n", ssid, password);
        g_BoardState.wifiState = WIFI_STATE_CLIENT;
        strcpy(g_BoardState.ssid, ssid);
        strcpy(g_BoardState.password, password);
    }
    else
    {
        /* No credentials are stored, the board will start its own AP */
        WC_DEBUG("[WiFi] No SSID, password stored on mflash\r\n");
        strcpy(g_BoardState.ssid, WIFI_SSID);
        strcpy(g_BoardState.password, WIFI_PASSWORD);
        g_BoardState.wifiState = WIFI_STATE_AP;
    }

    g_BoardState.connected = false;

    /* Initialize WiFi board */
    WC_DEBUG("[WiFi] Initializing WiFi module... \r\n");
    WPL_Init();
    if ((result = WPL_Start()) != WPL_SUCCESS)
    {
        PRINTF("[WiFi!] Could not initialize WiFi module %d\r\n", (uint32_t)result);
        __BKPT(0);
    }
    else
    {
        WC_DEBUG("[WiFi] Successfully initialized WiFi module\r\n");
    }

    /* Start WebServer */
    if (xTaskCreate(http_srv_task, "http_srv_task", HTTPD_STACKSIZE, NULL, HTTPD_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("[Http] HTTPD Task creation failed.");
        while (1)
            __BKPT(0);
    }

    /* Here other tasks can be created that will run the enduser app.... */

    /* Main Loop */
    while (1)
    {
        /* The SetBoardTo<state> function will configure the board Wifi to that given state.
         * After that, this task will suspend itself. It will remain suspended until it is time
         * to switch the state again. Uppon resuming, it will clean up the current state.
         * Every time the WiFi state changes, this loop will perform an iteration switching back
         * and fourth between the two states as required.
         */
        switch (g_BoardState.wifiState)
        {
            case WIFI_STATE_CLIENT:
                SetBoardToClient();
                /* Suspend here until its time to switch back to AP */
                if (g_BoardState.wifiState != WIFI_STATE_AP) {
                	wait_command();
                }
                CleanUpClient();
                break;
            case WIFI_STATE_OFF:
            	/* TURN OFF WIFI module, check the power consumption on the dongle */
            	vTaskDelay(5000 / portTICK_PERIOD_MS);
            	deepSleepOn(1);
            	WC_DEBUG("[WiFi] Deep sleep ON \r\n");
            	wait_command();
            	g_BoardState.connected = false;
            	deepSleepOn(0);
            	WC_DEBUG("[WiFi] Deep sleep OFF \r\n");
            	vTaskDelay(4000 / portTICK_PERIOD_MS);
            	WC_DEBUG("[WiFi] Initializing WiFi module... \r\n");
            	break;
            case WIFI_STATE_AP:
            default:
                SetBoardToAP();
                /* Suspend here until its time to stop the AP */
                wait_command();
                CleanUpAP();
        }
        vTaskDelay(5);
    }
}

/*!
 * @brief Initialize and start local AP
 *
 * @status int32_t		returns 0 /dummy return status/
 */
static uint32_t SetBoardToAP()
{
    uint32_t result;

    /* Set the global ssid and password to the default AP ssid and password */
    strcpy(g_BoardState.ssid, WIFI_SSID);
    strcpy(g_BoardState.password, WIFI_PASSWORD);

    /* Start the access point */
    PRINTF("[WiFi] Starting Access Point: SSID: %s, Chnl: %d\r\n", g_BoardState.ssid, WIFI_AP_CHANNEL);
    result = WPL_Start_AP(g_BoardState.ssid, g_BoardState.password, WIFI_AP_CHANNEL);

    if (result != WPL_SUCCESS)
    {
        PRINTF("[WiFi!] Failed to start access point\r\n");
        while (1)
            __BKPT(0);
    }
    g_BoardState.connected = true;
    /* Start DHCP server */
    WPL_StartDHCPServer(WIFI_AP_IP_ADDR, WIFI_AP_NET_MASK);

    char ip[16];
    WPL_GetIP(ip, 0);
    PRINTF("[WiFi] Now join the network on your device and connect to this IP: %s\r\n\r\n", ip);

    return 0;
}

/*!
 * @brief Clean up the local AP after waiting for all tasks to clean up
 *
 * @status int32_t		returns 0 /dummy return status/
 */
static uint32_t CleanUpAP()
{
    /* Give time for reply message to reach the web interface before destorying the conection */
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    WC_DEBUG("[WiFi] Stopping Wireless AP!\r\n");
    if (WPL_Stop_AP() != WPL_SUCCESS)
    {
        PRINTF("[WiFi!] Error while stopping ap\r\n");
        while (1)
            __BKPT(0);
    }

    WPL_StopDHCPServer();
    return 0;
}

/*!
 * @brief Connect to the external AP in g_BoardState.ssid
 *
 * @status int32_t		returns 0 /dummy return status/
 */
static uint32_t SetBoardToClient()
{
    int32_t result;
    // If we are already connected, skip the initialization
    if (!g_BoardState.connected)
    {
        PRINTF("[WiFi] Connecting as client to ssid: %s with password %s\r\n\t", g_BoardState.ssid, g_BoardState.password);

        result = WPL_Join(g_BoardState.ssid, g_BoardState.password);
        if (result != WPL_SUCCESS)
        {
            PRINTF("[WiFi!] Cannot connect to Wi-Fi\r\n[!]ssid: %s\r\n[!]passphrase: %s\r\n", g_BoardState.ssid,
                   g_BoardState.password);
            char c;
            do
            {
                PRINTF("[WiFi] To reset the board to AP mode, press 'r'.\r\n");
                PRINTF("[WiFi] In order to try connecting again press 'a'.\r\n");

                do
                {
                    c = GETCHAR();
                    // Skip over \n and \r and don't print the prompt again, just get next char
                } while (c == '\n' || c == '\r');

                switch (c)
                {
                    case 'r':
                    case 'R':
                        if (reset_saved_wifi_credentials(CONNECTION_INFO_FILENAME) != 0)
                        {
                            PRINTF("[WiFi] Error occured during resetting of saved credentials!\r\n");
                            while (1)
                                __BKPT(0);
                        }
                        else
                        {
                            // Reset back to AP mode
                            g_BoardState.wifiState = WIFI_STATE_AP;
                            return 0;
                        }
                        break;
                    case 'a':
                    case 'A':
                        // Try connecting again...
                        return 0;
                    default:
                        PRINTF("[WiFi] Unknown command %c, please try again.\r\n", c);
                }

            } while (1);
        }
        else
        {
            PRINTF("[WiFi] Connected to Wi-Fi\r\n\t\tssid: %s\r\n\t\t[!]passphrase: %s\r\n", g_BoardState.ssid,
                   g_BoardState.password);
            g_BoardState.connected = true;
            char ip[16];
            WPL_GetIP(ip, 1);
            PRINTF("[WiFi] Now join that network on your device and connect to this IP: %s\r\n", ip);
        }
    }
    return 0;
}

/*!
 * @brief Wait for any transmissions to finish and clean up the Client connection
 *
 * @status int32_t		returns 0 /dummy return status/
 */
static uint32_t CleanUpClient()
{
    /* Give time for reply message to reach the web interface before destroying the connection */
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* Leave the external AP */
    if (WPL_Leave() != WPL_SUCCESS)
    {
        PRINTF("[WiFi!] Error Leaving from Client network.\r\n");
        __BKPT(0);
    }
    return 0;
}


