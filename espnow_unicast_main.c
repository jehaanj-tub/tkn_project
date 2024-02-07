/*
 * espnow_unicast_main.c
 * Networked Embedded Systems
 * Telecommunication Networks Group
 * TU Berlin 2023
 * Based on example codes of Espressif
 */

#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "espnow_example.h"
#include "esp_task_wdt.h"

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_receiver";

static QueueHandle_t s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

//uint8_t mac_address_list[];

bool sender = true;

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    /* Fill all remaining bytes after the data with random values */
    //esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint16_t *seq, uint8_t **payload)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;
    uint8_t payload_len = data_len - sizeof(example_espnow_data_t);

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *seq = buf->seq_num;
    *payload = malloc(payload_len);
    if (*payload == NULL) {
        ESP_LOGE(TAG, "Malloc payload buffer");
        return -1;
    }
    memcpy(*payload, buf->payload, payload_len);
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

void add_peer(uint8_t* mac_addr, bool encrypt)
{
    /* If MAC address does not exist in peer list, add it to peer list. */
    if (esp_now_is_peer_exist(mac_addr) == false) {
        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
        if (peer == NULL) {
            ESP_LOGE(TAG, "Malloc peer information fail");
            esp_now_deinit();
            vTaskDelete(NULL);
        }
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = CONFIG_ESPNOW_CHANNEL;
        peer->ifidx = ESPNOW_WIFI_IF;
        peer->encrypt = encrypt;
        if(encrypt){
            memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
        }
        memcpy(peer->peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
        ESP_LOGI(TAG, "See peer for the first time and add it to my list.");
        free(peer);
    } else {
        ESP_LOGI(TAG, "Already known peer.");
    }
}

static void send_broadcast(void *pvParameter)
{
    
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    example_espnow_data_prepare(send_param);
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }
    vTaskDelete(NULL);
}


static void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;
    uint16_t recv_seq = 0;
    int ret;
    bool is_broadcast = false;
    
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    //example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    /*if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }*/

    static uint8_t my_peer[ESP_NOW_ETH_ALEN] = { 0x34, 0x85, 0x18, 0xb9, 0x1b, 0x9c };

    add_peer(my_peer, true);

    if(sender == true) {
        ESP_LOGI(TAG, "Send data to "MACSTR", data: RTS", MAC2STR(my_peer));

        example_espnow_send_param_t *send_param = malloc(sizeof(example_espnow_send_param_t));
        if (send_param == NULL) {
            ESP_LOGE(TAG, "Malloc send parameter fail");
            vSemaphoreDelete(s_example_espnow_queue);
            esp_now_deinit();
            return;
        }
        memset(send_param, 0, sizeof(example_espnow_send_param_t));
        send_param->len = sizeof(example_espnow_data_t) + 4;
        send_param->buffer = malloc(send_param->len);
        if (send_param->buffer == NULL) {
            ESP_LOGE(TAG, "Malloc send buffer fail");
            free(send_param);
            vSemaphoreDelete(s_example_espnow_queue);
            esp_now_deinit();
            return;
        }
        example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
        memcpy(buf->payload, "RTS", 4);
        memcpy(send_param->dest_mac, my_peer, ESP_NOW_ETH_ALEN);
        example_espnow_data_prepare(send_param);

        ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_param->dest_mac));
        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
            ESP_LOGE(TAG, "Send error");
            example_espnow_deinit(send_param);
            vTaskDelete(NULL);
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
                
                if (esp_now_is_peer_exist(send_cb->mac_addr) == false) {
                    is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);
                    
                    if(is_broadcast){
                        xTaskCreate(send_broadcast, "send_broadcast", 4096, send_param, 4, NULL);
                    }
                }
                
                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                uint8_t* payload;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_seq, &payload);
                free(recv_cb->data);
                if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d, data: %s", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len, payload);
                    
                    /*********************************************/
                    /*************** STEP 1 **********************/
                    /* Remove the comment in the next line to    */
                    /* add the broadcast node to your peer list  */
                    /*********************************************/
                    
                    add_peer(recv_cb->mac_addr, true);
                    
                    
                    /* Initialize sending parameters. */
                    
                    /*********************************************/
                    /*************** STEP 2 **********************/
                    /* Remove the comment in the next line to    */
                    /* send a unicast message                    */
                    /*********************************************/

                    example_espnow_send_param_t *send_param = malloc(sizeof(example_espnow_send_param_t));
                    if (send_param == NULL) {
                        ESP_LOGE(TAG, "Malloc send parameter fail");
                        vSemaphoreDelete(s_example_espnow_queue);
                        esp_now_deinit();
                        return;
                    }
                    memset(send_param, 0, sizeof(example_espnow_send_param_t));
                    send_param->len = sizeof(example_espnow_data_t) + 4;
                    send_param->buffer = malloc(send_param->len);
                    if (send_param->buffer == NULL) {
                        ESP_LOGE(TAG, "Malloc send buffer fail");
                        free(send_param);
                        vSemaphoreDelete(s_example_espnow_queue);
                        esp_now_deinit();
                        return;
                    }
                    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
                    memcpy(buf->payload, "RTS", 4);
                    memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                    example_espnow_data_prepare(send_param);

                    ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_param->dest_mac));
                    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                        ESP_LOGE(TAG, "Send error");
                        example_espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                }
                else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d, data: %s", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len, payload);
                    if(strcmp((char *)payload,"CTS") == 0){

                        ESP_LOGI(TAG, "Send data to "MACSTR", data: some data", MAC2STR(my_peer));

                        example_espnow_send_param_t *send_param = malloc(sizeof(example_espnow_send_param_t));
                        if (send_param == NULL) {
                            ESP_LOGE(TAG, "Malloc send parameter fail");
                            vSemaphoreDelete(s_example_espnow_queue);
                            esp_now_deinit();
                            return;
                        }
                        memset(send_param, 0, sizeof(example_espnow_send_param_t));
                        send_param->len = sizeof(example_espnow_data_t) + 10;
                        send_param->buffer = malloc(send_param->len);
                        if (send_param->buffer == NULL) {
                            ESP_LOGE(TAG, "Malloc send buffer fail");
                            free(send_param);
                            vSemaphoreDelete(s_example_espnow_queue);
                            esp_now_deinit();
                            return;
                        }
                        example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
                        memcpy(buf->payload, "some data", 10);
                        memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        example_espnow_data_prepare(send_param);

                        ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_param->dest_mac));
                        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                            ESP_LOGE(TAG, "Send error");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }

                        vTaskDelay(500 / portTICK_PERIOD_MS);
                    }
                    else if(strcmp((char *)payload,"ACK") == 0){
                        example_espnow_send_param_t *send_param = malloc(sizeof(example_espnow_send_param_t));

                        ESP_LOGI(TAG, "Send data to "MACSTR", data: RTS", MAC2STR(my_peer));

                        if (send_param == NULL) {
                            ESP_LOGE(TAG, "Malloc send parameter fail");
                            vSemaphoreDelete(s_example_espnow_queue);
                            esp_now_deinit();
                            return;
                        }
                        memset(send_param, 0, sizeof(example_espnow_send_param_t));
                        send_param->len = sizeof(example_espnow_data_t) + 4;
                        send_param->buffer = malloc(send_param->len);
                        if (send_param->buffer == NULL) {
                            ESP_LOGE(TAG, "Malloc send buffer fail");
                            free(send_param);
                            vSemaphoreDelete(s_example_espnow_queue);
                            esp_now_deinit();
                            return;
                        }
                        example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
                        memcpy(buf->payload, "RTS", 4);
                        memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        example_espnow_data_prepare(send_param);

                        ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_param->dest_mac));
                        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                            ESP_LOGE(TAG, "Send error");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }

                        vTaskDelay(500 / portTICK_PERIOD_MS);
                    }
                    else if(strcmp((char *)payload, "RTS") == 0){
                        example_espnow_send_param_t *send_param = malloc(sizeof(example_espnow_send_param_t));
                        if (send_param == NULL) {
                            ESP_LOGE(TAG, "Malloc send parameter fail");
                            vSemaphoreDelete(s_example_espnow_queue);
                            esp_now_deinit();
                            return;
                        }
                        memset(send_param, 0, sizeof(example_espnow_send_param_t));
                        send_param->len = sizeof(example_espnow_data_t) + 4;
                        send_param->buffer = malloc(send_param->len);
                        if (send_param->buffer == NULL) {
                            ESP_LOGE(TAG, "Malloc send buffer fail");
                            free(send_param);
                            vSemaphoreDelete(s_example_espnow_queue);
                            esp_now_deinit();
                            return;
                        }
                        example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
                        memcpy(buf->payload, "CTS", 4);
                        memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        example_espnow_data_prepare(send_param);
                        

                        ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_param->dest_mac));
                        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                            ESP_LOGE(TAG, "Send error");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }

                        vTaskDelay(500 / portTICK_PERIOD_MS);
                    }
                    else if(strcmp((char *)payload, "some data") == 0){
                        example_espnow_send_param_t *send_param = malloc(sizeof(example_espnow_send_param_t));
                        if (send_param == NULL) {
                            ESP_LOGE(TAG, "Malloc send parameter fail");
                            vSemaphoreDelete(s_example_espnow_queue);
                            esp_now_deinit();
                            return;
                        }
                        memset(send_param, 0, sizeof(example_espnow_send_param_t));
                        send_param->len = sizeof(example_espnow_data_t) + 4;
                        send_param->buffer = malloc(send_param->len);
                        if (send_param->buffer == NULL) {
                            ESP_LOGE(TAG, "Malloc send buffer fail");
                            free(send_param);
                            vSemaphoreDelete(s_example_espnow_queue);
                            esp_now_deinit();
                            return;
                        }
                        example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
                        memcpy(buf->payload, "ACK", 4);
                        memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        example_espnow_data_prepare(send_param);
                        

                        ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_param->dest_mac));
                        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                            ESP_LOGE(TAG, "Send error");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }

                        vTaskDelay(500 / portTICK_PERIOD_MS);
                    }
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                free(payload);
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;
    
    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
    ESP_ERROR_CHECK( esp_now_set_wake_window(65535) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    add_peer(s_example_broadcast_mac, false);
    
    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->len = sizeof(example_espnow_data_t) + 7;
    send_param->buffer = malloc(send_param->len);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
    memcpy(buf->payload, "add_me", 7);
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    xTaskCreate(example_espnow_task, "example_espnow_task", 4096, send_param, 4, NULL);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_example_espnow_queue);
    esp_now_deinit();
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    example_wifi_init();
    example_espnow_init();
    
    /*********************************************/
    /*************** STEP 3 **********************/
    /* Remove the comment in the next line to    */
    /* go to periodic sleep                      */
    /*********************************************/
    /*while(1){
        vTaskDelay(500 / portTICK_PERIOD_MS);
        //esp_wdt_reset();
        esp_wifi_stop();
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(1000));
        esp_light_sleep_start();
        esp_wifi_start();
    }*/
}
