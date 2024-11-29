#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_http_server.h>
#include <esp_netif.h>
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <esp_wifi.h>
#include "esp_log.h"
#include "cJSON.h"
#include "esp_event.h"

#include "connect_wifi.h"

#define PIN_CS_SENS 23
#define PIN_CS_SD 22
#define PIN_TX 17
#define PIN_RX_SD 18
#define PIN_RX_SENS 4
/*#define TXD_PIN 17
#define RXD_PIN 4
#define CS_SENS_PIN 23
#define CS_SD_PIN 22*/

#define SAVE_CMD 36

httpd_handle_t server = NULL;
struct async_resp_arg
{
    httpd_handle_t hd;
    int fd;
};

static const char *TAG = "WebSocket Server"; // TAG for debug
int led_state = 0;

#define INDEX_HTML_PATH "/spiffs/index.html"
char index_html[4096];
char response_data[4096];
const char mi_ssid[] = "Proyecto";
EventGroupHandle_t wifi_event_group;
uint8_t FIRST_CLIENT = BIT0, UART_EN_USO = BIT1;

uint32_t to_big_endian(uint32_t num) {
    return ((num >> 24) & 0xFF) | ((num >> 8) & 0xFF00) | ((num << 8) & 0xFF0000) | ((num << 24) & 0xFF000000);
}

// Función para convertir un bloque de 4 bytes en un float, considerando el endianness


void send_temperature(void* args){
    char buffer[20];
    int8_t len = 0;
    while (1){
        xEventGroupWaitBits(wifi_event_group, FIRST_CLIENT, pdFALSE, pdTRUE, portMAX_DELAY);
        uint8_t data[4];
        gpio_set_level(PIN_CS_SENS, 1);
        len = uart_read_bytes(UART_NUM_1, data, 4, 1000 / portTICK_PERIOD_MS);
        if (len >= 0)
        {
            xEventGroupWaitBits(wifi_event_group, UART_EN_USO, pdTRUE, pdTRUE, portMAX_DELAY);
            gpio_set_level(PIN_CS_SENS, 1);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            gpio_set_level(PIN_CS_SENS, 0);
            len = uart_read_bytes(UART_NUM_2, buffer, 5, 1000 / portTICK_PERIOD_MS);
            buffer[len] = '\0';
            ESP_LOGI(TAG, "Len: %d", len);
            ESP_LOGI(TAG, "Medicion: %s", buffer);

            // Create JSON object
            cJSON *root = cJSON_CreateObject();
            cJSON_AddNumberToObject(root, "lectura", 1);
            cJSON_AddStringToObject(root, "medicion", buffer);
            char *json_string = cJSON_Print(root);

            httpd_ws_frame_t ws_pkt;
            memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
            ws_pkt.payload = (uint8_t *)json_string;
            ws_pkt.len = strlen(json_string);
            ws_pkt.type = HTTPD_WS_TYPE_TEXT;

            size_t max_clients = CONFIG_LWIP_MAX_LISTENING_TCP;
            size_t fds = max_clients;
            int client_fds[max_clients];
            esp_err_t ret = httpd_get_client_list(server, &max_clients, client_fds);
            for (int i = 0; i < fds; i++){
                int client_info = httpd_ws_get_fd_info(server, client_fds[i]);
                if (client_info == HTTPD_WS_CLIENT_WEBSOCKET){
                    httpd_ws_send_frame_async(server, client_fds[i], &ws_pkt);
                }
                ESP_LOGI(TAG, "Enviando a %d", client_fds[i]);
            }

            // Clean up
            cJSON_Delete(root);
            free(json_string);

            gpio_set_level(PIN_CS_SD, 1);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            gpio_set_level(PIN_CS_SD, 0);
            char salida[30];
            sprintf(salida, "%c%s", SAVE_CMD, buffer);
            vTaskDelay(1500 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "Salida: %s", salida);
            uart_write_bytes(UART_NUM_1, salida, strlen(salida));
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            xEventGroupSetBits(wifi_event_group, UART_EN_USO);
        }
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(mi_ssid),
            .channel = 1,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };

    memcpy(wifi_config.ap.ssid, mi_ssid, strlen(mi_ssid));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "ESP32 AP iniciado con SSID: %s, Password: %s channel: %d", wifi_config.ap.ssid, wifi_config.ap.password, wifi_config.ap.channel);
}

static void initi_web_page_buffer(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    memset((void *)index_html, 0, sizeof(index_html));
    struct stat st;
    if (stat(INDEX_HTML_PATH, &st))
    {
        ESP_LOGE(TAG, "index.html not found");
        return;
    }

    FILE *fp = fopen(INDEX_HTML_PATH, "r");
    if (fread(index_html, st.st_size, 1, fp) == 0)
    {
        ESP_LOGE(TAG, "fread failed");
    }
    fclose(fp);
}

esp_err_t get_req_handler(httpd_req_t *req)
{
    int response;
    if (led_state)
    {
        sprintf(response_data, index_html, "ON");
    }
    else
    {
        sprintf(response_data, index_html, "OFF");
    }
    response = httpd_resp_send(req, response_data, HTTPD_RESP_USE_STRLEN);
    return response;
}

static void ws_async_send(void *arg)
{
    httpd_ws_frame_t ws_pkt;
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;


    char buff[4];
    memset(buff, 0, sizeof(buff));
    sprintf(buff, "%d", led_state);

    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t *)buff;
    ws_pkt.len = strlen(buff);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    static size_t max_clients = CONFIG_LWIP_MAX_LISTENING_TCP;
    size_t fds = max_clients;
    int client_fds[max_clients];

    esp_err_t ret = httpd_get_client_list(server, &fds, client_fds);

    if (ret != ESP_OK)
    {
        return;
    }

    for (int i = 0; i < fds; i++)
    {
        int client_info = httpd_ws_get_fd_info(server, client_fds[i]);
        if (client_info == HTTPD_WS_CLIENT_WEBSOCKET)
        {
            httpd_ws_send_frame_async(hd, client_fds[i], &ws_pkt);
        }
    }
    free(resp_arg);
}

static esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req)
{
    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    return httpd_queue_work(handle, ws_async_send, resp_arg);
}

static esp_err_t handle_ws_req(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        xEventGroupSetBits(wifi_event_group, FIRST_CLIENT);
        xEventGroupSetBits(wifi_event_group, UART_EN_USO);
        return ESP_OK;
    }
        return ESP_OK;
}


httpd_handle_t setup_websocket_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t uri_get = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_req_handler,
        .user_ctx = NULL};

    httpd_uri_t ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = handle_ws_req,
        .user_ctx = NULL,
        .is_websocket = true};

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &ws);
    }

    return server;
}

void gpio_init(){
    gpio_reset_pin(PIN_CS_SENS);
    gpio_set_direction(PIN_CS_SENS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_CS_SENS, 0);

    gpio_reset_pin(PIN_CS_SD);
    gpio_set_direction(PIN_CS_SD, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_CS_SD, 0);
}

void uart_init(){
    uart_config_t uart_config2 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_2, &uart_config2);
    uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, PIN_RX_SENS, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, 1024, 0, 0, NULL, 0);

        uart_config_t uart_config1 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config1);
    uart_set_pin(UART_NUM_1, PIN_TX, PIN_RX_SENS, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);
}

void app_main()
{
    gpio_init();
    uart_init();
    wifi_event_group = xEventGroupCreate();
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();

    led_state = 0; // Estado inicial del LED (apagado)
    ESP_LOGI(TAG, "ESP32 ESP-IDF WebSocket Web Server is running ... ...\n");
    initi_web_page_buffer();
    setup_websocket_server();

    // Iniciar la tarea para hacer el toggle del LED cada 10 segundos
    xTaskCreate(send_temperature, "Control temperatura", 2048, NULL, 5, NULL);
}