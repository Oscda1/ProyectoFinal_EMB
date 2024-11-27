#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_http_server.h>
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "connect_wifi.h"

#define HEADER 0x27
#define CMD_SENSOR_READ 0x9F
#define CMD_SD_LOAD 0x9E
#define CMD_SD_SAVE 0x9D

#define I2C_MASTER_SCL_IO    22    // Pin SCL
#define I2C_MASTER_SDA_IO    21    // Pin SDA
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000
#define SLAVE_DIR_SD         0x12 // Dirección del slave del sd card
#define SLAVE_DIR_SENSOR     0x28 // Dirección del slave del sensor

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

void master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 256, 256, 0);
}

esp_err_t pedir_data_slave(uint8_t *data, size_t size) {
    i2c_cmd_handle_t comand = i2c_cmd_link_create();   

    uint8_t form[2] = {HEADER, CMD};
 //escritura para leer
    i2c_master_start(comand); //SEÑAL DE INICIO
    i2c_master_write_byte(comand, (SLAVE_DIR << 1) | I2C_MASTER_WRITE, true); //direccion del slave
    i2c_master_write(comand, form, sizeof(form), true); //manda los datos
    i2c_master_stop(comand);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, comand, 500 / portTICK_PERIOD_MS); //se manda los comandos en la cola
    
    if(ret == ESP_OK){ //si es exitoso empiza  a leer
        i2c_cmd_link_delete(comand);
        comand = i2c_cmd_link_create();
        i2c_master_start(comand);
        i2c_master_write_byte(comand, (SLAVE_DIR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(comand, data, size, I2C_MASTER_LAST_NACK);
        i2c_master_stop(comand);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, comand, 500 / portTICK_PERIOD_MS);
    }
    i2c_cmd_link_delete(comand);
    return ret;
}

// void recibir_datos(void *args) {
//     uint8_t data[256];
//     char temp[256];
//     int i = 0;
//     while (1) {
//         esp_err_t ret = pedir_data_slave(data, 256);
//         if(ret == ESP_OK){
//             if(data[0] == HEADER && data[1] == CMD){ 
//                 i = 0; //restaura el contador
//                 sprintf(temp, "Temperatura: %c%c.%c%c °C\n", data[2], data[3], data[5], data[6]);
//                 uart_puts(temp);
//                 vTaskDelay(2000 / portTICK_PERIOD_MS);
//             }
//         } else if(i >= 2) {
//             uart_puts("Comunicacion terminada, el periferico no responde\n");
//             break;
//         } else {
//             i++;
//             uart_puts("Intentando nuevamente...\n");
//             vTaskDelay(500 / portTICK_PERIOD_MS); 
//         }
//     }
//     vTaskDelete(NULL); 
// }

static void send_temperature(httpd_handle_t hd, int fd) {
    uint8_t data[2]; // Ajusta el tamaño de acuerdo con los datos que el sensor devuelve
    esp_err_t ret = pedir_data_slave(data, sizeof(data));
    
    if (ret == ESP_OK) {
        // Convertir los datos recibidos en temperatura (esto depende del formato del sensor)
        float temperature = (float)(data[0] << 8 | data[1]) / 10.0; // Ajusta según el formato de los datos
        
        // Enviar la temperatura al WebSocket
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.payload = (uint8_t *)malloc(64);
        sprintf((char *)ws_pkt.payload, "%.2f", temperature); // Formatear como string
        ws_pkt.len = strlen((char *)ws_pkt.payload);
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;

        httpd_ws_send_frame_async(hd, fd, &ws_pkt);
        free(ws_pkt.payload); // Liberar la memoria después de enviar
    } else {
        ESP_LOGE(TAG, "Error reading sensor data");
    }
}

static void temperatura(void *arg)
{
    while (1)
    {
        uint8_t data[4];
        esp_err_t ret = pedir_data_slave(data, 4);
        if (ret == ESP_OK)
        {
            float temp = (float)(data[0] << 8 | data[1]) / 100;

            httpd_handle_t hd = req -> handle;
            int fd = httpd_req_to_sockfd(req);
            send_temperature(hd, fd);
        }else{
            ESP_LOGE(TAG, "Error reading sensor data");
        }
        // Esperar 10 segundos antes de hacer el siguiente toggle
        vTaskDelay(10000 / portTICK_PERIOD_MS);
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

    led_state = !led_state;
    gpio_set_level(LED_PIN, led_state);

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
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }

    if (ws_pkt.len)
    {
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
    }

    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char *)ws_pkt.payload, "toggle") == 0)
    {
        free(buf);
        return trigger_async_send(req->handle, req);
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

void app_main()
{
    master_init();
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();

    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    led_state = 0; // Estado inicial del LED (apagado)
    ESP_LOGI(TAG, "ESP32 ESP-IDF WebSocket Web Server is running ... ...\n");
    initi_web_page_buffer();
    setup_websocket_server();

    // Iniciar la tarea para hacer el toggle del LED cada 10 segundos
    xTaskCreate(temperatura, "Control temperatura", 2048, NULL, 5, NULL);
}