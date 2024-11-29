#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/uart.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include <ctype.h>
#include "driver/uart.h"
#include "esp_sleep.h"

#define EXAMPLE_MAX_CHAR_SIZE    64
#define BUFFER_SIZE 1024
#define MOUNT_POINT "/sdcard"
#define PIN_NUM_MISO 14
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK 26
#define PIN_NUM_CS 25
#define PIN_WAKEUP 13
#define PIN_TX 22
#define PIN_RX 23
#define SAVE_CMD 36
#define LOAD_CMD 37
#define DELETE_CMD 38
#define BORRAR 1


static const char *TAG = "SD_CARD_SLAVE";
char bufferfile[BUFFER_SIZE];
uint8_t i=0,j=0, primero=0, n=0;
sdmmc_card_t *card;
sdmmc_host_t host = SDSPI_HOST_DEFAULT();
QueueHandle_t queueUART = NULL;
const char mount_point[] = MOUNT_POINT;
uint8_t data[26] = {0};
char cad[EXAMPLE_MAX_CHAR_SIZE]={0};
char salida[200] = ""; 
RTC_DATA_ATTR static uint8_t firstrun = 0;

#define CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED

void initSD(){
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif 
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    ESP_LOGI(TAG, "Inicializando SD card");
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Usando periférico SPI");
    host.max_freq_khz = 5000;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fallo inicializacion del bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;


    ESP_LOGI(TAG, "Montando el filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Fallo montar el filesystem. "
                     "Si desea formatear la tarjeta, configure la opcion CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED.");
        } else {
            ESP_LOGE(TAG, "Fallo la inicializacion de la tarjeta (%s). "
                     "Asegurese de que las lineas de la tarjeta SD tengan resistencias pull-up en su lugar.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem montado");
    sdmmc_card_print_info(stdout, card);
    vTaskDelay(1000/portTICK_PERIOD_MS);
}

// Función para verificar si una cadena representa un número flotante válido


static void uart_init(){
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0);
}

void gpio_init(){
    gpio_reset_pin(PIN_WAKEUP);
    gpio_set_direction(PIN_WAKEUP, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_WAKEUP, GPIO_PULLDOWN_ONLY);
}

//Genera una funcion imprimir que imprima el contenido del archivo montado en la SDCARD en el archivo valoores.txt
void imprimir() {
    FILE *F = fopen(MOUNT_POINT"/valores.txt", "r");
    if (F == NULL) {
        ESP_LOGE(TAG, "Fallo al abrir el archivo para lectura!");
        return;
    }
    ESP_LOGI(TAG, "Contenido del archivo valores.txt:");
    while (fgets(cad, EXAMPLE_MAX_CHAR_SIZE, F) != NULL) {
        printf("%s", cad);
    }
    fclose(F);
}

//Genera una funcion que borre el contenido del archivo valores.txt en la SDCARD sin borrar el archivo
void borrar_contenido() {
    FILE *F = fopen(MOUNT_POINT"/valores.txt", "w");
    if (F == NULL) {
        ESP_LOGE(TAG, "Fallo al abrir el archivo para borrar contenido!");
        return;
    }
    fclose(F);
    ESP_LOGI(TAG, "Contenido del archivo valores.txt borrado!");
}

void slave_listening(void *args){
    while(1){
        uint8_t len = uart_read_bytes(UART_NUM_1, salida, sizeof(salida), 500/portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Recibido: %s con %d bytes", salida, len);
        if(salida[0] == SAVE_CMD){
            FILE *F = fopen(MOUNT_POINT"/valores.txt", "a+");
            ESP_LOGI(TAG, "Numero recibido: %s", &salida[1]);
            fprintf(F, "%s\n", &salida[1]);
            fclose(F);
        }else{
            ESP_LOGI(TAG, "Comando no reconocido");
        }
        imprimir();
        vTaskDelay(100/portTICK_PERIOD_MS);
        esp_deep_sleep_start();
    }
}

void app_main()
{
    initSD();
    uart_init();
    gpio_init();
    
    ESP_LOGI(TAG, "SPI SD incializado exitosamente");
    esp_sleep_enable_ext0_wakeup(PIN_WAKEUP, 1);
    if (firstrun == 0)
    {
        firstrun = 1;
        ESP_LOGI(TAG, "Primer deep sleep");
        #ifdef BORRAR
            borrar_contenido();
        #endif
        esp_deep_sleep_start();
    }
    else
    {
        xTaskCreate(slave_listening, "slave_listening", 4096, NULL, 10, NULL);
    }
    do
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } while (1);
}
