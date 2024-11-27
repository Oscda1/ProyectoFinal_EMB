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
#include "driver/i2c.h"
#include "esp_log.h"
#include <ctype.h>

#define EXAMPLE_MAX_CHAR_SIZE    64
#define BUFFER_SIZE 1024
#define MOUNT_POINT "/sdcard"
#define PIN_NUM_MISO  13
#define PIN_NUM_MOSI  12
#define PIN_NUM_CLK   14
#define PIN_NUM_CS    27

#define HEADER 0x27
#define CMD_SAVE 0x9D
#define CMD_LOAD 0x9E

#define SLAVE_DIR 0X12 //Posible cambio
#define I2C_SLAVE_NUM 1
#define I2C_MASTER_NUM 0

#define dig_t1_lsb 0x88
#define dig_t1_msb 0x89
#define dig_t2_lsb 0x8A
#define dig_t2_msb 0x8B
#define dig_t3_lsb 0x8C
#define dig_t3_msb 0x8D

static const char *TAG = "SD_CARD_SLAVE";
uint8_t i=0,j=0, primero=0, n=0;
sdmmc_card_t *card;
sdmmc_host_t host = SDSPI_HOST_DEFAULT();
QueueHandle_t queueUART = NULL;
const char mount_point[] = MOUNT_POINT;
TaskHandle_t taskImpresion = NULL, taskUART;
char data[26] = {0};
char cad[EXAMPLE_MAX_CHAR_SIZE]={0};
char salida[100] = ""; 

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
int esNumeroFlotanteValido(const char* str) {
    int puntoEncontrado = 0;
    
    // Verificar cada carácter de la cadena
    for (int i = 0; str[i] != '\0'; i++) {
        if (str[i] == '.') {
            // Solo puede haber un punto decimal
            if (puntoEncontrado) {
                return 0;  // No es un número flotante válido
            }
            puntoEncontrado = 1;
        } else if (!(str[i] >= '0' && str[i] <= '9') && str[i] != '+' && str[i] != '-' && str[i] != '.') {
            return 0;  // Contiene caracteres no válidos para un número flotante
        }
    }
    
    // Una cadena válida debe contener al menos un dígito
    return (str[0] != '\0');
}

// Función para extraer números flotantes de una cadena de texto
int extraerNumerosFlotantes(const char* texto, float** resultados) {
    int cantidad = 0;
    char buffer[100];  // Buffer para almacenar el número flotante como cadena temporal
    size_t i = 0;
    
    while (texto[i] != '\0') {
        // Buscar el corchete de apertura
        if (texto[i] == '[') {
            i++;  // Saltamos el corchete de apertura
            size_t j = 0;
            
            // Copiar caracteres hasta encontrar el corchete de cierre o fin de la cadena
            while (texto[i] != ']' && texto[i] != '\0') {
                buffer[j++] = texto[i++];
            }
            buffer[j] = '\0';  // Terminar la cadena en el buffer
            
            // Si el contenido es un número flotante válido, lo convertimos y almacenamos
            if (esNumeroFlotanteValido(buffer)) {
                float num;
                if (sscanf(buffer, "%f", &num) == 1) {
                    cantidad++;
                    // Redimensionar el arreglo de resultados dinámicamente
                    *resultados = realloc(*resultados, cantidad * sizeof(float));
                    (*resultados)[cantidad - 1] = num;
                }
            }
        }
        i++;
    }
    
    return cantidad;
}


void guardarValor(uint8_t data[]){
    FILE *f = fopen("/sdcard/valores.txt", "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Fallo al abrir el archivo");
    }else{
        float data_float;
        memcpy(&data_float, &data[2], 4);
        fprintf(f, "Medicion %d: [%f]\n",n, data_float);
        fclose(f);
        n++;
        ESP_LOGI(TAG, "Valor guardado: %s", data);
    }
    return;
}

void enviarValores(){
    FILE *f = fopen("/sdcard/valores.txt", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Fallo al abrir el archivo");
    }else{
        memcpy(salida, f, sizeof(f));
        float* resultados = NULL;
        uint16_t cantidad = extraerNumerosFlotantes(salida, &resultados);
        fclose(f);
        i2c_slave_write_buffer(I2C_SLAVE_NUM, &resultados, cantidad, 300 / portTICK_PERIOD_MS);
        memset(salida, 0, sizeof(salida));
    }
    return;
}

static void slave_init(){
    i2c_config_t slave_config={
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = 21, //pin sda
        .scl_io_num = 22, //pin scl
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = SLAVE_DIR,
    };
    i2c_param_config(I2C_SLAVE_NUM, &slave_config);
    i2c_driver_install(I2C_SLAVE_NUM, slave_config.mode, 256, 256, 0);
}

void slave_enviar_datos(void *args){
    
    while (1)
    {
        uint8_t formato[5];
        int solicitud = i2c_slave_read_buffer(I2C_SLAVE_NUM, formato, sizeof(formato), portMAX_DELAY);
        if (solicitud == 2 && formato[0] == HEADER && formato[1] == CMD_SAVE){
            ESP_LOGI(TAG, "Guardando datos");
            guardarValor(formato);
        }
        else if(solicitud == 2 && formato[0] == HEADER && formato[1] == CMD_LOAD)
        {
            ESP_LOGI(TAG, "Enviando datos");
            enviarValores();
        }else{
            ESP_LOGI(TAG, "Comando no reconocido");
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_SLAVE_NUM));
}


void app_main() {
    slave_init();
    initSD();
    ESP_LOGI(TAG, "I2C incializado exitosamente");
    ESP_LOGI(TAG, "WHO_AM_I = %X", SLAVE_DIR);
    xTaskCreate(slave_enviar_datos, "slave_enviar_datos", 4096, NULL, 10, NULL);
    do{
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }while(1);   
}