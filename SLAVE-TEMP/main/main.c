#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include <string.h>
#include "esp_log.h"

#define HEADER_SENSOR 0x27
#define CMD_SENSOR_READ 0x9F

#define HEADER_SIM 0x28
#define CMD_SIM_WRITE 0x9F
#define CMD_SIM_LOAD 0x9E

#define SLAVE_DIR 0X28 //Posible cambio
#define I2C_SLAVE_NUM 1
#define I2C_MASTER_NUM 0

#define BME280_SENSOR_ADDR 0xF6
#define BME280_CHIP_ID 0xD0
#define BME280_TEMP_MSB 0xFA // 
#define BME280_TEMP_LSB 0xFB // Contiene la salida del dato de la temperatura de 20 bits
#define BME280_TEMP_XLSB 0xFC //
//registros para inicializar el sensor
#define BME280_RESET_ADDRESS 0xE0
#define BME280_RESET 0x68
#define BME280_CONFIG 0xF5
#define BME280_CTRL_MEAS 0xF4

#define dig_t1_lsb 0x88
#define dig_t1_msb 0x89
#define dig_t2_lsb 0x8A
#define dig_t2_msb 0x8B
#define dig_t3_lsb 0x8C
#define dig_t3_msb 0x8D

static const char *TAG= "I2C_SLAVE";

uint16_t Temp[3];
int32_t t_fine; 


static void master_init(){
    i2c_config_t master_config={
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 19, //PIN SDA
        .scl_io_num = 18, //PIN SCL
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    int i2c_master = I2C_MASTER_NUM;

    i2c_param_config(i2c_master, &master_config);
    i2c_driver_install(i2c_master, master_config.mode, 0 , 0, 0); 
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

static esp_err_t register_read(uint8_t reg_addr, uint8_t *data, size_t len){
    return i2c_master_write_read_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

static esp_err_t register_write_byte(uint8_t reg_addr, uint8_t data){
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
    return ret;
}

uint32_t read_temp(){ // Combinar los 3 bytes de temperatura
    uint8_t data;
    uint32_t temperatura;
    register_read(BME280_TEMP_MSB, &data, 1);
    temperatura = (data << 12); //Bits mas significativos
    register_read(BME280_TEMP_LSB, &data, 1);
    temperatura |= (data<<4); //Bits de en medio o menos significativos
    register_read(BME280_TEMP_XLSB, &data, 1);
    temperatura |= (data >> 4); //Los bits estan en la parte mas alta de los 8 bits
    return temperatura;
}

static void compensation_temp(){ //lectura y lo guarda
    uint8_t data;
    register_read(dig_t1_lsb, &data, 1);
    Temp[0] = (uint16_t)data;
    register_read(dig_t1_msb, &data, 1); //digito 1 de temperatura
    Temp[0] |= (uint16_t)(data<<8);
    register_read(dig_t2_lsb, &data, 1);
    Temp[1] = (uint16_t)data;
    register_read(dig_t2_msb, &data, 1); //digito 2 de temperatura
    Temp[1] |= (uint16_t)(data<<8);
    register_read(dig_t3_lsb, &data, 1);
    Temp[2] = (uint16_t)data;
    register_read(dig_t3_msb, &data, 1); //digito 3 de temperatura
    Temp[2] |= (uint16_t)(data<<8);
}

float conversion_Temp(){
    int32_t no_conv_temp = read_temp();
    int32_t var1,var2;
    var1 =  ((((no_conv_temp >> 3) - ((int32_t)Temp[0]<<1))) * ((int32_t)Temp[1])) >> 11;
    var2 = (((((no_conv_temp >> 4) - ((int32_t)Temp[0])) * ((no_conv_temp >> 4) - ((int32_t)Temp[0]))) >> 12) * ((int32_t)Temp[2])) >> 14;
    t_fine = var1 + var2;
    float T = (t_fine * 5 + 128)>>8;
    return (float)(T/100.00);
}


void sensor_init(){
    uint8_t ctrl_meas = 0x27; //temperatura
    uint8_t config = 1<<4;
    ESP_ERROR_CHECK(register_write_byte(BME280_RESET_ADDRESS, BME280_RESET)); //reiniciar el sensor
    ESP_LOGI(TAG, "BME280 reset done");
    ESP_ERROR_CHECK(register_write_byte(BME280_CTRL_MEAS, ctrl_meas));
    ESP_ERROR_CHECK(register_write_byte(BME280_CONFIG, config));
}

void slave_enviar_datos(void *args){
    
    while (1)
    {
        uint8_t formato[2];
        int solicitud = i2c_slave_read_buffer(I2C_SLAVE_NUM, formato, sizeof(formato), 500/ portTICK_PERIOD_MS);
        if (solicitud == 2 && formato[0] == HEADER_SENSOR && formato[1] == CMD_SENSOR_READ){
            // Calcular y mostrar la temperatura compensada
            float temperatura = conversion_Temp();
            uint8_t data[48];

            snprintf((char *)&data, sizeof(data), "%.2f", temperatura); // Convierte a cadena con 2 decimales
            ESP_LOGI(TAG, "Temperatura: %s °C", &data[2]);

            // Enviar los datos
            i2c_slave_write_buffer(I2C_SLAVE_NUM, (uint8_t*) &temperatura, 4, 300 / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGI(TAG, "Error al recibir encabezado");
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_SLAVE_NUM));
}


void app_main() {
     uint8_t dataId[2];

    master_init();
    slave_init();
    ESP_LOGI(TAG, "I2C incializado exitosamente");
    ESP_ERROR_CHECK(register_read(BME280_CHIP_ID, dataId, 1)); // Leer WHO_AM_I
    ESP_LOGI(TAG, "WHO_AM_I = %X", dataId[0]);
    sensor_init(); // Inicializar el sensor BME280
    compensation_temp(); // Leer los datos de calibración
    ESP_LOGI(TAG, "Lectura de calibracion exitosa");
    xTaskCreate(slave_enviar_datos, "slave_enviar_datos", 4096, NULL, 10, NULL);
    
    
}
