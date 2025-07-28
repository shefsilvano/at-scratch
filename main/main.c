#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"


//private variables, handles, abd prototype functions 
//defined macros
#define UART_BUFFER_SIZE (1024)
#define UART_SIM_BUFFER (248)
#define UART_PORT_SIM (UART_NUM_2) //uart port for the sim card module
#define TSIM_PWR (GPIO_NUM_4)
#define UART_SIM_TX (27) //GSM Rx -> IO27
#define UART_SIM_RX (26) //GSM Tx -> IO26 


//handles
static QueueHandle_t uart_queue2;
static SemaphoreHandle_t xSensorReady;
static TaskHandle_t xInit; 
static BaseType_t init; 
//variables
char uart_at_buffer[UART_SIM_BUFFER]; 
uint16_t uart_at_index =0; 
bool success =false; 

//functions 
static void uart_event_task_sim (void *arg);
void init_uart (uart_port_t uart_num, int tx, int rx, int baud); 
void sim7000G_init (void); 
void module_init(void *arg); 
bool sendATCommand(char* command, char* expectedResponse, int timeoutMs); 
bool parse_data(uint8_t* data, size_t len, int timeout_ms , const char* word); 
//tags 
static const char *TAG_SIM = "UART_SIM_LOG";


void app_main(void)
{
// xSensorReady = xSemaphoreCreateBinary();
    //xTaskCreate(uart_event_task_sim,"uart_event_sim", 4096, NULL,5,NULL); 
    init = xTaskCreate(module_init, "mod-init", 4096, NULL, 5, &xInit);
    configASSERT(init==pdPASS); 
  

   


}
void init_uart (uart_port_t uart_num, int tx, int rx, int baud){
     uart_config_t uart_config = {
    .baud_rate =baud, 
    .data_bits = UART_DATA_8_BITS, 
    .parity = UART_PARITY_DISABLE, 
    .stop_bits = UART_STOP_BITS_1, 
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB, 
    }; 

    uart_param_config(uart_num, &uart_config); 
    uart_set_pin(uart_num, tx, rx, UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE); 
    uart_driver_install(uart_num, UART_BUFFER_SIZE*2, 0, 0, NULL,0);
    
}

void sim7000G_init (void){
    ESP_LOGI(TAG_SIM,"UART for Sim7000G has been initialized"); 
    init_uart(UART_PORT_SIM, UART_SIM_TX, UART_SIM_RX, 115200);
    uart_flush_input(UART_PORT_SIM);

     gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TSIM_PWR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    //copied from the ino file used for initial testing of the board 
    gpio_set_level(TSIM_PWR,1); 
    ESP_LOGI(TAG_SIM, "TSIM power pin set to low for 1 sec"); 
    //vTaskDelay(1000/portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(TSIM_PWR, 0);
    ESP_LOGI(TAG_SIM, "waiting to boot..."); 
    vTaskDelay(pdMS_TO_TICKS(10000)); //wait to boot 
}

void module_init(void *arg){
    int counter = 0; 
    sim7000G_init(); 
    while (1){
        
        success = sendATCommand("AT\r\n", "OK", 500);

            if (success)
            {
                printf("Module Initialization done successfully with final count: %d", counter);
            } 

            else
            {
                printf("Module Initialization fail, with count:%d\n ",counter);
            }
            counter++; 
    }
}


bool sendATCommand(char* command, char* expectedResponse, int timeoutMs) // Sending AT Command
{
    uint8_t buffer[UART_SIM_BUFFER];

    memset(buffer , 0 ,UART_SIM_BUFFER);

    uart_write_bytes(UART_PORT_SIM, command, strlen(command));  

    vTaskDelay(pdMS_TO_TICKS(100)); 

    printf("\nWrite done\n");

    bool responseReceived = parse_data(buffer, UART_SIM_BUFFER , timeoutMs, expectedResponse);

    if (responseReceived) 
    {
        printf("\nCommand sent successfully!\n");

        printf("Response: %s\n", buffer); // printing response

        return true;

    } else 
    {
        printf("Failed to receive expected response!\n");
        return false;
    }
}

bool parse_data(uint8_t* data, size_t len, int timeout_ms , const char* word) // function for pasing data 
{
    char resp[UART_SIM_BUFFER] = {0};   // Allocate a buffer to hold the received data

    TickType_t start_time = xTaskGetTickCount(); // get the start time of the loop
    
    printf("\nparsing data start\n");

    while ((xTaskGetTickCount() - start_time) < (timeout_ms / portTICK_PERIOD_MS))
    {
        int bytes_read = uart_read_bytes(UART_PORT_SIM, data, len, pdMS_TO_TICKS(500));

        if (bytes_read > 0)
        {
            // Append the received data to the buffer
            // strncat(resp, (const char*)data, bytes_read); a
            
            

            // Check if the word is present in the buffer
            if (strstr(resp, word) != NULL)
            {
                return true;
            }
        }
    }
    
    return false;
}
