#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nmea_parser.h"
#include "ssd1306.h"
#include "font8x8_basic.h"

static const char *GPS = "Node Coordinates";
static const char *DISTANCE = "Distance";
static const char *tag = "SSD1306";

#define TIME_ZONE (+8)   // Beijing Time
#define YEAR_BASE (2000) // date in GPS starts from 2000
#define EARTH_RADIUS 6371

#define SDA_GPIO 21
#define SCL_GPIO 22
#define RESET_GPIO -1 // Update with the appropriate GPIO pin if used

SSD1306_t dev;
QueueHandle_t Q1;

double hav(double lat1, double lon1, double lat2, double lon2)
{
    // Convert the Geographic Location from degree to Radian.
    lon1 = (lon1 * M_PI) / 180;
    lon2 = (lon2 * M_PI) / 180;
    lat1 = (lat1 * M_PI) / 180;
    lat2 = (lat2 * M_PI) / 180;

    // Geographical Location Differences
    double diff_lon = lon2 - lon1;
    double diff_lat = lat2 - lat1;

    double sin2_lat = sin(diff_lat / 2) * sin(diff_lat / 2);
    double sin2_lon = sin(diff_lon / 2) * sin(diff_lon / 2);

    double a = sin2_lat + cos(lat1) * cos(lat2) * sin2_lon;

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    double distance = c * EARTH_RADIUS;

    return distance;
}

static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    float coordinate[3];
    gps_t *gps = NULL;
    switch (event_id)
    {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        /* print information parsed from GPS statements */
        ESP_LOGI(GPS, "%d/%d/%d %d:%d:%d => \r\n"
                      "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                      "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                      "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                      "\t\t\t\t\t\tspeed      = %fm/s",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);
        coordinate[0] = gps->latitude;
        coordinate[1] = gps->longitude;
        coordinate[2] = hav(11.570665, 104.898652, gps->latitude, gps->longitude);
        
        xQueueSend(Q1, &coordinate, portMAX_DELAY);

        ESP_LOGI(DISTANCE, "%.3f km", coordinate[2]);
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(GPS, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

void display_task(void *pvParameter)
{
    float coordinate[3];
    while (1)
    {
        if (xQueueReceive(Q1, &coordinate, portMAX_DELAY) == pdPASS)
        {
            float latitude2 = coordinate[0];
            float longitude2 = coordinate[1];
            float distance = coordinate[2];

            char text_lat[20], text_lon[20], text_d[20];

            snprintf(text_lat, sizeof(text_lat), "Lat: %.5f-N", latitude2);
            snprintf(text_lon, sizeof(text_lon), "Lon: %.5f-E", longitude2);
            snprintf(text_d, sizeof(text_d), "Dis: %.3fkm", distance);

            ssd1306_clear_screen(&dev, false);
            ssd1306_contrast(&dev, 0xff);
            ssd1306_display_text(&dev, 2, text_lat, strlen(text_lat), false);
            ssd1306_display_text(&dev, 4, text_lon, strlen(text_lon), false);
            ssd1306_display_text(&dev, 6, text_d, strlen(text_d), false);

            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}

void app_main(void)
{
    Q1 = xQueueCreate(10, sizeof(float) * 3);

    ESP_LOGI(tag, "INTERFACE is i2c");
    ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d", SDA_GPIO);
    ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d", SCL_GPIO);
    ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d", RESET_GPIO);
    i2c_master_init(&dev, SDA_GPIO, SCL_GPIO, RESET_GPIO);
    ESP_LOGI(tag, "Panel is 128x64");
    ssd1306_init(&dev, 128, 64);

    /* NMEA parser configuration */
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    /* init NMEA parser library */
    nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
    /* register event handler for NMEA parser library */
    nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

    /* Create display task */
    xTaskCreate(&display_task, "display_task", 2048, NULL, 5, NULL);

    /* Allow the main task to run for some time */
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    /* unregister event handler */
    // nmea_parser_remove_handler(nmea_hdl, gps_event_handler);
    // /* deinit NMEA parser library */
    // nmea_parser_deinit(nmea_hdl);
}
