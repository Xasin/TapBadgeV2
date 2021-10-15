/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include <cmath>

#include <xasin/neocontroller.h>

#include <xasin/audio.h>
#include <xasin/TrekAudio.h>

Xasin::NeoController::NeoController test(GPIO_NUM_25, RMT_CHANNEL_0, 16);
Xasin::Audio::TX audio_out(I2S_NUM_0);

void update_pixels() {
    for(int i=0; i<7; i++) {
        auto c = test.colors[i];

        test.colors[i].r = c.g;
        test.colors[i].g = c.r;
    }

    test.update();
}

void audio_processing_loop(void *args) {
    while (true)
    {
        xTaskNotifyWait(0, 0, nullptr, portMAX_DELAY);

        audio_out.largestack_process();
    }
}

extern "C"
void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    test.colors.fill(0);
    test.update();

	gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_NUM_4, true);

    vTaskDelay(10);

    test.update();



    TaskHandle_t processing_handle;
	xTaskCreatePinnedToCore(audio_processing_loop, "Audio", 30000, nullptr, 10, &processing_handle, 1);

    i2s_pin_config_t speaker_tx_cfg = {
        GPIO_NUM_19,
        GPIO_NUM_26,
        GPIO_NUM_22,
        -1
    };
    audio_out.init(processing_handle, speaker_tx_cfg);
    audio_out.calculate_volume = true;

    audio_out.volume_mod = 255;

    Xasin::Trek::init(audio_out);

    vTaskDelay(100);
    Xasin::Trek::play(Xasin::Trek::PROG_DONE);


    TickType_t last_beep = 0;
    for (int i = 10000; i >= 0; i--) {
        vTaskDelay(4);

        for(int j = 0; j<16; j++)
            test.colors[j] = Xasin::NeoController::Color::HSV(xTaskGetTickCount()/5 + 360/16 * j, 255, 50);

        update_pixels();

        if(xTaskGetTickCount() > last_beep) {
            Xasin::Trek::play(Xasin::Trek::INPUT_REQ);
            last_beep += 5000/portTICK_PERIOD_MS;
        }
    }

    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
