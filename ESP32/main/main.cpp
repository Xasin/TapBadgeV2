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
#include <array>

#include <xnm/neocontroller.h>

#include <xasin/audio.h>
#include <xasin/TrekAudio.h>

#include <MasterAction.h>
#include <xasin/xai2c/DRV2605.h>

#include "click_a.h"
#include "click_b.h"

Xasin::I2C::DRV2605 vibro;

XNM::Neo::NeoController test(GPIO_NUM_25, RMT_CHANNEL_0, 16);
Xasin::Audio::TX audio_out(I2S_NUM_0);

std::array<XNM::Neo::IndicatorBulb, 16> t_bulbs = {};

void update_pixels() {
    static TickType_t next_switch = 0;

    if(next_switch < xTaskGetTickCount())
        next_switch = xTaskGetTickCount();

    for(int i=0; i<7; i++) {
        auto c = test.colors[i];

        if(xTaskGetTickCount() >= next_switch) {
            auto l = t_bulbs[i].switch_tick();
            if(l) {
                vibro.trig_sequence(2);

                auto cassette = cassette_click_a;
                if(l > 0)
                    cassette = cassette_click_b;
                
                auto tmp = new Xasin::Audio::ByteCassette(audio_out, cassette.data_start, 
                    cassette.data_end, 
                    cassette.data_samplerate * (0.995F + 0.05F * (esp_random() % 1024) / 1024.0F));
                
                tmp->volume = 10;
                tmp->start(true);

                next_switch += 40/portTICK_PERIOD_MS;
            }
        }
        auto bulb_c = t_bulbs[i].color_tick();
        bulb_c.bMod(50);

        c = 0;
        c.merge_overlay(bulb_c);

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

    XaI2C::MasterAction::init(GPIO_NUM_21, GPIO_NUM_14);
	gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_4, false);

    vTaskDelay(200/portTICK_PERIOD_MS);

    vibro.autocalibrate_lra();
    vibro.sequence_mode();

    test.colors.fill(0);
    test.update();

    for(auto &bulb : t_bulbs) {
        bulb.set(Material::GREEN, XNM::Neo::IDLE);
    }

    t_bulbs[1].set(Material::AMBER, XNM::Neo::FLASH);
    t_bulbs[2].set(Material::RED, XNM::Neo::DFLASH);

    t_bulbs[5].set(Material::CYAN, XNM::Neo::VAL_RISING);

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

    Xasin::Trek::init(audio_out);

    vTaskDelay(100);

    Xasin::Trek::play(Xasin::Trek::PROG_DONE);

    while(true) {
        vTaskDelay(40/portTICK_PERIOD_MS);

        update_pixels();
    }

    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
