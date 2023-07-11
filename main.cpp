/*------------------------------------------------------/
/ Copyright (c) 2023, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "spdif_rx.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "pico/rand.h"
#include "hardware/timer.h"

static constexpr uint8_t PIN_DCDC_PSM_CTRL = 23;
static constexpr uint8_t PIN_PICO_SPDIF_RX_DATA = 15;
volatile static bool stable_flg = false;
volatile static bool lost_stable_flg = false;


void on_stable_func(spdif_rx_samp_freq_t samp_freq)
{
    // callback function should be returned as quick as possible
    stable_flg = true;
}

void on_lost_stable_func()
{
    // callback function should be returned as quick as possible
    lost_stable_flg = true;
}
void core1_entry();
void init(){
    stdio_init_all();

    // DCDC PSM control
    // 0: PFM mode (best efficiency)
    // 1: PWM mode (improved ripple)
    gpio_init(PIN_DCDC_PSM_CTRL);
    gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
    gpio_put(PIN_DCDC_PSM_CTRL, 1); // PWM mode for less Audio noise

    spdif_rx_config_t config = {
        .data_pin = PIN_PICO_SPDIF_RX_DATA,
        .pio_sm = 0,
        .dma_channel0 = 0,
        .dma_channel1 = 1,
        .alarm = 0,
        .flags = SPDIF_RX_FLAGS_ALL
    };

    spdif_rx_start(&config);
    spdif_rx_set_callback_on_stable(on_stable_func);
    spdif_rx_set_callback_on_lost_stable(on_lost_stable_func);
}

int main()
{
    init();
    
    multicore_launch_core1(core1_entry);
    

    int count = 0;
    while (true) {
        if (stable_flg) {
            stable_flg = false;
            printf("detected stable sync\n");
        }
        if (lost_stable_flg) {
            lost_stable_flg = false;
            printf("lost stable sync. waiting for signal\n");
        }
        if (count % 100 == 0 && spdif_rx_get_state() == SPDIF_RX_STATE_STABLE) {
            spdif_rx_samp_freq_t samp_freq = spdif_rx_get_samp_freq();
            float samp_freq_actual = spdif_rx_get_samp_freq_actual();
            static uint32_t c_bits;
            spdif_rx_get_c_bits(&c_bits, sizeof(c_bits), 0);
            printf("%08d Samp Freq = %d Hz (%7.4f KHz)\n",time_us_32(), (int) samp_freq, samp_freq_actual / 1e3);
            printf("c_bits = %08x\n",c_bits);
            printf("parity errors = %d\n", spdif_rx_get_parity_err_count());
        }
        tight_loop_contents();
        sleep_ms(10);
        count++;
    }

    return 0;
}
static constexpr int CLK = 2;
static constexpr int DAT = 3;
static constexpr int iomask = 1<<CLK | 1 << DAT;
void _write_display_byte(char byte){
    for(char i = 0; i < 8;i++){
        gpio_put(DAT,(byte >> i) & 1 );
        sleep_us(1);
        gpio_put(CLK, 1);
        sleep_us(1);
        gpio_put(CLK, 0);
        sleep_us(1);
    }
}

void write_display(char command, char* data, size_t len){
    gpio_put_masked(iomask,iomask);
    sleep_us(1);
    gpio_put(DAT,0);
    sleep_us(1);
    gpio_put(CLK,0);
    sleep_us(1);
    _write_display_byte(command);
    for(char* d = data; d < data+len;d++){
        _write_display_byte(*d);
    }
    sleep_us(1);
    gpio_put(CLK,1);
    sleep_us(1);
    gpio_put(DAT,1);
    sleep_us(1);
}
void init_display(){
    static union{
        rng_128_t _128;
        uint64_t _64[2];
        char _8[16];
        } digits;
    get_rand_128(&digits._128);
    gpio_init_mask(iomask);
    gpio_set_dir_out_masked(iomask);
    gpio_set_mask(iomask);
    sleep_ms(150);
    write_display(0x40,nullptr,0);
    write_display(0xc0,digits._8,16);
    write_display(0x8a,nullptr,0);



}
void core1_entry(){
    printf("\n\n\t***Core 1 started***\n\n");
    init_display();
    uint64_t t = 0;
    while(1){
        // int32_t samples[192][2];
        // int32_t peak[2];
        // get_latest_samples(&samples);
        // get_peak(&samples,&peak);
        // display(peak);
        if(t < time_us_64()){
            t = time_us_64() + 1e6;

            init_display();
            printf("**1**"); 
        }else{
            sleep_ms(10);
        }
        
    }
}
