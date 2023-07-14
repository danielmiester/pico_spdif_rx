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
#include "hardware/interp.h"
#include "string.h"
#include "math.h"
#include "pico/divider.h"

static constexpr uint8_t PIN_DCDC_PSM_CTRL = 23;
static constexpr uint8_t PIN_PICO_SPDIF_RX_DATA = 15;
static constexpr uint8_t PIN_DEBUG = 4;
static constexpr uint8_t PIN_DEBUG2 = 5;
volatile static bool stable_flg = false;
volatile static bool lost_stable_flg = false;
volatile static bool wide_mode = false;
static constexpr uint8_t Z = 0b1111;
static constexpr uint8_t Y = 0b0111;
static constexpr uint8_t X = 0b1011;

void on_stable_func(spdif_rx_samp_freq_t samp_freq)
{
    // callback function should be returned as quick as possible
    stable_flg = true;
}

void on_lost_stable_func()
{
    // callback function should be returned as quick as possible
    lost_stable_flg = true;
    wide_mode = false;
}
void core1_entry();
void init()
{
    stdio_init_all();
    printf("\n$\t*** Core 0 Running  ***\n\n");
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
        .flags = SPDIF_RX_FLAGS_ALL};

    spdif_rx_start(&config);
    spdif_rx_set_callback_on_stable(on_stable_func);
    spdif_rx_set_callback_on_lost_stable(on_lost_stable_func);

    gpio_init_mask(0b1110000);
    gpio_set_dir_out_masked(0b1110000);
    gpio_put(PIN_DEBUG2, 0);
}

int main()
{
    init();

    multicore_launch_core1(core1_entry);

    int count = 0;
    while (true)
    {
        if (stable_flg)
        {
            stable_flg = false;
            printf("\n$detected stable sync\n");
        }
        if (lost_stable_flg)
        {
            lost_stable_flg = false;
            printf("\n$lost stable sync. waiting for signal\n");
        }
        if (count % 100 == 0 && spdif_rx_get_state() == SPDIF_RX_STATE_STABLE)
        {
            spdif_rx_samp_freq_t samp_freq = spdif_rx_get_samp_freq();
            float samp_freq_actual = spdif_rx_get_samp_freq_actual();
            static uint8_t c_bits[8];
            spdif_rx_get_c_bits(&c_bits, 8, 0);
            printf("\n$% 12.12d Samp Freq = %d Hz \n", time_us_32(), (int)samp_freq);
            // printf("c_bits = %02x %02x %02x %02x %02x %02x %02x %02x\n",c_bits[0],c_bits[1],c_bits[2],c_bits[3],c_bits[4],c_bits[5],c_bits[6],c_bits[7]);
            // printf("parity errors = %d\n", spdif_rx_get_parity_err_count());
        }
        tight_loop_contents();
        sleep_ms(10);
        count++;
    }

    return 0;
}
static constexpr int CLK = 2;
static constexpr int DAT = 3;
static constexpr int iomask = 1 << CLK | 1 << DAT;
static constexpr int delay = 10;
#define NS_TO_CYCLES(X) ((X)*SYS_CLK_MHZ / 1000)

void set_clk()
{
    gpio_put(CLK, 1);
    busy_wait_at_least_cycles(NS_TO_CYCLES(100));
}
void clr_clk()
{
    gpio_put(CLK, 0);
    busy_wait_at_least_cycles(NS_TO_CYCLES(100));
}
static void inline toggle_clk()
{
    set_clk();
    busy_wait_at_least_cycles(NS_TO_CYCLES(300));
    clr_clk();
    busy_wait_at_least_cycles(NS_TO_CYCLES(200));
}
static void inline set_dat(bool val = true)
{
    gpio_put(DAT, val);
    busy_wait_at_least_cycles(NS_TO_CYCLES(100));
}
static void inline clr_dat()
{
    gpio_put(DAT, 0);
    busy_wait_at_least_cycles(NS_TO_CYCLES(100));
}
void _write_display_byte(char byte)
{
    for (char i = 0; i < 8; i++)
    {
        set_dat(((byte >> i) & 1));
        toggle_clk();
    }
}

void write_display(char command, char *data, size_t len)
{
    clr_dat();
    clr_clk();
    _write_display_byte(command);
    for (char *d = data; d < data + len; d++)
    {
        _write_display_byte(*d);
    }
    clr_dat();
    set_clk();
    set_dat();
    sleep_us(5 * delay);
}
static char digits[16] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
void init_display()
{
    gpio_init_mask(iomask);
    gpio_set_drive_strength(CLK, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(DAT, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_dir_out_masked(iomask);
    gpio_set_mask(iomask);
    sleep_ms(150);
    write_display(0x40, nullptr, 0);
    write_display(0xc0, digits, 16);
    write_display(0x88, nullptr, 0);
}

void update_display()
{
    write_display(0xc0, digits, 16);
}
uint32_t inline unpad(uint32_t raw)
{
    // interp0->accum[0] = raw;
    // return abs((int32_t)interp0.peek[0]);
    return abs((int32_t)raw << 4 >> 8);

}
volatile uint32_t samples[SPDIF_BLOCK_SIZE / 2][2];
volatile uint64_t peak[2];
volatile uint64_t avg[2];

void build_bar(uint8_t l, uint8_t r)
{
    uint32_t pl = peak[0] >> 18;
    uint32_t pr = peak[1] >> 18;
    for (int i = 0; i < 16; i++)
    {
        uint8_t bits = 0;
        int8_t ll = l - 2 * i;
        if (ll <= 0)
        {
            // do nothing
        }
        else if (ll == 1)
        {
            bits |= 0b00100000;
        }
        else
        {
            bits |= 0b00100010;
        }
        int8_t rr = r - 2 * i;
        if (rr <= 0)
        {
            // do nothing
        }
        else if (rr == 1)
        {
            bits |= 0b00010000;
        }
        else
        {
            bits |= 0b00010100;
        }
        // digits[i] = 0;
        digits[i] = bits;
    }
    digits[pl/2] |= pl%2? 0b00000010:0b00100000;
    digits[pr/2] |= !pr%2? 0b00010000:0b00000100;
}
void core1_entry()
{
    constexpr uint32_t SPDIF_SAMPLE_MASK = 0x0FFFFFF0;
    printf("\n*\t*** Core 1 started***\n\n");
    init_display();
    uint32_t t = 0;
    uint32_t tt;

    // Initialise lane 0 on interp0 on this core
    interp0->accum[0] = 0;
    interp0->base[0] = 0;
    interp0->ctrl[0] = SIO_INTERP0_CTRL_LANE0_RESET;                 // initialize
    interp0->ctrl[0] |= (4 << SIO_INTERP0_CTRL_LANE0_SHIFT_LSB);     // Set to right-shift by 4
    interp0->ctrl[0] |= (0 << SIO_INTERP0_CTRL_LANE0_MASK_LSB_LSB);  // choose digits 0...
    interp0->ctrl[0] |= (23 << SIO_INTERP0_CTRL_LANE0_MASK_MSB_LSB); // through 23 inclusive
    interp0->ctrl[0] |= (1 << SIO_INTERP0_CTRL_LANE0_SIGNED_LSB);    // enable sign-extention;
    sleep_ms(500);
    static uint32_t frames = 0;
    constexpr uint32_t peak_decay = 64;
    while (1)
    {
        // int32_t samples[192][2];
        // int32_t peak[2];
        // get_latest_samples(&samples);
        // get_peak(&samples,&peak);
        // display(peak);
        tt = time_us_32();
        if (t < tt)
        {
            t = tt + 1e6 / 30;
            frames++;
            uint8_t l = avg[0] >>17;
            uint8_t r = avg[1] >>17;
            build_bar(l, r);
            update_display();
            peak[0] = (peak[0] * (peak_decay - 1) + avg[0]) / peak_decay;
            peak[1] = (peak[1] * (peak_decay - 1) + avg[1]) / peak_decay;
            if(frames %10 == 0){
                printf("\n*s[0]:%06.6x avg[0]:%06.6x peak[0]:%06.6x",samples[0][0],avg[0],peak[0]);
            }
            // printf("\n* buffer count:%d ",spdif_rx_get_fifo_count());
            // printf("sample: % 8ld  %06.6lx\t",unpad(samples[0]),interp0->peek[0]);
            
        }
        else
        {
            sleep_ms(1);
        }
    }
}
extern "C"
{
    void spdif_rx_callback_func(uint32_t *buff, uint32_t sub_frame_count, uint8_t c_bits[SPDIF_BLOCK_SIZE / 16], bool parity_err)
    {
        static constexpr uint32_t avg_decay = 512;
        gpio_put(PIN_DEBUG, 1);
        volatile uint_fast8_t lr;
        volatile uint_fast32_t n;
        volatile int_fast32_t val;
        for (int i = 0; i < sub_frame_count; i++)
        {
            lr = i & 1;
            n = i >> 1;
            val = unpad(buff[i]);
            samples[n][lr] = val;

            avg[lr] = (avg[lr] * (avg_decay - 1) + val) / avg_decay;
            peak[lr] = MAX(peak[lr], val);
            
        }

        // memcpy(&samples, buff,sizeof(uint32_t)*sub_frame_count);
        //  printf("*");
        gpio_put(PIN_DEBUG, 0);
        return;
    }
}