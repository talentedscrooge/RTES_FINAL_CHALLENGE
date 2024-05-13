#include <mbed.h>
#include "arm_math.h"  // Include CMSIS DSP library for FFT functions
#include "stm32f4xx_hal.h"  // Include the correct header file for your STM32 series
#include "drivers/LCD_DISCO_F429ZI.h" 
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28 | 0x80

EventFlags flags;


void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

// FFT specific definitions
#define FFT_SIZE 64
#define SAMPLE_FREQUENCY 100  // Adjust based on your actual sampling rate

float32_t fft_input[FFT_SIZE]; // Complex buffer: real and imaginary parts
float32_t fft_output[FFT_SIZE / 2 + 1];    // output buffer
float32_t magnitudes[FFT_SIZE / 2 + 1];    // STORES THE MAGNITUDES
arm_rfft_fast_instance_f32 s;

void draw_smile() {
    LCD_DISCO_F429ZI lcd;

    uint8_t spacing = 100;

    // Draw the left eye (white part)
    lcd.FillCircle(70 + 0 * spacing, LINE(5), 25);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.FillCircle(75 + 0 * spacing, LINE(5), 5);

    // Draw the right eye (white part)
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.FillCircle(70 + 1 * spacing, LINE(5), 25);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.FillCircle(65 + 1 * spacing, LINE(5), 5);

    // Draw the smile (red part)
    lcd.SetTextColor(LCD_COLOR_RED);
    Point smile[] = {{70, LINE(10)}, {120, LINE(15)}, {170, LINE(10)}};
    lcd.FillPolygon(smile, 3);
}
    
void draw_exclaim() {
    LCD_DISCO_F429ZI lcd;

    uint8_t spacing = 10;
    lcd.SetTextColor(LCD_COLOR_RED);
    lcd.FillRect(10 * spacing, LINE(4), 40, 100);
    lcd.FillCircle(12 * spacing, LINE(12), 20);
    lcd.SetTextColor(LCD_COLOR_RED);
    //lcd.SetBackColor(LCD_COLOR_YELLOW);
}

float32_t manualFFT(float32_t fft_input[FFT_SIZE])
{
    float32_t output = 0;
    printf("output 0 - %f\n", output);
    int i, j;
    for(i = 0; i < FFT_SIZE; i++)
    {
        for(j = 0; j < FFT_SIZE; j++)
        {
            if(i % 2 == 0)
            {
                if(j % 2 == 0)
                {
                    output += fft_input[i] * fft_output[j];
                }
                else
                {
                    output -= fft_input[i] * fft_output[j];
                }
            }
            else
            {
                if(j % 2 == 0)
                {
                    output -= fft_input[i] * fft_output[j];
                }
                else
                {
                    output += fft_input[i] * fft_output[j];
                }
            }
        }
    }
    printf("%f\n", output);
    return output;
}

int main()
{
    printf("main start\n");
    // draw_exclaim();
    draw_smile();
    // Buffers for sending and receiving data over SPI.
    uint8_t write_buf[32], read_buf[32];

    // Initialize the SPI object with specific pins.
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    // Configure SPI format and frequency.
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Configure CTRL_REG1 register.
    //printf("debug:about to configure ctrl_reg1.\n");
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);
    //printf("debug:ctrl_reg1 configured.\n");

    //printf("debug:about to configure ctrl_reg4.\n");
    // Configure CTRL_REG4 register.
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);
    //printf("debug:ctrl_reg1 configured.\n");
   // Initialize FFT
    arm_rfft_fast_init_f32(&s, FFT_SIZE);

    uint32_t sample_count = 0;
    uint32_t fft_index = 0;

    while(1){
        // Define variables to store raw and scaled gyroscope data
        uint16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;
    
        // printf("loop %d start\n", fft_index);

        // Prepare to read the gyroscope values starting from OUT_X_L
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        // Perform the SPI transfer to read 6 bytes of data (for x, y, and z axes)
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Convert the received data into 16-bit integers for each axis
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t) read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t) read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t) read_buf[5]);

        // Print the raw values for debugging 
        // printf("RAW -> \t\tgx: %d \t gy: %d \t gz: %d \t\n", raw_gx, raw_gy, raw_gz);

        // Convert raw data to actual values using a scaling factor
        gx = ((float) raw_gx) * SCALING_FACTOR;
        gy = ((float) raw_gy) * SCALING_FACTOR;
        gz = ((float) raw_gz) * SCALING_FACTOR;

        // Print the actual values
        // printf("Actual -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", gx, gy, gz);

        // Use one of the axes (e.g., gx) for FFT analysis
        fft_input[++fft_index] = sqrt(gx * gx + gy * gy + gz * gz); // Real part
        printf("fft ip - %f\n", fft_input[fft_index]);-
        // //draw_smile();

        sample_count++;
        // Reset for next FFT processing
        if(sample_count >= FFT_SIZE){
            printf("b4 fft\n");
            // // Perform FFT on the collected data
            // arm_rfft_fast_f32(&s, fft_input, fft_output, 0);

            float32_t frequency = manualFFT(fft_input);

            printf("fft init\n");

            // // Compute magnitude of the FFT results
            // for (int i = 0; i <= FFT_SIZE / 2; i++) {
            //     float real = fft_output[2*i];
            //     float imag = fft_output[2*i +1];
            //     float magn_squared = real*real + imag*imag;
            //     arm_sqrt_f32(magn_squared, &magnitudes[i]);
            //     magnitudes[i] /= FFT_SIZE; 

            // }

            // printf("got mag\n");

            // // Determine frequency resolution
            // float frequency_resolution = (float)SAMPLE_FREQUENCY / (float)FFT_SIZE;

            // printf("freq res\n");

            // for (int i = 0; i <= FFT_SIZE / 2; i++) {
            //     float frequency = i * frequency_resolution;
            //     // Process the frequencies of interest (3 Hz to 6 Hz)
            //     if (frequency >= 3.0f && frequency <= 6.0f)
            //     {
            //         if (magnitudes[i] > 0.5)
            //         {
            //             draw_exclaim();
            //         }
            //     }
            //     else{
            //         draw_smile();
            //     }
            // }    

        

            if (frequency >= 3.0f && frequency <= 6.0f)
            {
                draw_exclaim();
            }
            else if (frequency <= -3.0f && frequency >= -6.0f)
            {
                draw_exclaim();
            }
            else
            {
                draw_smile();
            }

            printf("drawing done - freq = %f\n", frequency);

            sample_count = 0;
            fft_index = 0;
        }
        
        thread_sleep_for(1000 / SAMPLE_FREQUENCY);
    }
}
