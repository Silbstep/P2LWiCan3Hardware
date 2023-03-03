#ifndef __Global_h
#define __Global_h

//Define Inputs
#define KEY1 (gpio_num_t)3
#define KEY2 (gpio_num_t)7
#define BKEY1 BIT3
#define BKEY2 BIT7
//Define Outputs
#define Buzzer (gpio_num_t) 2
#define LEDG (gpio_num_t)0
#define LEDB (gpio_num_t)1
#define LEDR (gpio_num_t)10
#define CLK (gpio_num_t)5
#define OE (gpio_num_t)6
#define LE (gpio_num_t)4
#define SDO (gpio_num_t)7
#define BLEDG BIT0
#define BLEDB BIT1
#define BLEDR BIT10
#define BCLK BIT5
#define BOE BIT6
#define BLE BIT4
#define BSDO BIT7
#define I2C_MASTER_SDA_IO 18
#define I2C_MASTER_SCL_IO 19
//I2C 
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0     //I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0     //I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS 500
//Define timer values
#define TIMER_DIVIDER 10
#define TIMER_VALUE 511                 //interrupt every 63.874uS i.e. 1 / (80000000 / TIMER_DIVIDER / TIMER_VALUE). Default clock source is APB (80MHz)
//LED and 7 segment
#define SEGMENTS 4						//Number of 7 segment led's
#define FLASHCOUNT 107					//0.5s
#define POWERUPTIMEOUT 488				//2s
//Buzzer
#define BUZZERCOUNT 700
//Notes
#define A7 3520
#define B7 3951
#define C7 2093
#define D7 2349
#define E7 2637
#define F7 2794
#define G7 3136
//LEDC (used for buzzer)
#define LEDC_TIMER              LEDC_TIMER_1
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define BUZZER                  (2) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (2730) // Frequency in Hertz. 
//Circular buffer sizes
#define ROOTBUFFERSIZE 900
#define NODEBUFFERSIZE 900
#define ACTIONBUFFERSIZE 80
//Mode definitions
#define CAN false
#define WIFI true
//States
#define LOW 0
#define HIGH 1

#endif