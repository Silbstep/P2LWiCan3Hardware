// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mdf_common.h"
#include "mwifi.h"
#include "driver/uart.h"
#include "Global.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

// #define MEMORY_DEBUG
#define BUF_SIZE (1024)

bool DataAvailable = false;

static int g_sockfd    = -1;
static const char *TAG = "P2LWiCan";
static esp_netif_t *netif_sta = NULL;

//Create handlers
TaskHandle_t LedFlashRateTHandle = NULL;
TaskHandle_t LedFlashPeriodTHandle = NULL;
TaskHandle_t BuzzerTHandle = NULL;
TaskHandle_t InputsTHandle = NULL;
QueueHandle_t NodeWriteQHandle = NULL;
QueueHandle_t BuzzerQHandle = NULL;

//Constants
const uint16_t BuzzerFrequency1[8] = {2730, 2730, 2730, 2730, 2730, 2730, 2730, 2730};
const uint16_t BuzzerFrequency2[8] = {A7, C7, E7, C7, A7, C7, D7, C7};
const uint16_t BuzzerFrequency3[8] = {C7, B7, C7, B7, C7, B7, C7, B7};

//Global variables
//Keypad
uint8_t KeyPressed = 0;
//Buzzer
bool BuzzerContinous = false;
bool BuzzerSound = false;
uint8_t BuzzerTone = 0;
uint8_t BuzzerFreq = 0;
uint8_t FreqCount = 0;
uint16_t BuzzerCount = 0;
//LED
volatile bool WriteToLedBuzz = false;
bool FlashLedActive = false;
bool FlashLedOn = false;
uint8_t Blue = 0;
uint8_t Green = 0;
uint8_t Red = 0;
uint8_t BlueTemp, GreenTemp, RedTemp;
uint16_t PowerUpTimeout = POWERUPTIMEOUT;
typedef struct LEDFlash
{
	int DelayOn;
    int DelayOff;
    int Period;
}LEDFlash;
volatile uint16_t Led;
uint16_t LedSwitch = 0;
//7 Segments
uint8_t Seg[4] = {0x00, 0x00, 0x00, 0x00};              //Value that is written to segments
uint8_t SegValue[4] = {0x00, 0x00, 0x00, 0x00};         //Value that is received, still needs to be converted
//Inputs
typedef struct Inputs
{
    uint8_t DIPAddress;
    uint8_t DIPOptions;
    bool Key1;
    bool Key2;
    bool Key3;
    bool Key4;
}Inputs;
//Reply string
typedef struct Reply
{
    char ResponseType;
    char Data[7];
}Reply;
//Address
uint8_t Address = 0;
//Circular buffers
uint8_t RootOut[ROOTBUFFERSIZE];                        //Each packet is 9 bytes so can contain 100 messages
uint16_t RootOutTail = 0;
uint16_t RootOutHead = 0;
uint8_t NodeOut[NODEBUFFERSIZE];                        //Each packet is 9 bytes so can contain 100 messages
uint16_t NodeOutTail = 0;
uint16_t NodeOutHead = 0;
uint8_t Action[ACTIONBUFFERSIZE];                       //Each packet is 8 bytes so can contain 10 messages
uint16_t ActionTail = 0;
uint16_t ActionHead = 0;
//Mode
bool Mode = WIFI;
//MACAddress (will be stored in FRAM)
uint8_t MACAddress[1536];

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//I2C
esp_err_t i2c_init(int i2c_number)
{
    int ret = 0;
    i2c_config_t conf = 
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_number, &conf);
    ret = i2c_driver_install(i2c_number, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    return ret;
}

esp_err_t i2c_read(i2c_port_t i2c_number, uint8_t i2c_address, const uint8_t *reg_address, size_t reg_size, uint8_t *read_buffer, size_t read_size)
{
    int ret = 0;

    ret = i2c_master_write_read_device(i2c_number, i2c_address, &reg_address, reg_size, read_buffer, read_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}
/*
esp_err_t i2c_current_read(i2c_port_t i2c_number, uint8_t i2c_address, uint8_t *read_buffer, size_t read_size)
{
    int ret = 0;

    ret = i2c_master_read_from_device(i2c_number, i2c_address, &read_buffer, read_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

esp_err_t i2c_write(i2c_port_t i2c_number, uint8_t i2c_address, uint8_t *write_buffer, size_t write_size)
{
    int ret = 0;

    ret = i2c_master_write_to_device(i2c_number, i2c_address, &write_buffer, write_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}
*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Convert ascii to seven segment value
//Codes as per Wikipedia https://en.wikipedia.org/wiki/Seven-segment_display_character_representations
//From schematic a - 10000000, f - 01000000, b - 00100000, e - 00010000, g - 00001000, c - 00000100, d - 00000010, dp - 00000001
//or g - 00001000, f - 01000000, e - 00010000, d - 00000010, c - 00000100, b - 00100000, a - 10000000, dp - 00000001
uint8_t AsciiToSeven(uint8_t Data)
{
    uint8_t ReturnValue;

    switch(Data)
    {
    case 'a':	ReturnValue = 0x0E;     //g, d, c
    			break;
    case 'b':	ReturnValue = 0x5E;     //g, f, e, d, c
    			break;
    case 'c':   ReturnValue = 0x1A;     //g, e, d
                break;
    case 'd':   ReturnValue = 0x3E;     //g, e, d, c, b
                break;
    case 'e':	ReturnValue = 0x12;     //e, d
    			break;
    case 'f':	ReturnValue = 0x58;     //g, f, e
    			break;
    case 'g':	ReturnValue = 0x9A;     //g, e, d, a
    			break;
    case 'h':   ReturnValue = 0x5C;     //g, f, e, c
                break;
    case 'i':	ReturnValue = 0x92;     //e, d, a
    			break;
    case 'j':	ReturnValue = 0x86;     //d, c, a
    			break;
    case 'k':	ReturnValue = 0xCA;     //g, f, d, a
    			break;
    case 'l':   ReturnValue = 0x50;     //f, e
                break;
    case 'm':	ReturnValue = 0x9C;     //g, e, c, a
    			break;
    case 'n':	ReturnValue = 0x1C;     //g, e, c
    			break;
    case 'o':   ReturnValue = 0x1E;     //g, e, d, c
                break;
    case 'p':	ReturnValue = 0xF8;     //g, f, e, b, a
    			break;
    case 'q':	ReturnValue = 0xEC;     //g, f, c, b, a
    			break;
    case 'r':	ReturnValue = 0x18;     //g, e
    			break;
    case 's':	ReturnValue = 0x06;     //d, c
    			break;
    case 't':   ReturnValue = 0x5A;     //g, f, e, d
                break;
    case 'u':   ReturnValue = 0x16;     //e, d, c
                break;
    case 'v':	ReturnValue = 0x06;     //d, c
    			break;
    case 'w':	ReturnValue = 0x62;     //f, d, b
    			break;
    case 'x':	ReturnValue = 0x0A;     //g, d
    			break;
    case 'y':	ReturnValue = 0x6E;     //g, f, d, c, b
    			break;
    case 'z':	ReturnValue = 0x0A;     //g, d
    			break;
    case 'A':   ReturnValue = 0xFC;     //g, f, e, c, b, a
                break;
    case 'B':	ReturnValue = 0xFE;
    			break;
    case 'C':   ReturnValue = 0xD2;
                break;
    case 'D':	ReturnValue = 0xB6;
    			break;
    case 'E':	ReturnValue = 0xDA;
    			break;
    case 'F':   ReturnValue = 0xD8;
                break;
    case 'G':	ReturnValue = 0xDE;
    			break;
    case 'H':   ReturnValue = 0x7C;
                break;
    case 'I':	ReturnValue = 0x50;
    			break;
    case 'J':   ReturnValue = 0x26;
                break;
    case 'K':	ReturnValue = 0xDC;
    			break;
    case 'L':   ReturnValue = 0x52;
                break;
    case 'M':	ReturnValue = 0xE2;
    			break;
    case 'N':   ReturnValue = 0xF4;
                break;
    case 'O':	ReturnValue = 0xF6;
    			break;
    case 'P':   ReturnValue = 0xF8;
                break;
    case 'Q':	ReturnValue = 0xEA;
    			break;
    case 'R':	ReturnValue = 0xFA;
    			break;
    case 'S':	ReturnValue = 0xCE;
    			break;
    case 'T':	ReturnValue = 0xD0;
    			break;
    case 'U':	ReturnValue = 0x76;
    			break;
    case 'V':	ReturnValue = 0x66;
    			break;
    case 'W':	ReturnValue = 0x96;
    			break;
    case 'X':	ReturnValue = 0x8A;
    			break;
    case 'Y':	ReturnValue = 0x6A;
    			break;
    case 'Z':	ReturnValue = 0x9A;
    			break;
    case '0':   ReturnValue = 0xF6;
                break;
    case '1':   ReturnValue = 0x24;
                break;
    case '2':   ReturnValue = 0xBA;
                break;
    case '3':   ReturnValue = 0xAE;
                break;
    case '4':   ReturnValue = 0x6C;
                break;
    case '5':   ReturnValue = 0xCE;
                break;
    case '6':   ReturnValue = 0xDE;
                break;
    case '7':   ReturnValue = 0xEC;
                break;
    case '8':   ReturnValue = 0xFE;
                break;
    case '9':   ReturnValue = 0xEE;
                break;
    case '"':   ReturnValue = 0x60;
                break;
    case '\\':  ReturnValue = 0x38;
                break;
    case '-':   ReturnValue = 0x08;
                break;
    case '=':   ReturnValue = 0x0A;
                break;
    case '.':   ReturnValue = 0x01;
                break;
    case '_':   ReturnValue = 0x02;
                break;
    case ' ':   ReturnValue = 0x00;
                break;
    default:    ReturnValue = Data;
                break;
    }
    return(ReturnValue);
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Convert binary to seven segment value
//From schematic a - 10000000, f - 01000000, b - 00100000, e - 00010000, g - 00001000, c - 00000100, d - 00000010, dp - 00000001
//or g - 00001000, f - 01000000, e - 00010000, d - 00000010, c - 00000100, b - 00100000, a - 10000000, dp - 00000001
uint8_t BinaryToSeven(uint8_t Data)
{
    uint8_t ReturnValue = 0;

    if((Data & 0x40) != 0)                          //g
        ReturnValue = ReturnValue + 0x08;
    if((Data & 0x20) != 0)                          //f
        ReturnValue = ReturnValue + 0x40;
    if((Data & 0x10) != 0)                          //e
        ReturnValue = ReturnValue + 0x10; 
    if((Data & 0x08) != 0)                          //d
        ReturnValue = ReturnValue + 0x02; 
    if((Data & 0x04) != 0)                          //c
        ReturnValue = ReturnValue + 0x04; 
    if((Data & 0x02) != 0)                          //b
        ReturnValue = ReturnValue + 0x20; 
    if((Data & 0x01) != 0)                          //a
        ReturnValue = ReturnValue + 0x80;
    return(ReturnValue);                                                 
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//GPIO functions
inline void Pin_High(int Pin)
{
    REG_WRITE(GPIO_OUT_W1TS_REG, Pin); 
}

inline void Pin_Low(int Pin)
{
    REG_WRITE(GPIO_OUT_W1TC_REG, Pin); 
}

inline void Pin_Level(int Pin, bool Level)
{
    if(Level != 0)
        REG_WRITE(GPIO_OUT_W1TS_REG, Pin);
    else
        REG_WRITE(GPIO_OUT_W1TC_REG, Pin);
}

inline void Pin_PullUp(gpio_num_t Pin)
{
    gpio_set_pull_mode(Pin, GPIO_PULLUP_ONLY);
}

inline void Pin_PullDown(gpio_num_t Pin)
{
    gpio_set_pull_mode(Pin, GPIO_PULLDOWN_ONLY);    
}

void Pin_Init(gpio_num_t Pin, gpio_mode_t Direction, uint32_t Level)
{
    gpio_reset_pin(Pin);
    gpio_set_direction(Pin, Direction); 
    if(Direction == GPIO_MODE_OUTPUT)
        gpio_set_level(Pin, Level);   
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Create a tcp client
static int socket_tcp_client_create(const char *ip, uint16_t port)
{
    MDF_PARAM_CHECK(ip);

    MDF_LOGI("Create a tcp client, ip: %s, port: %d", ip, port);

    mdf_err_t ret = ESP_OK;
    int sockfd    = -1;
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = inet_addr(ip),
    };

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    MDF_LOGI("Socketfd1: %d", sockfd);
    MDF_ERROR_GOTO(sockfd < 0, ERR_EXIT, "socket create, sockfd: %d", sockfd);

    ret = connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in));
    MDF_ERROR_GOTO(ret < 0, ERR_EXIT, "socket connect, ret: %d, ip: %s, port: %d",
                   ret, ip, port);
    MDF_LOGI("Socketfd2: %d", sockfd);
    return sockfd;

ERR_EXIT:

    if (sockfd != -1) {
        close(sockfd);
    }

    return -1;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void tcp_client_read_task(void *arg)
{
    mdf_err_t ret                     = MDF_OK;
    char *data                        = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size                       = MWIFI_PAYLOAD_LEN;
 
    MDF_LOGI("TCP client read task is running");
    while (mwifi_is_connected()) 
    {
        if (g_sockfd == -1) 
        {
            g_sockfd = socket_tcp_client_create(CONFIG_SERVER_IP, CONFIG_SERVER_PORT);

            if (g_sockfd == -1) 
            {
                vTaskDelay(500 / portTICK_RATE_MS);
                continue;
            }
        }
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = read(g_sockfd, data, size);
        MDF_LOGD("Data from controller, %d, size: %d, data: %d %d %d %d %d %d %d %d %d", g_sockfd, size, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
        if (ret <= 0) 
        {
            MDF_LOGW("<%s> TCP read error", strerror(errno));
            close(g_sockfd);
            g_sockfd = -1;
            continue;
        }
        vTaskSuspendAll();
        if(data[0] == Address)                                                  //Controller wants to send message to root
        {
            for(uint8_t i = 0; i < 8; i++)
            {
                Action[ActionHead++] = data[i + 1];
                if(ActionHead >= ACTIONBUFFERSIZE)
                    ActionHead = ActionHead - ACTIONBUFFERSIZE;
            }
        }
        else                                                                    //Controller wants to send message to node
        {
            for(uint8_t i = 0; i < 9; i++)
            {
                NodeOut[NodeOutHead++] = data[i];
                if(NodeOutHead >= NODEBUFFERSIZE)
                    NodeOutHead = NodeOutHead - NODEBUFFERSIZE;
            }
        }
        xTaskResumeAll();
    }
    MDF_LOGI("TCP client read task is exit");
    close(g_sockfd);
    g_sockfd = -1;
    MDF_FREE(data);
    vTaskDelete(NULL);
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
void tcp_client_write_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    char *data    = MDF_CALLOC(1, MWIFI_PAYLOAD_LEN);
    size_t size   = MWIFI_PAYLOAD_LEN;
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type      = {0x0};

    MDF_LOGI("TCP client write task is running");

    while (mwifi_is_connected()) 
    {
        if (g_sockfd == -1) 
        {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        size = MWIFI_PAYLOAD_LEN - 1;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = mwifi_root_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_read", mdf_err_to_name(ret));

        MDF_LOGD("TCP write, size: %d, data: %s", size, data);
        ret = write(g_sockfd, data, size);
        MDF_ERROR_CONTINUE(ret <= 0, "<%s> TCP write", strerror(errno));
    }

    MDF_LOGI("TCP client write task is exit");

    close(g_sockfd);
    g_sockfd = -1;
    MDF_FREE(data);
    vTaskDelete(NULL);
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
static void root_task(void *arg)
{
    uint8_t i;
    mdf_err_t ret                    = MDF_OK;
    char *data                       = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size                      = 15;
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type      = {0};

    MDF_LOGI("Root is running");
    for (int i = 0;; ++i) 
    {
        if (!mwifi_is_started())
        {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }
        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = mwifi_root_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_read", mdf_err_to_name(ret));
        vTaskSuspendAll();
        for(i = 0; i < 9; i++)
        {
            RootOut[RootOutHead++] = data[6 + i];
            if(RootOutHead >= ROOTBUFFERSIZE)
                RootOutHead = RootOutHead - ROOTBUFFERSIZE;
        }
        //Save MAC address (ToDo - Check for new, duplicates etc.)
        for(i = 0; i < 6; i++)
            MACAddress[data[6] * 6 + i] = src_addr[i];
        xTaskResumeAll();
        MDF_LOGI("Root receive, addr: " MACSTR ", size: %d, data: %d, %d, %d, %d, %d, %d, %d, %d, %d", MAC2STR(src_addr), size, data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14]);

    }
    MDF_LOGW("Root is exit");
    MDF_FREE(data);
    vTaskDelete(NULL);
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
static void node_read_task(void *arg)
{
    mdf_err_t ret                    = MDF_OK;
    char *data                       = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size                      = MWIFI_PAYLOAD_LEN;
    mwifi_data_type_t data_type      = {0x0};
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};

    MDF_LOGI("Note read task is running");
    for (;;) 
    {
        if (!mwifi_is_connected()) 
        {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }
        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_read", mdf_err_to_name(ret));
        MDF_LOGD("Node receive: " MACSTR ", size: %d, data: %d %d %d %d %d %d %d %d", MAC2STR(src_addr), size, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        vTaskSuspendAll();
        for(uint8_t i = 0; i < 8; i++)
        {
            Action[ActionHead++] = data[i];
            if(ActionHead >= ACTIONBUFFERSIZE)
                ActionHead = ActionHead - ACTIONBUFFERSIZE;
        }
        xTaskResumeAll();        
    }

    MDF_LOGW("Note read task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
static void node_write_task(void *p)
{
    uint8_t i;
    struct Reply ReplyNow;
    size_t size                     = 15;
    int count                       = 0;
    char data[15]                   = {0};
    mdf_err_t ret                   = MDF_OK;
    mwifi_data_type_t data_type     = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
    Inputs *InputsNow = (Inputs *) p;

    MDF_LOGI("NODE task is running");

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

    for (;;) 
    {
        if (!mwifi_is_connected()) 
        {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }
        if(xQueueReceive(NodeWriteQHandle, &ReplyNow, 10))
        {
            for(i = 0; i < 6; i++)
                data[i] = sta_mac[i];
            data[6] = Address;
            data[7] = ReplyNow.ResponseType;
            for(i = 0; i < 7; i++)
                data[8 + i] = ReplyNow.Data[i];
            MDF_LOGD("Node send, size: %d, data: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", size, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14]);
            ret = mwifi_write(NULL, &data_type, data, size, true);
            MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100)); 
        }
    }

    MDF_LOGW("NODE task is exit");

    vTaskDelete(NULL);
}

/*
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Timed printing system information
static void print_system_info_timercb(void *timer)
{
    uint8_t primary                 = 0;
    wifi_second_chan_t second       = 0;
    mesh_addr_t parent_bssid        = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
    wifi_sta_list_t wifi_sta_list   = {0x0};

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);
    esp_mesh_get_parent_bssid(&parent_bssid);

    MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, node num: %d, free heap: %u", primary,
             esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr),
             mwifi_get_parent_rssi(), esp_mesh_get_total_node_num(), esp_get_free_heap_size());

    for (int i = 0; i < wifi_sta_list.num; i++) {
        MDF_LOGI("Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }

#ifdef MEMORY_DEBUG

    if (!heap_caps_check_integrity_all(true)) {
        MDF_LOGE("At least one heap is corrupt");
    }

    mdf_mem_print_heap();
    mdf_mem_print_record();
    mdf_mem_print_task();
#endif //< MEMORY_DEBUG 
}*/

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Initialise WiFi
static mdf_err_t wifi_init()
{
    mdf_err_t ret          = nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        MDF_ERROR_ASSERT(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    MDF_ERROR_ASSERT(ret);

    MDF_ERROR_ASSERT(esp_netif_init());
    MDF_ERROR_ASSERT(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));
    MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
    MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
    MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
    MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
    MDF_ERROR_ASSERT(esp_wifi_start());

    return MDF_OK;
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//All module events will be sent to this task in esp-mdf
//@Note:
//     1. Do not block or lengthy operations in the callback function.
//     2. Do not consume a lot of memory in the callback function.
//        The task memory of the callback function is only 4KB.
static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)
{
    MDF_LOGI("event_loop_cb, event: %d", event);

    switch (event) 
    {
        case MDF_EVENT_MWIFI_STARTED:
            MDF_LOGI("MESH is started");
            break;

        case MDF_EVENT_MWIFI_PARENT_CONNECTED:
            MDF_LOGI("Parent is connected on station interface");

            if (esp_mesh_is_root()) 
            {
                esp_netif_dhcpc_start(netif_sta);
            }

            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");
            break;

        case MDF_EVENT_MWIFI_ROUTING_TABLE_ADD:
        case MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE:
            MDF_LOGI("total_num: %d", esp_mesh_get_total_node_num());
            break;

        case MDF_EVENT_MWIFI_ROOT_GOT_IP: 
        {
            MDF_LOGI("Root obtains the IP address. It is posted by LwIP stack automatically");
            //xTaskCreate(tcp_client_write_task, "tcp_client_write_task", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
            xTaskCreate(tcp_client_read_task, "tcp_server_read", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
            break;
        }

        default:
            break;
    }

    return MDF_OK;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Timer interrupt
static bool IRAM_ATTR TimerInt(void *args)
{
    uint8_t i;
	uint16_t Mask = 0x8000; 
	static uint8_t Brightness = 0;
	static uint8_t SegmentCount = 0;
	static uint16_t Segment = 0;
	static uint16_t LedSegment = 0;

	//WiFi Timeout
	/*if(WiFiTimeout > 0)
	{
		WiFiTimeout--;
		if(WiFiTimeout == 0)
			WiFiReset = true;
	}*/

	//Buzzer
	if (BuzzerSound)
	{
        BuzzerCount++;
        if(BuzzerCount >= BUZZERCOUNT)
        {
            BuzzerFreq++;
			FreqCount++;
            if(BuzzerFreq >= 8)
                BuzzerFreq = 0;
            switch(BuzzerTone)
            {
            case 0: ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, LEDC_TIMER, BuzzerFrequency1[BuzzerFreq]));
                    break;
            case 1: ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, LEDC_TIMER, BuzzerFrequency2[BuzzerFreq]));
                    break;
            case 2: ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, LEDC_TIMER, BuzzerFrequency3[BuzzerFreq]));
                    break;
            default:ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, LEDC_TIMER, BuzzerFrequency1[BuzzerFreq]));
                    break;
            }
            
            BuzzerCount = 0;
        }
	}

	//LED
	//Switch on LED
	if (Brightness == 0)
	{
		LedSwitch = 0;
		if (!FlashLedOn)
		{
			//Wurth LED is red = 2, green = 1, blue = 4
			if (Red > 0)
				LedSwitch = LedSwitch | 2;
			if (Green > 0)
				LedSwitch = LedSwitch | 1;
			if (Blue > 0)
				LedSwitch = LedSwitch | 4;
		}
	}
	//Switch off LED
	//Wurth LED is red 0xFFFD, green 0xFFFE, blue 0xFFFB
	if ((Red < 255) & (Red == Brightness))
		LedSwitch = LedSwitch & 0xFFFD;
	if ((Green < 255) & (Green == Brightness))
		LedSwitch = LedSwitch & 0xFFFE;
	if ((Blue < 255) & (Blue == Brightness))
		LedSwitch = LedSwitch & 0xFFFB;
	Brightness++;

    /*
	//Seven segment
	if ((Brightness == 32) | (Brightness == 96) | (Brightness == 160) | (Brightness == 224))
	{
		SegmentCount++;
		if (SegmentCount >= SEGMENTS)
			SegmentCount = 0;
		Segment = Seg[SegmentCount] << 8;
		switch (SegmentCount)
		{
		case 0:	LedSegment = 0x0080;									//Segment 1
			break;
		case 1:	LedSegment = 0x0040;									//Segment 2
			break;
		case 2:	LedSegment = 0x0020;									//Segment 3
			break;
		case 3:	LedSegment = 0x0010;									//Segment 4
			break;
		default:LedSegment = 0x0010;									//Segment 4
			break;
		}
		if ((PowerUpTimeout > 0) & (PowerUpTimeout < 0xFFFF))			//Used to switch off power up message
		{
			PowerUpTimeout--;
			if (PowerUpTimeout == 0)
			{
				Seg[0] = 0x00;
				Seg[1] = 0x00;
				Seg[2] = 0x00;
				Seg[3] = 0x00;
				PowerUpTimeout = 0xFFFF;
			}
		}
	}
	Led = LedSwitch | Segment | LedSegment;
	Pin_Low(BCLK);
	Pin_Low(BLE);
	for (i = 0; i < 16; i++)
	{
        Pin_Level(BSDO, (Mask & Led));
		Pin_High(BCLK);
		Pin_Low(BCLK);
		Mask = Mask >> 1;
	}
	Pin_High(BLE);
	Pin_Low(BLE);
    return(true);
    */
   //Test
    if((LedSwitch & 1) == 0)
        Pin_High(BLEDG);
    else
        Pin_Low(BLEDG);
    if((LedSwitch & 2) == 0)
        Pin_High(BLEDR);
    else
        Pin_Low(BLEDR);        
    if((LedSwitch & 4) == 0)
        Pin_High(BLEDB);
    else
        Pin_Low(BLEDB); 
    return(true);
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Initialise timer
static void TimerInit(int group, int timer)
{
    //Timer interrupt period is defined in Global.h
    timer_config_t config = 
    {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    };
    timer_init(group, timer, &config);                  //Timer basic parameters
    timer_set_counter_value(group, timer, 0);           //Start counter from zero, this value will be automatically reload on alarm
    timer_set_alarm_value(group, timer, TIMER_VALUE);   //Set the alarm value
    timer_enable_intr(group, timer);                    //Enable interrupt on alarm
    timer_isr_callback_add(group, timer, TimerInt, NULL, 0);    //Attach interrupt to callback routine
    timer_start(group, timer);                          //Start timer
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Flash LED task
//Switch led on and off at a rate given in structure
void LedFlashRateTask(void *p)
{
	LEDFlash *LEDNow = (LEDFlash *) p;
	while (1)
	{
        FlashLedOn = true;
        vTaskDelay(pdMS_TO_TICKS(LEDNow->DelayOff));							//Delay	
        FlashLedOn = false;
        vTaskDelay(pdMS_TO_TICKS(LEDNow->DelayOn));
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Flash LED for a period given in structure
void LedFlashPeriodTask(void *p)
{
    uint8_t Counter = 0;
    LEDFlash *LEDNow = (LEDFlash *) p;

    while(1)
    {
        if(FlashLedActive == true)
        {
            if(Counter == 1)
            {
                FlashLedActive = false; 
                Counter = 0;
            }
            Counter++;
        }
        vTaskDelay(pdMS_TO_TICKS(LEDNow->Period));    
    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Switch buzzer on for Delay(mS) period
void BuzzerOn(void* p)
{
	int Delay;

	while (1)
	{
		if (xQueueReceive(BuzzerQHandle, &Delay, 1000))
		{
			BuzzerSound = true;
            BuzzerFreq = 0;
            BuzzerCount = 0;
			FreqCount = 0;
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
			switch(BuzzerTone)
            {
            case 0: ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, LEDC_TIMER, BuzzerFrequency1[BuzzerFreq]));
                    break;
            case 1: ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, LEDC_TIMER, BuzzerFrequency2[BuzzerFreq]));
                    break;
            case 2: ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, LEDC_TIMER, BuzzerFrequency3[BuzzerFreq]));
                    break;
            default:ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, LEDC_TIMER, BuzzerFrequency1[BuzzerFreq]));
                    break;
            }
			vTaskDelay(pdMS_TO_TICKS(Delay));							//Delay
            if(BuzzerContinous == false)
            {
                BuzzerSound = false;
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));  
                vTaskDelay(pdMS_TO_TICKS(100));
            }
		}
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Read inputs (for now just GPIO7)
void InputsTask(void *p)
{
    bool Key2Debounce = true, Key2Once = true;
    Inputs *InputsNow = (Inputs *) p;
    while(1)
    {
        if(gpio_get_level(KEY2) == 0)
        {
            if((Key2Debounce == false) & Key2Once)
            {
                Key2Once = false;
                InputsNow->Key2 = true;
            }
            Key2Debounce = false;
        }
        else
        {
            InputsNow->Key2 = false;
            Key2Debounce = true;
            Key2Once = true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//Main Routine
void app_main()
{
    //Define variables
    bool MACPass = false;
    uint8_t i;
    mdf_err_t ret = MDF_OK;
    char data[9];//                       = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size;   
    int Delay = 100; 
    mwifi_node_type_t MeshType;
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type      = {0};
    struct LEDFlash LEDNow;
    struct Inputs InputsNow;
    struct Reply ReplyNow;

    //Initialise structure values
    LEDNow.DelayOn = 100;
    LEDNow.DelayOff = 100;
    LEDNow.Period = 1000;
    InputsNow.Key2 = false;
    
    //Setup pins
    //Inputs
    Pin_Init(Buzzer, GPIO_MODE_OUTPUT, LOW);
    Pin_Init(KEY1, GPIO_MODE_INPUT, 0);
    Pin_Init(KEY2, GPIO_MODE_INPUT, 0);
    Pin_PullUp(KEY2);
    //Temporary inputs
    Pin_Init(LE, GPIO_MODE_INPUT, 0);
    Pin_Init(CLK, GPIO_MODE_INPUT, 0); 
    Pin_PullUp(LE);
    Pin_PullUp(CLK);
    //Outputs
    Pin_Init(LEDG, GPIO_MODE_OUTPUT, 1);
    Pin_Init(LEDB, GPIO_MODE_OUTPUT, 1);
    Pin_Init(LEDR, GPIO_MODE_OUTPUT, 1);

    //Determine if Root or Node
    if(gpio_get_level(KEY1) == 0)
        MeshType = MESH_NODE;
    else
        MeshType = MESH_ROOT;

    //Temporarily get address from pins (normally from DIP switch)
    if(gpio_get_level(LE) != 0)
        Address = Address | 0x01;
    if(gpio_get_level(CLK) != 0)
        Address = Address | 0x02;    
    InputsNow.DIPAddress = Address;
    InputsNow.DIPOptions = Address;

    mwifi_init_config_t cfg   = MWIFI_INIT_CONFIG_DEFAULT();
    mwifi_config_t config     = {
        .router_ssid     = "Murray36_2ghz",
        .router_password = "Steph@321",
        .channel = 1,
        .mesh_id         = CONFIG_MESH_ID,
        .mesh_password   = "",
        .mesh_type = MeshType,
    };

    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    /**
     * @brief Initialize wifi mesh.
     */
    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
    MDF_ERROR_ASSERT(wifi_init());
    MDF_ERROR_ASSERT(mwifi_init(&cfg));
    MDF_ERROR_ASSERT(mwifi_set_config(&config));
    MDF_ERROR_ASSERT(mwifi_start());

    /**
     * @brief select/extend a group memebership here
     *      group id can be a custom address
     */
    const uint8_t group_id_list[2][6] = {{0x01, 0x00, 0x5e, 0xae, 0xae, 0xae},
                                        {0x01, 0x00, 0x5e, 0xae, 0xae, 0xaf}};

    MDF_ERROR_ASSERT(esp_mesh_set_group_id((mesh_addr_t *)group_id_list,
                                           sizeof(group_id_list) / sizeof(group_id_list[0])));

    //Setup RTOS
    if (MeshType == MESH_ROOT) 
    {
        xTaskCreate(root_task, "root_task", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    } 
    else 
    {
        xTaskCreate(node_write_task, "node_write_task", 4 * 1024, (void *) NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
        xTaskCreate(node_read_task, "node_read_task", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    }
	NodeWriteQHandle = xQueueCreate(1, sizeof(struct Reply));   
    //Input Task
    xTaskCreate(InputsTask, "Input Task", 2000, (void *) &InputsNow, 1, &InputsTHandle);
    //Buzzer task
    xTaskCreate(BuzzerOn, "Buzzer On", 2000, NULL, 1, &BuzzerTHandle);
    BuzzerQHandle = xQueueCreate(1, sizeof(int));

    //Initialise timer
    TimerInit(TIMER_GROUP_0, TIMER_0); 
	ESP_LOGI(TAG, "Timer initialized");

    //Initialise LEDC
    //Prepare and then apply the LEDC PWM timer configuration
     ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer)); 
    //Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BUZZER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
	ESP_LOGI(TAG, "PWM initialized");

    //Clear memory
    memset(MACAddress, 0, 1536);

    //Main loop
    for(;;)
    {
        if(InputsNow.Key2 == true)
        {
            ReplyNow.ResponseType = 0x00;
            ReplyNow.Data[0] = 0x01;
            ReplyNow.Data[1] = 0x02;
            ReplyNow.Data[2] = 0x03;
            ReplyNow.Data[3] = 0x04;
            ReplyNow.Data[4] = 0x05;
            ReplyNow.Data[5] = 0x00;
            ReplyNow.Data[6] = 0x00;
            if(MeshType == MESH_NODE)
                xQueueSend(NodeWriteQHandle, &ReplyNow, 10);
            if(MeshType == MESH_ROOT)
            {
                vTaskSuspendAll();
                RootOut[RootOutHead++] = Address;
                if(RootOutHead >= ROOTBUFFERSIZE)
                    RootOutHead = RootOutHead - ROOTBUFFERSIZE;        
                RootOut[RootOutHead++] = ReplyNow.ResponseType; 
                if(RootOutHead >= ROOTBUFFERSIZE)
                    RootOutHead = RootOutHead - ROOTBUFFERSIZE;           
                for(i = 0; i < 7; i++)
                {
                    RootOut[RootOutHead++] = ReplyNow.Data[i];
                    if(RootOutHead >= ROOTBUFFERSIZE)
                        RootOutHead = RootOutHead - ROOTBUFFERSIZE;
                }
                xTaskResumeAll();                
            }
            xQueueSend(BuzzerQHandle, &Delay, 100);
            InputsNow.Key2 = false;
        }
        if(FlashLedActive)
        {
            if(LedFlashRateTHandle == NULL)
            {
                xTaskCreate(LedFlashRateTask, "LED Flash Rate", 2000, (void *) &LEDNow, 1, &LedFlashRateTHandle);
            }
        }
        else
        {
            if(LedFlashPeriodTHandle!= NULL)  
            {
                vTaskDelete(LedFlashPeriodTHandle);
                LedFlashPeriodTHandle= NULL;
            }
            if(LedFlashRateTHandle != NULL)
            {
                vTaskDelete(LedFlashRateTHandle);
                LedFlashRateTHandle = NULL;
                FlashLedOn = false;
            }                            
        }
        //Data to be sent from Root to controller, for CAN all nodes are roots
        if(RootOutHead != RootOutTail)
        {
            if (mwifi_is_connected()) 
            {
                if (g_sockfd != -1) 
                {
                    size = 9;
                    vTaskSuspendAll();
                    for( i = 0; i < size; i++)
                    {
                        data[i] = RootOut[RootOutTail++];
                        if(RootOutTail >= ROOTBUFFERSIZE)
                            RootOutTail = RootOutTail - ROOTBUFFERSIZE;
                    }
                    xTaskResumeAll(); 
                    MDF_LOGD("Data to controller, size: %d, data: %d %d %d %d %d %d %d %d %d", size, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
                    if(Mode == WIFI)                                                        //Send data via WiFi
                    {
                        ret = write(g_sockfd, data, size);
                        MDF_ERROR_CONTINUE(ret <= 0, "<%s> TCP write", strerror(errno));
                    }
                    else                                                                    //Send data via CAN
                    {

                    }
                }
            }
        }
        //Data to be sent from Root to Node
        if(NodeOutHead != NodeOutTail)
        {
            size = 9;
            vTaskSuspendAll();
            for( i = 0; i < size; i++)
            {
                data[i] = NodeOut[NodeOutTail++];
                if(NodeOutTail >= NODEBUFFERSIZE)
                    NodeOutTail = NodeOutTail - NODEBUFFERSIZE;
            }
            xTaskResumeAll(); 
            if(Mode == WIFI)                                                        //Send data via WiFi
            {  
                //get MAC Address
                MACPass = false;
                for(i = 0; i < 6; i++)
                {
                    src_addr[i] = MACAddress[data[0] * 6 + i];  
                    if(src_addr[i] > 0) 
                        MACPass = true;
                }
                if(MACPass)     
                {                
                    size = 8;
                    for(i = 0; i < size; i++)                                       //Don't need to send node address so change to be the same as a CAN send
                        data[i] = data[i + 1];
                    ret = mwifi_root_write(src_addr, 1, &data_type, data, size, true);
                    MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_root_recv, ret: %x", ret);
                    MDF_LOGI("Root send, addr: " MACSTR ", size: %d, data: %d %d %d %d %d %d %d %d", MAC2STR(src_addr), size, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
                }
                else
                    MDF_LOGE("MAC Address not in database");
            }
            else                                                                    //Send data via CAN
            {

            }            
        } 
        //Data to be actioned by node (including root)
        if(ActionHead != ActionTail)
        {  
            vTaskSuspendAll();
            for( i = 0; i < 8; i++)
            {
                data[i] = Action[ActionTail++];
                if(ActionTail >= ACTIONBUFFERSIZE)
                    ActionTail = ActionTail - ACTIONBUFFERSIZE;
            }
            xTaskResumeAll();             
            switch(data[0] & 0x3F)
            {
            case 0: if((data[0] & 0x80) == 0)
                        FlashLedActive = false;
                    else
                        FlashLedActive = true;    
                    Red = data[1];
                    Green = data[2];
                    Blue = data[3];
                    for(i = 0; i < 4; i++)
                        SegValue[i] = data[4 + i];
                    //ToDo must store SegValue in FRAM
                    if((data[4] & 0x80) == 0)
                        Seg[0] = AsciiToSeven(data[4]);
                    else
                        Seg[0] = BinaryToSeven(data[4]);
                    if((data[5] & 0x80) == 0)
                        Seg[1] = AsciiToSeven(data[5]);
                    else
                        Seg[1] = BinaryToSeven(data[5]);
                    if((data[6] & 0x80) == 0)
                        Seg[2] = AsciiToSeven(data[6]);
                    else
                        Seg[2] = BinaryToSeven(data[6]);
                    if((data[7] & 0x80) == 0)
                        Seg[3] = AsciiToSeven(data[7]);
                    else
                        Seg[3] = BinaryToSeven(data[7]);  
                    MDF_LOGI("Data command 0 actioned Segments: %d %d %d %d", Seg[0], Seg[1], Seg[2], Seg[3]);      
                    break;
            case 1: if((data[0] & 0x80) == 0)
                        FlashLedActive = false;
                    else
                        FlashLedActive = true;    
                    Red = data[1];
                    Green = data[2];
                    Blue = data[3];
                    MDF_LOGI("Data command 1 actioned"); 
                    break;
            case 2: for(i = 0; i < 4; i++)
                        SegValue[i] = data[i];
                    //ToDo must store SegValue in FRAM
                    if((data[0] & 0x80) == 0)
                        Seg[0] = AsciiToSeven(data[0]);
                    else
                        Seg[0] = BinaryToSeven(data[0]);
                    if((data[1] & 0x80) == 0)
                        Seg[1] = AsciiToSeven(data[1]);
                    else
                        Seg[1] = BinaryToSeven(data[1]);
                    if((data[2] & 0x80) == 0)
                        Seg[2] = AsciiToSeven(data[2]);
                    else
                        Seg[2] = BinaryToSeven(data[2]);
                    if((data[3] & 0x80) == 0)
                        Seg[3] = AsciiToSeven(data[3]);
                    else
                        Seg[3] = BinaryToSeven(data[3]);  
                    MDF_LOGI("Data command 2 actioned");  
                    break;
            case 3: if((data[0] & 0x40) == 0) 
                        BuzzerContinous = false;
                    else
                        BuzzerContinous = true;
                    xQueueSend(BuzzerQHandle, &Delay, 100);
                    MDF_LOGI("Data command 3 actioned");        
                    break;
            case 4: if((data[0] & 0x80) == 0)
                        FlashLedActive = false;
                    else
                        FlashLedActive = true;   
                    MDF_LOGI("Data command 4 actioned");     
                    break;  
            case 5: 
                    break;
            case 6: if(data[1] > 2)
                        BuzzerTone = 0;
                    else
                        BuzzerTone = data[1];
                    MDF_LOGI("Data command 6 actioned"); 
                    break;
            case 7: if((data[1] > 100) | (data[1] == 0))
                        LEDNow.DelayOn = 500;
                    else
                        LEDNow.DelayOn = data[1] * 100;
                    if((data[2] > 100) | (data[2] == 0))
                        LEDNow.DelayOff = 500;
                    else
                        LEDNow.DelayOff = data[2] * 100;
                    MDF_LOGI("Data command 7 actioned"); 
                    break;
            case 16:
            case 17:
            case 18:
            case 19:for(i = 0; i < 7; i++)                          //Clear response
                    ReplyNow.Data[i] = 0;
                    if((data[0] & 0x3F) == 16)
                    {
                        ReplyNow.ResponseType = 0x01;               //Response type (response to Command 16)
                        ReplyNow.Data[0] = Red;
                        ReplyNow.Data[1] = Green;
                        ReplyNow.Data[2] = Blue;
                    }
                    if((data[0] & 0x3F) == 17)
                    {
                        ReplyNow.ResponseType = 0x02;               //Response type (response to Command 17)                 
                        ReplyNow.Data[0] = SegValue[0];
                        ReplyNow.Data[1] = SegValue[1];
                        ReplyNow.Data[2] = SegValue[2];
                        ReplyNow.Data[3] = SegValue[3];
                    }
                    if((data[0] & 0x3F) == 18)
                    {
                        ReplyNow.ResponseType = 0x03;               //Response type (response to Command 18)                      
                        ReplyNow.Data[0] = InputsNow.DIPAddress;
                        ReplyNow.Data[1] = InputsNow.DIPOptions;
                    }
                    if((data[0] & 0x3F) == 19)
                    {
                        ReplyNow.ResponseType = 0x04;               //Response type (response to Command 19)                      
                        ReplyNow.Data[0] = BuzzerTone;
                        ReplyNow.Data[1] = LEDNow.DelayOn / 100;
                        ReplyNow.Data[2] = LEDNow.DelayOff / 100;
                    }                    
                    if(MeshType == MESH_NODE)
                        xQueueSend(NodeWriteQHandle, &ReplyNow, 10);
                    if(MeshType == MESH_ROOT)
                    {
                        vTaskSuspendAll();
                        RootOut[RootOutHead++] = Address;
                        if(RootOutHead >= ROOTBUFFERSIZE)
                            RootOutHead = RootOutHead - ROOTBUFFERSIZE;        
                        RootOut[RootOutHead++] = ReplyNow.ResponseType; 
                        if(RootOutHead >= ROOTBUFFERSIZE)
                            RootOutHead = RootOutHead - ROOTBUFFERSIZE;           
                        for(i = 0; i < 7; i++)
                        {
                            RootOut[RootOutHead++] = ReplyNow.Data[i];
                            if(RootOutHead >= ROOTBUFFERSIZE)
                                RootOutHead = RootOutHead - ROOTBUFFERSIZE;
                        }
                        xTaskResumeAll();                
                    }            
                    break;
            case 32:
                    break; 
            default:break;                                                                                                                      
            }
        }             
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
