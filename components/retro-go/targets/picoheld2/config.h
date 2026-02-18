/* 
 * command to build: python rg_tool.py --target picoheld2 build-img --no-networking
 */

/****************************************************************************
 * Target definition for the Pico Held 2                                    *
 ****************************************************************************/
#define RG_TARGET_NAME             "Pico Held 2"

/****************************************************************************
 * Status LED                                                               *
 ****************************************************************************/
// #define RG_LED_DRIVER               1   // 1 = GPIO
// #define RG_GPIO_LED                 GPIO_NUM_NC
// #define RG_GPIO_LED_INVERT          // Uncomment if the LED is active LOW

/****************************************************************************
 * Storage                                                                  *
 ****************************************************************************/
// Storage
#define RG_STORAGE_ROOT             "/sd"
#define RG_STORAGE_SDMMC_HOST       SDMMC_HOST_SLOT_1
#define RG_STORAGE_SDMMC_SPEED      SDMMC_FREQ_DEFAULT   // SDMMC_FREQ_PROBING  //SDMMC_FREQ_HIGHSPEED
#define RG_GPIO_SDMMC_CLK           (GPIO_NUM_43)
#define RG_GPIO_SDMMC_CMD	        (GPIO_NUM_44)
#define RG_GPIO_SDMMC_D0	        (GPIO_NUM_39)
#define RG_GPIO_SDMMC_D1	        (GPIO_NUM_40)
#define RG_GPIO_SDMMC_D2	        (GPIO_NUM_41)
#define RG_GPIO_SDMMC_D3	        (GPIO_NUM_42)
#define RG_GPIO_SD_DET              (GPIO_NUM_47)
#define RG_STORAGE_SDMMC_VDDPST     (4)            // VDPST for VREF

/****************************************************************************
 * Audio                                                                    *
 ****************************************************************************/
#define RG_AUDIO_USE_INT_DAC        0   // 0 = Disable, 1 = GPIO25, 2 = GPIO26, 3 = Both
#define RG_AUDIO_USE_EXT_DAC        1   // 0 = Disable, 1 = Enable
#define RG_GPIO_SND_I2S_BCK         (GPIO_NUM_28)
#define RG_GPIO_SND_I2S_WS          (GPIO_NUM_29)
#define RG_GPIO_SND_I2S_DATA        (GPIO_NUM_25)
#define RG_GPIO_SND_AMP_ENABLE      (GPIO_NUM_30)
// #define RG_GPIO_SND_AMP_ENABLE_INVERT // Uncomment if the mute = HIGH


/****************************************************************************
 * Video                                                                    *
 ****************************************************************************/
#define RG_SCREEN_DRIVER            (3) // (RG_SCREEN_DRIVER_DSIMIPI)   // 3 = DSI/MIPI
#define RG_SCREEN_BACKLIGHT         (1)
#define RG_SCREEN_WIDTH             (360)
#define RG_SCREEN_HEIGHT            (360)
#define RG_SCREEN_ROTATION          (1)   // Possible values are 0-3 (you'll have to experiment)
#define RG_SCREEN_RGB_BGR           (1)   // Possible values are 0-1 (change if colors are bad)
#define RG_SCREEN_NOSWAP            (1)   // Don't swap bytes
#define RG_SCREEN_PIXEL_FORMAT      (1)

#define RG_GPIO_LCD_BCKL            (GPIO_NUM_22)
#define RG_GPIO_LCD_BCKL_EN         (GPIO_NUM_7)
#define RG_GPIO_LCD_RST             (GPIO_NUM_23)
#define RG_GPIO_LCD_TE              (GPIO_NUM_24)

#define RG_SCREEN_PARTIAL_UPDATES   (0)
// #define RG_GPIO_LCD_BCKL_INVERT  // Uncomment if the LED is active LOW


/****************************************************************************
 * Input                                                                    *
 ****************************************************************************/
#define USE_ADC_DRIVER_NG

// Refer to rg_input.h to see all available RG_KEY_* and RG_GAMEPAD_*_MAP types
// start needs to be pulled down and is pressed on level=1
// analog switch is currently menu, option is currently unassigned
#if 0
    {RG_KEY_START,  .num = GPIO_NUM_45, .pullup = 0, .level = 1},
    {RG_KEY_SELECT, .num = GPIO_NUM_35, .pullup = 1, .level = 0},
    {RG_KEY_MENU,   .num = GPIO_NUM_1,  .pullup = 1, .level = 0},
    {RG_KEY_A,      .num = GPIO_NUM_31, .pullup = 1, .level = 0},
    {RG_KEY_B,      .num = GPIO_NUM_5,  .pullup = 1, .level = 0},
    {RG_KEY_X,      .num = GPIO_NUM_32, .pullup = 1, .level = 0},
    {RG_KEY_Y,      .num = GPIO_NUM_54, .pullup = 1, .level = 0},
    {RG_KEY_LEFT,   .num = GPIO_NUM_14, .pullup = 1, .level = 0},
    {RG_KEY_RIGHT,  .num = GPIO_NUM_12, .pullup = 1, .level = 0},
    {RG_KEY_UP,     .num = GPIO_NUM_13, .pullup = 1, .level = 0},
    {RG_KEY_DOWN,   .num = GPIO_NUM_15, .pullup = 1, .level = 0},
    {RG_KEY_L,      .num = GPIO_NUM_11, .pullup = 1, .level = 0},
    {RG_KEY_R,      .num = GPIO_NUM_4,  .pullup = 1, .level = 0},
#endif

#define RG_GAMEPAD_GPIO_MAP {\
    {RG_KEY_START,  .num = GPIO_NUM_45, .pullup = 0, .level = 1},\
    {RG_KEY_SELECT, .num = GPIO_NUM_35, .pullup = 1, .level = 0},\
    {RG_KEY_MENU,   .num = GPIO_NUM_1,  .pullup = 1, .level = 0},\
    {RG_KEY_A,      .num = GPIO_NUM_31, .pullup = 1, .level = 0},\
    {RG_KEY_B,      .num = GPIO_NUM_5,  .pullup = 1, .level = 0},\
    {RG_KEY_X,      .num = GPIO_NUM_32, .pullup = 1, .level = 0},\
    {RG_KEY_Y,      .num = GPIO_NUM_54, .pullup = 1, .level = 0},\
    {RG_KEY_LEFT,   .num = GPIO_NUM_14, .pullup = 1, .level = 0},\
    {RG_KEY_RIGHT,  .num = GPIO_NUM_12, .pullup = 1, .level = 0},\
    {RG_KEY_UP,     .num = GPIO_NUM_13, .pullup = 1, .level = 0},\
    {RG_KEY_DOWN,   .num = GPIO_NUM_15, .pullup = 1, .level = 0},\
    {RG_KEY_L,      .num = GPIO_NUM_11, .pullup = 1, .level = 0},\
    {RG_KEY_R,      .num = GPIO_NUM_4,  .pullup = 1, .level = 0},\
}


/****************************************************************************
 * Power and battery                                                        *                                                            *
 ****************************************************************************/
// Power
#define RG_BATTERY_DRIVER            (0)
#define RG_BATTERY_ADC_UNIT          (ADC_UNIT_1)
#define RG_BATTERY_ADC_CHANNEL       (ADC_CHANNEL_3)
#define RG_BATTERY_CALC_PERCENT(raw) (((raw) * 2.f - 3500.f) / (4200.f - 3500.f) * 100.f)
#define RG_BATTERY_CALC_VOLTAGE(raw) ((raw) * 2.f * 0.001f)

#define RG_PICO_VERSION (3)

#if RG_PICO_VERSION==2
//v1.2
#define RG_GPIO_PWR_EN               (GPIO_NUM_53)
#define RG_GPIO_PWR_CHG_STAT         (GPIO_NUM_52)
#elif RG_PICO_VERSION==3
// v1.3
#define RG_GPIO_PWR_EN               (GPIO_NUM_52)
#define RG_GPIO_PWR_CHG_STAT         (GPIO_NUM_53)
#elif
#error Unsupported Pico Held version
#endif

/****************************************************************************
 * Updater                                                                  *
 ****************************************************************************/
// #define RG_UPDATER_ENABLE               1
// #define RG_UPDATER_APPLICATION          RG_APP_FACTORY
// #define RG_UPDATER_DOWNLOAD_LOCATION    RG_STORAGE_ROOT "/odroid/firmware"



/****************************************************************************
 * Miscellaneous                                                            *
 ****************************************************************************/
//#define RG_RECOVERY_BTN               (RG_KEY_MENU // Keep this button pressed to open the recovery menu

#define RG_CUSTOM_PLATFORM_INIT() \
    gpio_reset_pin(RG_GPIO_PWR_EN); \
    gpio_set_direction(RG_GPIO_PWR_EN, GPIO_MODE_OUTPUT); \
    gpio_set_level(RG_GPIO_PWR_EN, 1); \

#define RG_CUSTOM_PLATFORM_DEINIT() \
    gpio_set_level(RG_GPIO_SND_AMP_ENABLE, 0); \

#define RG_CUSTOM_PLATFORM_SHUTDOWN() \
    gpio_set_level(RG_GPIO_PWR_EN, 0); \

/* none */


// See components/retro-go/config.h for more things you can define here!
