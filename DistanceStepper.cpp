#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "stdio.h"
#include "external/LCD_I2C.hpp"
#include <pico/binary_info/code.h>
#include <hardware/i2c.h>
#include "pico/binary_info.h"
#include <string>

// distance sensor
#define IR_SENSOR_ADC_PIN 26  // GPIO 26 == ADC0

// GENERIC
#define LED_PIN 25 //

// motor pins
#define STEP_PIN 12
#define DIR_PIN  11
#define ENABLE_PIN 13
#define MOTOR_OUT_PIN 11
#define MOTOR_DELAY_US 175
#define MOTOR_STEPS_PER_FRAME 1

// limit switches
#define LIMIT_Z_MIN_PIN 3
#define LIMIT_Z_MAX_PIN 5

// encoder
#define ENCODER_SW 8
#define ENCODER_CLK 9
#define ENCODER_DATA 17

#define DISTANCE_PRECISION 1000

#define abs(value) ((value) < 0 ? (-value) : (value))
#define max(value,upper) ((value) < (upper) ? upper : value)
#define min(value,lower) ((value) > (lower) ? lower : value)


float min = 9999.0f;
float max = 3.3f;
float range = 0.0f;
float value = 0.0f;
volatile bool zLimitMin = false;
volatile bool zLimitMax = false;

volatile int encoder_position = 0;
volatile uint8_t last_state = 0;
// -1 is left, 0 is no direction, 1 is right
volatile int encoderDirection = 0;

uint8_t clk = 0;
uint8_t data = 0;
uint8_t new_state = 0;

volatile bool encoder_flag = false;

void fill_meter(char* meter, int size, int value)
{
    for(int i = 0; i < 100;i++)
    {
        meter[i] = ' ';
    }

    // fill meter
    for(int i = 0; i < value; i++)
    {
        meter[i] = '#';
    }
}

int get_distance()
{
    uint16_t raw = adc_read();  // 12-bit: 0–4095
    const float voltage = raw * 3.3f / 4095;

    fprintf(stdout, "%0.3f :", voltage);

    // keep track of mins and max to create a accurate meter
    if(voltage < min)
    {
        min = voltage;
        range = max - min;
    }

    int value = voltage * DISTANCE_PRECISION;

    fprintf(stdout, "%li :", value);

    return value;
}

void init_led_pins()
{
    printf("Initializing LED indicator\n");

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1); // turn on LED
}

void init_sensor_pins()
{
    printf("Initializing sensors\n");

    adc_init();
    adc_gpio_init(IR_SENSOR_ADC_PIN);
    adc_select_input(0);  // ADC0
}

void init_motor_pins()
{
    printf("Initializing motors\n");

    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, 0);

    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_OUT);
    gpio_put(ENABLE_PIN, 1);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 0);

    gpio_init(MOTOR_OUT_PIN);
    gpio_set_dir(MOTOR_OUT_PIN, GPIO_OUT);
    gpio_put(MOTOR_OUT_PIN, 1);
}

void init_limit_pins()
{
    gpio_init(LIMIT_Z_MIN_PIN);
    gpio_set_dir(LIMIT_Z_MIN_PIN, GPIO_IN);
    gpio_pull_down(LIMIT_Z_MIN_PIN);

    gpio_init(LIMIT_Z_MAX_PIN);
    gpio_set_dir(LIMIT_Z_MAX_PIN, GPIO_IN);
    gpio_pull_down(LIMIT_Z_MAX_PIN);
}

void encoder_update() {
    if ((last_state == 0b00 && new_state == 0b01) ||
        (last_state == 0b01 && new_state == 0b11) ||
        (last_state == 0b11 && new_state == 0b10) ||
        (last_state == 0b10 && new_state == 0b00)) 
    {
        if(encoderDirection != -1)
        {
            encoder_position++;  // CW
            last_state = new_state;
            encoderDirection = 1;
        }
        else
        {
            encoderDirection = 0;
        }
    } 
    else if (
        (last_state == 0b00 && new_state == 0b10) ||
        (last_state == 0b10 && new_state == 0b11) ||
        (last_state == 0b11 && new_state == 0b01) ||
        (last_state == 0b01 && new_state == 0b00)) 
    {
        if(encoderDirection != 1)
        {
            encoder_position--;  // CCW
            last_state = new_state;
            encoderDirection = -1;
        }
        else
        {
            encoderDirection = 0;
        }
    }
}

bool encoderIsDifferent()
{
    clk = gpio_get(ENCODER_CLK);
    data = gpio_get(ENCODER_DATA);
    
    new_state = (clk << 1) | data;
    
    return new_state != last_state;
}

void init_encoder_pins()
{
    printf("Initializing encoder\n");

    gpio_init(ENCODER_CLK);
    gpio_set_dir(ENCODER_CLK, GPIO_IN);
    gpio_pull_up(ENCODER_CLK);

    gpio_init(ENCODER_DATA);
    gpio_set_dir(ENCODER_DATA, GPIO_IN);
    gpio_pull_up(ENCODER_DATA);

    gpio_init(ENCODER_SW);
    gpio_set_dir(ENCODER_SW, GPIO_IN);
    gpio_pull_up(ENCODER_SW);

    last_state = (gpio_get(ENCODER_CLK) << 1) | gpio_get(ENCODER_DATA);
}

void step_motor(int steps, bool direction, int delay_us) {
    gpio_put(ENABLE_PIN, 0);
    gpio_put(DIR_PIN, direction);
    for (int i = 0; i < steps; ++i) {
        zLimitMin = gpio_get(LIMIT_Z_MIN_PIN) && !direction;
        zLimitMax = gpio_get(LIMIT_Z_MAX_PIN) && direction;
        if(zLimitMin || zLimitMax)
        {
            gpio_put(STEP_PIN, 0);
            return;
        }
        else{
            gpio_put(STEP_PIN, 1);
            sleep_us(delay_us);
            gpio_put(STEP_PIN, 0);
            sleep_us(delay_us);
        }
    }
    gpio_put(ENABLE_PIN, 1);
}

bool EncoderSwitch()
{
    return !gpio_get(ENCODER_SW);
}

int scanForDisplay()
{
    printf("Attempting to find serial devices\n");
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        int ret = i2c_write_blocking(i2c0, addr, NULL, 0, false);
        if (ret >= 0)
        {
            printf("Found device at 0x%02x\n", addr);
            return addr;
        }
    }

    printf("No serial devices found\n");

    return 0x27;
}

LCD_I2C* init_display()
{
    printf("Initializing dislay\n");
    i2c_inst* I2C = i2c0;
    const int SDA = 0;
    const int SCL = 1;
    const uint8_t  I2C_ADDRESS = 0x27;
    const char  LCD_COLUMNS = 20;
    const char  LCD_ROWS = 4;

    bi_decl(bi_1pin_with_name(SDA, "[SDA] LCD screen serial data pin"))
    bi_decl(bi_1pin_with_name(SCL, "[SCL] LCD screen serial clock pin"))

    /* @param address The I2C address
     * @param columns The LCD's number of columns
     * @param rows The LCD's number of rows
     * @param I2C The I2C instance
     * @param SDA The serial data pin
     * @param SCL The serial clock pin
     * LCD_I2C(byte address, byte columns, byte rows, i2c_inst * I2C = PICO_DEFAULT_I2C_INSTANCE,
            uint SDA = PICO_DEFAULT_I2C_SDA_PIN, uint SCL = PICO_DEFAULT_I2C_SCL_PIN) noexcept;
     */

     LCD_I2C* result =  new LCD_I2C(I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS, I2C, SDA, SCL);

    printf("Initialized Display\n");

    return result;
}

uint64_t last_time = time_us_64();

float calculateDeltaTime()
{
    uint64_t now = time_us_64();
    float dt = (now - last_time) / 1e6f; // delta time in seconds
    last_time = now;

    return dt;
}

struct ProgramState
{
    int SelectedItem;
    int PreviousSelectedItem;
    bool ChangedMenu;
};

#define NUMBER_OF_MAIN_MENU_ITEMS 4

#define MENU(name) void UI_##name(const float deltaTime, ProgramState* state, LCD_I2C* display)

MENU(DrawMainMenu)
{
    if(state->PreviousSelectedItem != state->SelectedItem)
    {
        const char* options[NUMBER_OF_MAIN_MENU_ITEMS] = {
            "Calibrate",
            "Start",
            "Info",
            "Back"
        };

        if(state->ChangedMenu)
        {
            state->ChangedMenu = false;

            display->Clear();
            for(int i = 0; i < NUMBER_OF_MAIN_MENU_ITEMS; i++)
            {
                display->SetCursor(i,1);

                display->PrintString(options[i]);
            }
        }

        // clear arrows
        display->SetCursor(state->PreviousSelectedItem,0);
        display->PrintChar(' ');

        state->PreviousSelectedItem = state->SelectedItem;

        display->SetCursor(state->SelectedItem,0);
        display->PrintChar('>');
    }
}

int main() {
    stdio_init_all();

    printf("Initialized stdio\n");
    sleep_ms(2000);
    
    init_sensor_pins();
    init_limit_pins();

    init_led_pins();

    init_motor_pins();
    step_motor(1000, true, MOTOR_DELAY_US);
    sleep_ms(50);
    step_motor(1000, false, MOTOR_DELAY_US);

    init_encoder_pins();

    auto lcd = init_display();

    lcd->SetBacklight(true);
    lcd->SetCursor(0,1);
    lcd->PrintString("Arfsuits.com");

    int defaultDesiredDistance = 3133;
    const int defaultVariation = 10;
    const int maxFramesOutsideRangeBeforeMovement = 7;

    int baseDesiredDistace = defaultDesiredDistance;

    float holdButtonCurrentTimer = 0.0f;
    float holdButtonTimerLength = 1.5f;

    bool paused = false;

    bool buttonPreviouslyPressed = false;

    int framesOutsideRange = 0;
    bool previouslyMoving = false;

    bool encoderChangedThisFrame = false;

    bool startupLoop = true;
    int currentStartupLocation = 0;

    ProgramState state
    {
        .ChangedMenu = true
    };

    bool inMenu = true;

    while (true) {

        const float deltaTime = calculateDeltaTime();
        
        if(encoderIsDifferent())
        {
            encoderChangedThisFrame =true;
            encoder_update();
            lcd->SetCursor(0,20-5);
            char num[32];
            sprintf(num, "%5d", encoder_position);
            printf("%5d\n",encoder_position);
            lcd->PrintString(num);
        }
        else{
            encoderChangedThisFrame = false;
            encoderDirection = 0;
        }

        if(EncoderSwitch())
        {
            holdButtonCurrentTimer += deltaTime;

            buttonPreviouslyPressed = true;
        }else
        {
            if(holdButtonCurrentTimer >= holdButtonTimerLength)
            {
                // reset everything
                fprintf(stdout,"RESET\n");

                baseDesiredDistace = defaultDesiredDistance;
                paused = false;
                buttonPreviouslyPressed = false;
                holdButtonCurrentTimer = 0.0f;
                encoder_position = 0;
                encoderDirection = 0;
                framesOutsideRange = 0;
                previouslyMoving = false;
                encoderChangedThisFrame = false;
                startupLoop = true;
                currentStartupLocation = 0;
                last_state  = 0;
                inMenu = true;
            }
            else if(buttonPreviouslyPressed)
            {
                buttonPreviouslyPressed = false;

                paused = !paused;

                fprintf(stdout,"%s\n",paused ? "PAUSED" : "UNPAUSED");
            }

            holdButtonCurrentTimer = 0.0f;
        }


        if(inMenu)
        {
            if(encoderChangedThisFrame)
            {
                if(encoderDirection > 0)
                {
                    state.SelectedItem = min(state.SelectedItem + 1, NUMBER_OF_MAIN_MENU_ITEMS-1);
                }
                else if(encoderDirection < 0)
                {
                    state.SelectedItem = max(state.SelectedItem - 1 ,0);
                }
            }

            UI_DrawMainMenu(deltaTime,&state, lcd);

            continue;
        }

        // when we first start or reset the device enter a mode
        // where the user can use the wheel to set the desired height
        // and when the user pressed the pause/resume button start moving
        if(startupLoop)
        {
            // if the user presses "pause" while in startup loop
            // resume normal operation
            if(paused)
            {   
                startupLoop = false;
                paused = false;
                // since the user uses the wheel when starting up
                // reset the position when they resume
                encoder_position = 0;

                // whenever the user resumes default to the distance they chose
                baseDesiredDistace = get_distance();

                continue;
            }

            if(currentStartupLocation != encoder_position)
            {
                const int difference = encoder_position - currentStartupLocation;
                
                const bool direction = difference < 0;

                currentStartupLocation = encoder_position;

                step_motor(abs(difference), direction, MOTOR_DELAY_US);
            }
            
            continue;
        }

        if(paused)
        {
            continue;
        }

        int distance  = get_distance();

        const int desiredDistance = baseDesiredDistace + encoder_position;

        // figure what direction the motor needs to go in order to reach the desired distance
        const int difference = distance - desiredDistance;

        const bool outsideOfRange  = abs(difference) > defaultVariation;

        if(outsideOfRange)
        {
            framesOutsideRange++;
            // if we were moving last frame we should move again this frame if we're still outside
            // the range, EVEN if we haven't be outside the range again for several frames
            if((framesOutsideRange >= maxFramesOutsideRangeBeforeMovement) || encoderChangedThisFrame)
            {
                framesOutsideRange = 0;

                const bool direction = difference > 0;

                step_motor(MOTOR_STEPS_PER_FRAME, direction, MOTOR_DELAY_US);
            }
        }
        else
        {
            framesOutsideRange = 0;
        }
    }
}