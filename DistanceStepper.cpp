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

#define abs(value) ((value) < 0 ? (-value) : (value))
#define max(value,upper) ((value) < (upper) ? upper : value)
#define min(value,lower) ((value) > (lower) ? lower : value)


float min = 9999.0f;
float max = 3.3f;
float range = 0.0f;
float value = 0.0f;
volatile bool zLimitMin = false;
volatile bool zLimitMax = false;


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

    int value = voltage * 1000;

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

struct RotaryEncoder
{
private:
    uint8_t LastState = 0;
    uint8_t NewState = 0;

    uint8_t clk = 0;
    uint8_t data = 0;
public:
    int Position = 0;
    int Direction = 0;

    void Init()
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

        LastState = (gpio_get(ENCODER_CLK) << 1) | gpio_get(ENCODER_DATA);
    }

    void Update() 
    {
        if ((LastState == 0b00 && NewState == 0b01) ||
            (LastState == 0b01 && NewState == 0b11) ||
            (LastState == 0b11 && NewState == 0b10) ||
            (LastState == 0b10 && NewState == 0b00)) 
        {
            if(Direction != -1)
            {
                Position++;  // CW
                LastState = NewState;
                Direction = 1;
            }
            else
            {
                Direction = 0;
            }
        } 
        else if (
            (LastState == 0b00 && NewState == 0b10) ||
            (LastState == 0b10 && NewState == 0b11) ||
            (LastState == 0b11 && NewState == 0b01) ||
            (LastState == 0b01 && NewState == 0b00)) 
        {
            if(Direction != 1)
            {
                Position--;  // CCW
                LastState = NewState;
                Direction = -1;
            }
            else
            {
                Direction = 0;
            }
        }
    }

    bool Different()
    {
        clk = gpio_get(ENCODER_CLK);
        data = gpio_get(ENCODER_DATA);
        
        NewState = (clk << 1) | data;
        
        return NewState != LastState;
    }

    bool Pressed()
    {
        return !gpio_get(ENCODER_SW);
    }
};

uint64_t last_time = time_us_64();

float calculateDeltaTime()
{
    uint64_t now = time_us_64();
    float dt = (now - last_time) / 1e6f; // delta time in seconds
    last_time = now;

    return dt;
}

struct Timer
{
    float Length;
    float Time;

    void Update(float deltaTime)
    {
        Time += deltaTime;
    }

    bool UpdateAndCheck(float deltaTime)
    {
        Update(deltaTime);

        return Time >= Length;
    }

    bool Check()
    {
        return Time >= Length;
    }

    void Reset()
    {
        Time = 0.0f;
    }

    bool CheckAndReset()
    {
        bool result = Time >= Length;

        Reset();

        return result;
    }
};

struct ProgramState
{
    RotaryEncoder Encoder;

    int SelectedItem;
    int PreviousSelectedItem;

    bool Paused;

    bool ChangedMenu;

    bool ButtonPressedThisFrame;
    bool ButtonUpThisFrame;
    bool ButtonPreviouslyPressed;

    bool EncoderChangedThisFrame;

    Timer ButtonHeldTimer;
};

#define NUMBER_OF_MAIN_MENU_ITEMS 4

#define MENU(name) void UI_##name(const float deltaTime, ProgramState* state, LCD_I2C* display)

MENU(DrawMainMenu)
{
    static const char* options[NUMBER_OF_MAIN_MENU_ITEMS] = {
            "Calibrate",
            "Start",
            "Info",
            "Back"
        };

    if(state->PreviousSelectedItem != state->SelectedItem)
    {
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

    if(state->ButtonUpThisFrame)
    {
        printf("%s",options[state->SelectedItem]);
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

    ProgramState state
    {
        .ChangedMenu = true,
        .ButtonHeldTimer = {
            .Length = 2
        }
    };

    state.Encoder.Init();

    auto lcd = init_display();

    lcd->SetBacklight(true);
    lcd->SetCursor(0,1);
    lcd->PrintString("ARFsuits.com");

    

    bool inMenu = true;

    while (true) {

        const float deltaTime = calculateDeltaTime();
        
        if(state.Encoder.Different())
        {
            state.EncoderChangedThisFrame =true;
            state.Encoder.Update();

            lcd->SetCursor(0,20-5);
            char num[32];
            sprintf(num, "%5d", state.Encoder.Position);
            printf("%5d\n",state.Encoder.Position);
            lcd->PrintString(num);
        }
        else{
            state.EncoderChangedThisFrame = false;
            state.Encoder.Direction = 0;
        }

        if(state.Encoder.Pressed())
        {
            state.ButtonHeldTimer.Update(deltaTime);

            state.ButtonPressedThisFrame = true;
        }else
        {
            if(state.ButtonHeldTimer.Check())
            {
                // reset everything
                fprintf(stdout,"RESET\n");
                inMenu = true;
            }
            else if(state.ButtonPreviouslyPressed)
            {
                state.ButtonPressedThisFrame = false;
                state.ButtonUpThisFrame = true;
                state.ButtonPreviouslyPressed = false;

                state.Paused = !state.Paused;

                fprintf(stdout,"%s\n",state.Paused ? "PAUSED" : "UNPAUSED");
            }
            
            state.ButtonHeldTimer.Reset();
        }

        if(inMenu)
        {
            if(state.EncoderChangedThisFrame)
            {
                if(state.Encoder.Direction > 0)
                {
                    state.SelectedItem = min(state.SelectedItem + 1, NUMBER_OF_MAIN_MENU_ITEMS-1);
                }
                else if(state.Encoder.Direction < 0)
                {
                    state.SelectedItem = max(state.SelectedItem - 1 ,0);
                }
            }

            UI_DrawMainMenu(deltaTime,&state, lcd);

            continue;
        }
    }
}