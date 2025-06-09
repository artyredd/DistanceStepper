#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "stdio.h"
#include "external/LCD_I2C.hpp"
#include <pico/binary_info/code.h>
#include <hardware/i2c.h>
#include "pico/binary_info.h"
#include <string>
#include <functional>
#include <vector>

#define VERBOSE 1
#define ENCODER_LOGGING 1
#define SENSOR_LOGGING 0

#define abs(value) ((value) < 0 ? (-value) : (value))
#define max(value,upper) ((value) < (upper) ? upper : value)
#define min(value,lower) ((value) > (lower) ? lower : value)
#define INFO(format,...) if(VERBOSE){printf(format,__VA_ARGS__);} 
#define ENCODER_INFO(format,...) if(ENCODER_LOGGING){INFO(format,__VA_ARGS__)}
#define SENSOR_INFO(format,...) if(SENSOR_LOGGING){INFO(format,__VA_ARGS__)}

typedef LCD_I2C* Display;

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

// encoder
#define ENCODER_SW 8
#define ENCODER_CLK 9
#define ENCODER_DATA 17

void init_led_pins()
{
    printf("Initializing LED indicator\n");

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1); // turn on LED
}

Display init_display()
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

struct GlobalTime
{
public:
    // Last Calculated deltaTime, needs Update() to be ran at the beginning of the main loop
    // before this will update
    float deltaTime;

    float Update()
    {
        uint64_t now = time_us_64();
        float dt = (now - last_time) / 1e6f; // delta time in seconds
        last_time = now;

        deltaTime = dt;

        return dt;
    }

private:
    uint64_t last_time = time_us_64();
};

static GlobalTime Time;

struct Timer
{
    float Length;
    float Value;

    Timer() = default;
    Timer(float length) : Length(length){}

    void Update()
    {
        Value += Time.deltaTime;
    }

    bool UpdateAndCheck()
    {
        Update();

        return Value >= Length;
    }

    bool Check()
    {
        return Value >= Length;
    }

    void Reset()
    {
        Value = 0.0f;
    }

    bool CheckAndReset()
    {
        bool result = Value >= Length;

        Reset();

        return result;
    }
};

struct DistanceSensor
{
public:
    int Distance = 3300;

    void Init()
    {
        SENSOR_INFO("Initializing sensors\n","");

        adc_init();
        adc_gpio_init(IR_SENSOR_ADC_PIN);
        adc_select_input(0);  // ADC0

        // check the voltage immediately
        // so the caller knows at startup if the
        // sensor is connected
        Update();
    }

    bool Connected()
    {
        // 3300 means no sensor connected
        return Distance != 3300;
    }

    int Update()
    {
        uint16_t raw = adc_read();  // 12-bit: 0–4095
        
        const float voltage = raw * 3.3f / 4095;

        const int value = voltage * 1000;

        SENSOR_INFO("%li :", value);

        Distance = value;

        return Distance;
    }
};

struct StepMotor
{
public:
    int Position = 0;
    bool Direction;

    void Init()
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

    void Enable()
    {
        gpio_put(ENABLE_PIN, 0);
    }

    void Disable()
    {
        gpio_put(ENABLE_PIN, 1);
    }

    void SetDirection(bool direction)
    {
        gpio_put(DIR_PIN, direction);
        Direction = direction;
    }

    void Step(int delay_us)
    {
        if(Direction)
        {
            Position++;
        }else{
            Position--;
        }
        
        gpio_put(STEP_PIN, 1);
        sleep_us(delay_us);
        gpio_put(STEP_PIN, 0);
        sleep_us(delay_us);
    }

    void TurnSimple(int steps, bool direction, int delay_us) {
        Enable();
        SetDirection(direction);
        for (int i = 0; i < steps; ++i) {
            Step(delay_us);
        }
        Disable();
    }
};

struct RotaryEncoder
{
private:
    uint8_t LastState = 0;
    uint8_t NewState = 0;

    uint8_t clk = 0;
    uint8_t data = 0;
public:
    // ignore button presses if they happened within
    // this length of time
    Timer ButtonDebounceTimer = Timer(1.0/10.0);
    Timer ButtonHeldTimer = Timer(2);

    int PositionLastFrame = 0;
    int Position = 0;
    int Direction = 0;

    bool ChangedThisFrame = false;

    // Whether or not the user is pressing the button, does not neccessarily mean
    // it started or ended this frame
    bool ButtonPressedThisFrame = false;
    bool ButtonPreviouslyPressed = false;
    // first frame the button was released after pressing
    bool ButtonUpThisFrame = false;
    // First frame that the button was held for longer than the timer length
    bool ButtonHeldThisFrame = false;
    // First frame the button is pressed
    bool ButtonDownThisFrame = false;
    
    bool ButtonUpLastFrame = false;


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

    int Update()
    {
        ButtonHeldThisFrame = false;
        ButtonUpThisFrame = false;
        ButtonDownThisFrame = false;

        // rotation
        if(Different())
        {
            Recalculate();
            ChangedThisFrame = PositionLastFrame != Position;
            PositionLastFrame  = Position;
        }
        else{
            ChangedThisFrame = false;
            Direction = 0;
        }

        bool pressingButton = Pressed();

        ButtonDebounceTimer.Update();

        if(!ButtonDebounceTimer.Check())
        {
            return Position;
        }

        ButtonPressedThisFrame = pressingButton;

        if(!ButtonPressedThisFrame)
        {
            if(ButtonHeldTimer.CheckAndReset())
            {
                ENCODER_INFO("Button Held\n","");
                ButtonHeldThisFrame = true;

                // debounce cooldown
                ButtonDebounceTimer.Reset();
            }
            else if(ButtonPreviouslyPressed)
            {
                ENCODER_INFO("Button Up\n","");
                ButtonUpThisFrame = true;
                ButtonPreviouslyPressed = false;

                // debounce cooldown
                ButtonDebounceTimer.Reset();
            }
        }

        if(ButtonPressedThisFrame)
        {
            // keep track of how long it's pressed
            ButtonHeldTimer.Update();

            if(!ButtonPreviouslyPressed)
            {
                ENCODER_INFO("Button Down\n","");
                ButtonPreviouslyPressed = true;
                ButtonDownThisFrame = true;

                // debounce cooldown
                ButtonDebounceTimer.Reset();
            }
        }

        return Position;
    }

    void Recalculate() 
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

    void Print(Display display, int row, int col)
    {
        display->SetCursor(row,col);
        
        ENCODER_INFO("%5d\n",Position);

        char num[32];
        sprintf(num, "%5d", Position);
        display->PrintString(num);
    }

    void Print(Display display)
    {
        Print(display, 0, 20-5);
    }
};

struct ProgramState
{
    RotaryEncoder Encoder;
    StepMotor Motor;
    DistanceSensor Sensor;

    int SelectedItem;
    int PreviousSelectedItem;

    bool Paused;

    bool ChangedMenu;
};

#define NUMBER_OF_MAIN_MENU_ITEMS 4

#define MENU(name) void UI_##name(ProgramState* state, LCD_I2C* display)

MENU(UpdateSelectedItem)
{
    if(state->Encoder.ChangedThisFrame)
    {
        state->Encoder.Print(display);

        if(state->Encoder.Direction > 0)
        {
            state->SelectedItem = min(state->SelectedItem + 1, NUMBER_OF_MAIN_MENU_ITEMS-1);
        }
        else if(state->Encoder.Direction < 0)
        {
            state->SelectedItem = max(state->SelectedItem - 1 ,0);
        }
    }
}

MENU(DrawMainMenu)
{
    static const char* options[NUMBER_OF_MAIN_MENU_ITEMS] = {
            "Calibrate",
            "Start",
            "Info",
            "Back"
        };

    display->Clear();
    for(int i = 0; i < NUMBER_OF_MAIN_MENU_ITEMS; i++)
    {
        display->SetCursor(i,1);

        display->PrintString(options[i]);
    }

    do{
        UI_UpdateSelectedItem(state, display);

        if(state->PreviousSelectedItem != state->SelectedItem)
        {
            // clear arrows
            display->SetCursor(state->PreviousSelectedItem,0);
            display->PrintChar(' ');

            state->PreviousSelectedItem = state->SelectedItem;

            display->SetCursor(state->SelectedItem,0);
            display->PrintChar('>');
        }

        if(state->Encoder.ButtonUpThisFrame)
        {
            // since we selected a different menu exit this UI loop
            printf("%s:%li\n",options[state->SelectedItem],state->SelectedItem);
            return;
        }
    } while(Time.Update() + state->Encoder.Update());
}

MENU(DrawInfo)
{
    display->Clear();
    display->SetCursor(3,0);
    display->PrintString("> Back");

    display->SetCursor(0,0);
    display->PrintString("   Encoder Pos:");

    // motor can't move in info screen
    char motorPos[5];
    sprintf(motorPos,"%5d",state->Motor.Position);
    display->SetCursor(1,0);
    display->PrintString("     Motor Pos:" + std::string(motorPos));

    display->SetCursor(2,0);
    bool hasSensorAtStart = state->Sensor.Connected();
    display->PrintString("      Sensor: " + std::string( hasSensorAtStart ? "Y" : "N"));

    int previousDistance = 0;
    bool hadSensorLastCheck = hasSensorAtStart;
    char previousSensorString[4];
    do{

        if(state->Encoder.ChangedThisFrame)
        {
            state->Encoder.Print(display,0,20-5);
        }

        int distance = state->Sensor.Update();

        if(previousDistance != distance)
        {
            previousDistance = distance;

            char str[4];
            sprintf(str,"%4d", distance);

            for(int i = 0; i < 4; i++)
            {
                char previous = previousSensorString[i];
                char current = str[i];
                if(previous != current)
                {
                    display->SetCursor(2,20-4+i);
                    display->PrintChar(current);
                    previousSensorString[i] = current;
                }
            }
        }

        bool hasSensor = state->Sensor.Connected();

        if(hadSensorLastCheck != hasSensor)
        {
            hadSensorLastCheck = hasSensor;
            
            display->SetCursor(2, 14);
            display->PrintChar(hasSensor ? 'Y' : 'N');
        }
        
        if(state->Encoder.ButtonUpThisFrame)
        {
            // go back to main menu
            state->SelectedItem = 0;
            return;
        }
    }while(Time.Update() + state->Encoder.Update());
}

int main() {
    stdio_init_all();

    printf("Initialized stdio\n");
    sleep_ms(2000);
    
    auto display = init_display();

    display->SetBacklight(true);
    display->SetCursor(0,4);
    display->PrintString("ARFsuits.com");

    display->SetCursor(2, 0);
    display->PrintString("Z-Axis Laser Ranger");

    display->SetCursor(3,4);
    display->PrintString("[turn knob]");

    ProgramState state
    {
        .ChangedMenu = true,
    };

    init_led_pins();

    state.Sensor.Init();
    state.Motor.Init();

    state.Motor.TurnSimple(1000, true, MOTOR_DELAY_US);
    sleep_ms(50);
    state.Motor.TurnSimple(1000, false, MOTOR_DELAY_US);

    state.Encoder.Init();

    bool inMenu = true;

    while (true) {

        // make sure to update deltaTime
        Time.Update();
        state.Encoder.Update();

        if(inMenu)
        {

            switch(state.SelectedItem)
            {
                case 2:
                    UI_DrawInfo(&state, display);
                default:
                    UI_DrawMainMenu(&state, display);
            }
        }
    }
}