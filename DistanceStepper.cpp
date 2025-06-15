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
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

#define VERBOSE 1
#define ENCODER_LOGGING 0
#define SENSOR_LOGGING 0
#define MOTOR_LOGGING 0
#define UI_LOGGING 1

#define abs(value) ((value) < 0 ? (-value) : (value))
#define max(value,upper) ((value) < (upper) ? upper : value)
#define min(value,lower) ((value) > (lower) ? lower : value)
#define INFO(format,...) if(VERBOSE){printf(format,__VA_ARGS__);} 
#define INFO_IF(condition,format,...) if(VERBOSE && (condition)){printf(format,__VA_ARGS__);} 
#define ENCODER_INFO(format,...) if(ENCODER_LOGGING){INFO(format,__VA_ARGS__)}
#define ENCODER_INFO_IF(condition,format,...) if(ENCODER_LOGGING && (condition)){INFO(format,__VA_ARGS__)}
#define SENSOR_INFO(format,...) if(SENSOR_LOGGING){INFO(format,__VA_ARGS__)}
#define MOTOR_INFO(format,...) if(MOTOR_LOGGING){INFO(format,__VA_ARGS__)}
#define UI_INFO(format,...) if(UI_LOGGING){INFO(format,__VA_ARGS__)}

#define assert_soft(condition,format,...) if(!(condition)){INFO(format,__VA_ARGS__); }
#define wait_while(condition) while(condition){sleep_us(1);}
#define sign(num) ((num) < 0 ? -1 : 1);


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
#define MOTOR_DELAY_US 25
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
private:
    volatile bool Enabled = false;
    volatile bool PreviousDirection;
    volatile int DesiredPosition = 0;
public:
    volatile int Position = 0;

    volatile bool Synchronous = false;

    volatile bool Direction;
    volatile bool LimitMotion = false;

    volatile int LowerLimit = 0;
    volatile int UpperLimit = 0;

    volatile int MotorSpeed_us = MOTOR_DELAY_US;

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
        if(!Enabled)
        {
            Enabled = true;
            gpio_put(ENABLE_PIN, 0);
        }
    }

    void Disable()
    {
        if(Enabled)
        {
            Enabled = false;
            gpio_put(ENABLE_PIN, 1);
        }
    }

    void SetDirection(bool direction)
    {
        if(Synchronous)
        {
            gpio_put(DIR_PIN, Direction);
            PreviousDirection = direction;
        }
        Direction = direction;
    }

    void Step(int delay_us)
    {
        if(Direction != PreviousDirection)
        {
            gpio_put(DIR_PIN, Direction);
            PreviousDirection = Direction;
        }

        if(Direction)
        {
            if(LimitMotion && (Position + 1) > UpperLimit)
            {
                return;
            }

            Position++;
        }else{
            if(LimitMotion && (Position - 1) < LowerLimit)
            {
                return;
            }

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

    void GotoPosition(int pos, int delay_us)
    {
        bool direction = pos >= Position;
        int steps = pos - Position;

        if(steps != 0)
        {
            MOTOR_INFO("Moving motor to position: %li (%li steps to the %s)\n", pos, steps, direction ? "right" : "left");

            TurnSimple(abs(steps), direction, delay_us);
        }
    }

    void SetPositionAsync(int position)
    {
        assert_soft(!Synchronous, "ERROR: attempted to set the async position of the motor while it's in a synchronous state\n","");

        DesiredPosition = position;
        Direction = DesiredPosition > Position;
    }

    // BLOCKING;
    // This method blocks the current thread and monitors
    // for commands for the motor and moves the motor appropriately
    void MonitorForCommands()
    {
        do
        {
            if(Enabled && !Synchronous)
            {
                if(Position != DesiredPosition)
                {
                    Step(MotorSpeed_us);
                }
            }
        } while (1);
    }

    std::string PositionToString(int value)
    {
        char motorPos[6];
        sprintf(motorPos,"%6d", value);
        return std::string(motorPos);
    }

    void Print(Display display, int row, int col)
    {
        char motorPos[5];
        sprintf(motorPos,"%5d",Position);
        display->SetCursor(row,col);
        display->PrintString(motorPos);
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
    Timer ButtonDebounceTimer = Timer(2.0/10.0);
    Timer ButtonHeldTimer = Timer(1);
    float LongHoldLength = 2.0f;

    int PositionLastFrame = 0;
    int Position = 0;
    int Direction = 0;
    int StepScale = 1;

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
    bool ButtonLongHeldThisFrame = false;
    
    bool ButtonUpLastFrame = false;
    bool ButtonHeldLastFrame = false;
    bool PreviouslyHeldButton = false;



    void Init()
    {
        ENCODER_INFO("Initializing encoder\n","");

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
        ButtonLongHeldThisFrame = false;
        ButtonHeldThisFrame = false;
        ButtonUpThisFrame = false;
        ButtonDownThisFrame = false;

        // rotation
        if(Different())
        {
            Recalculate();
            ChangedThisFrame = PositionLastFrame != Position;
            
            ENCODER_INFO_IF(Position != PositionLastFrame, "pos: %li\n", Position);

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
            if(ButtonHeldTimer.Check())
            {
                if(ButtonHeldTimer.Value >= LongHoldLength)
                {
                    ENCODER_INFO("Button Long Held\n","");
                    ButtonLongHeldThisFrame = true;
                }

                ENCODER_INFO("Button Held\n","");
                ButtonHeldThisFrame = true;
                ButtonHeldLastFrame = true;
                PreviouslyHeldButton = true;
                ButtonPreviouslyPressed = false;

                // debounce cooldown
                ButtonDebounceTimer.Reset();
            }
            else if(ButtonPreviouslyPressed)
            {
                // ignore first up since held
                if(!PreviouslyHeldButton)
                {
                    ENCODER_INFO("Button Up\n","");
                    ButtonUpThisFrame = true;
                    ButtonPreviouslyPressed = false;

                    // debounce cooldown
                    ButtonDebounceTimer.Reset();
                }
                
                PreviouslyHeldButton = false;
            }

            ButtonHeldTimer.Reset();
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
                PreviouslyHeldButton = false;

                // debounce cooldown
                ButtonDebounceTimer.Reset();
            }
        }

        ENCODER_INFO_IF(
            ButtonHeldThisFrame || ButtonUpThisFrame || ButtonDownThisFrame,
            "Down: %s, Up: %s, Held: %s\n",
            ButtonDownThisFrame ? "true" : "false",
            ButtonUpThisFrame ? "true" : "false",
            ButtonHeldThisFrame ? "true" : "false"
        );

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
                Position += StepScale;  // CW
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
                Position -= StepScale;  // CCW
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
};

struct ProgramState
{
    RotaryEncoder Encoder;
    StepMotor Motor;
    DistanceSensor Sensor;

    int CurrentMenu;

    bool Paused;

    int CurrentCalibrationStep;
    int CurrentMainStep;

    int DesiredDistance;
};

#define NUMBER_OF_MAIN_MENU_ITEMS 3

#define MENU(name) void UI_##name(ProgramState* state, LCD_I2C* display)

MENU(Main)
{
    static const char* options[NUMBER_OF_MAIN_MENU_ITEMS] = {
            "Calibrate",
            "Start",
            "Info"
        };

    display->Clear();

    static char mainMenuInteractionTable [] = {
        90+7<<7>>7,
        110+4<<4>>4,
        100+2<<2>>2,
        110+5<<5>>5,
        110+7<<7>>7,
        100+5<<5>>5,
        110+6<<6>>6,
        110+5<<5>>5,
        40+6<<6>>6,
        90+9<<9>>9,
        110+1<<1>>1,
        100+9<<9>>9
    };

    for(int i = 0; i < NUMBER_OF_MAIN_MENU_ITEMS; i++)
    {
        display->SetCursor(i,1);

        display->PrintString(options[i]);
    }

    char* s = mainMenuInteractionTable;

    int selectedItem = 0;
    int previousSelectedItem = 1;
    
    display->SetCursor(3,20-12);
    display->PrintString(s);

    do{
        if(state->Encoder.ChangedThisFrame)
        {
            if(state->Encoder.Direction > 0)
            {
                selectedItem = min(selectedItem + 1, NUMBER_OF_MAIN_MENU_ITEMS-1);
            }
            else if(state->Encoder.Direction < 0)
            {
                selectedItem = max(selectedItem - 1 ,0);
            }

            display->SetCursor(0,19);
            display->PrintString("" + selectedItem);
        }

        if(previousSelectedItem != selectedItem)
        {
            // clear arrows
            display->SetCursor(previousSelectedItem, 0);
            display->PrintChar(' ');

            previousSelectedItem = selectedItem;

            display->SetCursor(selectedItem, 0);
            display->PrintChar('>');
        }

        if(state->Encoder.ButtonUpThisFrame)
        {
            // since we selected a different menu exit this UI loop
            UI_INFO("%s:%li\n",options[selectedItem],selectedItem);
            state->CurrentMenu = selectedItem;
            return;
        }
    } while(Time.Update() + state->Encoder.Update());
}

MENU(Info)
{
    display->Clear();
    display->SetCursor(3,0);
    display->PrintString(">Back");

    // motor can't move in info screen
    display->SetCursor(0,0);
    display->PrintString("Motor: " + state->Motor.PositionToString(state->Motor.Position) + " " + state->Motor.PositionToString(state->Motor.LowerLimit));
    display->SetCursor(1,20-6);
    display->PrintString(state->Motor.PositionToString(state->Motor.UpperLimit));

    display->SetCursor(2,0);
    bool hasSensorAtStart = state->Sensor.Connected();
    display->PrintString("      Sensor: " + std::string( hasSensorAtStart ? "Y" : "N"));
    
    display->SetCursor(3,6);
    display->PrintString("Encodr:");
    state->Encoder.Print(display, 3, 20-5);

    int previousDistance = -1;
    bool hadSensorLastCheck = hasSensorAtStart;
    char previousSensorString[4];
    do{

        if(state->Encoder.ChangedThisFrame)
        {
            state->Encoder.Print(display,3,20-5);
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
            state->CurrentMenu = -1;
            return;
        }
    }while(Time.Update() + state->Encoder.Update());
}

MENU(SetHeight)
{
    display->Clear();

    display->SetCursor(0,5);

    display->PrintString("Set Height");

    display->SetCursor(1,6);
    display->PrintString("Distance");

    // reset position to be the motors position
    state->Encoder.Position = state->Motor.Position / 100;

    // turn off the limits of the motor so it can extend past its motion limits
    state->Motor.Enable();

    bool fastMode = false;

    int previousDistance = 0;
    char previousSensorString[4];
    bool hadSensorLastCheck = state->Sensor.Connected();

    do{
        // print sensor status
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
                    display->SetCursor(2,8+i);
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
            
        if(state->Encoder.ChangedThisFrame)
        {
            int newPosition = state->Encoder.Position * 100;
            bool direction = newPosition >= state->Motor.Position;
            int steps = newPosition - state->Motor.Position;

            state->Motor.SetDirection(direction);

            for(int i = 0; i < abs(steps);i++)
            {
                state->Motor.Step(MOTOR_DELAY_US);
            }
        }

        if(state->Encoder.ButtonHeldThisFrame)
        {   
            fastMode = !fastMode;
            state->Encoder.StepScale = fastMode ? 5 : 1;
            display->SetCursor(3,20-4);
            display->PrintString(fastMode ? "fast" : "    ");
        }

        if(state->Encoder.ButtonUpThisFrame)
        {
            state->Encoder.StepScale = 1;
            state->Motor.Disable();

            UI_INFO("Set desired distance: %4d\n", previousDistance);
            state->DesiredDistance = previousDistance;

            // go to ranging menu to start ranging
            state->CurrentMenu = 40;
            
            return;
        }
    }while(Time.Update() + state->Encoder.Update());
}

MENU(DrawRangeElements)
{
    display->Clear();

    display->SetCursor(0,7);
    display->PrintString("PAUSED");

    display->SetCursor(1,3);
    display->PrintString("Distance: " + std::to_string(state->DesiredDistance));
    
    display->SetCursor(3,0);
    display->PrintString("Push:Paws  Hold:Quit");
}

MENU(Range)
{
    UI_DrawRangeElements(state, display);

    bool paused = true;

    DistanceSensor& sensor = state->Sensor;
    StepMotor& motor = state->Motor;

    bool displayingErrorScreen = false;

    int previousDistance = 0;
    char previousSensorString[4];

    // the time that it takes before the motor resumes
    // after the sensor has been reconnected
    Timer reconnectSensorTimer = Timer(0.5);
    Timer sensorMissingTimer = Timer(0.1);

    bool enabled = false;

    const int sensorNoiseMargin = 10;
    const int desiredDistance = state->DesiredDistance;

    // allow movement
    motor.Synchronous = false;
    motor.Enable();
    motor.LimitMotion = false;

    do{
        sensor.Update();

        // make sure the sensor is working before moving motor to avoid crashes
        if(!sensor.Connected())
        {
            reconnectSensorTimer.Reset();

            if(!displayingErrorScreen && sensorMissingTimer.UpdateAndCheck())
            {
                displayingErrorScreen = true;

                display->Clear();
                display->SetCursor(0,7);
                display->PrintString("ERROR");

                display->SetCursor(1,0);
                display->PrintString("SENSOR NOT DETECTED");

                display->SetCursor(3,0);
                display->PrintString("Push:Paws  Hold:Quit");
            }

            continue;
        }
        else if(displayingErrorScreen)
        {
            if(reconnectSensorTimer.UpdateAndCheck())
            {
                // reset GUI
                displayingErrorScreen = false;
                UI_DrawRangeElements(state, display);
            }
            else
            {
                continue;
            }
        }
        
        int distance = sensor.Distance;

        if(!paused)
        {
            distance = sensor.Distance;

            // update display
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
                        display->SetCursor(2,8+i);
                        display->PrintChar(current);
                        previousSensorString[i] = current;
                    }
                }
            }

            int difference = desiredDistance-distance;

            int absDiff = abs(difference);

            if(absDiff > sensorNoiseMargin)
            {
                // move motor
                int steps = (difference >> 1) * 100;

                motor.SetPositionAsync(motor.Position + steps);
            }
        }

        if(state->Encoder.ButtonHeldThisFrame)
        {
            UI_INFO("quitting to main menu\n","");
            state->CurrentMenu = -1;
            motor.Disable();
            return;
        }

        if(state->Encoder.ButtonUpThisFrame)
        {
            paused = !paused;
            UI_INFO("%s\n",paused ? "paused" : "unpaused");

            display->SetCursor(0,7);
            display->PrintString(paused ? "PAUSED" : "ACTIVE");
        }
    }while(Time.Update() + state->Encoder.Update());
}

MENU(Calibrate)
{
    if(state->CurrentCalibrationStep > 1)
    {
        state->CurrentCalibrationStep = 0;
    }

    display->Clear();

    display->SetCursor(0,2);

    display->PrintString(state->CurrentCalibrationStep == 0 ? "Set Lower Height" : "Set Upper Height");

    display->SetCursor(1,3);
    display->PrintString("Motor Position");

    display->SetCursor(3,0);
    display->PrintString("Previously: " + state->Motor.PositionToString(state->CurrentCalibrationStep == 0 ? state->Motor.LowerLimit / 100 : state->Motor.UpperLimit / 100));

    // reset position to be the motors position
    state->Encoder.Position = state->Motor.Position / 100;

    // turn off the limits of the motor so it can extend past its motion limits
    state->Motor.LimitMotion = false;
    state->Motor.Enable();

    int previousNumberLength = 0;
    
    bool fastMode = false;

    do{
        if(state->Encoder.ChangedThisFrame)
        {
            int newPosition = state->Encoder.Position * 100;
            bool direction = newPosition >= state->Motor.Position;
            int steps = newPosition - state->Motor.Position;

            state->Motor.SetDirection(direction);

            for(int i = 0; i < abs(steps);i++)
            {
                state->Motor.Step(MOTOR_DELAY_US);
            }
            
            int numberLength = snprintf(NULL, 0, "%d", state->Encoder.Position);
            int column = 10 - (numberLength/2);
            if(numberLength != previousNumberLength)
            {
                previousNumberLength = numberLength;
                display->SetCursor(2,5);
                display->PrintString("           ");
            }

            display->SetCursor(2,column);
            display->PrintString(std::to_string(state->Encoder.Position));
        }

        if(state->Encoder.ButtonHeldThisFrame)
        {   
            fastMode = !fastMode;
            state->Encoder.StepScale = fastMode ? 5 : 1;
            display->SetCursor(2,20-4);
            display->PrintString(fastMode ? "fast" : "    ");
        }

        if(state->Encoder.ButtonUpThisFrame)
        {
            if(state->CurrentCalibrationStep == 0)
            {
                state->Motor.LowerLimit = state->Motor.Position;
            }
            if(state->CurrentCalibrationStep == 1)
            {
                state->Motor.UpperLimit = state->Motor.Position;
            }
            
            state->Encoder.StepScale = 1;
            state->Motor.Disable();
            
            // resume forcing the motor to stop when exceeding the limits
            state->Motor.LimitMotion = true;
            
            // go back to main menu
            if(state->CurrentCalibrationStep == 1)
            {
                state->CurrentCalibrationStep = 0;
                state->CurrentMenu = -1;
            }
            else{
                state->CurrentCalibrationStep++;
                // return to this menu
                state->CurrentMenu = 0;
            }
            
            return;
        }
    }while(Time.Update() + state->Encoder.Update());
}

ProgramState* Global_State;

// responsible for driving the motor asynchronously
void main_core_1()
{
    INFO("Starting Second Core","");

    Global_State->Motor.MonitorForCommands();
}

int main() {
    stdio_init_all();

    printf("Initialized stdio\n");
    sleep_ms(2000);

    
    ProgramState state
    {
        .CurrentMenu = -1
    };
    
    Global_State = &state;
    
    multicore_launch_core1(main_core_1);
    
    auto display = init_display();
    
    display->SetBacklight(true);
    display->SetCursor(0,4);
    display->PrintString("ARFsuits.com");
    
    display->SetCursor(2, 0);
    display->PrintString("Z-Axis Laser Ranger");
    
    init_led_pins();

    state.Sensor.Init();

    // initialize and test motor for movement
    state.Motor.Init();

    state.Motor.Enable();
    state.Motor.Synchronous = false;
    state.Motor.SetPositionAsync(6400);
    wait_while(state.Motor.Position != 6400);
    state.Motor.SetPositionAsync(0);
    wait_while(state.Motor.Position != 0);
    state.Motor.Disable();

    sleep_ms(1000);

    state.Motor.Synchronous = true;
    state.Motor.TurnSimple(6400, true, MOTOR_DELAY_US);
    sleep_ms(100);
    state.Motor.TurnSimple(6400, false, MOTOR_DELAY_US);

    state.Encoder.Init();

    while (true) {

        // make sure to update deltaTime
        Time.Update();
        state.Encoder.Update();

        UI_INFO("Selecting Menu: %li\n", state.CurrentMenu);
        
        switch(state.CurrentMenu)
        {
            case 0:
                UI_Calibrate(&state, display);
                break;
            case 1:
                UI_SetHeight(&state, display);
                break;
            case 2:
                UI_Info(&state, display);
                break;
            case 40:
                UI_Range(&state, display);
                break;
            default: // -1
                UI_Main(&state, display);
                break;
        }
    }
}