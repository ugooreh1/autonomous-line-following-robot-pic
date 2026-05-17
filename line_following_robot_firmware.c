// ELE105 Robot Control Program
// Features:
// - Line following using proportional control
// - Perpendicular line detection
// - Lane changes between outer and inner tracks
// - Encoder-based 180 degree turn
// - Automatic braking using AN0 and AN1 distance sensors
// - Final stop with LED flash

#include <xc.h>

#define _XTAL_FREQ 10000000

#pragma config OSC = HS
#pragma config WDT = OFF
#pragma config LVP = OFF
#pragma config PWRT = ON

// Motor direction pins
#define Leftmotor1A  LATAbits.LA4
#define Leftmotor2A  LATAbits.LA5
#define Rightmotor3A LATBbits.LB0
#define Rightmotor4A LATBbits.LB1

// Function declarations
void configPWM(void);
void configI2C(void);
void configEncoders(void);
void configADC(void);

void goforward(void);
void spinRight(void);

void I2C_checkbus_free(void);
void I2C_Start(void);
void I2C_RepeatedStart(void);
void I2C_Stop(void);
void I2C_Write(unsigned char data);
unsigned char I2C_Read(void);
unsigned char readLineSensors(void);

int getLinePosition(unsigned char sensor);
unsigned char detectPerpendicularLine(unsigned char linesensor);
unsigned char confirmPerpendicularLine(void);
void handlePerpendicularLine(void);

void changeLaneToInner(void);
void changeLaneToOuter(void);

void resetEncoders(void);
void checkEncoders(void);
void turn180WithEncoders(void);

int readADC(unsigned char channel);
int getBrakingBaseSpeed(void);

// Global motor speed variables
int markspaceL;
int markspaceR;

// Robot route stage
unsigned char robotStage = 0;

// Last valid line position for line-loss recovery
int lastPosition = 0;

// Encoder variables
unsigned int leftEncoderCount = 0;
unsigned char previousLeftEncoder = 0;

void main(void)
{
    int i;
    int position;
    int baseSpeed;
    int error;
    int correction;

    int K = 15;        // Proportional line-following gain
    int lambda = 1;    // Steering correction multiplier

    unsigned char linesensor;

    // Configure pins
    ADCON1 = 0b00001101;
    TRISA = 0b11001111;
    TRISB = 0b00000000;
    TRISC = 0b00111001;

    LATA = 0;
    LATB = 0;

    // Startup LED flash for 5 seconds
    for(i = 0; i < 5; i++)
    {
        LATBbits.LB2 = 1;
        LATBbits.LB3 = 1;
        LATBbits.LB4 = 1;
        LATBbits.LB5 = 1;
        __delay_ms(500);

        LATBbits.LB2 = 0;
        LATBbits.LB3 = 0;
        LATBbits.LB4 = 0;
        LATBbits.LB5 = 0;
        __delay_ms(500);
    }

    // Configure modules
    configPWM();
    configI2C();
    configEncoders();
    configADC();

    markspaceL = 250;
    markspaceR = 250;

    while(1)
    {
        checkEncoders();

        // Invert sensor reading so 1 means line detected
        linesensor = ~readLineSensors();

        // Detect and handle route-stage perpendicular lines
        if(confirmPerpendicularLine())
        {
            handlePerpendicularLine();
            continue;
        }

        position = getLinePosition(linesensor);

        // Line-loss recovery for invalid or unknown sensor patterns
        if(position == 99)
        {
            if(lastPosition < 0)
            {
                markspaceL = 80;
                markspaceR = 260;
            }
            else
            {
                markspaceL = 260;
                markspaceR = 80;
            }

            goforward();
            continue;
        }

        // Automatic braking adjusts base speed before line-following correction
        baseSpeed = getBrakingBaseSpeed();

        if(baseSpeed == 0)
        {
            markspaceL = 0;
            markspaceR = 0;
            goforward();
            continue;
        }

        // Proportional line-following control
        // reference = 0, so error = 0 - position
        error = 0 - position;
        correction = error * K;

        // Motor speed correction
        markspaceL = baseSpeed - (lambda * correction);
        markspaceR = baseSpeed + (lambda * correction);

        lastPosition = position;

        // Limit PWM values
        if(markspaceL > 1023) markspaceL = 1023;
        if(markspaceR > 1023) markspaceR = 1023;
        if(markspaceL < 0) markspaceL = 0;
        if(markspaceR < 0) markspaceR = 0;

        goforward();
    }
}

// PWM configuration
void configPWM(void)
{
    PR2 = 0b11111111;
    T2CON = 0b00000111;
    CCP1CON = 0b00001100;
    CCP2CON = 0b00001100;
    CCPR1L = 0;
    CCPR2L = 0;
}

// Drive both motors forward using current markspace values
void goforward(void)
{
    Leftmotor1A = 0;
    Leftmotor2A = 1;

    Rightmotor3A = 0;
    Rightmotor4A = 1;

    CCP1CON = (0x0c) | ((markspaceR & 0x03) << 4);
    CCPR1L = markspaceR >> 2;

    CCP2CON = (0x0c) | ((markspaceL & 0x03) << 4);
    CCPR2L = markspaceL >> 2;
}

// I2C configuration for line sensor array
void configI2C(void)
{
    TRISCbits.RC3 = 1;
    TRISCbits.RC4 = 1;

    SSPCON1 = 0b00101000;
    SSPCON2 = 0b00000000;
    SSPADD  = 0x63;
    SSPSTAT = 0b00000000;
}

// ADC configuration for AN0 and AN1 distance sensors
void configADC(void)
{
    ADCON1 = 0b00001101;   // AN0 and AN1 analogue, others digital
    ADCON2 = 0b10101010;   // Right justified ADC result
    ADCON0 = 0b00000001;   // ADC on, channel AN0 selected
}

void I2C_checkbus_free(void)
{
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

void I2C_Start(void)
{
    I2C_checkbus_free();
    SEN = 1;
}

void I2C_RepeatedStart(void)
{
    I2C_checkbus_free();
    RSEN = 1;
}

void I2C_Stop(void)
{
    I2C_checkbus_free();
    PEN = 1;
}

void I2C_Write(unsigned char data)
{
    I2C_checkbus_free();
    SSPBUF = data;
}

unsigned char I2C_Read(void)
{
    unsigned char temp;

    I2C_checkbus_free();
    RCEN = 1;

    I2C_checkbus_free();
    temp = SSPBUF;

    I2C_checkbus_free();
    ACKEN = 1;

    return temp;
}

// Read line sensor byte over I2C
unsigned char readLineSensors(void)
{
    unsigned char linesensor;

    I2C_Start();
    I2C_Write(0x7C);
    I2C_Write(0x11);
    I2C_RepeatedStart();
    I2C_Write(0x7D);
    linesensor = I2C_Read();
    I2C_Stop();

    return linesensor;
}

// Lookup table for line position angle
// Returns 99 for invalid or unrecognised sensor patterns
int getLinePosition(unsigned char sensor)
{
    switch(sensor)
    {
        case 0b00000001: return 12;
        case 0b00000011: return 10;
        case 0b00000010: return 9;
        case 0b00000110: return 7;
        case 0b00000100: return 5;
        case 0b00001100: return 3;
        case 0b00001000: return 2;
        case 0b00011000: return 0;
        case 0b00010000: return -2;
        case 0b00110000: return -3;
        case 0b00100000: return -5;
        case 0b01100000: return -7;
        case 0b01000000: return -9;
        case 0b11000000: return -10;
        case 0b10000000: return -12;

        default: return 99;
    }
}

// Perpendicular line detection
// Detects when at least 6 sensors are active
unsigned char detectPerpendicularLine(unsigned char linesensor)
{
    unsigned char sensorCount = 0;

    if(linesensor & 0b00000001) sensorCount++;
    if(linesensor & 0b00000010) sensorCount++;
    if(linesensor & 0b00000100) sensorCount++;
    if(linesensor & 0b00001000) sensorCount++;
    if(linesensor & 0b00010000) sensorCount++;
    if(linesensor & 0b00100000) sensorCount++;
    if(linesensor & 0b01000000) sensorCount++;
    if(linesensor & 0b10000000) sensorCount++;

    return (sensorCount >= 6);
}

// Confirms perpendicular line using three readings to reduce false triggers
unsigned char confirmPerpendicularLine(void)
{
    unsigned char sensor1;
    unsigned char sensor2;
    unsigned char sensor3;

    sensor1 = ~readLineSensors();
    __delay_ms(20);

    sensor2 = ~readLineSensors();
    __delay_ms(20);

    sensor3 = ~readLineSensors();

    if(detectPerpendicularLine(sensor1) &&
       detectPerpendicularLine(sensor2) &&
       detectPerpendicularLine(sensor3))
    {
        return 1;
    }

    return 0;
}

// Handles route behaviour at each perpendicular line
void handlePerpendicularLine(void)
{
    int i;

    // Short feedback flash when perpendicular line is detected
    LATBbits.LB2 = 1;
    LATBbits.LB3 = 1;
    LATBbits.LB4 = 1;
    LATBbits.LB5 = 1;
    __delay_ms(200);

    LATBbits.LB2 = 0;
    LATBbits.LB3 = 0;
    LATBbits.LB4 = 0;
    LATBbits.LB5 = 0;

    if(robotStage == 0)
    {
        // Outer clockwise complete: change to inner lane
        changeLaneToInner();

        lastPosition = 0;
        robotStage = 1;
        return;
    }
    else if(robotStage == 1)
    {
        // Inner clockwise complete: stop, turn 180 degrees, continue anticlockwise
        markspaceL = 0;
        markspaceR = 0;
        goforward();
        __delay_ms(5000);

        turn180WithEncoders();

        markspaceL = 280;
        markspaceR = 280;
        goforward();
        __delay_ms(300);

        lastPosition = 0;
        robotStage = 2;
        return;
    }
    else if(robotStage == 2)
    {
        // Inner anticlockwise complete: change back to outer lane
        changeLaneToOuter();

        lastPosition = 0;
        robotStage = 3;
        return;
    }
    else if(robotStage == 3)
    {
        // Final perpendicular line: stop and flash LEDs for 5 seconds
        markspaceL = 0;
        markspaceR = 0;
        goforward();

        for(i = 0; i < 5; i++)
        {
            LATBbits.LB2 = 1;
            LATBbits.LB3 = 1;
            LATBbits.LB4 = 1;
            LATBbits.LB5 = 1;
            __delay_ms(500);

            LATBbits.LB2 = 0;
            LATBbits.LB3 = 0;
            LATBbits.LB4 = 0;
            LATBbits.LB5 = 0;
            __delay_ms(500);
        }

        markspaceL = 0;
        markspaceR = 0;
        goforward();

        while(1)
        {
            // Final stop state
        }
    }

    // Move forward briefly to clear the perpendicular line
    markspaceL = 230;
    markspaceR = 230;
    goforward();
    __delay_ms(350);
}

// Tuned lane change from outer to inner lane
void changeLaneToInner(void)
{
    markspaceL = 470;
    markspaceR = 130;
    goforward();
    __delay_ms(650);

    markspaceL = 230;
    markspaceR = 230;
    goforward();
    __delay_ms(150);

    markspaceL = 110;
    markspaceR = 500;
    goforward();
    __delay_ms(650);

    markspaceL = 300;
    markspaceR = 300;
    goforward();
    __delay_ms(200);

    lastPosition = 0;
}

// Tuned lane change from inner back to outer lane
void changeLaneToOuter(void)
{
    markspaceL = 470;
    markspaceR = 130;
    goforward();
    __delay_ms(650);

    markspaceL = 230;
    markspaceR = 230;
    goforward();
    __delay_ms(150);

    markspaceL = 110;
    markspaceR = 500;
    goforward();
    __delay_ms(650);

    markspaceL = 300;
    markspaceR = 300;
    goforward();
    __delay_ms(200);

    lastPosition = 0;
}

// Encoder input configuration
void configEncoders(void)
{
    TRISCbits.RC0 = 1;
    previousLeftEncoder = PORTCbits.RC0;
}

void resetEncoders(void)
{
    leftEncoderCount = 0;
    previousLeftEncoder = PORTCbits.RC0;
}

// Counts rising edges from left encoder on RC0
void checkEncoders(void)
{
    unsigned char currentLeftEncoder;

    currentLeftEncoder = PORTCbits.RC0;

    if(currentLeftEncoder == 1 && previousLeftEncoder == 0)
    {
        leftEncoderCount++;
    }

    previousLeftEncoder = currentLeftEncoder;
}

// Encoder-based 180 degree spin
void turn180WithEncoders(void)
{
    resetEncoders();

    markspaceL = 350;
    markspaceR = 350;
    spinRight();

    while(leftEncoderCount < 220)   // Tuned encoder count for 180 degree turn
    {
        checkEncoders();
    }

    markspaceL = 0;
    markspaceR = 0;
    goforward();
    __delay_ms(300);

    lastPosition = 0;
}

// Spin right on the spot: left wheel forward, right wheel backward
void spinRight(void)
{
    Leftmotor1A = 0;
    Leftmotor2A = 1;

    Rightmotor3A = 1;
    Rightmotor4A = 0;

    CCP1CON = (0x0c) | ((markspaceR & 0x03) << 4);
    CCPR1L = markspaceR >> 2;

    CCP2CON = (0x0c) | ((markspaceL & 0x03) << 4);
    CCPR2L = markspaceL >> 2;
}

// Read ADC value from selected analogue channel
int readADC(unsigned char channel)
{
    unsigned int timeout = 0;

    ADCON0 = ((channel & 0x0F) << 2) | 0b00000001;

    __delay_us(30);

    ADCON0bits.GO = 1;

    while(ADCON0bits.GO && timeout < 1000)
    {
        timeout++;
    }

    if(timeout >= 1000)
    {
        return 0;
    }

    return (((int)ADRESH << 8) | ADRESL);
}

// Automatic braking using AN0 and AN1 distance sensors
int getBrakingBaseSpeed(void)
{
    int leftSensor;
    int rightSensor;
    int sensorValue;
    int baseSpeed;

    leftSensor = readADC(0);
    rightSensor = readADC(1);

    // Use the sensor with the highest reading, meaning closest object
    if(leftSensor > rightSensor)
        sensorValue = leftSensor;
    else
        sensorValue = rightSensor;

    if(sensorValue > 350)
    {
        baseSpeed = ((1023 - sensorValue) * 330) / 1023;

        if(sensorValue > 550)
        {
            baseSpeed = 0;
        }
        else if(baseSpeed < 100)
        {
            baseSpeed = 100;
        }
    }
    else
    {
        baseSpeed = 330;
    }

    return baseSpeed;
}
