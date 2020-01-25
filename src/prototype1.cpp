#include <Arduino.h>
#include "prototype1.h"

//Bluetooth serial reading
String msg;
char serial_read;

//Motor driver variables
int x;
int y;

bool isColliding = false;

float btTimeoutTimer;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(9600);

    //Initialize range finder
    rangeFinderInit();

    //Initialize motor driver
    motorDriverInit();
}

void rangeFinderInit()
{
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void motorDriverInit()
{
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(OUT1_LB, OUTPUT);
    pinMode(OUT2_LR, OUTPUT);
    pinMode(OUT3_RB, OUTPUT);
    pinMode(OUT4_RR, OUTPUT);

    digitalWrite(OUT1_LB, LOW);
    digitalWrite(OUT2_LR, LOW);
    digitalWrite(OUT3_RB, LOW);
    digitalWrite(OUT4_RR, LOW);
}

void loop()
{
    // put your main code here, to run repeatedly:
    if (Serial.available() > 0)
    {
        serial_read = char(Serial.read());
        if (serial_read == '<')
        {
            getXYValues();
            btTimeoutTimer = millis();
        }

        runMotorDriver();
    }
    else
    {
        //Stop car if did not receive read for certain duration (BT_TIMEOUT = 1 second)
        if (x != 0 && y != 0 && millis() - btTimeoutTimer >= BT_TIMEOUT)
        {
            x = 0;
            y = 0;
            runMotorDriver();
            Serial.println("Timeout. Stopped.");
        }
    }
}

bool checkCollision()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    float duration = pulseIn(ECHO_PIN, HIGH);
    float distance = (duration / 2) * SOUND_SPEED_CM;
    distance = distance > 0 ? distance + OFFSET : distance;

    return distance < MIN_COL_DISTANCE;
}

void runMotorDriver()
{
    if (!checkCollision() || y < 0)
    {
        //Map the speed to the min and max value defined
        int absoluteY = abs(y);
        int absoluteX = abs(x);
        int leftSpeed, rightSpeed;
        int blackOutput, redOutput;
        int lb, lr, rb, rr;

        //If y is more than the min speed to rotate the motors (motors require minimum voltage to start turning)
        //Then move forward else check x value (create the rotation behaviour)
        if (absoluteY >= MIN_SPD)
        {
            //Use X value to calculated the speed ratio of the motors
            //To simulate the ability to turn left and right while moving forward
            leftSpeed = absoluteY * min(1, (float)(TRUEMAX_SPD + x) / TRUEMAX_SPD);
            rightSpeed = absoluteY * min(1, (float)(TRUEMAX_SPD - x) / TRUEMAX_SPD);

            //Map the speed to the min and max speed defined
            leftSpeed = map(leftSpeed, 0, TRUEMAX_SPD, MIN_SPD, MAX_SPD);
            rightSpeed = map(rightSpeed, 0, TRUEMAX_SPD, MIN_SPD, MAX_SPD);

            //Sets the motors rotation direction (lb & rb = 1 => forward, lr & rr = 1 => reverse)
            //Get output for black and red wire of motor output (returns 0 or 1, red is opposite of black)
            int normalizedY = y / absoluteY;
            blackOutput = max(0, normalizedY);
            redOutput = max(0, -normalizedY);

            //Use x value to invert the rotation when y is 0
            //This is to allow full rotation where
            lb = blackOutput;
            lr = redOutput;
            rb = blackOutput;
            rr = redOutput;
        }
        else if (absoluteX > 0)
        {
            //map the value to min and max speed to get variable rotation speed
            leftSpeed = rightSpeed = map(absoluteX, 0, TRUEMAX_SPD, MIN_SPD, MAX_TURN_SPD);

            int normalizedX = x / absoluteX;

            //Rotate right = lb(1), lr(0), rb(0), rr(1)
            //Rotate left = lb(0), lr(1), rb(1), rr(0)
            blackOutput = max(0, normalizedX);
            redOutput = max(0, -normalizedX);

            lb = blackOutput;
            lr = redOutput;

            //Basically inverts the left outputs
            rb = redOutput;
            rr = blackOutput;
        }
        changeDirection(lb, lr, rb, rr);
        changeSpeed(leftSpeed, rightSpeed);        
    }
    else
    {
        changeSpeed(0, 0);
    }
}

void changeDirection(int LB, int LR, int RB, int RR)
{
    digitalWrite(OUT1_LB, LB);
    digitalWrite(OUT2_LR, LR);
    digitalWrite(OUT3_RB, RB);
    digitalWrite(OUT4_RR, RR);
}

void changeSpeed(int leftSpeed, int rightSpeed)
{
    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
}

void getXYValues()
{
    msg = "";
    //Read x value (format x[val]y[val])
    while (serial_read != 'y')
    {
        delay(READ_DELAY);
        serial_read = char(Serial.read());

        if (serial_read != 'y')
        { //skips the y char
            if (serial_read != 'x')
            { //skips the x char
                msg += serial_read;
            }
        }
        else
        {
            x = msg.toInt();
            msg = "";
        }
    };

    while (serial_read != '>')
    {
        delay(READ_DELAY);
        serial_read = char(Serial.read());

        if (serial_read != '>')
        {
            msg += serial_read;
        }
        else
        {
            y = msg.toInt();
        }
    }

    //Serial.print("Received: ");
    //Serial.println("X: " + String(x) + ", Y: " + String(y));
}
