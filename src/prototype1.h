#define READ_DELAY 10
#define BT_TIMEOUT 1000

//Constants for range finder
#define ECHO_PIN 2
#define TRIG_PIN 3
#define SOUND_SPEED_CM 0.0344
#define OFFSET 4
#define MIN_COL_DISTANCE 30

//Constants for motor driver
#define ENA 11
#define ENB 10
#define OUT1_LB 9 //LB = left black
#define OUT2_LR 8 //LR = left red
#define OUT3_RB 7 //RB = right black
#define OUT4_RR 6 //RR = right red
#define TRUEMAX_SPD 255 //Real speed limit that the motor can go which is 255
#define MAX_SPD 100
#define MAX_TURN_SPD 80
#define MIN_SPD 60

void setup();
void rangeFinderInit();
void motorDriverInit();
void loop();
bool checkCollision();
void runMotorDriver();
void changeDirection(int , int , int , int );
void changeSpeed(int , int );
int clampSpeed(int , int = MIN_SPD, int = MAX_SPD);
void getXYValues();
