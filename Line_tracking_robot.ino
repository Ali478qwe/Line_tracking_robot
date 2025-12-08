//_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_www.behnamrobotic.com

// white -> IR SENOSR -> 0

// black -> IR SENOSR -> 1

// IN1 = 0 , IN2 = 0 || IN1 = 1 , IN2 = 1 -> motor off


/*----IR Sensors Connection----*/

uint8_t Left_IR = 2;    // left sensor

int Right_IR = 3;    // right sensor

/*-------defining Outputs------*/

uint8_t IN1 = 8;       // left motor

int IN2 = 9;     // left motor

int IN3 = 10;    

int IN4 = 11;   

int ENB_Left = 5;   //left motor PWM(ENA)

int ENB_Right = 6;  //right motor PWM(ENB)



static int Speed = 70; //MAX speed is 255

void setup()

{
  

  pinMode(Left_IR,  INPUT);

  pinMode(Right_IR,  INPUT);

  pinMode(IN1, OUTPUT);

  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);

  pinMode(IN4, OUTPUT);

  pinMode(ENB_Left,OUTPUT);

  pinMode(ENB_Right,OUTPUT);

}

void loop(){

  int Dig_IR_Left = digitalRead(Left_IR);

  int Dig_IR_Right = digitalRead(Right_IR);

if ((Dig_IR_Left == 0) && (Dig_IR_Right == 0)) 
{

    MoveForward();//CONDITION-1 FORWARD

}

if ((Dig_IR_Left == 1) && (Dig_IR_Right == 1)) 
{

      Stop();//CONDITION-2 STOP

}

if((Dig_IR_Left == 0) && (Dig_IR_Right == 1))  
{
      TurnRight();// RIGHT
}

if((Dig_IR_Left == 1) && (Dig_IR_Right == 0))
{

      TurnLeft();//Left

}

}

void MoveForward()

{

    digitalWrite(IN1,LOW);

    digitalWrite(IN2,HIGH);

    digitalWrite(IN3,LOW);

    digitalWrite(IN4,HIGH);

    analogWrite(ENB_Left,Speed);

    analogWrite(ENB_Right,Speed);

    delay(20);

}

void TurnRight()

{

    digitalWrite(IN1,LOW);

    digitalWrite(IN2,HIGH);

    digitalWrite(IN3,HIGH);

    digitalWrite(IN4,HIGH);

    analogWrite(ENB_Left,Speed);

    analogWrite(ENB_Right,Speed);

    delay(20);

}

void TurnLeft()

{

    digitalWrite(IN1,HIGH);

    digitalWrite(IN2,HIGH);

    digitalWrite(IN3,LOW);

    digitalWrite(IN4,HIGH);

    analogWrite(ENB_Left,Speed);

    analogWrite(ENB_Right,Speed);

    delay(20);

}

void Stop()

{

    digitalWrite(IN1,LOW);

    digitalWrite(IN2,LOW);

    digitalWrite(IN3,LOW);

    digitalWrite(IN4,LOW);

    delay(20);

}