//_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_www.behnamrobotic.com

// white -> IR SENOSR -> 0 -> -> 24 -> led on

// black -> IR SENOSR -> 1 -> 690 -> azad -> led off

// IN1 = 0 , IN2 = 0 || IN1 = 1 , IN2 = 1 -> motor off

static int last_time  = 0;

float kp = 2 ; 
float ki = 0 ;
float kd = 1 ;

static int last_error = 0 ; 
static float integral = 0 ;




typedef struct {
    int left_speed;
    int right_speed;
    long min_speed;
    int avrage_speed;
    long max_speed;
    } Motor_speed;

static Motor_speed obj; 


void speed_control(int ADC_Left , int  ADC_Right ){
   static int left_buf [5];
   static int right_buf [5];
   int total_left  = 0;
   int total_right = 0;
   bool flag_rihgt = false;
   bool flag_left =  false;
   static int idx = 0;

    left_buf[idx]  = ADC_Left;
    right_buf[idx] = ADC_Right;
    idx++;
    if(idx >= 5) idx = 0;

    for(int i = 0 ; i < 5 ; i++)
    {
      total_left  += left_buf  [i];
      total_right += right_buf [i];
    }

    int avg_left  = total_left / 5 ;
    int avg_right = total_right / 5 ; 


     int left_ratio = 1 , right_ratio = 1;
    // int white_color = min(ADC_Left , ADC_Right);
    // int black_color = max(ADC_Left , ADC_Right);
    // int left_speed = map(ADC_Left,1023,0,obj.min_speed,obj.max_speed);
    // int right_speed = map(ADC_Right,1023,0,obj.min_speed,obj.max_speed);
    // obj.left_speed = left_speed; 
    // obj.right_speed = right_speed;
    int error = ADC_Left - ADC_Right ;
    integral += error ;
    int derivative = error - last_error ;
    last_error = error;
    int output = (kp * error) + (ki * integral) + (kd * derivative);

    int  left_motor  = obj.avrage_speed - output ; 
    int  right_motor = obj.avrage_speed + output ; 

    int left_speed = map(left_motor,-65025,65025,obj.min_speed,obj.max_speed);
    int right_speed = map(right_motor,-65025,65025,obj.min_speed,obj.max_speed);

    // constrain(   left_motor    , obj.min_speed , obj.max_speed )
    // constrain(right_motor, obj.min_speed , obj.max_speed )
    if(error < 1 && avg_right > avg_left){ right_ratio = 1.5 ; left_ratio = 0;}
    if(error > 1 && avg_left > avg_right){ right_ratio = 0 ; left_ratio = 1.5;  }

    obj.left_speed  = (left_speed * left_ratio)  + ((right_ratio == 0)? 20 : 0) ;
    obj.right_speed = (right_speed * right_ratio) + ((left_ratio == 0)? 20 : 0) ;

}

void findmotion_speed(){
    for(uint8_t i = 0 ; i <= 255 ; i++){
        obj.avrage_speed = (int)i ; 
        Serial.println((int)i);
        delay(50);

    }
}

/*----IR Sensors Connection----*/

uint8_t Left_IR = 2;    // left sensor
uint8_t Right_IR = 3;    // right sensor

uint8_t Right_IR_ADC = A0; 
uint8_t Left_IR_ADC = A1; 

bool flag = false;
bool subflag = false;

/*-------defining Outputs------*/

uint8_t IN1 = 8;       
uint8_t IN2 = 9;
uint8_t IN3 = 10;    
uint8_t IN4 = 11;   
uint8_t ENB_Left = 5;  
uint8_t ENB_Right = 6;




// static int Speed = 70; //MAX speed is 255

void setup()

{
  Serial.begin(9600);
  obj.min_speed = 65;//80 
  obj.max_speed = 100;//90
  obj.avrage_speed = 70;//83


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
    
   int now = millis();

  int Dig_IR_Left  = digitalRead(Left_IR);

  int Dig_IR_Right = digitalRead(Right_IR);

  int ADC_Left  = analogRead(Left_IR_ADC);

  int ADC_Right = analogRead(Right_IR_ADC);

  speed_control(ADC_Left, ADC_Right);
   
//   analogWrite(ENB_Left,obj.left_speed);//

//   analogWrite(ENB_Right,obj.right_speed);//
  
   

    if(flag){
    if(now - last_time >= 2000){
        subflag = true;
    Serial.print("analog left : ");
    Serial.println(ADC_Left);
    Serial.print("analog right : ");
    Serial.println(ADC_Right);
    Serial.print("digital left : ");
    Serial.println(Dig_IR_Left);
    Serial.print("digital right : ");
    Serial.println(Dig_IR_Right);
     Serial.print("left speed: ");
    Serial.println(obj.left_speed);
    Serial.print("right speed: ");
    Serial.println(obj.right_speed);
     last_time = now;
    }
    flag = false;
   
    }
    


    if (Dig_IR_Left == 0 && Dig_IR_Right == 0) 
    {

    flag = true;
    MoveForward();//CONDITION-1 FORWARD
    

    }
    
  

    if(Dig_IR_Left == 0 && Dig_IR_Right == 1) 
    {
    // Serial.println("right");
    flag = true;        
    TurnRight();// RIGHT
    }

    if(Dig_IR_Left == 1 && Dig_IR_Right == 0)
    {
  
    flag = true;
    TurnLeft();//Left

    }

    //  if (Dig_IR_Left == 1 && Dig_IR_Right == 1) 
    // {

    // flag = true;
    // Stop();//CONDITION-2 STOP

    // }

}

void MoveForward()

{
    if(subflag) {Serial.println(__func__);subflag = false;}

    // for( uint8_t i = obj.min_speed ; i <= obj.avrage_speed ; i++){
    //     // obj.avrage_speed = (int)i ; 
    //     Serial.println((int)i);

    //  }

    digitalWrite(IN1,LOW);

    digitalWrite(IN2,HIGH);

    digitalWrite(IN3,LOW);

    digitalWrite(IN4,HIGH);
 

    analogWrite(ENB_Left,obj.left_speed);//

    analogWrite(ENB_Right,obj.right_speed);//150
}

void TurnRight()

{
    if(subflag) {Serial.println(__func__);subflag = false;}
    digitalWrite(IN1,LOW);

    digitalWrite(IN2,HIGH);

    digitalWrite(IN3,LOW);

    digitalWrite(IN4,HIGH);


    analogWrite(ENB_Left,obj.left_speed);//

    analogWrite(ENB_Right,obj.right_speed);//



}

void TurnLeft()

{
  if(subflag) {Serial.println(__func__);subflag = false;}
    digitalWrite(IN1,LOW);

    digitalWrite(IN2,HIGH);

    digitalWrite(IN3,LOW);

    digitalWrite(IN4,HIGH);

   analogWrite(ENB_Left,obj.left_speed);//

   analogWrite(ENB_Right,obj.right_speed);//

    

}

void Stop()

{
     if(subflag) {Serial.println(__func__);subflag = false;}
    digitalWrite(IN1,LOW);

    digitalWrite(IN2,LOW);

    digitalWrite(IN3,LOW);

    digitalWrite(IN4,LOW);

    

}