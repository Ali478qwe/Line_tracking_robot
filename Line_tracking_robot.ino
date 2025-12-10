//_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_www.behnamrobotic.com

// white -> IR SENOSR -> 0 -> -> 24 -> led on

// black -> IR SENOSR -> 1 -> 690 -> azad -> led off

// IN1 = 0 , IN2 = 0 || IN1 = 1 , IN2 = 1 -> motor off

static int last_time  = 0;

float kp = 0.097 ; // har che kamtr deqat bishtar
// float ki = 10;
float kd = 0.05 ;

static int last_error = 0 ; 
static float integral = 0 ;
static float output = 0; 




typedef struct {
    int left_speed;
    int right_speed;
    long min_speed;
    int normal_speed;
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

    for(int i = 0 ; i < 5 ; i++) // --->>>> errr dare benazram
    {
      total_left  += left_buf  [i];
      total_right += right_buf [i];
    }

    int avg_left  = total_left / 5 ;
    int avg_right = total_right / 5 ; 


     int left_ratio = 1 , right_ratio = 1;

    int error = avg_left - avg_right ; // -1023 to +1023 -> mizan enheraf az line 
    int derivative = error - last_error ; // -2046 to +2046 -> tanzim sorat va pishbin jahat
    last_error = error; 
    
    
    // Serial.println(error);
    // integral += error ;  // niazz nist (ki * integral) // baray jelo gir az enheraf va khatay gozashteh majmo kol khataha

    output = ((kp * error) +  (kd * derivative));
    output  = constrain((int)output,0,obj.max_speed);
    
    int  left_motor  = obj.normal_speed - output ; 
    int  right_motor = obj.normal_speed + output ; 

    int left_speed  = constrain(   left_motor    , 0, 255 );
 
    int right_speed = constrain(right_motor, 0 , 255 );


    // if(error < 1 && avg_right > avg_left){ right_ratio = 1.5 ; left_ratio = 0;}
    // if(error > 1 && avg_left > avg_right){ right_ratio = 0 ; left_ratio = 1.5;  }

    obj.left_speed  = (left_speed * left_ratio) ;// + ((right_ratio == 0)? 20 : 0) ;
    obj.right_speed = (right_speed * right_ratio); // + ((left_ratio == 0)? 20 : 0) ;

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

void findmotion_speed(){
    digitalWrite(IN1,LOW);

    digitalWrite(IN2,HIGH);

    digitalWrite(IN3,LOW);

    digitalWrite(IN4,HIGH);
    for(uint8_t i = 0 ; i <= 255 ; i++){
       
    analogWrite(ENB_Left,i);//

    analogWrite(ENB_Right,i);//150
    Serial.println((int)i);
        delay(50);

    }
}


// static int Speed = 70; //MAX speed is 255

void setup()

{
  Serial.begin(9600);
  obj.min_speed = 65;//80 
  obj.max_speed = 100;//90
  obj.normal_speed = 65;//83


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

//   findmotion_speed();

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
    Serial.println(output);
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