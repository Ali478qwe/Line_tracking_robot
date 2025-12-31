/*____________________DATA ANLYSIS____________________*/

// white -> IR SENOSR VALUE : Digital -> 0 , Analog -> 24  -> led on

// black -> IR SENOSR VALUE : Digital -> 1 , Analog -> 690 -> led off

// IN1 = 0 , IN2 = 0 || IN1 = 1 , IN2 = 1 -> motor off

// [ IN1 = 0 , IN2 = 1  - > Moving forward ] , [ IN1 = 1 , IN2 = 0 -> Move backward ]

/*____________________STRUCTER TYPE____________________*/

typedef struct {
    int left_speed;
    int right_speed;
    int min_speed;
    int max_speed;
    int normal_speed;
    int min_pwm;
    int max_pwm;
    float output_pwm;
    } Motor_speed;

static Motor_speed obj; // INESTANCE FOR MOTOR CONTROL


/*____________________VAIABLES FOR SAMPLING____________________*/

static int last_time = 0; // FOR TIME SAMPLING
int AVG_Left  = 0;
int AVG_Right = 0; 

bool PrintSerialflag  = false;
bool ChangeMethodflag = false;



/*____________________IR SENSOR CONNECTION____________________*/

const uint8_t Left_IR = 2; 
const uint8_t Right_IR = 3;

const uint8_t Right_IR_ADC = A0; 
const uint8_t Left_IR_ADC = A1; 

/*____________________OUTPUTS PIN FOR L298N____________________*/

const uint8_t IN1 = 8;       
const uint8_t IN2 = 9;
const uint8_t IN3 = 10;    
const uint8_t IN4 = 11;   
const uint8_t ENB_Left = 5;  
const uint8_t ENB_Right = 6;

/*____________________PID____________________*/

float kp = 0.065; // max_speed / 1023 = 0.117 -> 1023 * 0.117 = speed
float ki = 0.0009;// ratio -> 10 / 10230 -> 0.0048 * 10230 = 50
float kd = 0.017 ; // max_speed / 2046 = 0.058  -> 2046 * 0.058   = accurc

static int   last_error = 0; 
static float integral   = 0;



void ADC_AVG_Sampling(int * ADC_Left , int * ADC_Right){
  static int Left_Buf  [5];
  static int Right_Buf [5];
  static int idx = 0;
  static int count = 0;
  int Total_Left  = 0;
  int Total_Right = 0;
  

  Left_Buf[idx]  = *ADC_Left;
  Right_Buf[idx] = *ADC_Right;
  idx++;
  if(idx >= 5) idx = 0;

  if (count < 5)
        count++;

  for(int i = 0 ; i < count ; i++)
  {
  Total_Left  += Left_Buf  [i];
  Total_Right += Right_Buf [i];
  }

  AVG_Left  = Total_Left / (count ? count : 1);
  AVG_Right = Total_Right / (count ? count : 1); 

}


void speed_control(int AVG_Left , int  AVG_Right ){

  int error = AVG_Left - AVG_Right ; // -1023 to +1023 -> ERROR RANGE FOR LINE DIRECTION DETECTION

  int derivative = error - last_error ; // -2046 to +2046 -> THIS RANGE -> TIP : SPEED ​​AND SMOOTH MOVEMENT ADAPTATION AND PATH PREDICATION

  last_error = error; 

  integral += error;  
  if(integral > 10230) integral = 10230;
  if(integral < 10230) integral = -10230;
  //
  obj.output_pwm = (kp * error) +  (kd * derivative) + (ki * integral);
  obj.output_pwm  = constrain((int)obj.output_pwm,0,obj.max_speed);
  
  int  left_motor  = obj.normal_speed - obj.output_pwm ; 
  int  right_motor = obj.normal_speed + obj.output_pwm ; 

  int left_speed  = constrain(left_motor, obj.min_pwm,obj.max_pwm);
  int right_speed = constrain(right_motor,obj.min_pwm,obj.max_pwm);

  obj.left_speed  = left_speed  ; 
  obj.right_speed = right_speed ;

}

void findmotion_speed(){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);

    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);

    for(uint8_t i = 0 ; i <= 255 ; i++)
    { 
      analogWrite(ENB_Left,i);
      analogWrite(ENB_Right,i);

      Serial.println((int)i);
      delay(50);
    }
}

void setup(){


  Serial.begin(9600);

  //obj.min_speed = 70;//65 
  obj.max_speed = 100;//90
  obj.normal_speed = 65;//83
  obj.min_pwm = 0;
  obj.max_pwm = 255;

  pinMode(Left_IR, INPUT);
  pinMode(Right_IR, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENB_Left, OUTPUT);
  pinMode(ENB_Right, OUTPUT);

}

void loop(){
    
  int now = millis();

  int Dig_IR_Left  = digitalRead(Left_IR);
  int Dig_IR_Right = digitalRead(Right_IR);

  int  ADC_Left  = analogRead(Left_IR_ADC);
  int  ADC_Right = analogRead(Right_IR_ADC);

  ADC_AVG_Sampling(&ADC_Left,&ADC_Right);

  speed_control(AVG_Left,AVG_Right);

  // set_speed();   


  if(PrintSerialflag && now - last_time >= 2000)
  {
    ChangeMethodflag = true;
    Serial.println(String("analog avg left : "  + String(AVG_Left)));
    Serial.println(String("analog avg right : "  + String(AVG_Right)));;
    Serial.println(String("digital left : "  + String(Dig_IR_Left)));
    Serial.println(String("digital right : " + String(Dig_IR_Right)));
    Serial.println(String("left speed : "    + String(obj.left_speed)));
    Serial.println(String("right speed : "   + String(obj.right_speed)));
    Serial.println(String("output : "        + String(obj.output_pwm)));
     Serial.println(String("integral : "     + String(integral)));
    last_time = now;    
    PrintSerialflag = false; 
  }

  if(Dig_IR_Left == 0 && Dig_IR_Right == 0){set_speed(obj.left_speed,obj.right_speed);}
  if(Dig_IR_Left == 1 && Dig_IR_Right == 0){set_speed(0,obj.right_speed);}
  if(Dig_IR_Left == 0 && Dig_IR_Right == 1){set_speed(obj.left_speed,0);}



  if (Dig_IR_Left == 1 && Dig_IR_Right == 1) 
  {
    PrintSerialflag = true;
    Stop();
  }else
  {
    PrintSerialflag = true;        
    MoveForward();
  }

}

void set_speed(int left_speed,int right_speed){
    analogWrite(ENB_Left,left_speed);//obj.left_speed
    analogWrite(ENB_Right,right_speed);//obj.right_speed
}

void MoveForward(){
    if(ChangeMethodflag) {Serial.println(__func__);ChangeMethodflag = false;}
  
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);

    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
}

void Stop(){
    if(ChangeMethodflag) {Serial.println(__func__);ChangeMethodflag = false;}
    ki = 0;
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);

    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
}