#include <SoftwareSerial.h>   //가상 시리얼 통신을 위한 라이브러리 선언
#include <IRremote.h> //리모컨 통신을 위한 라이브러리 선언 
#include <Servo.h> //서보모터 통신을 위한 라이브러리 선언 

#define BT_RXD 3  // 아두이노의 4번핀을 RX(받는 핀)로 설정
#define BT_TXD 4 // 아두이노 3번핀을 TX(보내는 핀)로 설정
SoftwareSerial bluetooth(BT_RXD,BT_TXD); //블루투스 통신을 위한 설정 

Servo EduServo;

int RECV_PIN = A0;              // 적외선 수신센서 핀(아날로그 입력 A0)
IRrecv irrecv(RECV_PIN);        // 적외선 송수신 통신을 위한 객체
decode_results IR_Signal;       // 적외선 수신값 해석을 위한 객체


//출력핀(trig)과 입력핀(echo) 설정
int trigPin = 13;                  // 디지털 13번 핀에 연결
int echoPin = 12;                  // 디지털 12번 핀에 연결
int Ultra_d = 0;

int RightMotor_E_pin = 5;       // 오른쪽 모터의 Enable & PWM
int LeftMotor_E_pin = 6;        // 왼쪽 모터의 Enable & PWM
int RightMotor_1_pin = 8;       // 오른쪽 모터 제어선 IN1
int RightMotor_2_pin = 9;       // 오른쪽 모터 제어선 IN2
int LeftMotor_3_pin = 10;       // 왼쪽 모터 제어선 IN3
int LeftMotor_4_pin = 11;       // 왼쪽 모터 제어선 IN4


int L_Line = A5; // 왼쪽 라인트레이서 센서는 A3 핀에 연결
int C_Line = A4; // 가운데 라인트레이서 센서는 A4 핀에 연결
int R_Line = A3; // 오른쪽 라인트레이서 센서는 A5 핀에 연결

int motor_s = 165; // 아두이노 속도 

int R_Motor = 0; //모터
int L_Motor = 0;
int mode = 0;
int val = 0;
int count = 0; //초음파 count 변수생성 2번 초음파 후 while문 빠져나오게 설정 

int SL = 1; //라인트레이서 
int SC = 1;
int SR = 1;

void setup() {
  EduServo.attach(2);  // 서보모터 PWM 디지털입출력 2번핀 연결
  
  pinMode(echoPin, INPUT);   // echoPin 입력
  pinMode(trigPin, OUTPUT);  // trigPin 출력
    
  pinMode(RightMotor_E_pin, OUTPUT); // 출력모드로 설정
  pinMode(RightMotor_1_pin, OUTPUT);
  pinMode(RightMotor_2_pin, OUTPUT);
  pinMode(LeftMotor_3_pin, OUTPUT);
  pinMode(LeftMotor_4_pin, OUTPUT);
  pinMode(LeftMotor_E_pin, OUTPUT);

  Serial.begin(9600);                       // PC와의 시리얼 통신 9600bps로 설정
  bluetooth.begin(9600); //블루투스와 아두이노간 시리얼 통신 속도를 9600bps로 설정  
  irrecv.enableIRIn(); // 적외선 통신 수신 시작
}

void loop() { 
  if(irrecv.decode(&IR_Signal)){      // 적외선(IR) 수신값이 있는지 판단.
    control_SmartCar((String)IR_Signal.value);
    if(mode == 0){
      motor_role(R_Motor, L_Motor, motor_s);
    }
    else if(mode == 1){
      Right_role(R_Motor, L_Motor, motor_s);
    }
    else if(mode == 2){
      Left_role(R_Motor, L_Motor, motor_s);
    }
    else if(mode == 7){ //자동 장애물 회피 (초음파)
      while(count<2){ //회전 2번 후 while문 빠져 나옴 
        Ultra_d = Ultrasonic();
        motor_role(HIGH, HIGH, motor_s);  // 직진
      
        if(Ultra_d < 250) {
          if (Ultra_d < 150) {
            motor_role(LOW, LOW, motor_s); // 후진
            delay(1000);
            motor_role(HIGH, HIGH, 0);  // 정지
            delay(200);
          }
          else {
            motor_role(HIGH, HIGH, 0);  // 정지
            delay(200);
            val =  Servo_con();
            if (val == 0) {
              Serial.println("우회전.");
              motor_role(HIGH, HIGH, 0);  // 정지
              delay(500);
              
              Right_role(LOW, HIGH, motor_s);  // 우회전
              delay(800);
            }
            else if (val == 1) {
              Serial.println("좌회전.");
              motor_role(HIGH, HIGH, 0);  // 정지
              delay(500);
             
              Left_role(HIGH, LOW, motor_s);  // 좌회전
              delay(800);
            }
          }
        }
      }
    }
    else if(mode == 8){ //블루투스 제어 기능 
      while(1){
          if(bluetooth.available()){      
          char Blue_Val = bluetooth.read();
          
          control_SmartCar(Blue_Val);
      
          if(mode == 0){
            motor_role(R_Motor, L_Motor, motor_s);
          }
          else if(mode == 1){
            Right_role(R_Motor, L_Motor, motor_s);
          }
          else if(mode == 2){
            Left_role(R_Motor, L_Motor, motor_s);
          }
          else{
            motor_role(R_Motor, L_Motor, 0);
          }   
        }
      } 
    }
    else{
      motor_role(R_Motor, L_Motor, 0);
    }
    irrecv.resume();  // 다음 적외선 값 수신   
  } 
}

void control_SmartCar(String Remote_Val){ //리모컨 버튼 기능 
  if( Remote_Val == "16754775" ){      // "+" 버튼, 명령 : 속도 증가
    motor_s = motor_s + 20;
    motor_s = min(motor_s, 255);
  }
  else if( Remote_Val == "16769055" ){ // "-" 버튼, 명령 : 속도 감소
    motor_s = motor_s - 20;
    motor_s = max(motor_s, 50);
  }
  else if(Remote_Val == "16718055" ){  // "2" 버튼, 명령 : 전진
    R_Motor = HIGH; L_Motor = HIGH; mode = 0;
  }  
  else if( Remote_Val == "16716015" ){ // "4" 버튼, 명령 : 좌회전
    mode = 2;
  } 
  else if( Remote_Val == "16734885" ){ // "6" 버튼, 명령 : 우회전
    mode = 1;
  }  
  else if( Remote_Val == "16730805" ){ // "8" 버튼, 명령 : 후진
    R_Motor = LOW; L_Motor = LOW; mode = 0;
  }
  else if( Remote_Val == "16726215" ){ // "5" 버튼, 명령 : 정지
    mode = 3;
  }
  else if( Remote_Val =="16728765"){ //"7"버튼, 명령 : 자율주행 
    mode = 7;
  }
  else if( Remote_Val =="16724175"){ //"1"버튼, 명령 : 블루투스 주행 
    mode = 8;
  }
  else{
    Serial.print("Not Defined : ");  // 지정하지 않은 주소입력.
  }
}
void control_SmartCar(char Blue_val){ //블루투스 버튼 조작 기능 
  if( Blue_val == '+' ){      // "+" 버튼, 명령 : 속도 증가
    motor_s = motor_s + 20;
    motor_s = min(motor_s, 255); 
  }
  else if( Blue_val == '-' ){ // "-" 버튼, 명령 : 속도 감소
    motor_s = motor_s - 20;
    motor_s = max(motor_s, 50);
  }
  else if( Blue_val == 'g' ){  // "g" 버튼, 명령 : 전진
    R_Motor = HIGH; L_Motor = HIGH; mode = 0;
  }  
  else if( Blue_val == 'l' ){ // "l" 버튼, 명령 : 좌회전
    mode = 2; 
  }
  else if( Blue_val == 'r' ){ // "r" 버튼, 명령 : 우회전
    mode = 1;
  }
  else if( Blue_val == 'b' ){ // "b" 버튼, 명령 : 후진
    R_Motor = LOW; L_Motor = LOW; mode = 0;
  }
  else if( Blue_val == 's' ){ // "s" 버튼, 명령 : 정지
    mode = 3;
  }
  else if( Blue_val == 'k' ){ // "a" 버튼, 명령 : 라인트레이서
     while(1){ //라인트레이서 기능 while문으로 계속 선을 읽어줌  
        int L = digitalRead(L_Line);
        int C = digitalRead(C_Line);
        int R = digitalRead(R_Line);
        if( L == LOW && C == LOW && R == LOW ){     // 0 0 0
          L = SL; C = SC; R = SR; 
        }
        if( L == LOW && C == HIGH && R == LOW ){    // 0 1 0
          motor_role(HIGH, HIGH, motor_s);
          Serial.println("직진");  
        }
        else if( L == LOW && R == HIGH ){           // 0 1 1, 0 0 1
          motor_role(LOW, HIGH, motor_s); 
          Serial.println("우회전");   
        }
        else if( L == HIGH && R == LOW ){           // 1 1 0, 1 0 0 
          motor_role(HIGH, LOW, motor_s);
          Serial.println("좌회전");    
        }
        else if( L == HIGH &&  R == HIGH ){         // 1 1 1, 1 0 1
          motor_role(HIGH, HIGH, 0); 
          Serial.println("정지");    
        } 
        SL = L; SC = C; SR = R;
      }
  }
}
void motor_role(int R_motor, int L_motor, int Speed){
   digitalWrite(RightMotor_1_pin, R_motor);
   digitalWrite(RightMotor_2_pin, !R_motor);
   digitalWrite(LeftMotor_3_pin, L_motor);
   digitalWrite(LeftMotor_4_pin, !L_motor);
   
   analogWrite(RightMotor_E_pin, Speed);  // 우측 모터 속도값
   analogWrite(LeftMotor_E_pin, Speed);   // 좌측 모터 속도값  
}

void Right_role(int R_motor, int L_motor, int Speed){
   digitalWrite(RightMotor_1_pin, R_motor);
   digitalWrite(RightMotor_2_pin, !R_motor);
   digitalWrite(LeftMotor_3_pin, L_motor);
   digitalWrite(LeftMotor_4_pin, !L_motor);
   
   analogWrite(RightMotor_E_pin, max(Speed*0.2,50));  // 우측 모터 속도값
   analogWrite(LeftMotor_E_pin, min(Speed*1.2,255));   // 좌측 모터 속도값
}

void Left_role(int R_motor, int L_motor, int Speed){
   digitalWrite(RightMotor_1_pin, R_motor);
   digitalWrite(RightMotor_2_pin, !R_motor);
   digitalWrite(LeftMotor_3_pin, L_motor);
   digitalWrite(LeftMotor_4_pin, !L_motor);
   
   analogWrite(RightMotor_E_pin, min(Speed*1.4,255));  // 우측 모터 속도값
   analogWrite(LeftMotor_E_pin, max(Speed*0.2,50));   // 좌측 모터 속도값   
}
//초음파 센서 자율주행 
int Ultrasonic(){
  long duration, distance;
  digitalWrite(trigPin, HIGH);      // trigPin에서 초음파 발생(echoPin도 HIGH)        
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);   // echoPin 이 HIGH를 유지한 시간을 저장 한다.
  distance = ((float)(340 * duration) / 1000) / 2; 

  //Serial.print("DIstance:");        // 물체와 초음파 센서간 거리를 표시.        
  //Serial.println(distance);

  return distance;
}

int Servo_con(){
  EduServo.write(30);
  int Ult_30 = Ultrasonic();
  delay(1000);
  EduServo.write(150);
  int Ult_150 = Ultrasonic();
  delay(1000);

  if(Ult_30 > Ult_150){
     val = 1;
  }
  else{
     val = 0;
  }
  EduServo.write(90);
   
  return val;
}
