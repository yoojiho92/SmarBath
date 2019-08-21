#include <Adafruit_MLX90614.h> // 비접촉식 온도측정센서 라이브러리 불러오기

#define DEBUG 1

const int watersensor1 = A10;   // 찬물 수위 감지 센서
const int watersensor2 = A9;    // 뜨거운물 수위 감지 센서
const int watersensor3 = A8;    // 욕조물 수위 감지 센서


// 모터 1, 2, 3
const int RELAYPIN1 = 2;        // 찬물 모터 제어 릴레이 핀
const int RELAYPIN2 = 3;        // 뜨거운물 모터 제어 릴레이 핀
const int RELAYPIN3 = 4;        // 욕조 모터 제어 릴레이 핀


const int WATERSENSOR1_LIMIT = 250;   // 찬물 한계 값
const int WATERSENSOR2_LIMIT = 250;   // 뜨거운물 한계 값
const int WATERSENSOR3_LIMIT = 250;   // 욕조물 한계 값

int sensor1_val = 0;          // 찬물 전류변화값 변수선언
int sensor2_val = 0;          // 뜨거운물 전류변화값 변수선언
int sensor3_val = 0;          // 욕조 전류변화값 변수선언

String bt_Response_Str = "";
boolean btFlag = false;
boolean bath = false;


boolean mortor_1_enable = false;
boolean mortor_2_enable = false;
boolean mortor_3_enable = false;


Adafruit_MLX90614 mlx = Adafruit_MLX90614();     //적외선 온도센서 생성
unsigned char Re_buf[11], counter=0;
unsigned char sign = 0;
float T0=0, TA=0;

void Mortor1_run();       // 모터 1 작동 함수
void Mortor1_stop();      // 모터 1 정지 함수

void Mortor2_run();       // 모터 2 작동 함수
void Mortor2_stop();      // 모터 2 정지 함수

void Mortor3_run();       // 모터 3 작동 함수
void Mortor3_stop();      // 모터 3 정지 함수

void Watersensor1_ck();       // 수위센서1(차가운물)를 검사하여 물이 없을시 모터 정지
void Watersensor2_ck();       // 수위센서2(뜨거운물)를 검사하여 물이 없을시 모터 정지
void Watersensor3_ck();       // 수위센서3(욕조물)를 검사하여 물이 없을시 모터 정지
void SetBath(int water, int temp_in); // 욕조에 temp_in만큼의 온도에 water(물의양)만큼의 물을 받기
void testBath(int water);     // 테스트
void sendState();             // 블루투스로 현재 수위 전송

void Mortor1_run(){
  sendState();
  if(!mortor_1_enable){
    mortor_1_enable = true;
    if(DEBUG){
      Serial.println("모터 1 작동");
    }
    digitalWrite(RELAYPIN1,LOW);
  } 
}

void Mortor1_stop(){
  if(mortor_1_enable){
   mortor_1_enable = false;
   if(DEBUG){
    Serial.println("모터 1 멈춤");
   }
   digitalWrite(RELAYPIN1,HIGH);
  }
   
}

void Mortor2_run(){
  sendState();
  if(!mortor_2_enable){
    mortor_2_enable = true;
    if(DEBUG){
      Serial.println("모터 2 작동");
    }
    digitalWrite(RELAYPIN2,LOW);
  }
} 

void Mortor2_stop(){
  if(mortor_2_enable){
    mortor_2_enable = false;
    if(DEBUG){
      Serial.println("모터 2 멈춤");
    }
    digitalWrite(RELAYPIN2,HIGH);
  }
}

void Mortor3_run(){
  sendState();
  if(!mortor_3_enable){
    mortor_3_enable = true;
    if(DEBUG){
      Serial.println("모터 3 작동");
    }
    digitalWrite(RELAYPIN3,LOW);
  }
}

void Mortor3_stop(){
  if(mortor_3_enable){
    mortor_3_enable = false;
    if(DEBUG){
      Serial.println("모터 3 멈춤");
    }
    digitalWrite(RELAYPIN3,HIGH);
  }
}

void Watersensor1_ck(){
  sensor1_val = analogRead(watersensor1);   // watersansor1 의 변화값(전류값)을 읽음
  
  if (sensor1_val < WATERSENSOR1_LIMIT){                    // 찬물의 전류값이 WATERSENSOR1_LIMIT 이하일 경우 
    Serial.println("찬물이 부족합니다.");
    Mortor1_stop();
  }
  if(DEBUG){
    Serial.print("수위센서 1 :");
    Serial.println(sensor1_val);
  }
}

void Watersensor2_ck(){
  sensor2_val = analogRead(watersensor2);   // watersansor2 의 변화값(전류값)을 읽음
  
  if (sensor2_val < WATERSENSOR2_LIMIT){                    // 찬물의 전류값이 WATERSENSOR2_LIMIT 이하일 경우 
    Serial.println("뜨거운물이 부족합니다.");
    Mortor2_stop();
  }
  if(DEBUG){
    Serial.print("수위센서 2 :");
    Serial.println(sensor2_val);
  }
}

void Watersensor3_ck(){
  sensor3_val = analogRead(watersensor3);   // watersansor3 의 변화값(전류값)을 읽음
  
  if (sensor1_val < WATERSENSOR3_LIMIT){                    // 찬물의 전류값이 WATERSENSOR3_LIMIT 이하일 경우 
    Serial.println("욕조물이 부족합니다.");
    Mortor3_stop();
  }
  if(DEBUG){
    Serial.print("수위센서 2 :");
    Serial.println(sensor2_val);
  }
}
void MortorAll_Stop(){
   Mortor1_stop();
   Mortor2_stop();
   Mortor3_stop();
}

void SetBath(int water, int temp_in){       // water 목표 수위 ,temp_in 목표 온도         //temp in = 40
  temp_in = temp_in - 1;
  sensor3_val = analogRead(watersensor3);   // watersansor3 의 변화값(전류값)을 읽음
  boolean mortorFlag = true;                // 어떤 모터 실행 구분 Flag 
  boolean coldFlag = false;                 // 차가운물이 있는지 없는지 구분 Flag
  boolean hotFlag = false;                  // 뜨거운물이 있는지 없는지 구분 Flag
  int temp = (int)(mlx.readObjectTempC()); // 현재 온도 읽음
  restart:
  while(sensor3_val < water){               // 목표 수위보다 현재 수위가 낮을경우 Loop
    //각 센서의 값을 받아옴
    sensor1_val = analogRead(watersensor1);
    sensor2_val = analogRead(watersensor2);
    sensor3_val = analogRead(watersensor3);
    temp = (int)(mlx.readObjectTempC()); // 현재 온도 읽음

    //온도 에따라 모터 전환
    if(temp < temp_in){  // 10 < 40     
      mortorFlag = false;
    }else if(temp > temp_in){   // 50 > 40 
      mortorFlag = true;
    }
    if(DEBUG){
      Serial.print("수위센서 1 :");
      Serial.println(sensor1_val);
      Serial.print("수위센서 2 :");
      Serial.println(sensor2_val);
      Serial.print("수위센서 3 :");
      Serial.println(sensor3_val);
      Serial.print("현재 온도 :");
      Serial.println(temp);
    }
    if(mortorFlag){
      if(sensor1_val < WATERSENSOR1_LIMIT){
        Serial.println("차가운물이 부족합니다.");
        coldFlag = true; 
      }else {
        coldFlag = false; 
        Mortor2_stop();
        Mortor1_run();
        delay(5000);
      } 
    }else {
      if(sensor2_val < WATERSENSOR2_LIMIT){
        Serial.println("뜨거운물이 부족합니다.");
        hotFlag = true;
      }else {
        hotFlag = false;
        Mortor1_stop();
        Mortor2_run();
        delay(5000);
      } 
    }
    if(hotFlag && coldFlag){                        // 차가운물과 뜨거운물 모두 없을 경우 
      Serial.println("양쪽 물이 모두 부족합니다.");
      Serial.println("물이 채워주세요."); 
      Mortor1_stop();
      Mortor2_stop();   
      delay(1000);
    }
  }
  while(temp < temp_in){
    sensor2_val = analogRead(watersensor2);
    temp = (int)(mlx.readObjectTempC());
    
    if(sensor2_val < WATERSENSOR2_LIMIT){
        Serial.println("뜨거운물이 부족합니다.");
        hotFlag = true;
    }else {
        Mortor2_run();
        Mortor3_run();
        Serial.print("현재온도 : ");
        Serial.print(temp); 
        Serial.print("|목표온도 : ");
        Serial.println(temp_in+1);
    } 
  }
  
  Mortor2_stop();
  Mortor3_stop();

  while(sensor3_val > WATERSENSOR3_LIMIT){
    sensor3_val = analogRead(watersensor3);
    Mortor3_run();
    if(DEBUG){
      Serial.print("수위센서 3 :");
      Serial.println(sensor3_val);
    }
    Mortor3_run();
  }
  Mortor3_stop();
}

void serialEvent(){
 while (Serial2.available()){
 Re_buf[counter]=(unsigned char)Serial2.read();
 if(counter == 0 && Re_buf[0] !=0x5A) return;
  counter++;
 if(counter++);
   if(counter == 9); {
     counter = 0;
     sign = 1;
   }
 }
}


void testBath(int water){
  sensor3_val = analogRead(watersensor3);   // watersansor3 의 변화값(전류값)을 읽음
  boolean mortorFlag = true;
  boolean coldFlag = false;
  boolean hotFlag = false;
  
  while(sensor3_val < water){
    sensor1_val = analogRead(watersensor1);
    sensor2_val = analogRead(watersensor2);
    sensor3_val = analogRead(watersensor3);
    
    if(DEBUG){
      Serial.print("수위센서 1 :");
      Serial.println(sensor1_val);
      Serial.print("수위센서 2 :");
      Serial.println(sensor2_val);
      Serial.print("수위센서 3 :");
      Serial.println(sensor3_val);
    }
    if(mortorFlag){
      mortorFlag = false;
      if(sensor1_val < WATERSENSOR1_LIMIT){
        Serial.println("차가운물이 부족합니다.");
        Serial3.print("%");
        Serial3.print("c");
        Serial3.print("?");
        coldFlag = true; 
      }else {
        coldFlag = false; 
        Mortor2_stop();
        Mortor1_run();
        delay(5000);
      } 
    }else {
      mortorFlag = true;
      if(sensor2_val < WATERSENSOR2_LIMIT){
        Serial.println("뜨거운물이 부족합니다.");
        Serial3.print("%");
        Serial3.print("h");
        Serial3.print("?");
        hotFlag = true;
      }else {
        hotFlag = false;
        Mortor1_stop();
        Mortor2_run();
        delay(5000);
      } 
    }
    if(hotFlag && coldFlag){
      Serial.println("양쪽 물이 모두 부족합니다.");
      Serial.println("물이 채워주세요."); 
      Serial3.print("%");
      Serial3.print("b");
      Serial3.print("?");
      Mortor1_stop();
      Mortor2_stop();   
      delay(1000);
    }
  }
  Mortor1_stop();
  Mortor2_stop();
  
  while(sensor3_val > WATERSENSOR3_LIMIT){
    sensor3_val = analogRead(watersensor3);
    Mortor3_run();
    if(DEBUG){
      Serial.print("수위센서 3 :");
      Serial.println(sensor3_val);
    }
    Mortor3_run();
  }
  Mortor3_stop();
}

void sendState(){
  sensor3_val = analogRead(watersensor3);
  Serial3.print("%");
  Serial3.print(sensor3_val);
  Serial3.print("?");
}



void setup ()
{
  Serial3.begin(9600);         // 블루투스 통신속도 설정
  Serial.begin (9600);           // 시리얼모니터 설정
  mlx.begin();                   // mlx 모듈을 읽기 시작.
  
  pinMode(RELAYPIN1,OUTPUT);
  pinMode(RELAYPIN2,OUTPUT);
  pinMode(RELAYPIN3,OUTPUT);
  
  digitalWrite(RELAYPIN1,HIGH);
  digitalWrite(RELAYPIN2,HIGH);
  digitalWrite(RELAYPIN3,HIGH);
}
 
void loop()
{
  sendState();
  if(Serial3.available()){
    char toSend = Serial3.read();
    Serial.println(toSend);
    if(toSend == '%'){
      bt_Response_Str = "";
    }else if(toSend == '?'){
      if(bt_Response_Str.charAt(0) == 'c'){
        bt_Response_Str = bt_Response_Str.substring(1,bt_Response_Str.length());
        int lex = bt_Response_Str.indexOf(",");
        String water = bt_Response_Str.substring(0,lex);
        String temp = bt_Response_Str.substring((lex+1),(bt_Response_Str.length()));
       
        Serial.print("water :");
        Serial.print(water);
        Serial.println("");
        Serial.print(water);
        Serial.print("만큼의 물받기 실행");
        testBath(water.toInt());
      }else if(bt_Response_Str.charAt(0) == 'm'){
        bt_Response_Str = bt_Response_Str.substring(1,bt_Response_Str.length());
        int lex = bt_Response_Str.indexOf(",");
        String water = bt_Response_Str.substring(0,lex);
        String temp = bt_Response_Str.substring((lex+1),(bt_Response_Str.length()));
        Serial.print("water :");
        Serial.print(water);
        Serial.print("\temp :");
        Serial.print(temp);
        Serial.println("");
        Serial.print(water);
        Serial.print("만큼의 물받기 실행");
        SetBath(water.toInt(),temp.toInt());
      }
      else if(bt_Response_Str.equals("1")){          // 찬물 On
        Mortor1_run();
        
      }else if(bt_Response_Str.equals("2")){    // 뜨거운물 On
        Mortor2_run();
        
      }else if(bt_Response_Str.equals("3")){    // 욕조 배수로 On
        Mortor3_run();
        
      }else if(bt_Response_Str.equals("4")){    // 찬물 Off
        Mortor1_stop();
        
      }else if(bt_Response_Str.equals("5")){    // 뜨거운물 Off
        Mortor2_stop();
        
      }else if(bt_Response_Str.equals("6")){    // 욕조 배수로 Off
        Mortor3_stop();
        
      }
    }else{
      bt_Response_Str += toSend;
    }
  }

   if(Serial.available()){
    int num = Serial.parseInt();
    if(num == 1){          // 찬물 On
      Mortor1_run();
    }else if(num == 2){    // 뜨거운물 On
      Mortor2_run();
    }else if(num == 3){    // 욕조 배수로 On
      Mortor3_run();
    }else if(num == 4){    // 찬물 Off
       Mortor1_stop();
    }else if(num == 5){    // 뜨거운물 Off
      Mortor2_stop();
    }else if(num == 6){    // 욕조 배수로 Off
      Mortor3_stop();
    }else if(num == 7){
      SetBath(500, 27);
    }
   }
  delay(100);
}
