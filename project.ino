#include <string.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>

bool check = false;
String receive_str; 
int stop_distance = 10;// Khoảng cách phát hiện vật cản
bool isInIntersection = false; //bien check co dang o cho giao nhau hay khong true: co, false: khong 

//L298 kết nối arduino
/**
 * Rules: 
 * A1: HIGH && A2: LOW -> đi lùi
 * A1: LOW && A2: HiGH -> đi tiến
 * B2: HIGH && B1: LOW -> đi lùi
 * B2: LOW && B1: HiGH -> đi tiến
 */
const int motorA1      = 3;  // kết nối chân IN1 với chân 3 arduino
const int motorA2      = 4;  // kết nối chân IN2 với chân 4 arduino
const int motorAspeed  = 5;  // kết nối chân ENA với chân 5 arduino
const int motorB1      = 7; // kết nối chân IN3 với chân 7 arduino
const int motorB2      = 8; // kết nối chân IN4 với chân 8 arduino
const int motorBspeed  = 6;  // kết nối chân ENB với chân 6 arduino

//kết nối của 3 cảm biến hồng ngoại (dò line )
const int L_S = 9; // cb dò line phải
const int S_S = 2; // cb dò line giữa
const int R_S = 10; //cb dò line trái

int left_sensor_state;// biến lưu cảm biến hồng ngoại line trái
int s_sensor_state;   // biến lưu cảm biến hồng ngoại line giữa
int right_sensor_state;// biến lưu cảm biến hồng ngoại line phải

long duration; //
int distance;  // biến khoảng cách
const int time_delay_to_turn = 30; // Thoi gian de quay xe, note la can thuc nghiem de biet thoi gian de quay dc gan 90 do nhat
int carSpeed = 100;

TaskHandle_t go_handler;
TaskHandle_t data_handler;

void setup() {
  pinMode(L_S, INPUT); // chân cảm biến khai báo là đầu vào
  pinMode(R_S, INPUT);
  pinMode(S_S, INPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorAspeed, OUTPUT);
  pinMode(motorBspeed, OUTPUT);
  
  Serial.begin(9600);
  analogWrite(motorAspeed, carSpeed); // tốc độ động cơ a ban đầu 120 ( 0 - 255)
  analogWrite(motorBspeed, carSpeed);// tốc độ động cơ b ban đầu 120 ( 0 - 255)
  xTaskCreate(carGo, "carGO", 128,NULL ,1, &go_handler);
  xTaskCreate(nhanBienBao, "nhanBienBao", 128, NULL,1, &data_handler);
}

void loop() {
  // Khong code trong nay khi dung freeRTOS
}

/**
 * Cho xe quay vong tai cho theo chieu kim dong ho
 */
void turnAroundClockwise(int delayTime) {
    digitalWrite (motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    digitalWrite (motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    delay(delayTime);
    stopCar();
}

/**
 * quay xe tai cho ngc chieu kim dong ho
 */
void turnAroundCounterClockwise(int delayTime) {
    digitalWrite (motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    digitalWrite (motorB1, HIGH);
    digitalWrite(motorB2, LOW);
    delay(delayTime);
    stopCar();
}

void backward(int runTime) {
    digitalWrite (motorA1, HIGH); // cho xe robot chạy lùi 1 đoạn
    digitalWrite(motorA2, LOW);
    digitalWrite (motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    delay(runTime);
}

void forward(int runTime) { // chương trình con xe robot đi tiến
//  Serial.println("forward:");
  digitalWrite (motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite (motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  delay(runTime);
}

void turnRight() {
//  Serial.println("re phai:");
  digitalWrite (motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite (motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void turnLeft() {
  //Serial.println("re trai:");
  digitalWrite (motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite (motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void stopCar() {
//  Serial.println("stop: ");
  digitalWrite (motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite (motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void carGo() {
  Serial.println("carGo: ");
  while(true) {
    left_sensor_state = digitalRead(L_S);
    s_sensor_state = digitalRead(S_S);
    right_sensor_state = digitalRead(R_S);
  
    if ((digitalRead(L_S) == 0) && (digitalRead(S_S) == 1) && (digitalRead(R_S) == 0)) {
      forward(0); // đi tiến
    }
    if ((digitalRead(L_S) == 1) && (digitalRead(S_S) == 1) && (digitalRead(R_S) == 0)) {
      turnLeft(); // rẻ trái
    }
    if ((digitalRead(L_S) == 1) && (digitalRead(S_S) == 0) && (digitalRead(R_S) == 0)) {
      turnLeft(); // rẻ trái
    }
    if ((digitalRead(L_S) == 0) && (digitalRead(S_S) == 1) && (digitalRead(R_S) == 1)) {
      turnRight(); // rẻ phải
    }
    if ((digitalRead(L_S) == 0) && (digitalRead(S_S) == 0) && (digitalRead(R_S) == 1)) {
      turnRight(); // rẻ phải
    }
    if ((digitalRead(L_S) == 1) && (digitalRead(S_S) == 1) && (digitalRead(R_S) == 1)) {
      isInIntersection = true;
      if ( receive_str == "0") {
        stopCar();
      } else if(receive_str == "1") {
        stopCar();
        delay(100);
        forward(500);
      } else if(receive_str == "2") {
        stopCar();
        delay(1000);
        forward(1000);
        turnLeft();
        delay(200);
      } else if(receive_str == "3") {
        stopCar();
        delay(1000);
        forward(1000);
        turnRight();
        delay(200);
      } else if (receive_str == "5") {
        stopCar();
      }
      receive_str = "4";
      isInIntersection = false;
    }
    
    
  }

  vTaskDelete(NULL);
}

void nhanBienBao() {
  Serial.println("nhanBienBao: ");
  
  while(true) {
    delay(1000);
    if (Serial.available() && !isInIntersection) {
      receive_str = Serial.readStringUntil('\r');
      Serial.print("nhan data: ");
      Serial.println(receive_str);
     
    }
    if ( receive_str == "0") {
        stopCar();
        vTaskSuspend(go_handler);
      } else if (receive_str == "5") {

          vTaskResume(go_handler);
        
      }
      
    
  }

  vTaskDelete(NULL);
}
