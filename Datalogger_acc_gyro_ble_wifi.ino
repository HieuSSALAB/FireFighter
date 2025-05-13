#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <WiFiClient.h>
  #include <ESP8266WebServer.h>
  #include <ESP8266mDNS.h>
  #include <ESP8266HTTPUpdateServer.h>
    ESP8266WebServer server(80);
    ESP8266HTTPUpdateServer httpUpdater;
#elif defined(ESP32)
  #include <WiFiClient.h>
  #include <WebServer.h>
  #include <ESPmDNS.h>
  #include <HTTPUpdateServer.h>
  #include <WiFi.h>
    WebServer server(80);
    HTTPUpdateServer httpUpdater;
#endif
#include <WiFiUdp.h>
WiFiUDP UDP;
char packet[1024];
#define UDP_PORT 2205
IPAddress myIP;
const char* ssid     = "SSA_LAB";
const char* password = "12345687990";
const char* ssid1     = "SSA_step3";
const char* password1 = "999999999";
const char* host = "SSALAB-datalogger";
MDNSResponder mdns;


bool client_connect = false;

#include "BluetoothSerial.h"
#include <HardwareSerial.h>
HardwareSerial SerialPort(1); // use UART1
#include <TinyGPS++.h>
BluetoothSerial Bluetooth;
// The TinyGPS++ object
TinyGPSPlus gps;
#include <TaskScheduler.h>
Scheduler runner;
void task_loger();
Task TareaLED(1, TASK_FOREVER, &task_loger);
#include <Wire.h>
#include <ADXL345.h>
ADXL345 adxl;
#include "ITG3200.h"
ITG3200 gyro;
#include <EEPROM.h>
#define EEPROM_SIZE 512
#include "Arduino.h"
#include <SPI.h>
#include <SD.h>
const int chipSelect = 5;  // used for ESP32
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

bool logger = false;
int Wait_reset = 0;
int frequency = 100;
int var_time = 3;//giây=> biến quyết định thời gian giới hạn khi lấy mẫu cho 1 số hành động
long limit_time=var_time*frequency;
long int countTime = 0;
String S1,S2;
boolean delText = false;
unsigned long int time_crr;
float accX = 0,accY=0,accZ= 0,gyrX=0, gyrY=0, gyrZ = 0,magX=0, magY=0, magZ = 0;
String dataGPS_ ;
unsigned long int time1=0, time2=0;
String data;
int indexText = 0;
//String activity[]={"Walking-fw","Walking-bw","Jogging","Squatting-down","Bending","Bending-pick-up","Limp","TripOver","Sit-chair","Sit-sofa","Sit-air","Sit-bed","Lying-bed","Rising-bed","Standing","Sitting","Lying","Upstairs","Downstairs", "Stand-up","dikhomlung","roads"};
String activity[]={"Jogging"};
int index_ac = 0;
String name_text = "";
int cycle =1000/frequency;
File dataFile;
long int starup = 0;
QueueHandle_t adcQueue;
QueueHandle_t sensorQueue;
boolean view_ = false;
String dataGPS = "0,0,0,0.0,0.0.0,0";
struct ADCData {
  float ax_,ay_,az_,gx_, gy_, gz_, mx_, my_,mz_;
};
const long bufferSize = 6000;
long bufferIndex = 0; 
void setup() {  
  
  Serial.begin(115200);

    Serial.println("Free Heap before SD: " + String(ESP.getFreeHeap()));
 int i = 20;
  while (!SD.begin(chipSelect)) {
    i--;
    if (i <= 0) {
      Serial.println("Failed to initialize SD card after multiple attempts.");
      break;
    }
    delay(10);
    
  }
  if (i > 0) {
    Serial.println("SD card initialized successfully.");
  } 



      Serial.println("Free Heap before WiFi: " + String(ESP.getFreeHeap()));
  AccessPoint();
  UDP.begin(UDP_PORT);
  Serial.println("Free Heap after WiFi: " + String(ESP.getFreeHeap()));
    delay(2000);
WiFi.setSleep(true);  // Bật chế độ tiết kiệm năng lượng

   Serial.println("Free Heap before Bluetooth: " + String(ESP.getFreeHeap()));
  Bluetooth.begin("ESP32NCKH"); 
  
  SerialPort.begin(9600, SERIAL_8N1, 14, 12); 
   adxl.powerOn();
  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg mỗi lần tăng
  adxl.setInactivityThreshold(75); //62.5mg mỗi lần tăng
  adxl.setTimeInactivity(10); // có bao nhiêu giây không hoạt động?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625us per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interrupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);

  adxl.setRangeSetting(2);           // Give the range settings
                                    // Accepted values are 2g, 4g, 8g or 16g
                                    // Higher Values = Wider Measurement Range
                                    // Lower Values = Greater Sensitivity
  //===============
  gyro.init();
  gyro.zeroCalibrate(200,10);//sample 200 times to calibrate and it will take 200*10ms
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  }
  sensor_t sensor;
  mag.getSensor(&sensor);
  EEPROM.begin(EEPROM_SIZE);
  pinMode(13, OUTPUT);
  digitalWrite(13,1);
  delay(100);
  digitalWrite(13,0);
  delay(100);
 
  
//  index_ac = EEPROM.read(0);
//  printDirectory(30); //load 20 file moi nhat
  double xyz[3];
  adxl.getAcceleration(xyz); //read the accelerometer values and store them in variables  x,y,z
  accX = xyz[0];
  accY = xyz[1];
  accZ = xyz[2];
  float gx = 0,gy = 0,gz=0;
  gyro.getAngularVelocity(&gx,&gy,&gz);
  gyrX = gx*PI/180; gyrY=gy*PI/180; gyrZ=gz*PI/180;
  sensors_event_t event; 
  mag.getEvent(&event);
  magX = event.magnetic.x;
  magY = event.magnetic.y;
  magZ = event.magnetic.z;
  // Tạo hàng đợi ADC
  adcQueue = xQueueCreate(10, sizeof(ADCData));
  xTaskCreatePinnedToCore(readAdcTask, "ADC Task", 2048, NULL, 1, NULL, 0);
  runner.addTask(TareaLED);
  // We activate the task
  TareaLED.enable();

  
}
int flas = 0;
int buttonState;          // Trạng thái hiện tại của nút nhấn
int lastButtonState = 1; // Trạng thái trước đó của nút nhấn
int count = 0;            // Biến đếm số lần nhả nút
unsigned long lastDebounceTime = 0;  // Thời điểm debounce trước đó
unsigned long debounceDelay = 50;    // Thời gian debounce (ms)
int changeACc= 0;
String trangthai = "";
String trangthaiOptions[] = {"Walking","Standing", "Lying", "Sitting","Jogging","WalkStoop","Crawling","Slithering","Falling", "DownStair", "UpStair"};
void loop() {

  runner.execute();
   int reading = digitalRead(flas);
//   if(Serial){Serial.print("flas: ");Serial.println(reading);}
  // Kiểm tra nếu trạng thái nút thay đổi (do nhấn hoặc nhả)
  if (reading != lastButtonState) {
    // Ghi nhận thời điểm thay đổi
    lastDebounceTime = millis();
  }

  // Kiểm tra nếu đã đủ thời gian debounce
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Nếu trạng thái thay đổi
    if (reading != buttonState) {
      buttonState = reading;

      // Nếu nút nhả (trạng thái HIGH)
      if (buttonState == 1) {
         changeACc++;
         if(changeACc>10)changeACc = 0;     
         if(Serial){Serial.print("activity: ");Serial.println(trangthaiOptions[changeACc]);}
      }
    }

  }
    // Lưu trạng thái hiện tại để sử dụng cho lần kiểm tra tiếp theo
  lastButtonState = reading;
}
int countTimeSend = 0;
void readAdcTask(void *parameter) {
  ADCData dataADC;
  while (1) {
      double xyz[3];
      adxl.getAcceleration(xyz); //read the accelerometer values and store them in variables  x,y,z
      dataADC.ax_ = xyz[0];    dataADC.ay_ = xyz[1];    dataADC.az_ = xyz[2];
      float gx = 0,gy = 0,gz=0;
      gyro.getAngularVelocity(&gx,&gy,&gz);
      dataADC.gx_ = gx*PI/180.0;   dataADC.gy_=gy*PI/180.0;   dataADC.gz_=gz*PI/180.0;
      sensors_event_t event; 
      mag.getEvent(&event);
      dataADC.mx_ = event.magnetic.x;    dataADC.my_ = event.magnetic.y;    dataADC.mz_ = event.magnetic.z;
//      String var = getGPS();
      // Gửi cấu trúc dataADC vào hàng đợi
      xQueueSend(adcQueue, &dataADC, portMAX_DELAY);
        vTaskDelay(1);
  }
}


void AccessPoint()
{
  WiFi.mode(WIFI_AP);

    while (WiFi.softAP(ssid1, password1) == false) 
  {
    Serial.print(".");
    delay(10);
  }
//  myIP = WiFi.softAPIP();  
//  Serial.println("WiFi connected");
//  Serial.print("IP address: ");
//  Serial.println(myIP);
//  if (mdns.begin(host)) Serial.println("MDNS ok"); //cai này dùng cho chế độ AP
//  nếu.setup(&server);
//  server.begin();
}

int countSave = 0;
void task_loger()
{ 

     
  countTimeSend++;
  countSave++;
    if (Serial.available() > 0) { // Kiểm tra xem có dữ liệu nhận được không
    String data = Serial.readStringUntil('\n'); // Đọc dữ liệu đến khi gặp ký tự xuống dòng
    data.trim(); // Loại bỏ khoảng trắng và ký tự thừa

    int number = 0; // Giá trị mặc định nếu dữ liệu không hợp lệ

    if (isNumber(data)) { // Kiểm tra xem chuỗi có phải số hợp lệ không
      number = data.toInt(); // Chuyển đổi sang số nguyên
      if (number < 0) { // Nếu số âm, gán thành 0
        number = 0;
      }
      changeACc = number;
      if(changeACc>10)changeACc = 0;
      
      Serial.print("activity: ");Serial.println(trangthaiOptions[changeACc]);
    }
    }
    if (Bluetooth.available() > 0) { // Kiểm tra xem có dữ liệu nhận được không
    String data = Bluetooth.readStringUntil('\n'); // Đọc dữ liệu đến khi gặp ký tự xuống dòng
    data.trim(); // Loại bỏ khoảng trắng và ký tự thừa

    int number = 0; // Giá trị mặc định nếu dữ liệu không hợp lệ

    if (isNumber(data)) { // Kiểm tra xem chuỗi có phải số hợp lệ không
      number = data.toInt(); // Chuyển đổi sang số nguyên
      if (number < 0) { // Nếu số âm, gán thành 0
        number = 0;
      }
      changeACc = number;
      if(changeACc>10)changeACc = 0;
      
      if(Serial){Serial.print("activity: ");Serial.println(trangthaiOptions[changeACc]);}
      Bluetooth.print("activity: ");Bluetooth.println(trangthaiOptions[changeACc]);
    }
    }
  ADCData receivedDataADC;
  if (xQueueReceive(adcQueue, &receivedDataADC, 0) == pdTRUE) {
        // Xử lý dữ liệu từ cấu trúc ADCData ở đây
        accX =  receivedDataADC.ax_;
        accY =  receivedDataADC.ay_;
        accZ =  receivedDataADC.az_;
        gyrX =  receivedDataADC.gx_;
        gyrY =  receivedDataADC.gy_;
        gyrZ =  receivedDataADC.gz_;
        magX =  receivedDataADC.mx_;
        magY =  receivedDataADC.my_;
        magZ =  receivedDataADC.mz_;
    }
//   String var = getGPS();
//   if(var != "") 
//          {
////             if(Serial && !logger)Serial.println(var);        //DÒNG NÀY ĐỂ ĐỌC GPS+ TỐC ĐỘ TRÊN SERIAL
//             dataGPS = var;  
//           }
  String datalog = activity[index_ac]+": " +String (accX,3)+","+String (accY,3)+","+String (accZ,3)+ ","+String (gyrX,3)+","+String (gyrY,3)+","+String (gyrZ,3);
  if(countTimeSend > 20)
  {
    // Randomly select Tinhtrang, Trangthai, and huongDiChuyen values
    String tinhtrang = (random(0, 2) == 0) ? "Tốt" : "Xấu";    
    trangthai = trangthaiOptions[changeACc];
    String huongDiChuyenOptions[] = {"Trái", "Phải", "Lên", "Xuống"};
    String huongDiChuyen = huongDiChuyenOptions[random(0, 4)];
    // Construct the JSON data string with the random values
    String dataToSend = "[{\"toadoax\":\"" + String(accX, 3) + "\",\"toadoay\":\"" + String(accY, 3) + "\",\"toadoaz\":\"" + String(accZ, 3) + 
                        "\",\"toadogx\":\"" + String(gyrX, 3) + "\",\"toadogy\":\"" + String(gyrY, 3) + "\",\"toadogz\":\"" + String(gyrZ, 3) + 
                        "\",\"toadomx\":\"" + String(magX, 3) + "\",\"toadomy\":\"" + String(magY, 3) + 
                        "\",\"toadomz\":\"" + String(magZ, 3) + "\",\"Tinhtrang\":\"" + tinhtrang + 
                        "\",\"Trangthai\":\"" + trangthai + "\",\"Step\":\"" + "0" + 
                        "\",\"nhiptho\":\"" + "0" + "\",\"nhiptim\":\"" + "0" + 
                        "\",\"Nhietdo\":\"" + "0" + "\",\"spo2\":\"" + "0" + 
                        "\",\"huongDiChuyen\":\"" + "Trái" + "\"}]";
                      
 
    Bluetooth.println(dataToSend); 
    countTimeSend=0;
  }
   
}
// Hàm kiểm tra xem chuỗi có phải là số không
bool isNumber(String str) {
  for (unsigned int i = 0; i < str.length(); i++) {
    if (!isDigit(str[i])) { // Nếu ký tự không phải là số, trả về false
      return false;
    }
  }
  return true;
}
