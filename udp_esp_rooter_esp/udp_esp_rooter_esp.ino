//起動してWifi接続してIPを確保
//同時にもう一台で同じコードを実行
//BTSerialで一時的に接続し，IPを交換
//BTSerialを終了
//UDPで双方向テキスト通信し，結果を画面出力
//表示部分については無駄があるが試作品なのでここまでとしておく

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include "esp_wpa2.h"

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

uint8_t macBT[6];

String btName="ESP32test";

// OLED SSD1331用ピン
#define sclk 18
#define mosi 23
#define cs   17
#define rst  4
#define dc   16

//9軸フュージョンモジュール BNO055用ピン


Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);
// Adafruit_SSD1331 display = Adafruit_SSD1331(&SPI, cs, dc, rst);

// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

String udp_address; //送信先のアドレス
int udp_port;//ポート

WiFiUDP udp;
TaskHandle_t task_handl; //マルチタスクハンドル定義

int counter=0;
int otherCounter=0;
boolean connected = false;
boolean isSet_send_data = false;
uint32_t now_time = 0;
int16_t interval = 100; //UDPデータ送信間隔

#define INPUT_PIN 14

// char* ssid;
// char* password;
String ssid;
String password;
bool apFind=false;

String sendTxt="";
String receiveTxt="";
char receiveBuf[255];
char textBuf[255];
char bufOld;

bool isMaster;
bool btConnected=false;

esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT();

//********* core 1 task ************
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  esp_read_mac(macBT, ESP_MAC_BT);
  uint8_t address[6] = { 
    macBT[0],
    macBT[1], 
    macBT[2], 
    macBT[3], 
    macBT[4], 
    returnSlaveBTAddress(macBT[5])
  };

  Serial.println("scan start");

  while(!apFind){
    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n == 0) {
      Serial.println("no networks found");
    } else {
      Serial.print(n);
      Serial.println(" networks found");
      for (int i = 0; i < n; ++i) {
        Serial.println(WiFi.SSID(i));
        if(ssidSearch(WiFi.SSID(i))){
          Serial.println("find!");
          ssid=WiFi.SSID(i);
          // Serial.println(const_cast<char*>(WiFi.SSID(i).c_str()));
          apFind=true;
          break;
        }
      }
    }
  }
  
  // ssid = returnSSID();
  // password = returnPass();
  password=returnPassword(ssid);
  Serial.println(password);
  udp_port=portNum();
  Serial.println(ssid);

  if(isWpa2_ent(ssid)){
    Serial.print(ssid);
    Serial.println(" is wpa2 ent");
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)returnAnonymousIdentity(ssid), strlen(returnAnonymousIdentity(ssid)));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)returnIdentity(ssid),strlen(returnIdentity(ssid)));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)password.c_str(), strlen(password.c_str()));
    esp_wifi_sta_wpa2_ent_enable(&config);
  }

  //入力ピン設定
  pinMode(INPUT_PIN, INPUT_PULLUP);

  //wifiにつながるまで待つ
  connectToWiFi();
  while(!connected){
    delay(1);
  }

  Serial.println("connected!");

  // IPAddress sendIP = WiFi.localIP();


  // WiFi.disconnect();
  // delay(3000);

  //つながったらipアドレスをBTSerialで交換
  while(1){
    if(random(0,2)==0){
      SerialBT.begin("ESP32test");
      Serial.println("The device started, now you can pair it with bluetooth!");
      isMaster=false;
    }else{
      SerialBT.begin("ESP32test", true); 
      Serial.println("The device started in master mode, make sure remote BT device is on!");
      btConnected = SerialBT.connect(address);
      // if(connected) {
      //   Serial.println("Connected Succesfully!");
      // }
      isMaster=true;
    }

    if(SerialBT.connected(1000*random(1,11))){
      Serial.println("connected!");
      Serial.println(isMaster);
      break;
    }else{
      Serial.println("retry");
      SerialBT.end();
    }
  }

  const int btDelay=1000;
  if(isMaster){
    Serial.println("send IP");
    while(1){
      SerialBT.println(WiFi.localIP().toString()+";");
      Serial.println("send:"+WiFi.localIP().toString());
      // SerialBT.println(sendIP+';');
      // SerialBT.write(';');
      if (SerialBT.available()){
        // String tmpStr=SerialBT.readString();
        String tmpStr=SerialBT.readStringUntil(';');
        tmpStr.trim();
        Serial.println("response:"+tmpStr);
        if(tmpStr==WiFi.localIP().toString()){
          Serial.println("ok;");
          SerialBT.println("ok;");
          break;
        }
      }
      delay(btDelay);
    }
  }else{
    Serial.println("receive IP");
    while(1){
      if (SerialBT.available()){
        String tmpStr=SerialBT.readStringUntil(';');
        tmpStr.trim();
        Serial.println("received:"+tmpStr);
        // String tmpStr=SerialBT.readStringUntil(';');
        if(tmpStr=="ok"){
          Serial.println("receive chk!");
          break;
        }
        udp_address=tmpStr;
        SerialBT.println(tmpStr+";");
        // SerialBT.write(udp_address+';');
      }
      delay(btDelay);
    }
  }
  if(!isMaster){
    Serial.println("send IP");
    while(1){
      SerialBT.println(WiFi.localIP().toString()+";");
      Serial.println("send:"+WiFi.localIP().toString());
      // SerialBT.println(sendIP+';');
      // SerialBT.write(';');
      if (SerialBT.available()){
        // String tmpStr=SerialBT.readString();
        String tmpStr=SerialBT.readStringUntil(';');
        tmpStr.trim();
        Serial.println("response:"+tmpStr);
        if(tmpStr==WiFi.localIP().toString()){
          Serial.println("ok;");
          SerialBT.println("ok;");
          break;
        }
      }
      delay(btDelay);
    }
  }else{
    Serial.println("receive IP");
    while(1){
      if (SerialBT.available()){
        String tmpStr=SerialBT.readStringUntil(';');
        tmpStr.trim();
        Serial.println("received:"+tmpStr);
        // String tmpStr=SerialBT.readStringUntil(';');
        if(tmpStr=="ok"){
          Serial.println("receive chk!");
          break;
        }
        udp_address=tmpStr;
        SerialBT.println(tmpStr+";");
        // SerialBT.write(udp_address+';');
      }
      delay(btDelay);
    }
  }

  Serial.print("myIP:");
  Serial.println(WiFi.localIP());
  
  Serial.print("otherIP:");
  Serial.println(udp_address);

  //BTを切断
  SerialBT.disconnect();
  delay(1000);
  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Succesfully!");
  }
  SerialBT.end();

  delay(3000);

  //wifiにつながるまで待つ
  connectToWiFi();
  while(!connected){
    delay(1);
  }

  xTaskCreatePinnedToCore(&taskDisplay, "taskDisplay", 8192, NULL, 10, &task_handl, 0);
  delay(500); //これ重要。別タスクでM5.begin関数が起動するまで待つ。
}
 
void loop() {  
  // Serial.print("otherIP:");
  // Serial.println(udp_address);

  // delay(100);
  receiveUDP();
  // autoIncDec();
  sendUDP();
  delay(2);
}
//******** core 0 task *************
void taskDisplay(void *pvParameters){
  display.begin();
  while(true){
    display.fillScreen(BLACK);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.print("mine: "+sendTxt);
    // display.print("mine: "+String(counter));
    display.setCursor(0,16);
    display.print("yours: "+receiveTxt);
    // display.print("yours: "+String(otherCounter));
    delay(interval/2);
  }
}
//************************************
void receiveUDP(){
  int packetSize = udp.parsePacket();
  if(packetSize > 0){
    // otherCounter = udp.read();
    int len=udp.read(receiveBuf,255);
    receiveTxt=String(receiveBuf);
    Serial.print("receive: ");
    Serial.println(receiveTxt);
  }
}

void sendUDP(){
  if(Serial.available()){
    udp.beginPacket(udp_address.c_str(), udp_port);
    sendTxt=Serial.readString();
    Serial.print("send: ");
    Serial.println(sendTxt);
    udp.print(sendTxt);
    udp.endPacket();
  }
}

void autoIncDec() {
  if(LOW==digitalRead(INPUT_PIN)){
    counter=counter+1;
  }
  delay(interval);
}

void connectToWiFi(){
  Serial.println("Connecting to WiFi network: " + ssid);
  WiFi.disconnect(true, true);
  delay(1000);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid.c_str(), password.c_str());
  Serial.println("Waiting for WIFI connection...");
}

void WiFiEvent(WiFiEvent_t event){
  IPAddress myIP = WiFi.localIP();
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected!");
      Serial.print("My IP address: ");
      Serial.println(myIP);
      //udp.begin関数は自サーバーの待ち受けポート開放する関数である
      udp.begin(myIP, udp_port);
      delay(1000);
      connected = true;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
    default:
      break;
  }
}
