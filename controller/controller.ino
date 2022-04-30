//コントローラ側の制御/通信用ソースコード
//ラズパイと機体の単なる中継

//ラズパイから制御コードをUARTで受信
//機体側へ各アウトプットの制御コードをudpで送信
//機体からはインプットのデータをudpで受信
//インプットデータをラズパイにUARTで送信

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wpa2.h"

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

uint8_t macBT[6];

String btName="ESP32test";

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

String ssid;
String password;
bool apFind=false;

String sendTxt="";
String receiveTxt="";
char receiveBuf[255];
char bufOld;

bool isMaster;
bool btConnected=false;

esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT();

//********* core 1 task ************
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    //自身のMACを取得
    esp_read_mac(macBT, ESP_MAC_BT);

    //相手のMACを(関数から)取得
    uint8_t address[6] = { 
        macBT[0],
        macBT[1], 
        macBT[2], 
        macBT[3], 
        macBT[4], 
        returnSlaveBTAddress(macBT[5])
    };

    //周辺APのスキャンを開始．既知のものを探す
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
    
    //pwとudpに使うポートを関数から取得
    password=returnPassword(ssid);
    Serial.println(password);
    udp_port=portNum();
    Serial.println(ssid);

    //wpa2_entの場合のみ処理，アカウントのデータを読み出す
    if(isWpa2_ent(ssid)){
        Serial.print(ssid);
        Serial.println(" is wpa2 ent");
        esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)returnAnonymousIdentity(ssid), strlen(returnAnonymousIdentity(ssid)));
        esp_wifi_sta_wpa2_ent_set_username((uint8_t *)returnIdentity(ssid),strlen(returnIdentity(ssid)));
        esp_wifi_sta_wpa2_ent_set_password((uint8_t *)password.c_str(), strlen(password.c_str()));
        esp_wifi_sta_wpa2_ent_enable(&config);
    }

    //wifiにつながるまで待つ
    connectToWiFi();
    while(!connected){
        delay(1);
    }
    Serial.println("connected!");

    //wifiにつながったら，双方のipアドレスをBTSerialで交換するため接続を試みる
    //Master/Slaveを適当な時間間隔で切り換え，つながるまで待つ
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

    //BTで繋がったら，Masterから自身のIPを送る
    const int btDelay=1000;
    if(isMaster){
        Serial.println("send IP");
        while(1){
            //送信
            SerialBT.println(WiFi.localIP().toString()+";");
            Serial.println("send:"+WiFi.localIP().toString());
            // SerialBT.println(sendIP+';');
            // SerialBT.write(';');

            //折り返し受信
            if (SerialBT.available()){
                // String tmpStr=SerialBT.readString();
                String tmpStr=SerialBT.readStringUntil(';');
                tmpStr.trim();
                Serial.println("response:"+tmpStr);

                //送ったIPと折り返し受信した内容が同じなら送信終了を知らせて次のフェーズへ
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
            //受信
            String tmpStr=SerialBT.readStringUntil(';');
            tmpStr.trim();
            Serial.println("received:"+tmpStr);
            // String tmpStr=SerialBT.readStringUntil(';');

            //受信内容が通信終了通知なら次へ
            if(tmpStr=="ok"){
                Serial.println("receive chk!");
                break;
            }

            //内容チェックのために，折り返して送信しておく
            udp_address=tmpStr;
            SerialBT.println(tmpStr+";");
            // SerialBT.write(udp_address+';');
        }
        delay(btDelay);
        }
    }

    //送受信を交代してもう一回
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
}
 
void loop() {  
    receiveUDP();
    sendUDP();
    delay(2);
}

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

