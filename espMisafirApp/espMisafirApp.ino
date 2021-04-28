


#include <Servo.h>               //Servo kütüphanemizi ekliyoruz.

int pirPin = 35;                   //Sensörü takacağımız pin
int servoPin = 19;                 //Servoyu takacağımız pin
int hareket;                      //Sensörden aldığımız veri
Servo motor;                      //Servo motor değişkeni




#define kirmizi_led 25
#define yesil_led 26
#define buzzer_Pin 27


bool islem = false;
unsigned long time_now =0;
int period = 10000;


#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID "Yükleniyor..._plus"
#define WIFI_PASSWORD "q1w2e3r4t5"

#define MQTT_HOST IPAddress(143, 198, 71, 210)       // "mqtt.smartcozum.com"     143.198.71.210
#define MQTT_PORT 1883

const char *mqtt_client_username = "kekstra"; 
const char *mqtt_client_password = "Resul81Yag";

char *pub_topic = "esp-s";
char *sub_topic = "esp-r";

char *biri_geldi = "101";    // esp32 pir sensörü hareket algıladığında esp_s topiğine 101 gönderecek
char *kapiyi_ac = "201";     // mobil app, esp_s den 101 geldiğinde eğer isterse kapıyı aç demek için esp_r ye 201 gönderecek
char *kapi_acildi = "202";   // mobil app den 201 gelip kapıyı açtığında esp_s ye kapı açıldı demek için 202 gönderecek
char *alarm_cal = "301";     // mobil app, esp_s den 101 geldiğinde eğer isterse alarm çaldırmak için esp_r ye 301 gönderecek
char *alarm_calindi = "302"; // mobil app den 301 gelip alarm çaldığında esp_s ye alarm çalındı demek için 302 gönderecek

char *durumlari_sonlandir = "102"; // bunu bütün durumlar için her şeyi en baştaki stabil duruma geri döndermek için yani sıfırlamak için kullanacağız daha kodlanmadı
                                  // her başlayan programdan 10 saniye sonra program sonlanması için bu mesaj gönderilecek
                                  //ayrıca arduino tarafında da kapıyı aç dediğinde yanan yeşil led açık kalıyor ve alarm çal dediğinde yanan led ve öten buzzer açık kalıyor
char *durumlar_sonlandirildi = "103";  //arduino tarafında da ledlerin ve buzzerın durdurulması için her şeyin başa dönmesi için <ndroid tarafından 102ye karşılık gelen mesaj 

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient._username = mqtt_client_username;
  mqttClient._password = mqtt_client_password;

  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe(sub_topic, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("->onMqttMessage() topic: ");
  Serial.println(topic);
  Serial.println("msg: ");
  Serial.println(payload);

 
  if (isEquals(topic,sub_topic,5)){
    Serial.println("topic doğru");

      if(isEquals(payload, kapiyi_ac,3) ){
       
        time_now = millis();
        islem = true;



        
        Serial.println("msg: kapiyi_aç");
        motor.write(180);
        digitalWrite(yesil_led, HIGH);  
        digitalWrite(kirmizi_led, LOW);
        digitalWrite (buzzer_Pin, LOW);
        publishMessage(kapi_acildi);  // burada bu mesaj gittiğinde bildirimde ve veya normal gittiğinde text özelliği veya ayrıca bir ekrana mesaj olarak da olabilir '10 saniye sonra kapı otomatik kapanacaktır' mesajı da gitmeli
        Serial.println("msg: kapi açıldı");

        //delay(10000);
        //publishMessage(durumlari_sonlandir);
        //delay(20000);
        //motor.write(92);       // burda hata veriyor     kendine reset atıyor
      } else if(isEquals(payload, alarm_cal,3)){


        time_now = millis();
        islem = true;




        
        Serial.println("msg: alarm_cal");
        digitalWrite(yesil_led, LOW);
        digitalWrite(kirmizi_led, HIGH);
        digitalWrite (buzzer_Pin, HIGH);
        // burada kırmızı led yanacak ve buzzer ötecek
        publishMessage(alarm_calindi);
        //delay(10000);       buraya delay komutu olmuyor çünkü delay bekliyor ve program hata alıyor, buraya millis() fonksiyonu kullannılacak ve şimdiki zamandan öncek zaman çıkartılacak
        //publishMessage(durumlari_sonlandir);
      } else if(isEquals(payload, durumlar_sonlandirildi,3)){
        Serial.println("msg: durumlar sonlandırıldı");
        digitalWrite(yesil_led, LOW);
        digitalWrite(kirmizi_led, LOW);
        digitalWrite (buzzer_Pin, LOW);
        motor.write(90);
      }else {
        Serial.println("başka mesaj");
      }
    
    } else {
      Serial.println("topic yanlış");
    }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();


  motor.attach(servoPin);         //Servomuzu 19. pin ile ilişkilendiriyoruz.
  motor.write(90);
  pinMode(pirPin, INPUT);         //Sensör pinimizi giriş olarak ayarlyoruz.

  pinMode(kirmizi_led,OUTPUT);
  pinMode(yesil_led,OUTPUT);  
  pinMode(buzzer_Pin,OUTPUT);


  
  

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

}

void publishMessage(char* mesaj){
  uint16_t packetIdPub2 = mqttClient.publish(pub_topic, 0, true, mesaj);
  Serial.print("Paket Id:");
  Serial.println(packetIdPub2);
  Serial.print("Gönderilen mesaj sonucu:");
  Serial.println(mesaj);
  }


void subscribeTopic(char* mesaj){
  uint16_t packetIdSub = mqttClient.subscribe(sub_topic, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  Serial.print("Subscribe olunan topic : ");
  Serial.println(sub_topic);
}




bool isEquals(char* first, char* sec, int t) { //eşit mi
      for (int i = 0; i<t;i++){
        if (first[i] != sec[i]){
          return false;
        }
      }
      return true;
  }



void loop() {
  hareket = digitalRead(pirPin);  //Sensörden okuma yapıyoruz.
  
  if(hareket == HIGH){            //Hareketlilik var ise içerideki komutlar uygulanır.
    publishMessage(biri_geldi);
    delay(7000);
  }
  
  if(islem){
    Serial.println("İşlem if inin içine girildi");
    if(millis() > (time_now + period)){
      Serial.println("İşlem if inin içindeki mesaj gönderme if inin içine girildi ve mesaj gönderildi  ve işlem tamamlandı");
      publishMessage(durumlari_sonlandir);
      islem=false;
    }
  }

}
