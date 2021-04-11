/*
This example uses FreeRTOS softwaretimers as there is no built-in Ticker library
*/




#include <Servo.h>               //Servo kütüphanemizi ekliyoruz.

int pirPin = 35;                   //Sensörü takacağımız pin
int servoPin = 19;                 //Servoyu takacağımız pin
int hareket;                      //Sensörden aldığımız veri
Servo motor;                      //Servo motor değişkeni





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

char *biri_geldi = "101";
char *kapiyi_ac = "201";
char *kapiyi_acma = "301";
char *kapi_acildi = "401";
char *led_yakildi = "501";

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
//  publishMessage(biri_geldi);
  //mqttClient.publish("esp-s", 0, true, "test 1");
//  Serial.println("Publishing at QoS 0");
  //uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  //Serial.print("Publishing at QoS 1, packetId: ");
  //Serial.println(packetIdPub1);
  //uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  //Serial.print("Publishing at QoS 2, packetId: ");
  //Serial.println(packetIdPub2);
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
        Serial.println("msg: kapiyi_aç");
        motor.write(180);
        publishMessage(kapi_acildi);
      } else if(payload == kapiyi_acma){
        Serial.println("msg: kapiyi_kapat");
      } else {
        Serial.println("başka mesaj");
      }
    
    } else {
      Serial.println("topic yanlış");
    }
  /*
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
  Serial.print("  payload: ");
  Serial.println(payload);
  */
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


  motor.attach(servoPin);         //Servomuzu 9. pin ile ilişkilendiriyoruz.
  pinMode(pirPin, INPUT);         //Sensör pinimizi giriş olarak ayarlyoruz.

  

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
  Serial.print("Gönderilen mesaj sonucu:");
  Serial.println(packetIdPub2);
  }


void subscribeTopic(char* mesaj){
  uint16_t packetIdSub = mqttClient.subscribe(sub_topic, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
}






void callback(char* sub_topic, byte* payload, unsigned int length) {

  String response;
  
  Serial.print("Message arrived in topic: ");
  Serial.println(sub_topic);
 
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    response += (char)payload[i];
    Serial.print((char)payload[i]);
  }

 
  Serial.println();
  Serial.println("-----------------------");
  
  if(response == "201")  // Turn the servo
  {
    motor.write(180);
    /*
    delay(350);
    motor.write(1);
    delay(350);
    motor.write(180);
    delay(350);
    motor.write(1);
    delay(350);
    motor.write(180);
    delay(350);
    motor.write(1);
    delay(350);
    motor.write(90);
    */
    mqttClient.publish(pub_topic, 0, true, kapi_acildi);
  }
  else if(response == "301")  // Turn the light off
  {
  //Serial.print("Gelen mesaj : ");
  //Serial.println(response);
    mqttClient.publish(pub_topic, 0, true, led_yakildi);
  //client.publish(STATE_TOPIC,"off");
  }
}



bool isEquals(char* first, char* sec, int t) { //eşit mi
      for (int i = 0; i<t;i++){
        if (first[i] != sec[i]){
          return false;
        }
      }
      return true;
  }





/*

void callback(char* sub_topic, char* kapiyi_ac) {
  String response;
 unsigned int length;
  for (int i = 0; i < length; i++) {
    response += (char)kapiyi_ac[i];
  }
  Serial.print("Message arrived [");
  Serial.print(sub_topic);
  Serial.print("] ");
  Serial.println(response);
  if(response == "201")  // Turn the light on
  {
    motor.write(180);
    delay(350);
    motor.write(1);
    delay(350);
    motor.write(180);
    delay(350);
    motor.write(1);
    delay(350);
    motor.write(180);
    delay(350);
    motor.write(1);
    delay(350);
    motor.write(90);
    mqttClient.publish(pub_topic, 0, true, kapi_acildi);
  }
  else if(response == "301")  // Turn the light off
  {
  //Serial.print("Gelen mesaj : ");
  //Serial.println(response);
    mqttClient.publish(pub_topic, 0, true, led_yakildi);
  //client.publish(STATE_TOPIC,"off");
  }
}

*/



void loop() {
  hareket = digitalRead(pirPin);  //Sensörden okuma yapıyoruz.
  
  if(hareket == HIGH){            //Hareketlilik var ise içerideki komutlar uygulanır.
    publishMessage(biri_geldi);
    delay(3000);
  }
  else{                           
  }
}
