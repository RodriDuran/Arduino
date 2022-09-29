////////////////////////////////////////////////
//                  ///       /   /////       //
//==M-A-D===M-O-D===M-A-quina=D-e=M-O-D-ulos==//
//                  ///       /   /////       //
////////////////////////////////////////////////

/********************************************/
/*
  Rama: contingencia_no_conecta

  Ojetivo: 
  
  Si no puede conectar a wifi entonces
    toma un promedio de las medidas
    el máximo, el mínimo, la desviacion estandar
    almacena la fecha de inicio y el fin 
    del conjunto de medidas

*********************************************/


//cabezera WIFI 
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

//#include <NTPClient.h>
//#include <WiFiUdp.h>


#include "DHT.h"
#include <ArduinoJson.h>//version 5.13.4, n este programa se la version 5.8.0

#include "time.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>//definiciones DHT

#define Nsensores 2
#define DHTPIN 22 
#define DHTTYPE DHT22

#define DHTDOSPIN 21 
 
#define DHTTYPEDOS DHT22

#define MAX_OFFLINE_READINGS 5
#define MAX_PROMEDIO_READINGS 7

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */

#define TIME_TO_SLEEP 5      /* Time ESP32 will go to sleep (in seconds) */

#define wifi_tiempo_fuera_max 20 //cuanto intenta conectar

#define mqtt_tiempo_fuera_max 5 //intentos de conectarse a mqtt

//mas o menos 400 medidas para 2 sensores
RTC_DATA_ATTR bool primer_boot=true;                                      //1byte
RTC_DATA_ATTR unsigned short int bootCount = 0;                           //2byte
RTC_DATA_ATTR unsigned short int bootCountPromedio = 0;                   //2byte
RTC_DATA_ATTR unsigned long readings_time[MAX_OFFLINE_READINGS + 1];      //8bytes
RTC_DATA_ATTR float readings_temp[Nsensores][MAX_OFFLINE_READINGS + 1];   //4bytes
RTC_DATA_ATTR unsigned short int readings_humd[Nsensores][MAX_OFFLINE_READINGS + 1];   //2bytes
                                                                          //1+2+8*n_medidas+(4+2)*n_medidas*n_sensores = 8000 bytes
RTC_DATA_ATTR unsigned long readings_time_ini[MAX_PROMEDIO_READINGS + 1];      //8bytes
RTC_DATA_ATTR unsigned long readings_time_fin[MAX_PROMEDIO_READINGS + 1];      //8bytes

RTC_DATA_ATTR float readings_temp_PROMEDIO[Nsensores][MAX_PROMEDIO_READINGS + 1];   //4bytes
//RTC_DATA_ATTR float readings_temp_MAX[Nsensores][MAX_PROMEDIO_READINGS + 1];   //4bytes
//RTC_DATA_ATTR float readings_temp_MIN[Nsensores][MAX_PROMEDIO_READINGS + 1];   //4bytes
//RTC_DATA_ATTR float readings_temp_STD[Nsensores][MAX_PROMEDIO_READINGS + 1];   //4bytes

RTC_DATA_ATTR unsigned short int readings_humd_PROMEDIO[Nsensores][MAX_PROMEDIO_READINGS + 1];   //2bytes
//RTC_DATA_ATTR unsigned short int readings_humd_MAX[Nsensores][MAX_PROMEDIO_READINGS + 1];   //2bytes
//RTC_DATA_ATTR unsigned short int readings_humd_MIN[Nsensores][MAX_PROMEDIO_READINGS + 1];   //2bytes
//RTC_DATA_ATTR unsigned short int readings_humd_STD[Nsensores][MAX_PROMEDIO_READINGS + 1];   //2bytes


 unsigned long epochTime; 
String rama="contingencia_no_conecta";
bool pudo_conectar=false;
void goToDeepSleep(){
  Serial.println("Going to sleep...");
   Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  // Configure the timer to wake us up!
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  
  /*  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  */
  // Go to sleep! Zzzz
  esp_deep_sleep_start();
}
struct mqtt{
      char* mqtt_server;

      char* test_root_ca PROGMEM;
      char fingerprint[60] PROGMEM;

      char* dispositivo;

      unsigned int  puerto; // Here you keep all the Enter/Exit times during a year/month or whatever you like

      char* cliente;

      char* contra;
  }MQTT[]=
  {
   {"10.10.100.69","","-","Lenox",1883,"-","-"},      
    {"fedealbesa.sytes.net","","-","Lenox",1883,"-","-"},       
    {"dispositivos.onthewifi.com",\
    "-----BEGIN CERTIFICATE-----\n" \
    "MIIFYDCCBEigAwIBAgIQQAF3ITfU6UK47naqPGQKtzANBgkqhkiG9w0BAQsFADA/\n" \
    "MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n" \
    "DkRTVCBSb290IENBIFgzMB4XDTIxMDEyMDE5MTQwM1oXDTI0MDkzMDE4MTQwM1ow\n" \
    "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n" \
    "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwggIiMA0GCSqGSIb3DQEB\n" \
    "AQUAA4ICDwAwggIKAoICAQCt6CRz9BQ385ueK1coHIe+3LffOJCMbjzmV6B493XC\n" \
    "ov71am72AE8o295ohmxEk7axY/0UEmu/H9LqMZshftEzPLpI9d1537O4/xLxIZpL\n" \
    "wYqGcWlKZmZsj348cL+tKSIG8+TA5oCu4kuPt5l+lAOf00eXfJlII1PoOK5PCm+D\n" \
    "LtFJV4yAdLbaL9A4jXsDcCEbdfIwPPqPrt3aY6vrFk/CjhFLfs8L6P+1dy70sntK\n" \
    "4EwSJQxwjQMpoOFTJOwT2e4ZvxCzSow/iaNhUd6shweU9GNx7C7ib1uYgeGJXDR5\n" \
    "bHbvO5BieebbpJovJsXQEOEO3tkQjhb7t/eo98flAgeYjzYIlefiN5YNNnWe+w5y\n" \
    "sR2bvAP5SQXYgd0FtCrWQemsAXaVCg/Y39W9Eh81LygXbNKYwagJZHduRze6zqxZ\n" \
    "Xmidf3LWicUGQSk+WT7dJvUkyRGnWqNMQB9GoZm1pzpRboY7nn1ypxIFeFntPlF4\n" \
    "FQsDj43QLwWyPntKHEtzBRL8xurgUBN8Q5N0s8p0544fAQjQMNRbcTa0B7rBMDBc\n" \
    "SLeCO5imfWCKoqMpgsy6vYMEG6KDA0Gh1gXxG8K28Kh8hjtGqEgqiNx2mna/H2ql\n" \
    "PRmP6zjzZN7IKw0KKP/32+IVQtQi0Cdd4Xn+GOdwiK1O5tmLOsbdJ1Fu/7xk9TND\n" \
    "TwIDAQABo4IBRjCCAUIwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAQYw\n" \
    "SwYIKwYBBQUHAQEEPzA9MDsGCCsGAQUFBzAChi9odHRwOi8vYXBwcy5pZGVudHJ1\n" \
    "c3QuY29tL3Jvb3RzL2RzdHJvb3RjYXgzLnA3YzAfBgNVHSMEGDAWgBTEp7Gkeyxx\n" \
    "+tvhS5B1/8QVYIWJEDBUBgNVHSAETTBLMAgGBmeBDAECATA/BgsrBgEEAYLfEwEB\n" \
    "ATAwMC4GCCsGAQUFBwIBFiJodHRwOi8vY3BzLnJvb3QteDEubGV0c2VuY3J5cHQu\n" \
    "b3JnMDwGA1UdHwQ1MDMwMaAvoC2GK2h0dHA6Ly9jcmwuaWRlbnRydXN0LmNvbS9E\n" \
    "U1RST09UQ0FYM0NSTC5jcmwwHQYDVR0OBBYEFHm0WeZ7tuXkAXOACIjIGlj26Ztu\n" \
    "MA0GCSqGSIb3DQEBCwUAA4IBAQAKcwBslm7/DlLQrt2M51oGrS+o44+/yQoDFVDC\n" \
    "5WxCu2+b9LRPwkSICHXM6webFGJueN7sJ7o5XPWioW5WlHAQU7G75K/QosMrAdSW\n" \
    "9MUgNTP52GE24HGNtLi1qoJFlcDyqSMo59ahy2cI2qBDLKobkx/J3vWraV0T9VuG\n" \
    "WCLKTVXkcGdtwlfFRjlBz4pYg1htmf5X6DYO8A4jqv2Il9DjXA6USbW1FzXSLr9O\n" \
    "he8Y4IWS6wY7bCkjCWDcRQJMEhg76fsO3txE+FiYruq9RUWhiF1myv4Q6W+CyBFC\n" \
    "Dfvp7OOGAN6dEOM4+qR9sdjoSYKEBpsr6GtPAQw4dy753ec5\n" \
    
    "-----END CERTIFICATE-----\n"
,"A0 53 37 5B FE 84 E8 B7 48 78 2C 7C EE 15 82 7A 6A F5 A4 05","Lenox",8883,"-","-"},
   {"192.168.0.106","","-","Lenox",1883,"-","-"},
   {"-","","-","Lenox",1883,"-","-"},
   
  };
//aprendiendo c++ 21 dias davis forgewrti
struct redes{
  char* ssid;
  char* pwd;
}red_wifi[]={
  {"InencoAP01","Inenco.2016"},
  {"12monos","08.p3r1c0"},
  {"pim pum pam","2vecesmas"},
  {"-","-"}
}; 
 struct HyT{
  int h;
  float t;

};
//vivienda se refiere a dentro, y ambiente se refiere a afuera

struct array_dht {
  uint8_t puerto;
  uint8_t tipo;
  String nombre;

} a_dht[] = {

    {DHTPIN,
     DHTTYPE,
    "vivienda"},
     {DHTDOSPIN,
     DHTTYPEDOS,
     "ambiente"}
};

char msg1[500] = ""; //reserva para cadena json
char msg_prom[500] = ""; //reserva para cadena json
WiFiClient  espClient;
PubSubClient client(espClient);
/*
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);


// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;
*/


const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -3600*3;
const int   daylightOffset_sec = 0;

void callback(char* topic, byte* payload, unsigned int length) {
  return;
}

class RedWifi {
  protected:
  int red=0;
  bool conectado=false;
  
  int broker_i=0;
  bool broker_conectado=false;
  int mqtt_tiempo_fuera=0;
  char buf[10];

  public:
  RedWifi(){
     pudo_conectar=false;
  Serial.println("iniciando WIFI....");
  while (!this->conectado && this->red<2){
    WiFi.begin(red_wifi[this->red].ssid,red_wifi[this->red].pwd);  
    int wifi_tiempo_fuera=0;
    while (WiFi.status() != WL_CONNECTED && wifi_tiempo_fuera<wifi_tiempo_fuera_max) {
      delay(500);
      Serial.print(".");
      wifi_tiempo_fuera++;
      }
    if (WiFi.status() != WL_CONNECTED){
       Serial.println();
      Serial.print("no se conecto... ");
      this->red++;
      
     }
    else{
      this->conectado=true;
      this->broker_i=0;

      while (!this->broker_conectado && this->broker_i<2) {
    
          client.setServer(MQTT[this->broker_i].mqtt_server,MQTT[this->broker_i].puerto);
          client.setCallback(callback);
//          #if defined(ESP32)
//            espClient.setCACert(MQTT[broker_i].test_root_ca);
//          #elif defined(ESP8266)
//            espClient.setFingerprint(MQTT[broker_i].fingerprint);
//          #endif
          
          // client.setServer(MQTT[0].mqtt_server, MQTT[0].puerto);

          delay(500);
          
            while (!client.connected() && this->mqtt_tiempo_fuera<mqtt_tiempo_fuera_max) 
              {
              Serial.print("Attempting MQTT connection...");
              // Attempt to connect
              if (client.connect("Lenox")) 
                {
                Serial.print("Conectado a broker: ");
                Serial.println(MQTT[this->broker_i].mqtt_server);
                epochTime = getTime();
                client.publish("/nodo/1/conectado/", "ok");
          
                Serial.println("connected");
                // Once connected, publish an announcement...
                //String disp=String ("dispo")+"/conexion";
                //disp.toCharArray(buf,disp.length()+1);
                //client.publish(buf, "Conectado");
                // ... and resubscribe
                //disp=String ("dispositivo")+"/orden";
                //Serial.print (disp);
                //disp.toCharArray(this->buf,disp.length()+1);
                //client.subscribe(this->buf);
                //Serial.print (this->buf);
                this->broker_conectado=true;
                pudo_conectar=true;
                } 
              else 
                {
                Serial.print("failed, rc=");
                Serial.print(client.state());
                Serial.println(" try again in 5 seconds");
                // Wait 5 seconds before retrying
                delay(5000);
                this->mqtt_tiempo_fuera++;
                this->broker_conectado=false;
                }
        }
       this->broker_i++;
        this->mqtt_tiempo_fuera=0;

      }
      
 
 
     }

  }  


  }
  
  void publicar(char* mensaje){
    client.publish("/nodo/1/TyH/", mensaje);
  }
  void desconecta(){
     //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  }
};
//IBIS proyecto para parsear bases de datos independiente del motor
class SensorDHT {// uno de sus miembros apunta a un objeto DHT , nombre y valor {h,t}
                //devuelve par de flotantes, msj serial, cadena de caract en json, guarda en memoria
protected:
  DHT *sensor;
  double minValue;
  double maxValue;
  double currentValue;
  bool state;
  String nombre;
  HyT medidas;
 

public:
  SensorDHT(DHT *sensor, double minValue, double maxValue, String nombre)
      : sensor(sensor)
  {
    sensor->begin();
    this->nombre = nombre;
    this->minValue = minValue,
    this->maxValue = maxValue;
    this->currentValue = 0;
  }
  double getCurrentValue()
  {
    return currentValue;
  }
  double getMinValue()
  {
    return minValue;
  }
  double getMaxValue()
  {
    return maxValue;
  }
  bool getState()
  {
    return state;
  }
  void poll()
  {
    sensor->readHumidity();
    //@TODO: Implement poll method
  }
  HyT hyt()
  {
    int intentos=0;
    int max_intentos=0;
    bool lectura_correcta=false;
    while ((!lectura_correcta)&&(intentos<max_intentos)){
      Serial.print("Intento: ");
      Serial.println(intentos);
      
      this->medidas.h = (int) sensor->readHumidity();
      this->medidas.t = sensor->readTemperature();
      Serial.print("humedad: ");
      Serial.println(this->medidas.h);

      Serial.print("temperatura: ");
      Serial.println(this->medidas.t);
      if (isnan(this->medidas.h) || isnan(this->medidas.t) ){
        Serial.print("Falla ");
        lectura_correcta=false;
        delay(3000);
      }else{
        lectura_correcta=true;
      }
      intentos++;

    }
    if (!lectura_correcta){
      this->medidas.h=-99;
      this->medidas.t=-99;
    }
    HyT resultado = {0, 0};
    resultado.h = this->medidas.h;

    resultado.t = this->medidas.t;
    return resultado;
  }
  HyT leerMem(int j,int Count){
    //Serial.print ("leyendo memoria Sensor: ");
    //Serial.print (j);
    //Serial.print (" registro: ");
    //Serial.println (Count);
    HyT resultado = {0, 0};
    
    resultado.t=readings_temp[j][Count];
    resultado.h=readings_humd[j][Count];
    //this->medidas.t=resultado.t;
    //this->medidas.h=resultado.h;
    //Serial.println("Mostando dato guarado: ");
    //this->Json(msg1,resultado,epochTime);
    //Serial.println(msg1);
    return resultado;
  }
  void printSerial()
  {

    String n = this->nombre;
    Serial.print(F("Sensor: "));
    Serial.print(this->nombre);
    Serial.print(F(" - Humidity: "));
    Serial.print(this->medidas.h);
    Serial.print(F("% - Temperature: "));
    Serial.print(this->medidas.t);
    Serial.println(F("°C "));
  }
  void printJson()
  {

    StaticJsonBuffer<500> jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root["sensor"] = this->nombre;
    root["temp"] = this->medidas.t;
    root["humd"] = this->medidas.h;
    //root["tiempo"] = epochTime;
    char msg[500] = "";
    root.prettyPrintTo(msg);
    Serial.println(msg);
  }
  void Json(char *outStr,HyT ht,unsigned long tiempo)
  {
    //HyT r = this->hyt();
    StaticJsonBuffer<500> jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root["sensor"] = this->nombre;
    root["temp"] = ht.t;
    root["humd"] = ht.h;
    root["tiempo"] = tiempo;
    //root["tiempo"] = epochTime;
    char msg[500] = "";
    root.prettyPrintTo(msg);
    for (int i = 0; i < 500; ++i)
    {
      outStr[i] = msg[i];
    }
  }

  void Json_prom(char *outStr,float t, unsigned short int h,unsigned long tiempo_ini, unsigned long tiempo_fin)
  {
    //HyT r = this->hyt();
    StaticJsonBuffer<500> jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root["sensor"] = this->nombre;
    root["temp"] = t;
    root["humd"] = h;
    root["tiempo_ini"] = tiempo_ini;
    root["tiempo_fin"] = tiempo_fin;
    
    //root["tiempo"] = epochTime;
    char msg[500] = "";
    root.prettyPrintTo(msg);
    for (int i = 0; i < 500; ++i)
    {
      outStr[i] = msg[i];
    }
  }
  void Guarda_mem(int j,HyT ht)
  {
    //Serial.print ("Guardando Sensor: ");
    //Serial.print (j);
    //Serial.print (" registro: ");
    //Serial.println (bootCount);
    
    readings_temp[j][bootCount] = ht.t;
    readings_humd[j][bootCount] = ht.h;
  }
  

};

class Reloj {
  private:

    const char* ntpServer = "pool.ntp.org";
    const long  gmtOffset_sec = -3600*3;
    const int   daylightOffset_sec = 0;
  
  public:
  Reloj(){
    
     if (primer_boot){
       configTime(this->gmtOffset_sec, this->daylightOffset_sec, this->ntpServer);
       primer_boot=false;
     }
    this->printLocalTime();
     
  }
  
  void printLocalTime(){
      struct tm timeinfo;
      if(!getLocalTime(&timeinfo)){
        Serial.println("Failed to obtain time");
        return;
      }else{
        sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
      }
      
      Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
      Serial.print("Day of week: ");
      Serial.println(&timeinfo, "%A");
      Serial.print("Month: ");
      Serial.println(&timeinfo, "%B");
      Serial.print("Day of Month: ");
      Serial.println(&timeinfo, "%d");
      Serial.print("Year: ");
      Serial.println(&timeinfo, "%Y");
      Serial.print("Hour: ");
      Serial.println(&timeinfo, "%H");
      Serial.print("Hour (12 hour format): ");
      Serial.println(&timeinfo, "%I");
      Serial.print("Minute: ");
      Serial.println(&timeinfo, "%M");
      Serial.print("Second: ");
      Serial.println(&timeinfo, "%S");

      Serial.println("Time variables");
      char timeHour[3];
      strftime(timeHour,3, "%H", &timeinfo);
      Serial.println(timeHour);
      char timeWeekDay[10];
      strftime(timeWeekDay,10, "%A", &timeinfo);
      Serial.println(timeWeekDay);
      Serial.println();

      time_t now;

      char strftime_buf[64];

      time(&now);
      // Set timezone to China Standard Time
      setenv("TZ", "CST-8", 1);
      tzset();

      localtime_r(&now, &timeinfo);
      strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
      ESP_LOGI(TAG, "The current date/time in Shanghai is: %s", strftime_buf);
      Serial.print( strftime_buf);  
      struct timeval tv_now;
      gettimeofday(&tv_now, NULL);
      int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
      settimeofday(&tv_now,NULL);

        sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
  }


};

unsigned long getTime() {
  time_t now;
  //struct tm timeinfo;
  
  time(&now);
  return now;
  };

DHT **dhts = new DHT *[Nsensores]; 

SensorDHT *Controller[Nsensores];

void setup() {
    Serial.begin(9600);

  
 
  
  if (primer_boot){
    // Initialize a NTPClient to get time
    RedWifi Red;
    

    Serial.print("Actualizando reloj: ");
    Reloj Relojo;
    Red.desconecta();
  } 
  

 pinMode(17, OUTPUT);  
 digitalWrite(17,HIGH);
  delay(1000);
  
  for (int j = 0; j < Nsensores; j++){   // Inicia objetos del array|SensorDHT Controller(DHT *objdht,a,b)[] 
                                // con objetos DHT adentro
   
    pinMode(a_dht[j].puerto, INPUT);   // pin de la señal
    Serial.print("pin:");
    Serial.println(a_dht[j].puerto);
    Serial.print("tipo:");
    Serial.println(a_dht[j].tipo);
    dhts[j] = new DHT(a_dht[j].puerto, a_dht[j].tipo);       // librería de clase DHT del fabricante
    Controller[j] = new SensorDHT(dhts[j], 0.0, 30.0, a_dht[j].nombre); // mi clase recibe DHT
                                                                        //, y un nombre
  }
  
}

void loop(){
  
if (bootCount==MAX_OFFLINE_READINGS)
{
    digitalWrite(17,LOW);

Serial.println("*************Toca mostrar los valores guardados en momoria*******************");
      RedWifi *R = new RedWifi;
      
      for (int j = 0; j < Nsensores; j++){
        float temp_sum=0;
        unsigned short int humd_sum=0;
        
        for (int i=0; i<MAX_OFFLINE_READINGS;i++){
          
          HyT mide = Controller[j]->leerMem(j,i);
          Serial.print("memoria temp:");
          Serial.println(mide.t);
          Serial.print("memoria humd:");
          Serial.println(mide.h);
          
          temp_sum+=mide.t;
          humd_sum+=mide.h;
          
        }
        
        readings_temp_PROMEDIO[j][bootCountPromedio] = temp_sum / bootCount-1;
        
        readings_humd_PROMEDIO[j][bootCountPromedio] = humd_sum / bootCount-1;
        Serial.print("memoria temp promedio:");
          Serial.println(readings_temp_PROMEDIO[j][bootCountPromedio]);
          Serial.print("memoria humd promedio:");
          Serial.println(readings_humd_PROMEDIO[j][bootCountPromedio]);
      }  
    readings_time_ini[ bootCountPromedio] = readings_time[0];
    readings_time_fin[ bootCountPromedio] = readings_time[bootCount-1];
        bootCountPromedio++;



  if (pudo_conectar){
    Serial.println("**********************************************************************************************************************");
    Serial.println("Enviando registros:");
    for (int j = 0; j < Nsensores; j++){
      for (int i=0; i<MAX_OFFLINE_READINGS;i++){
        Serial.print("Sensor: ");
        Serial.print(j);
        Serial.print(" - Registro: ");
        Serial.println(i);
        HyT mide = Controller[j]->leerMem(j,i);
        unsigned long tiempo=readings_time[i];
        Controller[j]->Json(msg1,mide,tiempo);
        Serial.println(msg1);
        R->publicar(msg1);
        
        
      }
    }
    Serial.println("**********************************************************************************************************************");
    Serial.println("Enviando promedio:");
    
    for (int j = 0; j < Nsensores; j++){
      for (int i=0; i<bootCountPromedio;i++){
        Serial.print("Sensor: ");
        Serial.print(j);
        Serial.print(" - Promedio: ");
        Serial.println(i);
        
        unsigned long tiempo_i=readings_time_ini[i];
        unsigned long tiempo_f=readings_time_fin[i];
        
        float tt=readings_temp_PROMEDIO[j][i];
        unsigned short int hh = readings_humd_PROMEDIO[j][bootCountPromedio];
        Controller[j]->Json_prom(msg_prom,tt,hh,tiempo_i,tiempo_f);
    
        
        Serial.println(msg_prom);
        R->publicar(msg_prom);
        
        
      }
    }
      

    // falta q mande los promedios también

  delay(6000);
  R->desconecta();
  delete R;
                    

    
  }
  bootCount=0;
//  else
//  {
//    
//    for (int j = 0; j < Nsensores; j++){
//      float temp_sum=0;
//      unsigned short int humd_sum=0;
//      
//      for (int i=0; i<MAX_OFFLINE_READINGS;i++){
//        
//        HyT mide = Controller[j]->leerMem(j,i);
//        
//        
//        temp_sum+=mide.t;
//        humd_sum+=mide.h;
//        
//      }
//      
//      readings_temp_PROMEDIO[j][bootCountPromedio] = temp_sum / bootCount-1;
//      readings_humd_PROMEDIO[j][bootCountPromedio] = humd_sum / bootCount-1;
//    }  
//    readings_time_ini[ bootCountPromedio] = readings_time[0];
//    readings_time_fin[ bootCountPromedio] = readings_time[bootCount-1];
//    bootCount=0;
//    bootCountPromedio++;
//  }
  
  
} 

digitalWrite(17,HIGH);
  delay(1000);
  epochTime = getTime();   
      
  readings_time[bootCount] = epochTime;         
    
  for (int j = 0; j < Nsensores; j++){
    
    HyT mide = Controller[j]->hyt();
    
    //Controller[j]->printSerial();
    
    Controller[j]->Json(msg1,mide,epochTime);
    Serial.println(msg1);
    
    //Controller[j]->printJson();
    
    Controller[j]->Guarda_mem(j,mide);

    
  }
  digitalWrite(17,LOW);
  Serial.println("fin de medidas");
  Serial.println(bootCount);
  Serial.println(MAX_OFFLINE_READINGS);
 
    
    bootCount++;
    goToDeepSleep();
  
  
}
