#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>


#include <secrets.h>



// Temperature MQTT Topics
#define MQTT_PUB_LED_S  "homeassistant/led/state"
#define MQTT_PUB_LED_C "homeassistant/led/command"
#define MQTT_PUB_TEMP "esp32/temperature"
#define MQTT_PUB_HUM  "esp32/humidity"


// Initialize DHT20 sensor

int LED = 38;
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

#define USE_UI    //if you want to use the ui export from Squareline, please do not annotate this line.

#if defined USE_UI
#include <lvgl.h>
#include "ui.h"
#endif

#include <Arduino_GFX_Library.h>
#define TFT_BL 2
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
class LGFX : public lgfx::LGFX_Device
{
  public:
    lgfx::Bus_RGB     _bus_instance;
    lgfx::Panel_RGB   _panel_instance;
    LGFX(void)
    {
      {
        auto cfg = _bus_instance.config();
        cfg.panel = &_panel_instance;

        cfg.pin_d0  = GPIO_NUM_15; // B0
        cfg.pin_d1  = GPIO_NUM_7;  // B1
        cfg.pin_d2  = GPIO_NUM_6;  // B2
        cfg.pin_d3  = GPIO_NUM_5;  // B3
        cfg.pin_d4  = GPIO_NUM_4;  // B4

        cfg.pin_d5  = GPIO_NUM_9;  // G0
        cfg.pin_d6  = GPIO_NUM_46; // G1
        cfg.pin_d7  = GPIO_NUM_3;  // G2
        cfg.pin_d8  = GPIO_NUM_8;  // G3
        cfg.pin_d9  = GPIO_NUM_16; // G4
        cfg.pin_d10 = GPIO_NUM_1;  // G5

        cfg.pin_d11 = GPIO_NUM_14; // R0
        cfg.pin_d12 = GPIO_NUM_21; // R1
        cfg.pin_d13 = GPIO_NUM_47; // R2
        cfg.pin_d14 = GPIO_NUM_48; // R3
        cfg.pin_d15 = GPIO_NUM_45; // R4

        cfg.pin_henable = GPIO_NUM_41;
        cfg.pin_vsync   = GPIO_NUM_40;
        cfg.pin_hsync   = GPIO_NUM_39;
        cfg.pin_pclk    = GPIO_NUM_0;
        cfg.freq_write  = 15000000;

        cfg.hsync_polarity    = 0;
        cfg.hsync_front_porch = 40;
        cfg.hsync_pulse_width = 48;
        cfg.hsync_back_porch  = 40;

        cfg.vsync_polarity    = 0;
        cfg.vsync_front_porch = 1;
        cfg.vsync_pulse_width = 31;
        cfg.vsync_back_porch  = 13;

        cfg.pclk_active_neg   = 1;
        cfg.de_idle_high      = 0;
        cfg.pclk_idle_high    = 0;

        _bus_instance.config(cfg);
      }
      {
        auto cfg = _panel_instance.config();
        cfg.memory_width  = 800;
        cfg.memory_height = 480;
        cfg.panel_width  = 800;
        cfg.panel_height = 480;
        cfg.offset_x = 0;
        cfg.offset_y = 0;
        _panel_instance.config(cfg);
      }
      _panel_instance.setBus(&_bus_instance);
      setPanel(&_panel_instance);
    }
};


LGFX lcd;

/*******************************************************************************
   Screen Driver Configuration  end
*******************************************************************************/


/*******************************************************************************
   Please config the touch panel in touch.h
 ******************************************************************************/
#include "touch.h"

#ifdef USE_UI
/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf[800 * 480 / 10];   //notice here!!!  5,7inch: lv_color_t disp_draw_buf[800*480/10]            4.3inch: lv_color_t disp_draw_buf[480*272/10]
//static lv_color_t disp_draw_buf;
static lv_disp_drv_t disp_drv;

int led_flag = 0;
int led_flag_Lock = 0;
int lcd_brightness = 128;
int lcd_brightness_flag_Lock = 0;
// Variables to hold sensor readings
int  temp;
int  hum;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval_time = 1000;        // Interval at which to publish sensor readings

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  lcd.pushImageDMA(area->x1, area->y1, w, h, (lgfx::rgb565_t*)&color_p->full);
#else
  lcd.pushImageDMA(area->x1, area->y1, w, h, (lgfx::rgb565_t*)&color_p->full);
#endif

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      /*
      Serial.print( "Data x :" );
      Serial.println( touch_last_x );

      Serial.print( "Data y :" );
      Serial.println( touch_last_y );
      */
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
  delay(15);
}
#endif

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.setHostname("ESP-Display"); //define hostname
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event) {
        case ARDUINO_EVENT_WIFI_READY: 
            Serial.println("WiFi interface ready");
            break;
        case ARDUINO_EVENT_WIFI_SCAN_DONE:
            Serial.println("Completed scan for access points");
            break;
        case ARDUINO_EVENT_WIFI_STA_START:
            Serial.println("WiFi client started");
            break;
        case ARDUINO_EVENT_WIFI_STA_STOP:
            Serial.println("WiFi clients stopped");
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.println("Connected to access point");
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("Disconnected from WiFi access point");
            xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
            xTimerStart(wifiReconnectTimer, 0);
            break;
        case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
            Serial.println("Authentication mode of access point has changed");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.print("Obtained IP address: ");
            Serial.println(WiFi.localIP());
            connectToMqtt();
            break;
        case ARDUINO_EVENT_WIFI_STA_LOST_IP:
            Serial.println("Lost IP address and IP address is reset to 0");
            break;
        case ARDUINO_EVENT_WPS_ER_SUCCESS:
            Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
            break;
        case ARDUINO_EVENT_WPS_ER_FAILED:
            Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
            break;
        case ARDUINO_EVENT_WPS_ER_TIMEOUT:
            Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
            break;
        case ARDUINO_EVENT_WPS_ER_PIN:
            Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
            break;
        case ARDUINO_EVENT_WIFI_AP_START:
            Serial.println("WiFi access point started");
            break;
        case ARDUINO_EVENT_WIFI_AP_STOP:
            Serial.println("WiFi access point  stopped");
            break;
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            Serial.println("Client connected");
            break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            Serial.println("Client disconnected");
            break;
        case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
            Serial.println("Assigned IP address to client");
            break;
        case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
            Serial.println("Received probe request");
            break;
        case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
            Serial.println("AP IPv6 is preferred");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
            Serial.println("STA IPv6 is preferred");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP6:
            Serial.println("Ethernet IPv6 is preferred");
            break;
        case ARDUINO_EVENT_ETH_START:
            Serial.println("Ethernet started");
            break;
        case ARDUINO_EVENT_ETH_STOP:
            Serial.println("Ethernet stopped");
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            Serial.println("Ethernet connected");
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Serial.println("Ethernet disconnected");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Serial.println("Obtained IP address");
            break;
        default: break;
    }
}

/*
void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
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
*/

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_PUB_LED_C, 1);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  Serial.println(printf("Disconnect Reason: %i\n", reason));
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

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  
  Serial.println("Publish received.");
  for (int i = 0; i < len; i++) {
    Serial.print((char) payload[i]);
  }
  Serial.println("");
  if (strncmp(payload, "ON", 2) == 0) {
    digitalWrite(LED, HIGH);
    mqttClient.publish(MQTT_PUB_LED_S, 1, true, "ON");
    lv_obj_add_state(ui_Switch2, LV_STATE_CHECKED);
    lv_label_set_text(ui_Label1, "Light ON");
  }
  if (strncmp(payload, "OFF", 3) == 0) {
    digitalWrite(LED, LOW);
    mqttClient.publish(MQTT_PUB_LED_S, 1, true, "OFF");
    lv_obj_clear_state(ui_Switch2, LV_STATE_CHECKED);
    lv_label_set_text(ui_Label1, "Light OFF");
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup()
{
  //IO Port Pins
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(17, OUTPUT);
  digitalWrite(17, LOW);
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  pinMode(42, OUTPUT);
  digitalWrite(42, LOW);
  Serial.begin(9600);
  Wire.begin(19, 20);
  Serial.println();
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.mode(WIFI_STA);
  

  WiFi.onEvent(WiFiEvent);
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials(mqtt_username, mqtt_password);
  connectToWifi();

  // Init Display
  lcd.begin();
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextSize(2);
  delay(200);

#ifdef USE_UI
  lv_init();

  delay(100);
  touch_init();

  screenWidth = lcd.width();
  screenHeight = lcd.height();
  lcd.setBrightness(128);
  //lcd.setRotation(2);
  

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 10);
  //  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, 480 * 272 / 10);
  /* Initialize the display */
  lv_disp_drv_init(&disp_drv);
  /* Change the following line to your display resolution */
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  //disp_drv.sw_rotate = 1;
  
  
  lv_disp_drv_register(&disp_drv);

  /* Initialize the (dummy) input device driver */
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
#endif

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  //digitalWrite(TFT_BL, HIGH);
  analogWrite(TFT_BL, 50);
  //lv_bar_set_start_value(ui_Slider1, 50, LV_ANIM_ON);
#endif

#ifdef USE_UI
  ui_init();//ui from Squareline or GUI Guider
  //lv_bar_set_start_value(ui_Slider2, lcd.getBrightness(), LV_ANIM_ON);
  
#else
  lcd.fillScreen(TFT_RED);
  delay(800);
  lcd.fillScreen(TFT_BLUE);
  delay(800);
  lcd.fillScreen(TFT_YELLOW);
  delay(800);
  lcd.fillScreen(TFT_GREEN);
  delay(800);
#endif
  Serial.println( "Setup done" );
}

char buffer_t[10];
char buffer_h[10];

void publishMqttSensor(){
      
 

}

char buffer_rssi[10];

void loop()
{
#ifdef USE_UI
  lv_timer_handler();
  delay(5);
#endif
  if (led_flag_Lock == 1)
  {
    //Serial.println(printf("Led_flag_lock %i", led_flag_Lock));
    //Serial.println(printf("Led_flag %i", led_flag));

    if (led_flag == 1)
    {
      led_flag_Lock = 0;
      digitalWrite(LED, HIGH);
      mqttClient.publish(MQTT_PUB_LED_S, 0, true, "ON");
    }
    else
    {
      led_flag_Lock = 0;
      digitalWrite(LED, LOW);
      mqttClient.publish(MQTT_PUB_LED_S, 0, true, "OFF");
    }
  }

  if( lcd_brightness_flag_Lock == 1){
    lcd.setBrightness(lcd_brightness);
    Serial.println(printf("Brightness: %i", lcd_brightness));
    lcd_brightness_flag_Lock = 0;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval_time)
  {
    
    previousMillis = currentMillis;
    itoa(WiFi.RSSI(), buffer_rssi, 10);
    //lv_label_set_text(ui_Label4,strcat( "RSSI ", buffer_rssi));
    lv_label_set_text(ui_Label4,buffer_rssi);
    if(mqttClient.connected()){
      lv_label_set_text(ui_Label6," MQTT connected");
    }
    else {
      lv_label_set_text(ui_Label6,"no");
    }
  }
}
