#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_PRIMARY_CORE 1
#define ARDUINO_AUX_CORE 0
#endif

#include "helloworld.h"


// Task handle definitions
TaskHandle_t hello_task_handle;
TaskHandle_t blink_task_handle;
TaskHandle_t blink_task_handle1;

// Preferences object creation
Preferences preferences;

void setupPreferences()
{
  // Preferences is good for single KVP storage.
  // We want to use SPIFF for large storage
  bool status = preferences.begin("my_app", false);
}

void loop() {}
void setup() {
  Serial.begin(115200);
  Serial.write("Setting up...");
  setup_GPIO();
  setupPreferences();

  // pinMode(LED_PIN_0, OUTPUT);

  xTaskCreatePinnedToCore(hello_task,            /*Function to call*/
                          "hello_task",          /*Task name*/
                          10000,                 /*Stack size*/
                          NULL,                  /*Function parameters*/
                          1,                     /*Priority*/
                          &hello_task_handle,    /*ptr to global TaskHandle_t*/
                          ARDUINO_PRIMARY_CORE); /*Core ID*/
  // xTaskCreatePinnedToCore(blinky, "blink_task_handle", 10000, NULL, 1, &blink_task_handle, 1);
  // xTaskCreatePinnedToCore(blink1, "blink1_task_handle", 10000, NULL, 1, &blink_task_handle1, 1);
}
void hello_task(void *pvParameter) {
  uint16_t arraySize = 11;
  float dataArray[arraySize];
  uint16_t taskIteration = 0;
  while (1) {
    Serial.print("Hello world!\n");
    Serial.print(taskIteration++);
    Serial.println();
    taskIteration %= arraySize;
    // dataArray[0] = readCO2PPM(co2Error, co2Sensor);
    read_all_sensors(&dataArray[0], arraySize);
    for(int i=0;i<arraySize;i++)
      Serial.println(dataArray[i]);
    
    vTaskDelay(5000 / portTICK_RATE_MS);
  }
}
void blinky(void *pvParameter) {
  // pinMode(LED_OUT_3, OUTPUT);
  int count = 0;
  while (1) {
    /* Blink off (output low) */
    count++;
    if (count >= 255) {
      count = 0;
    }
    // analogWrite(LED_OUT_3, count);
    vTaskDelay(10);
  }
}

void blink1(void *pvParameter) {
  // pinMode(LED_OUT_2, OUTPUT);
  int count = 0;
  while (1) {
    /* Blink off (output low) */
    count++;
    if (count >= 255) {
      count = 0;
    }
    // analogWrite(LED_OUT_2, count);
    vTaskDelay(15);
  }
}