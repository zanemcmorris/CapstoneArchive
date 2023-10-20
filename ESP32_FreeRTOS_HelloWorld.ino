#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_PRIMARY_CORE 1
#define ARDUINO_AUX_CORE 0
#endif

#include "helloworld.h"


// Task handle definitions
TaskHandle_t sensor_read_task_handle;
TaskHandle_t spiffs_storage_task_handle;
// Preferences object creation
Preferences preferences;

void setupPreferences() {
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

  xTaskCreatePinnedToCore(sensor_read_task,         /*Function to call*/
                          "sensor_read_task",       /*Task name*/
                          10000,                    /*Stack size*/
                          NULL,                     /*Function parameters*/
                          1,                        /*Priority*/
                          &sensor_read_task_handle, /*ptr to global TaskHandle_t*/
                          ARDUINO_PRIMARY_CORE);    /*Core ID*/
}
xTaskCreatePinnedToCore(sensor_read_task,         /*Function to call*/
                        "sensor_read_task",       /*Task name*/
                        10000,                    /*Stack size*/
                        NULL,                     /*Function parameters*/
                        1,                        /*Priority*/
                        &sensor_read_task_handle, /*ptr to global TaskHandle_t*/
                        ARDUINO_PRIMARY_CORE);    /*Core ID*/
}
void sensor_read_task(void *pvParameter) {
  uint16_t arraySize = 11;
  float dataArray[arraySize];
  uint16_t taskIteration = 0;
  while (1) {
    Serial.print("Hello world!\n");
    Serial.print(taskIteration++);
    Serial.println();
    taskIteration %= arraySize;
    read_all_sensors(&dataArray[0], arraySize);
    for (int i = 0; i < arraySize; i++)
      Serial.println(dataArray[i]);

    vTaskDelay(5000 / portTICK_RATE_MS);
  }
}
