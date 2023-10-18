#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_PRIMARY_CORE 1
#define ARDUINO_AUX_CORE 0
#endif

#include "helloworld.h"

// Standard GPIO Pin definitions
#define LED_OUT_3 A3
#define LED_OUT_2 A2
#define LED_OUT_1 A1
#define NGPin A0
#define COPin A1
// Task handle definitions
TaskHandle_t hello_task_handle;
TaskHandle_t blink_task_handle;
TaskHandle_t blink_task_handle1;
// Sensor object definitions
PASCO2Ino co2Sensor;
int16_t co2PPM;
Error_t co2Error;
Adafruit_BMP3XX bmp;
SensirionI2CSen5x sen5x;
// Preferences object creation
Preferences preferences;


void setupGPIO() {
  // Setup I2C
  // Wire.begin(); // For breadboard QT-PY ESP32
  Serial.println("Setting up GPIO...");
  Wire.begin(21, 22);          // Used for Sparkfun Thing
  Wire.setClock(I2C_FREQ_HZ);  // 400KHz

  setupCO2Sensor(co2Error, co2Sensor);  // Setup PASCO2 Sensor
  // setupBMPSensor();
  setupSENSensor();
  Serial.println("Successfully set up GPIO.");
}

void setupPreferences()
{
  // Preferences is good for single KVP storage.
  // We want to use SPIFF for large storage
  bool status = preferences.begin("my_app", false);
}

void read_all_sensors(float *ret_array, uint16_t array_size) {
  // Reads all sensors and outputs into array
  /*
  0: CO2 PPM - PASCO2
  1: Pressure - BMP390
  2: PPM 1.0
  3: PPM 2.5
  4: PPM 4.0
  5: PPM 10.0
  6: Humidity
  7: Temperature
  8: VOCs
  9: CO
  10: NG
  */
  // CO2
  float co2_ppm_return = readCO2PPM(co2Error, co2Sensor);
  String co2ppmString = String(co2_ppm_return);
  // Serial.print("CO2: ");
  // Serial.println(co2ppmString);
  ret_array[0] = co2_ppm_return;
  // Pressure
  // while (!bmp.performReading()) {
  //   Serial.println("Failed to perform BMP390 reading :(");
  // }
  // Serial.print("BMP390 Temperature = ");
  // Serial.print(bmp.temperature);
  // Serial.println(" *C");

  // Serial.print("BMP390 Pressure = ");
  // Serial.print(bmp.pressure / 100.0);
  // Serial.println(" hPa");
  // SEN
  // Uses ret_array[2] through ret_array[8]
  readSENSensor(&ret_array[2], 7);
  // CO
  float COReading = readCOSensor();
  ret_array[9] = COReading;
  // Serial.print("CO ppm: ");
  // Serial.println(ret_array[9]);
  // NG
  float NGReading = readNGSensor();
  ret_array[10] = NGReading;
  // Serial.print("NG ppm: ");
  // Serial.println(ret_array[10]);
}
void setupSENSensor() {
  Serial.println("Setting up SEN...");
  sen5x.begin(Wire);
  uint16_t error;
  char errorMessage[256];
  error = sen5x.deviceReset();
  if (error) {
    Serial.print("Error trying to execute deviceReset(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  error = sen5x.setTemperatureOffsetSimple(0);  // No temp offset
  error = sen5x.startMeasurement();
  Serial.println("SEN Setup!");
}
void setupBMPSensor() {
  while (!bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void setupCO2Sensor(Error_t errorPtr, PASCO2Ino CO2SensorPtr) {
  errorPtr = co2Sensor.begin();
  if (XENSIV_PASCO2_OK != errorPtr) {
    Serial.print("PAS CO2: initialization error: ");
    Serial.println(errorPtr);
  }
  /* We can set the reference pressure before starting 
     * the measure 
     */
  errorPtr = co2Sensor.setPressRef(PRESSURE_REFERENCE);
  if (XENSIV_PASCO2_OK != errorPtr) {
    Serial.print("PAS CO2: pressure reference error: ");
    Serial.println(errorPtr);
  }
  errorPtr = co2Sensor.startMeasure(5);
  if (XENSIV_PASCO2_OK != errorPtr) {
    Serial.print("PAS CO2: startmeasure error: ");
    Serial.println(errorPtr);
  }

  if (errorPtr == XENSIV_PASCO2_OK)
    Serial.println("PAS CO2: Set up sensor successfully.");
}

float readNGSensor() {
  float reading = (3.3 * analogRead(NGPin))/4095;
  return 10.938 * exp(1.7742 * (reading * 3.3 / 4095));
}

float readCOSensor() {
  float sensorVoltage =  (3.3 * analogRead(COPin))/4095;
  return 3.027*exp(1.0698*sensorVoltage);
}

void readSENSensor(float *retArray, uint8_t arraySize) {
  // Read Measurement
  // Serial.println("Reading SEN...");
  char errorMessage[256];
  // float massConcentrationPm1p0;
  // float massConcentrationPm2p5;
  // float massConcentrationPm4p0;
  // float massConcentrationPm10p0;
  // float ambientHumidity;
  // float ambientTemperature;
  // float vocIndex;
  // float noxIndex;
  uint16_t error;

  // error = sen5x.readMeasuredValues(
  //   massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
  //   massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
  //   noxIndex);  // Need to copy print statement and figure out how to pass stuff along
  // // Also need to figure out how to sort files in this thang
  error = sen5x.readMeasuredValues(
    retArray[0], retArray[1], retArray[2],
    retArray[3], retArray[4], retArray[5], retArray[6],
    retArray[7]);

  if (error) {
    Serial.print("Error trying to execute readMeasuredValues(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  // } else {
  //   Serial.print("MassConcentrationPm1p0:");
  //   Serial.print(retArray[0]);
  //   Serial.print("\t");
  //   Serial.print("MassConcentrationPm2p5:");
  //   Serial.print(retArray[1]);
  //   Serial.print("\n");
  //   Serial.print("MassConcentrationPm4p0:");
  //   Serial.print(retArray[2]);
  //   Serial.print("\t");
  //   Serial.print("MassConcentrationPm10p0:");
  //   Serial.print(retArray[3]);
  //   Serial.print("\n");
  //   Serial.print("AmbientHumidity:");
  //   if (isnan(retArray[4])) {
  //     Serial.print("n/a");
  //   } else {
  //     Serial.print(retArray[4]);
  //   }
  //   Serial.print("\t");
  //   Serial.print("AmbientTemperature:");
  //   if (isnan(retArray[5])) {
  //     Serial.print("n/a");
  //   } else {
  //     Serial.print(retArray[5]);
  //   }
  //   Serial.print("\t");
  //   Serial.print("VocIndex:");
  //   if (isnan(retArray[6])) {
  //     Serial.print("n/a");
  //   } else {
  //     Serial.print(retArray[6]);
  //   }
  //   Serial.print("\t");
  //   Serial.print("NoxIndex:");
  //   if (isnan(retArray[7])) {
  //     Serial.println("n/a");
  //   } else {
  //     Serial.println(retArray[7]);
  //   }
  }
}
uint16_t readCO2PPM(Error_t errorPtr, PASCO2Ino CO2SensorPtr) {
  co2PPM = 0;
  do {
    errorPtr = co2Sensor.getCO2(co2PPM);
    Serial.print("Error reading CO2 w/ error code:");
    Serial.println(errorPtr);
    // Serial.print("CO2 ppm: ");
    // Serial.println(co2PPM);
    delay(1000);
  } while (co2PPM == 0);
  return co2PPM;
}

void loop() {}
void setup() {
  Serial.begin(115200);
  Serial.write("Setting up...");
  setupGPIO();
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