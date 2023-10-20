#include "my_gpio.h"


// Sensor object definitions
PASCO2Ino co2Sensor;
int16_t co2PPM;
Error_t co2Error;
SensirionI2CSen5x sen5x;

void setupSENSensor() {
  Serial.println("SEN54: Setting up...");
  sen5x.begin(Wire);
  uint16_t error;
  char errorMessage[256];
  error = sen5x.deviceReset();
  if (error) {
    Serial.print("SEN54: Error trying to execute deviceReset(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  error = sen5x.setTemperatureOffsetSimple(0);  // No temp offset
  error = sen5x.startMeasurement();
  Serial.println("SEN54: Set up sensor successfully!");
}

void setupCO2Sensor(Error_t errorPtr, PASCO2Ino CO2SensorPtr) {
  errorPtr = co2Sensor.begin();
  Serial.println("PAS CO2: Setting up...");
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
    Serial.println("PAS CO2: Set up sensor successfully!");
}

float readNGSensor() {
  float reading = (3.3 * analogRead(NGPin)) / 4095;
  return 10.938 * exp(1.7742 * (reading * 3.3 / 4095));
}

float readCOSensor() {
  float sensorVoltage = (3.3 * analogRead(COPin)) / 4095;
  return 3.027 * exp(1.0698 * sensorVoltage);
}

void readSENSensor(float *retArray, uint8_t arraySize) {
  // Read Measurement
  // Serial.println("Reading SEN...");
  char errorMessage[256];
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
  }
}

uint16_t readCO2PPM(Error_t errorPtr, PASCO2Ino CO2SensorPtr) {
  co2PPM = 0;
  do {
    errorPtr = co2Sensor.getCO2(co2PPM);
    Serial.print("Error reading CO2 w/ error code:");
    Serial.println(errorPtr);
    delay(1000);
  } while (co2PPM == 0);
  return co2PPM;
}

void read_all_sensors(float *ret_array, uint16_t array_size) {
  // Reads all sensors and outputs into array
  /*
  0: CO2 PPM - PASCO2
  1: PPM 1.0 - SEN 
  2: PPM 2.5 - SEN
  3: PPM 4.0 - SEN
  4: PPM 10.0 - SEN
  5: Humidity - SEN
  6: Temperature - SEN
  7: VOCs - SEN
  8: CO - MQ7
  9: NG - MQ4
  */
  // CO2
  float co2_ppm_return = readCO2PPM(co2Error, co2Sensor);
  ret_array[0] = co2_ppm_return;
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

void setup_GPIO() {
  // Setup I2C
  // Wire.begin(); // For breadboard QT-PY ESP32
  Serial.begin(115200);
  Serial.println("Setting up GPIO...");
  Wire.begin(21, 22);          // Used for Sparkfun Thing
  Wire.setClock(I2C_FREQ_HZ);  // 400KHz

  setupCO2Sensor(co2Error, co2Sensor);  // Setup PASCO2 Sensor
  setupSENSensor();
  Serial.println("Successfully set up GPIO.");
}
