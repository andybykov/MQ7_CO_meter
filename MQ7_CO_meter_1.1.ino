/* Программа для калибровки датчика газа MQ-7 и измерения CO v1.1 */
/*
  - Запись и чтение значения R0 из EEPROM, что не потребует повторной калибровки
  - Улучшен вывод информации на экран
*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h> // Добавляем работу с EEPROM

// Определения
#define pinCO A2
#define pinPWM 5
#define pinVolt A0
#define VERSION 1.1

// Константы
#define MAX_DUTY_PWM 255
#define LOW_HEAT_TIME 90000
#define HIGH_HEAT_TIME 60000
const float stepsADC = 1023.0;
const float refVoltage = 4.8;
const float lowHeaterVoltage = 1.4;

// Адреса EEPROM
#define EEPROM_R0_ADDR 0
#define EEPROM_CHECKSUM_ADDR sizeof(float)

// Объект дисплея
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// Глобальные переменные
static float CO_R0 = 0.0;
static int targetDutyPWM = 0;
unsigned long lastUpdate = 0;

// Прототипы функций
void writeFloatToEEPROM(int addr, float value);
float readFloatFromEEPROM(int addr);
bool eepromDataValid();
void saveCalibrationData();
void loadCalibrationData();
void (*resetFunc)(void) = 0; // Указатель на функцию сброса 

// Функция записи float в EEPROM
void writeFloatToEEPROM(int addr, float value) {
  EEPROM.put(addr, value);
}

// Функция чтения float из EEPROM
float readFloatFromEEPROM(int addr) {
  float value;
  EEPROM.get(addr, value);
  return value;
}

// Проверка контрольной суммы
bool eepromDataValid() {
  float storedR0 = readFloatFromEEPROM(EEPROM_R0_ADDR);
  return !(isnan(storedR0) || storedR0 < 1.0 || storedR0 > 20.0);
}

// Сохранение калибровочных данных
void saveCalibrationData() {
  writeFloatToEEPROM(EEPROM_R0_ADDR, CO_R0);
}

// Загрузка калибровочных данных
void loadCalibrationData() {
  CO_R0 = readFloatFromEEPROM(EEPROM_R0_ADDR);
}

// Получаем напряжение с пина
float getVoltagePin(uint8_t pin) {
  float rawValue = 0;
  const int avg = 50;  // количество усреднений

  // Замер с пина pin
  for (int i = 0; i < avg; ++i) {
    rawValue += analogRead(pin);
    delay(10);
  }

  // Вычисление среднего значения
  rawValue /= avg;

  // Преобразование в напряжение
  float inputVolt = (rawValue * refVoltage) / stepsADC;

  // Возвращаем значение, если оно больше 0.1V, иначе возвращаем 0.0
  return (inputVolt < 0.1) ? 0.0 : inputVolt;
}

// Вывод напряжения в порт
void printVoltage(float voltage) {
  Serial.print("Voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");
}

// Тест пинов-измерителей
void testPin() {
  float resultCO = 0;
  float resultVolt = 0;
  int avg = 50;  // количество усреднений

  //замер с пина pinCO
  for (int i = 0; i < avg; ++i) {
    resultCO += analogRead(pinCO);
    delay(10);
  }
  //замер с пина pinVolt
  for (int i = 0; i < avg; ++i) {
    resultVolt += analogRead(pinVolt);
    delay(10);
  }

  float ValCO = resultCO / avg;
  float ValVolt = resultVolt / avg;
  // вывод
  Serial.print("Raw values: ");
  Serial.print("COPin = ");
  Serial.print(ValCO);
  Serial.print("  ");
  Serial.print(" | ");
  Serial.print("  ");
  Serial.print("VoltPin = ");
  Serial.print(ValVolt);
  Serial.print("  ");
  Serial.print(" || ");
  Serial.print("  ");
  Serial.print("Voltage: ");
  Serial.print("COPin = ");
  Serial.print((ValCO * refVoltage) / stepsADC);
  Serial.print("V");
  Serial.print("  ");
  Serial.print(" | ");
  Serial.print("  ");
  Serial.print("VoltPin = ");
  Serial.print((ValVolt * refVoltage) / stepsADC);
  Serial.print("V");
  Serial.print("\n");
}

// Heating cycles
void heatingCycles(int lowPwm) {
    // **** Процесс нагрева MQ-7 ****
  // Нагреваем датчик высоким напряжением (5V) в течение 60 секунд
  Serial.print("1. Start high-heat cycle for ");
  Serial.print(HIGH_HEAT_TIME / 1000);
  Serial.print(" seconds");
  Serial.print("\n");
  analogWrite(pinPWM, MAX_DUTY_PWM);  // Подача 5V на обогреватель
  delay(HIGH_HEAT_TIME);              // Задержка 60 секунд

  // После этого подаём низкое напряжение (1.4V) на обогреватель для измерения стабильного сигнала
  Serial.print("2. Start low-heat cycle for ");
  Serial.print(LOW_HEAT_TIME / 1000);
  Serial.print(" seconds");
  Serial.print("\n");
  analogWrite(pinPWM, lowPwm);  // Подача ~1.4V (значение рассчитано для конкретного датчика)
  delay(LOW_HEAT_TIME);         // Задержка 90 секунд для стабилизации датчика и получения показаний при 1.4V
}

// Калибровка
float calibratingCycles(int lowPwm) {
  
  // Считываем аналоговое значение с датчика
  float CO_sensorRead = 0.0;  // Считываемое значение АЦП датчика
  CO_sensorRead = analogRead(pinCO);
  if (CO_sensorRead <= 0 || CO_sensorRead >= 1024) {
    Serial.print("Wrong data of CO sensor!");
    return 0.0;
  }
  // Объявление переменных для измерений и вычислений
  float CO_VRL;               // Напряжение на нагрузочном резисторе (на выходе датчика)
  float CO_RS;                // Сопротивление датчика в текущих условиях
  float CO_RL = 10;           // Нагрузочный резистор (в килоомах)
  float CO_VC = refVoltage;   // Напряжение питания (постоянный ток)
  float CO_R0 = 0;                // Калибровочное сопротивление в чистом воздухе

  Serial.println("Calibrating....");
  heatingCycles(lowPwm); //вызов циклов нагрева
  // **** выполнение вычислений ****
  // Переводим значение АЦП в напряжение (диапазон 0–1023 соответствует 0–CO_VC)
  CO_VRL = CO_sensorRead * (CO_VC / stepsADC);

  // Вычисляем сопротивление датчика RS по формуле делителя напряжения:
  // RS = ((CO_VC / CO_VRL) - 1) * CO_RL
  CO_RS = ((CO_VC / CO_VRL) - 1) * CO_RL;  //Вычислите RS (сопротивление датчика) в свежем воздухе

  // Расчёт калибровочного сопротивления R₀:
  // CO_R0 = CO_RS / 25.75
  // Значение 25.75 получено по графику из технической документации датчика
  CO_R0 = CO_RS / 25.75;

  // Вывод результата измерения (R₀) в последовательный монитор
  Serial.println("Calibrating done!");
  Serial.print("R0 = ");
  Serial.println(CO_R0);
  return CO_R0;
}

// Основеные циклы измерения СО
float measuringCycles(int lowPwm, float R0) {
  
  float VRL = analogRead(pinCO) * (refVoltage / stepsADC); // Считываемое значение АЦП датчика преобразуем в напряжение
  // Проверка на минимальное напряжение
  if (VRL < 0.01) {
    Serial.println("Error: Sensor not connected!");
    return -1.0;
  }

  Serial.println("Measuring....");
  heatingCycles(lowPwm); //вызов циклов нагрева
  float RL = 10000.0; // RL = 10 кОм = 10000 Ом
  float RS = ((refVoltage / VRL) - 1.0) * RL; 
  float ratio = RS / R0; //коэффициент сопротивления
  
  // Проверка деления на ноль
  if (ratio < 0.001) {
    Serial.println("Error: Invalid ratio!");
    return -1.0;
  }
  
  return pow(19.709 / ratio, 1.0 / 0.652);
}

// Функция вывода на экран
void printLcd(float ppm) {
  if(millis() - lastUpdate > 1000) { // Обновление раз в секунду
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CO: ");
    lcd.print(ppm, 2);
    lcd.print(" ppm");
    
    lcd.setCursor(0, 1);
    lcd.print("R0: ");
    lcd.print(CO_R0, 4);
    
    lastUpdate = millis();
  }
}
// Тест дисплея
void testLcd() {
  lcd.home();
  for (uint8_t i = 0; i < 16; i++) {
    lcd.write(0xFF);  //255
    delay(50);
  }
  lcd.setCursor(0, 1);  // Переходим на следующую строку
  for (uint8_t i = 0; i < 16; i++) {
    lcd.write(0xFF);  //255
    delay(50);
  }
  delay(1000);  // Ждем 1 секунды
  lcd.clear();  // Очищаем экран
}

/* SETUP */
void setup() {
  // Инициализация LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Проверка EEPROM
  EEPROM.begin();
  if(eepromDataValid()) {
    loadCalibrationData();
    lcd.print("Loaded R0: ");
    lcd.setCursor(0, 1);
    lcd.print(CO_R0, 4);
    delay(2000);
  } else {
    lcd.print("Need calibration");
    delay(2000);
  }
  lcd.clear();

  // Инициализация портов
  Serial.begin(9600);
  pinMode(pinPWM, OUTPUT);
  pinMode(pinVolt, INPUT);
  pinMode(pinCO, INPUT);
  analogWrite(pinPWM, MAX_DUTY_PWM);

  // Калибровка при необходимости
  if(CO_R0 < 1.0 || CO_R0 > 20.0) {
    lcd.clear();
    lcd.print("Calibrating...");
    
    targetDutyPWM = (int)((lowHeaterVoltage / refVoltage) * MAX_DUTY_PWM);
    CO_R0 = calibratingCycles(targetDutyPWM);
    
    if(CO_R0 > 1.0 && CO_R0 < 20.0) {
      saveCalibrationData();
      lcd.clear();
      lcd.print("Calibration OK!");
      lcd.setCursor(0, 1);
      lcd.print("R0: ");
      lcd.print(CO_R0, 4);
      delay(2000);
    } else {
      lcd.clear();
      lcd.print("Calibration FAIL!");
      delay(2000);
      resetFunc();
    }
  }
  lcd.clear();
}


/* LOOP */
void loop() {
  float COppm = measuringCycles(targetDutyPWM, CO_R0);
  
  // Вывод на LCD
  printLcd(COppm);
  
  // Вывод в Serial
  Serial.print("CO: ");
  Serial.print(COppm, 2);
  Serial.println(" ppm");
  
  delay(500); // Небольшая задержка для стабильности
}