/*
termostat_for_beko_CN335220.ino

Термореле для бытового холодильника с OLED экраном SSD1306 128X64

При включении напряжения сети - пауза 10 минут до включения двигателя независимо от температуры.
При отключении, повторное включение не ранее 10 минут независимо от температуры.
При перезагрузке - пауза 10 минут до включения двигателя независимо от температуры.
Если компрессор работает более 1 часа - принудительная остановка независимо от температуры.
Если температурный датчик оборван или неисправен, то компрессор работает
в цикле (20 мин работа/20 мин отдых), при этом загорается аварийный светодиод
 
Version: 20220518
*/
#include <OLED_I2C.h>
OLED  myOLED(SDA, SCL);
extern uint8_t SmallFont[];
extern uint8_t BigNumbers[];

#include <avr/wdt.h> // WatchDog

// Температура MC морозилки включения компрессора
#define MCTX_TEMP_MAX -16.0
// Температура MC морозилки отключения компрессора
#define MCTX_TEMP_MIN -21.0
// Температура HC холодильная включения компрессора
#define HCTX_TEMP_MAX 10.0
// Температура HC холодильная отключения компрессора
#define HCTX_TEMP_MIN 4.0
// Пауза при включении в сеть (120000 = 2 минут)
#define DELAY_COMPRESSOR_PWR 120000
// Пауза перед включением компрессора (1200000 = 20 минут)
#define DELAY_COMPRESSOR 1200000
// Максимальное время работы компрессора (3600000 = 1 час)
#define MAXTIME_COMPRESSOR 3600000
// Минимальная темп ошибки
#define ERROR_MIN_TEMP -35.0
// Максимальная темп ошибки
#define ERROR_MAX_TEMP 35.0
// Время работы компрессора при ошибке датчика температуры (1200000 = 20 минут)
#define ERROR_COMPRESSOR_ON 1200000
// Время паузы компрессора при ошибке датчика температуры (1200000 = 20 минут)
#define ERROR_COMPRESSOR_OFF 1200000
//--------------------------------------
// Температура включения вентилятора HC t > 6
#define HC_TEMP_FAN_ON 7.0
// Температура выключения вентилятора HC t < 1
#define HC_TEMP_FAN_OFF 3.0
// Время задержки включения вентилятора HC (120000 = 2 минуты)
#define DELAY_HC_FAN 60500
// Температура включения вентилятора MC t > -17
#define MC_TEMP_FAN_ON -17.0
// Температура выключения вентилятора MC t < -22
#define MC_TEMP_FAN_OFF -22.0
// Время задержки включения вентилятора MC (120000 = 2 минуты)
#define DELAY_MC_FAN 61000
//--------------------------------------
// Температура включения no-frost при подаче питания
#define NOFR_TEMP_POWER -15.0
// Температура выключения no-frost
#define NOFR_TEMP_MAX 2.0
// Время после подачи питания при котором разрешено no-frost (60000 = 1 мин)
//#define DELAY_READY_NOFROST_PWR 60000
// Задержка на включение no-frost после подачи питания (если низкая темп) (60000 = 1 мин)
#define DELAY_NOFROST_PWR 60000
// Пауза перед включением no-frost (28800000 = 8 часов)
#define DELAY_NOFROST 28800000
// Максимальное время работы no-frost (600000 = 10 минут)
#define MAXTIME_NOFROST 600000
//--------------------------------------
// Время работы ионизатора (60000 = 1 мин)
#define DELAY_ION_ON 60000
// Время паузы ионизатора (1800000 = 30 min)
#define DELAY_ION_OFF 1800000
//--------------------------------------
// Реле1 двигатель подключено к 2 пину
#define RELE1 4
// Реле2 вентилятор в холодильной камере.
#define RELE2 5
// Реле3 вентилятор в морозильной камере.
#define RELE3 6
// Реле4 no-frost на 5 пину
#define RELE4 7
// Реле5 ионизации на 6 пину.
#define RELE5 8
//--------------------------------------
#define HCDT A0        // ДТ NTC-термистор в холодильной камере
#define HTDT A3        // ДТ NTC в тене холодильной камеры
#define MCDT A1        // ДТ NTC в морозильной камере
#define MTDT A2        // ДТ NTC в тене морозильной камеры
// сопротивление при 25 градусах по Цельсию
#define THERMISTORNOMINAL 10000
// temp. для номинального сопротивления (практически всегда равна 25 C)
#define TEMPERATURENOMINAL 25
// бета коэффициент термистора (обычно 3000-4000)
#define BCOEFFICIENT 3950
// емкость резистора в цепи R=10 КОм
#define SERIESRESISTOR 10000
// сколько показаний берется для определения среднего значения
// чем больше значений, тем дольше проводится калибровка,
// но и показания будут более точными
#define NUMSAMPLES 10
//--------------------------------------
// Светодиод активности подключён к 13 пину
#define LED_ACTIVITY 13
// Свечение светодиода активности в мс
#define LED_ACTIVITY_ON 1000
// Пауза светодиода активности в мс
#define LED_ACTIVITY_OFF 2000
// Светодиод ошибки датчика температуры подключён к 12 пину
//#define LED_ERROR 12
// Интервал печати сообщений в терминал (1000 = 1 sec)
#define PRINT_DATA_TIME 1000

float samples[NUMSAMPLES];
float HCTX0, MCTX1, MTTX2, HTTX3; // Переменные
uint32_t ul_offTime;              // Время отключения компрессора
uint32_t ul_workTime;             // Время включения компрессора
uint32_t ul_noFrostTime;          // время работы no-frost
uint32_t ul_noFrostPrevWorkT;     // последний момент времени, когда вкл no-frost
uint32_t ul_ionTimeWork;          // время работы ионизатора
uint32_t ul_ionTimePause;         // Время паузы ионизатора
uint32_t ul_newTime;              // Текущее время
uint32_t ul_deltaTime;            // Разница времени
uint32_t ul_onTime;               // Время старта программы
uint32_t ul_startTimeDelta;       // Общее время работы
uint32_t ul_printData;            // Время печати сообщений в терминале
uint32_t ul_actHeartLedTime;      // Время светодиода активности
boolean b_actLedOn;               // Светодиод активности включён
boolean b_compressorOn;           // Компрессор включен
boolean b_fanHcOn;                // Вентилятор холод. включён
boolean b_fanMcOn;                // Вентилятор мороз. включён
boolean b_noFrostOn;              // No-frost включён
boolean b_ionOn;                  // ion включён
boolean b_checkTempDT;            // Исправность датчиков
// Пищалка
//uint32_t piezoPin = 9;

void setup(){
  //Serial.begin(9600);
  // подключите AREF к 3.3 В и используйте именно этот контакт для питания ДТ,
  // так как он не так сильно "шумит"
  analogReference(EXTERNAL); // На attiny88 его нет

  if(!myOLED.begin(SSD1306_128X64))
    while(1);
 
  wdt_enable(WDTO_2S);

  pinMode(RELE1, OUTPUT);
  digitalWrite(RELE1, LOW); // При загрузке компрессор отключён
  b_compressorOn = false;
  
  pinMode(RELE2, OUTPUT);
  digitalWrite(RELE2, LOW); // При загрузке вентилятор выключен
  b_fanHcOn = false;
  
  pinMode(RELE3, OUTPUT);
  digitalWrite(RELE3, LOW); // При загрузке морозильный вентилятор выключен
  b_fanMcOn = false;
  
  pinMode(RELE4, OUTPUT);
  digitalWrite(RELE4, LOW); // При загрузке no-frost выключен !!!!! HIGH
  b_noFrostOn = false;
  
  pinMode(RELE5, OUTPUT);
  digitalWrite(RELE5, LOW); // При загрузке ion выключен
  b_ionOn = false;

  b_checkTempDT = true;
  digitalWrite(LED_ACTIVITY, LOW);
  b_actLedOn = false;

  // зуммер
  /*pinMode(piezoPin, OUTPUT);
  
  tone(piezoPin, 500);
  delay(100);
  noTone();
  delay(100);
  
  tone(piezoPin, 1000);
  delay(100);
  noTone();
  delay(100);
  
  tone(piezoPin, 2000);
  delay(100);
  noTone();*/
  
  ul_onTime = millis();  // Запоминаем время старта программы
  ul_offTime = millis(); // Запоминаем время начала работы
  ul_ionTimePause = millis();
  ul_actHeartLedTime = ul_offTime;
}

// Функция вычисления разницы времени
uint32_t deltamills(uint32_t t_old, uint32_t t_new){
  uint32_t delta;
  if (t_old <= t_new) {
    delta = t_new - t_old;
  }
  else {
    delta = (4294967295 - t_old) + t_new;
  }
  return delta;
}

// Функция моргания светодиодом активности
void heartIndication(uint32_t ul_newLedTime){
  uint32_t ul_deltaHeartTime = deltamills(ul_actHeartLedTime, ul_newLedTime);
  if (b_actLedOn){
    // Если светодиод активности включён, ждём интервала LED_ACTIVITY_ON мс
    if (ul_deltaHeartTime > LED_ACTIVITY_ON){
      digitalWrite(LED_ACTIVITY, LOW);
      b_actLedOn = false;
      ul_actHeartLedTime = ul_newLedTime;
    }
  }
  else {
    // Если светодиод активности выключен, ждём интервала LED_ACTIVITY_OFF мс
    if (ul_deltaHeartTime > LED_ACTIVITY_OFF) {
      digitalWrite(LED_ACTIVITY, HIGH);
      b_actLedOn = true;
      ul_actHeartLedTime = ul_newLedTime;
    }
  }
}

// Функция расчёта темп с датчиков NTC
float get_tx(float THERMISTORPIN){
  uint8_t i;
  float average;
  // формируем вектор из N значений с небольшой задержкой между считыванием данных
  for (i = 0; i < NUMSAMPLES; i++) {
    samples[i] = analogRead(THERMISTORPIN);
    delay(10);
  }
  // определяем среднее значение в сформированном векторе
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;
  // конвертируем значение в сопротивление
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  // конвертируем в температуру
  float steinhart;
  steinhart = average / THERMISTORNOMINAL; // (R/Ro)
  steinhart = log(steinhart); // ln(R/Ro)
  steinhart /= BCOEFFICIENT; // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart; // инвертируем
  steinhart -= 273.15; // конвертируем в градусы по Цельсию
  return steinhart;
}

boolean checkTemp() {
  HCTX0 = get_tx(HCDT) - 4;     //Считываем аналоговое значение VRT по номеру порта и Получаем температуру датчика 0...
  MCTX1 = get_tx(MCDT) - 4;
  MTTX2 = get_tx(MTDT);
  HTTX3 = get_tx(HTDT);
  if((HCTX0 < ERROR_MIN_TEMP)||(HCTX0 > ERROR_MAX_TEMP)){return false;}
  if((HTTX3 < ERROR_MIN_TEMP)||(HTTX3 > ERROR_MAX_TEMP)){return false;}
  if((MCTX1 < ERROR_MIN_TEMP)||(MCTX1 > ERROR_MAX_TEMP)){return false;}
  if((MTTX2 < ERROR_MIN_TEMP)||(MTTX2 > ERROR_MAX_TEMP)){return false;}
  return true;
}

void printData() {
   uint32_t sec = ul_startTimeDelta / 1000ul;      // полное количество секунд
   uint8_t timeHours = (sec / 3600ul);                 // часы
   uint8_t timeMins = (sec % 3600ul) / 60ul;           // минуты
   uint8_t timeSecs = (sec % 3600ul) % 60ul;           // секунды
   /*
   Serial.print("Time since start: ");
   Serial.print(timeHours);
   Serial.print(":");
   Serial.print(timeMins);
   Serial.print(":");
   Serial.println(timeSecs);

   Serial.print("Temp: HCTX0:");
   Serial.print(HCTX0);
   Serial.print(", HTTX3:");
   Serial.print(HTTX3);
   Serial.print(", MCTX1:");
   Serial.print(MCTX1);
   Serial.print(", MTTX2:");
   Serial.println(MTTX2);
   */
  // Температура в камерах
  myOLED.setFont(BigNumbers);
  myOLED.printNumI(HCTX0, LEFT, 10);
  myOLED.printNumI(MCTX1, RIGHT, 10);
 
  // Подпись и темп. на тенах
  myOLED.setFont(SmallFont);
  myOLED.print("Holod:", LEFT, 0);
  myOLED.print("Moroz:", RIGHT, 0);
  myOLED.print(String(int(HTTX3)) , LEFT, 40);
  myOLED.print(String(int(MTTX2)), RIGHT, 40);
  
  // Время работы программы
  if(timeHours<10){
    myOLED.print("0",80,55);
    myOLED.print(String(timeHours), 86, 55);
  } else {
  myOLED.print(String(timeHours), 80, 55);}
  myOLED.print(":", 92, 55);
  if(timeMins<10){
    myOLED.print("0",98,55);
    myOLED.print(String(timeMins), 104, 55);
  } else {
  myOLED.print(String(timeMins), 98, 55);}
  myOLED.print(":", 110, 55);
  if(timeSecs<10){
    myOLED.print("0",116,55);
    myOLED.print(String(timeSecs), 122, 55);
  } else {
    myOLED.print(String(timeSecs), 116, 55);
  }
  
  // индикатор работы круг
  if (b_actLedOn){
    myOLED.print("I",5,55);
    myOLED.drawCircle(8,58, 5);
  }
  // индикатор ошибки дт темп. треугольник
  if (b_checkTempDT == false) {
    myOLED.print("!",23,57);
    myOLED.drawLine(26,53,20,63);
    myOLED.drawLine(20,63,32,63);
    myOLED.drawLine(26,53,32,63);
  }
  
  myOLED.invertText(true);
  // статусы работы реле
  if(b_fanHcOn){myOLED.print("FanH", 52, 2);}
  if(b_ionOn){myOLED.print("ION ", 52, 12);}
  //if (b_doorOpen){myOLED.print("DOOR", 52, 22);}
  if(b_fanMcOn){myOLED.print("FanM", 52, 34);}
  if(b_noFrostOn){myOLED.print("NoFr", 52, 44);}
  if(b_compressorOn){myOLED.print("COMP", 52, 54);}

  // прямоугольники 
  myOLED.drawRect(50,0,77,32);
  myOLED.drawRect(50,32,77,63);
  
  myOLED.update();
  myOLED.clrScr();
}

void fanHc() {
    if (HCTX0 < HC_TEMP_FAN_OFF && b_fanHcOn) {
        digitalWrite(RELE2, LOW);   // Выключаем вентилятор 
        b_fanHcOn = false;
        //Serial.println("-----Вентилятор HC выключен-----");
    }
    if (HCTX0 > HC_TEMP_FAN_ON && !b_fanHcOn && !b_noFrostOn) {
        digitalWrite(RELE2, HIGH);   // Включаем вентилятор
        b_fanHcOn = true;
        //Serial.println("-----Вентилятор HC включён-----");
    }
}

void fanMc() {
    if (MCTX1 < MC_TEMP_FAN_OFF && b_fanMcOn) {
        digitalWrite(RELE3, LOW);   // Выключаем вентилятор 
        b_fanMcOn = false;
        //Serial.println("-----Вентилятор MC выключен-----");
    }
    if (MCTX1 > MC_TEMP_FAN_ON && !b_fanMcOn && !b_noFrostOn) {
        digitalWrite(RELE3, HIGH);   // Включаем вентилятор
        b_fanMcOn = true;
        //Serial.println("-----Вентилятор MC включён-----");
    }
}

void ion() {
    if (b_ionOn) { // Если ion включён
        ul_deltaTime = deltamills(ul_ionTimeWork, ul_newTime);
        if (ul_deltaTime > DELAY_ION_ON) {
            digitalWrite(RELE5, LOW); // Выключаем ионизатор
            b_ionOn = false;
            ul_ionTimePause = millis();
            //Serial.println("-----Ионизатор выключен-----");
        }
    } else {
        ul_deltaTime = deltamills(ul_ionTimePause, ul_newTime);
        if (ul_deltaTime > DELAY_ION_OFF) {
            digitalWrite(RELE5, HIGH); // Включаем ионизатор
            b_ionOn = true;
            ul_ionTimeWork = millis();
            //Serial.println("-----Ионизатор включен-----");
        }
    }
}

void noFrostOff() {
    // Выключаем no-frost
    ul_noFrostPrevWorkT = millis();
    digitalWrite(RELE4, LOW);
    b_noFrostOn = false;
    delay(500);
    ul_offTime = millis(); // ставим компрессор на паузу, чтоб тэн остыл.
    //Serial.println("-----NOFrost выключен-----");
}

void noFrostOn() {
    digitalWrite(RELE2, LOW);     // Отключаем верхний вентилятор
    b_fanHcOn = false;
    delay(500);
    digitalWrite(RELE3, LOW);     // Отключаем нижний вентилятор
    b_fanMcOn = false;
    delay(500);
    digitalWrite(RELE4, HIGH);      // Включаем no-frost
    b_noFrostOn = true;
    ul_noFrostTime = millis();
    //Serial.println("-----NO-Frost запущен-----");
}

void noFrost() {
    ul_deltaTime = deltamills(ul_noFrostPrevWorkT, ul_newTime);
    // если выключен
    if (!b_noFrostOn) {
        // Проверяем работоспособность датчиков температуры
        if (b_checkTempDT) {
        // запуск по времени: 8 часов
            if (ul_deltaTime > DELAY_NOFROST) {
                // Убедиться, что температура на тене ниже нуля
                if (HTTX3 < 0 || MTTX2 < 0) {
                //Serial.println("-----NF прошло 8 часов-----");
                noFrostOn();
                }
            }
            // запуск при старте программы при низкой температуре
            if (ul_startTimeDelta < DELAY_NOFROST_PWR && MTTX2 < NOFR_TEMP_POWER){
                //Serial.println("-----NF старт при низк.темп.-----");
                noFrostOn();
            }
        }
    }
    // если включён
    else {
        if (b_checkTempDT) {
            // таймер на 10 мин
            ul_deltaTime = deltamills(ul_noFrostTime, ul_newTime);
            if (ul_deltaTime > MAXTIME_NOFROST) {
                //Serial.println("-----NO-Frost 10 минут прошло-----");
                noFrostOff();
            }
            // Если температура на тене высокая
            if (HTTX3 > NOFR_TEMP_MAX && MTTX2 > NOFR_TEMP_MAX) {
                //Serial.println("-----NO-Frost высокая температура на тене-----");
                noFrostOff();
            }
        }
        else {
            noFrostOff();
        }
    }
}

void compressor() {
  // Проверяем работоспособность датчиков температуры
  if (b_checkTempDT)
  { // Температурный датчик исправен
    if (!b_compressorOn)
    { // Если компрессор выключен
      ul_deltaTime = deltamills(ul_offTime, ul_newTime); // Разница времени простоя
      if (ul_deltaTime > DELAY_COMPRESSOR 
        || (ul_deltaTime > DELAY_COMPRESSOR_PWR 
          && (HCTX0 > HCTX_TEMP_MAX || MCTX1 > MCTX_TEMP_MAX)
          )
        )
      { // Если разница времени простоя больше разрешенной
        // ИЛИ Если температура больше максимально допустимой и прошло минимум 5 мин
        digitalWrite(RELE1, HIGH); // Включаем компрессор
        b_compressorOn = true;
        ul_workTime = millis();    // Записываем время включения
        //Serial.println("-----Компрессор запущен-----");
      }
    }
    else {  // Если компрессор включен
      ul_deltaTime = deltamills(ul_workTime, ul_newTime); // Разница времени работы
      // Если температура меньше минимально допустимой или время работы больше допустимого
      if ((HCTX0 < HCTX_TEMP_MIN) || (MCTX1 < MCTX_TEMP_MIN) || (ul_deltaTime > MAXTIME_COMPRESSOR))
      {
        digitalWrite(RELE1, LOW); // Выключаем компрессор
        b_compressorOn = false;
        ul_offTime = millis();   // Запоминаем время выключения компрессора
        //Serial.println("-----Компрессор выключен-----");
      }
    }
  }
  else
  { // Температурный датчик неисправен или оборван
    if (!b_compressorOn)
    { // Если компрессор выключен при неисправном датчике
      ul_deltaTime = deltamills(ul_offTime, ul_newTime); // Разница времени простоя
      if (ul_deltaTime > ERROR_COMPRESSOR_OFF)
      { // Если разница времени простоя больше разрешенной
        digitalWrite(RELE1, HIGH); // Включаем компрессор
        b_compressorOn = true;
        ul_workTime = millis();    // Записываем время включения
        //Serial.println("-----Компрессор запущен при неисправном ДТ-----");
      }
    }
    else
    { // Если компрессор включен при неисправном датчике
      ul_deltaTime = deltamills(ul_workTime, ul_newTime);
      if (ul_deltaTime > ERROR_COMPRESSOR_ON)
      {
        digitalWrite(RELE1, LOW); // Выключаем компрессор
        b_compressorOn = false;
        ul_offTime = millis();    // Запоминаем время выключения компрессора
        //Serial.println("-----Компрессор выключен при неисправном ДТ-----");
      }
    }
  }
}

void loop()
{
  wdt_reset();                  // Обнуляем WDT (сторожевой таймер)

  ul_newTime = millis();        // Получаем текущее время

  heartIndication(ul_newTime);  // Моргаем светодиодом активности

  b_checkTempDT = checkTemp();    // Проверяем работоспособность датчиков

  ul_startTimeDelta = deltamills(ul_onTime, ul_newTime); // общее время работы

  // Print on display
  ul_deltaTime = deltamills(ul_printData, ul_newTime);
  if (ul_deltaTime > PRINT_DATA_TIME) {
    printData();
    ul_printData = millis();
  }
  // запуск верхнего вентилятора
  if (ul_startTimeDelta > DELAY_HC_FAN) {
    fanHc();
  }
    // запуск нижнего вентилятора
  if (ul_startTimeDelta > DELAY_MC_FAN) {
    fanMc();
  }
  // запуск ионизатора
  if (b_fanHcOn) {
    ion();
  }
  // запуск no-frost
  if (!b_compressorOn){
    noFrost();
  }
  // иначе запускаем компрессор
  if (!b_noFrostOn) {
    if (ul_startTimeDelta > DELAY_COMPRESSOR_PWR) {
      compressor();
    }
  }
}
