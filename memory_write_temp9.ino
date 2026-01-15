#include <AT24Cxx.h>
#include <Wire.h>
#include "max6675.h"
#include "Adafruit_LiquidCrystal.h"
#include "RTClib.h"
RTC_DS1307 rtc;



#define BUTTON_PIN A6
#define PAGE_SIZE 16  // размер страницы EEPROM

Adafruit_LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Определения кнопок
#define BTN_NONE  -1
#define BTN_PLUS   0
#define BTN_OK     3
#define BTN_MINUS  5
#define BTN_RESET 10

unsigned long lastButtonTime = 0;
const unsigned long buttonDebounce = 200; // антидребезг

#define I2C_ADDRESS 0x50
AT24Cxx eep(I2C_ADDRESS, 32); // AT24C256 = 32 КБ ; AT24C32= 4 КБ

// === Настройки эксперимента ===
unsigned long RECORD_DURATION = 2UL * 60UL * 60UL * 1000UL; // длительность записи
unsigned long RECORD_INTERVAL = 1UL * 60UL * 1000UL; // интервал записи
const int VALUES_PER_WRITE = 4; // 4 датчика
const unsigned long EEPROM_SIZE = 32768UL;

// === Настройки термопар ===
int thermoDO = 2;
int thermoCLK = 3;

int thermoCS1 = A1;
int thermoCS2 = A2;
int thermoCS3 = A3;
int thermoCS4 = A0;

MAX6675 thermocouple1(thermoCLK, thermoCS1, thermoDO);
MAX6675 thermocouple2(thermoCLK, thermoCS2, thermoDO);
MAX6675 thermocouple3(thermoCLK, thermoCS3, thermoDO);
MAX6675 thermocouple4(thermoCLK, thermoCS4, thermoDO);

// === Переменные состояния ===
bool isRecording = false;
unsigned long startTime = 0;
unsigned long nextWriteTime = 0;
unsigned int eepromAddress = 0;
unsigned int totalIterations = 0;    // Общее количество итераций
unsigned int currentIteration = 0;   // Текущая итерация

unsigned long lastTimeWrite = 0;
int currectTempIndex = 0;
float temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0;
bool btnPlusWriteActive, btnMinusWriteActive;

int stateProgram = 0;
int countProgram = 5;
DateTime now;
unsigned long lastRtcRead = 0;



void setup() 
{
    Serial.begin(9600);
    delay(100); // Дайте время на установку уровней
    Wire.begin();
    lcd.begin(16, 2);

    // Инициализация RTC с проверкой
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        lcd.print("RTC Error!");
        while (1);
    }

    // Проверка, запущены ли часы
    if (!rtc.isrunning()) {
        Serial.println("RTC is NOT running!");
        lcd.print("RTC Stopped!");
        // Установка времени компиляции (опционально)
        //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    lcd.clear();

    delay(1000);
    initTemp();
    lcd.print("EfLight LLC");
    lcd.setCursor(0, 1);
    printLcdTemps();

    delay(3000);
    Serial.println("=== Готово к работе (записи температур) ===");
}

void loop() 
{

    if (millis() - lastRtcRead > 1000) {
        now = rtc.now();
        lastRtcRead = millis();
    }

    int btn = readButton();

    if (btn == BTN_PLUS) {
        if (stateProgram < countProgram) stateProgram++;
        else stateProgram = 0;
    }

    useProgram(btn);
    if (isRecording) {
        if (millis() >= nextWriteTime) {
            recordStep();
        }
        else {
            // Показываем прогресс между записями
            displayRecordingProgress();
        }
    }
    delay(100);


}

// ===============================
// === ФУНКЦИИ ===
// ===============================

void startRecording() 
{
    clearEEPROM();
    Serial.println("=== НАЧАЛО ЗАПИСИ ===");
    isRecording = true;
    startTime = millis();
    nextWriteTime = startTime;
    eepromAddress = 0;

    totalIterations = RECORD_DURATION / RECORD_INTERVAL;
    currentIteration = 0;

    Serial.print("Интервал: ");
    Serial.print(RECORD_INTERVAL / 1000);
    Serial.println(" сек");
}

void recordStep() 
{
    unsigned long currentTime = millis();

    // Проверяем, не истекло ли общее время записи
    if (currentTime - startTime >= RECORD_DURATION) {
        writeRecordMetadata();
        Serial.println("=== ЗАПИСЬ ЗАВЕРШЕНА ===");
        isRecording = false;
        return;
    }

    // Проверка переполнения EEPROM
    if (eepromAddress + VALUES_PER_WRITE >= EEPROM_SIZE) {
        Serial.println("=== Конец памяти ===");
        writeRecordMetadata();
        isRecording = false;
        return;
    }

    // Увеличиваем счетчик итераций
    currentIteration++;

    updateAllTemperaturesBeforeWrite();
    // Обновляем отображение прогресса
    lcd.clear(); lcd.setCursor(0, 0);
    printLcdTemps();
    // Записываем температуры
    Serial.print("Запись температур по адресу ");
    Serial.println(eepromAddress);

    float temps[4] = { temp1, temp2, temp3, temp4 };

    for (int i = 0; i < VALUES_PER_WRITE; i++) {
        int value = isnan(temps[i]) ? 0 : (int)temps[i];
        eep.write(eepromAddress + i, value);
        Serial.print("T");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(value);
        Serial.print("°C  ");
        delay(2);
    }

    Serial.println("\n------------------------");

    eepromAddress += VALUES_PER_WRITE;

    // **Обновляем nextWriteTime сразу после записи**
    nextWriteTime = currentTime + RECORD_INTERVAL;

    // Проверяем, не завершилась ли запись
    if (nextWriteTime - startTime >= RECORD_DURATION) {
        writeRecordMetadata();
        Serial.println("=== ЗАПИСЬ ЗАВЕРШЕНА ===");
        isRecording = false;
    }
}

void printRecordedData() 
{
    Serial.println("=== ЧТЕНИЕ ЗАПИСАННЫХ ДАННЫХ ===");

    unsigned int addr = 0;
    unsigned int dataCount = 0;

    // Читаем только до начала метаданных
    unsigned int maxDataAddr = EEPROM_SIZE - 6 - 1;

    while (addr < maxDataAddr) {
        byte values[4];
        bool allEmpty = true;

        // Читаем 4 значения за один раз
        for (int i = 0; i < 4; i++) {
            values[i] = eep.read(addr + i);
            if (values[i] != 0xFF) {
                allEmpty = false;
            }
        }

        // Проверяем на конец данных (все значения 0xFF)
        if (allEmpty) {
            bool endOfData = true;
            for (int i = 4; i <= 20; i += 4) {
                if (addr + i >= maxDataAddr) break;
                for (int j = 0; j < 4; j++) {
                    if (eep.read(addr + i + j) != 0xFF) {
                        endOfData = false;
                        break;
                    }
                }
                if (!endOfData) break;
            }

            if (endOfData) {
                Serial.println("\n=== КОНЕЦ ДАННЫХ ===");
                break;
            }
        }

        // Выводим данные только если они не пустые
        if (!allEmpty) {
            Serial.print(values[0]);
            Serial.print(" ");
            Serial.print(values[1]);
            Serial.print(" ");
            Serial.print(values[2]);
            Serial.print(" ");
            Serial.print(values[3]);
            Serial.println();
            dataCount++;
        }

        addr += 4;

        if (addr >= maxDataAddr) break;
    }

    // Теперь читаем метаданные из конца EEPROM
    unsigned int metaAddr = EEPROM_SIZE - 6;

    Serial.println("=== МЕТАДАННЫЕ ===");

    // Читаем метаданные
    byte year_end = eep.read(metaAddr);
    byte month_end = eep.read(metaAddr + 1);
    byte day_end = eep.read(metaAddr + 2);
    byte hour_end = eep.read(metaAddr + 3);
    byte interval_sec = eep.read(metaAddr + 4);
    byte duration_min = eep.read(metaAddr + 5);

    // Вывод в Serial
    Serial.print("Дата окончания: 20");
    Serial.print(year_end); Serial.print(".");
    Serial.print(month_end); Serial.print(".");
    Serial.print(day_end); Serial.print(" ");
    Serial.print(hour_end); Serial.println(":00");

    Serial.print("Интервал: "); Serial.print(interval_sec); Serial.print(" сек, ");
    Serial.print("Длительность: "); Serial.print(duration_min); Serial.println(" мин");
    Serial.println("===================");
    Serial.println("=========КОНЕЦ ЧТЕНИЯ=========");

    delay(3000);
}

void clearEEPROM() 
{
    Serial.println("=== УСКОРЕННАЯ ОЧИСТКА EEPROM ===");

    const int checkEmptyCount = 128; // сколько подряд пустых байт для остановки
    unsigned int emptyStreak = 0;

    for (unsigned int addr = 0; addr < EEPROM_SIZE; addr++) {
        byte value = eep.read(addr);
        if (value == 0xFF) {
            emptyStreak++;
            if (emptyStreak >= checkEmptyCount) {
                Serial.println("=== Конец данных найден, очистка завершена ===");
                break;
            }
        }
        else {
            eep.write(addr, 0xFF); // очищаем только занятые ячейки
            emptyStreak = 0;       // сбрасываем счетчик
        }
    }

    eepromAddress = 0;
    Serial.println("=== EEPROM ОЧИЩЕНА ===");
}

void useProgram(int btn)
{
    if (isRecording && stateProgram != 4) {
        displayRecordingProgress();
        return;
    }
    lcd.clear();
    switch (stateProgram)
    {
        case 0: {
        int h = RECORD_DURATION / (60UL * 60UL * 1000UL);//в часах
        int m = RECORD_INTERVAL / 60000UL; // минуты
        lcd.setCursor(0, 0); lcd.print("START  ");
        lcd.print(h); lcd.print("h/");
        lcd.print(m); lcd.print("m");
        lcd.setCursor(0, 1); lcd.print("Press + and -  ");
        if (btn == BTN_OK) btnPlusWriteActive = true;
        if (btn == BTN_RESET) btnMinusWriteActive = true;
        if (btnPlusWriteActive && btnMinusWriteActive)
        {
            lcd.setCursor(0, 1);
            lcd.print("Erase!!!!      ");
            startRecording();
            btnPlusWriteActive = false;
            btnMinusWriteActive = false;
        }
        break;
    }
        case 1: {
        btnPlusWriteActive = false;
        btnMinusWriteActive = false;
        lcd.setCursor(0, 0); lcd.print("Output Data  ");
        lcd.setCursor(0, 1); lcd.print("Press + or -  ");
        if (btn == BTN_OK || btn == BTN_RESET)
        {
            lcd.setCursor(0, 1);
            lcd.print("Working...     ");
            printRecordedData();
        }
        break;
    }
        case 2: {
            int h = RECORD_DURATION / (60UL * 60UL * 1000UL);//в часах

            lcd.setCursor(0, 0); lcd.print("Period write: "); lcd.print(h); lcd.print("h");
            lcd.setCursor(0, 1); lcd.print("Press + or -   ");
            if (btn == BTN_OK)
            {
                if (RECORD_DURATION < 8UL * 60UL * 60UL * 1000UL) RECORD_DURATION += 60UL * 60UL * 1000UL;
            }
            if (btn == BTN_RESET)
            {
                if (RECORD_DURATION > 60UL * 60UL * 1000UL) RECORD_DURATION -= 60UL * 60UL * 1000UL;
            }
            break;
        }
        case 3: {
            int m = RECORD_INTERVAL / 60000UL; // минуты

            lcd.setCursor(0, 0); lcd.print("interval: "); lcd.print(m); lcd.print("m");
            lcd.setCursor(0, 1); lcd.print("Press + or -   ");
            if (btn == BTN_OK)
            {
                if (RECORD_INTERVAL < 15UL * 60UL * 1000UL) RECORD_INTERVAL += 60UL * 1000UL;
            }
            if (btn == BTN_RESET)
            {
                if (RECORD_INTERVAL > 60UL * 1000UL) RECORD_INTERVAL -= 60UL * 1000UL;
            }
            break;
        }
        case 4: {
            // === НАСТРОЙКА ДАТЫ/ВРЕМЕНИ ===
            static bool timeEditMode = false;   // в режиме редактирования или нет
            static int fieldIndex = 0;          // 0=год, 1=мес, 2=день, 3=час, 4=мин, 5=сек
            static unsigned long lastChange = 0;
            const unsigned long debounce = 200;

            // текущее время
            int year1307 = now.year();
            int month1307 = now.month();
            int day1307 = now.day();
            int hour1307 = now.hour();
            int min1307 = now.minute();
            int sec1307 = now.second();

            // === ВХОД / ВЫХОД В РЕЖИМ ===
            if (!timeEditMode && (btn == BTN_OK || btn == BTN_RESET) && millis() - lastChange > debounce) {
                timeEditMode = true;
                fieldIndex = 0; // начинаем с года
                lcd.clear();
                lastChange = millis();
            }

            // === ЕСЛИ В РЕЖИМЕ РЕДАКТИРОВАНИЯ ===
            if (timeEditMode) {
                // смена поля (по кнопке PLUS)
                if (btn == BTN_PLUS && millis() - lastChange > debounce) {
                    fieldIndex++;
                    if (fieldIndex > 5) { // прошли все поля — выходим
                        timeEditMode = false;
                        lcd.clear();
                    }
                    lastChange = millis();
                }

                // изменение значения (OK / RESET)
                if (millis() - lastChange > debounce) {
                    if (btn == BTN_OK) {
                        switch (fieldIndex) {
                        case 0: year1307++; break;
                        case 1: month1307++; break;
                        case 2: day1307++; break;
                        case 3: hour1307++; break;
                        case 4: min1307++; break;
                        case 5: sec1307++; break;
                        }
                        lastChange = millis();
                    }
                    if (btn == BTN_RESET) {
                        switch (fieldIndex) {
                        case 0: year1307--; break;
                        case 1: month1307--; break;
                        case 2: day1307--; break;
                        case 3: hour1307--; break;
                        case 4: min1307--; break;
                        case 5: sec1307--; break;
                        }
                        lastChange = millis();
                    }
                }

                // нормализация значений
                auto daysInMonth = [](int y, int m)->int {
                    const int md[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
                    int d = md[(m - 1) % 12];
                    if (m == 2 && ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0))) d = 29;
                    return d;
                };

                if (year1307 < 2000) year1307 = 2000;
                if (year1307 > 2099) year1307 = 2099;

                if (month1307 < 1) month1307 = 12;
                if (month1307 > 12) month1307 = 1;

                int maxDay = daysInMonth(year1307, month1307);
                if (day1307 < 1) day1307 = maxDay;
                if (day1307 > maxDay) day1307 = 1;

                if (hour1307 < 0) hour1307 = 23;
                if (hour1307 > 23) hour1307 = 0;

                if (min1307 < 0) min1307 = 59;
                if (min1307 > 59) min1307 = 0;

                if (sec1307 < 0) sec1307 = 59;
                if (sec1307 > 59) sec1307 = 0;

                // применяем изменения
                rtc.adjust(DateTime(year1307, month1307, day1307, hour1307, min1307, sec1307));
                now = rtc.now();

                // === ВЫВОД НА LCD ===
                lcd.setCursor(0, 0);
                switch (fieldIndex) {
                case 0: lcd.print("Change YEAR   "); break;
                case 1: lcd.print("Change MONTH  "); break;
                case 2: lcd.print("Change DAY    "); break;
                case 3: lcd.print("Change HOUR   "); break;
                case 4: lcd.print("Change MINUTE "); break;
                case 5: lcd.print("Change SECOND "); break;
                }

                lcd.setCursor(0, 1);
                char buf[17];
                switch (fieldIndex) {
                case 0: snprintf(buf, sizeof(buf), "%04d", now.year()); break;
                case 1: snprintf(buf, sizeof(buf), "%02d", now.month()); break;
                case 2: snprintf(buf, sizeof(buf), "%02d", now.day()); break;
                case 3: snprintf(buf, sizeof(buf), "%02d", now.hour()); break;
                case 4: snprintf(buf, sizeof(buf), "%02d", now.minute()); break;
                case 5: snprintf(buf, sizeof(buf), "%02d", now.second()); break;
                }
                lcd.print(buf);
            }

            // === ЕСЛИ НЕ В РЕЖИМЕ РЕДАКТИРОВАНИЯ ===
            else {
                lcd.setCursor(0, 0);
                lcd.print("Set clock      ");
                lcd.setCursor(0, 1);
                lcd.print("Press + or -   ");
            }

            break;
        }
        case 5: {
        int hour1307 = now.hour();
        int min1307 = now.minute();
        int sec1307 = now.second();

        String h13 = String(hour1307); if (hour1307 < 10) { h13 = "0" + h13; }
        String m13 = String(min1307); if (min1307 < 10) { m13 = "0" + m13; }
        String s13 = String(sec1307); if (sec1307 < 10) { s13 = "0" + s13; }

        lcd.setCursor(0, 0); lcd.print("Time         ");
        lcd.setCursor(0, 1); // место, ряд
        lcd.print(h13);lcd.print(":"); lcd.print(m13); lcd.print(":"); lcd.print(s13);
        break;
    }
    }
}

void writeRecordMetadata() 
{
    DateTime t = rtc.now();

    // Записываем метаданные в КОНЕЦ EEPROM (отдельно от данных)
    unsigned int metaAddr = EEPROM_SIZE - 6; // последние 6 байтов

    Serial.println("=== ЗАПИСЬ МЕТАДАННЫХ ===");

    // Сохраняем время (год, месяц, день, час) - 4 байта
    eep.write(metaAddr, (byte)(t.year() - 2000));
    eep.write(metaAddr + 1, (byte)t.month());
    eep.write(metaAddr + 2, (byte)t.day());
    eep.write(metaAddr + 3, (byte)t.hour());

    // Сохраняем параметры записи - 2 байта
    byte interval_sec = RECORD_INTERVAL / 1000UL;
    byte duration_min = RECORD_DURATION / (60UL * 1000UL);

    eep.write(metaAddr + 4, interval_sec);
    eep.write(metaAddr + 5, duration_min);

    Serial.print("Дата окончания: 20");
    Serial.print(t.year() - 2000); Serial.print(".");
    Serial.print(t.month()); Serial.print(".");
    Serial.print(t.day()); Serial.print(" ");
    Serial.print(t.hour()); Serial.println(":00");

    Serial.print("Интервал: "); Serial.print(interval_sec); Serial.print(" сек, ");
    Serial.print("Длительность: "); Serial.print(duration_min); Serial.println(" мин");
    Serial.println("===========================");

    lcd.setCursor(0, 1);
    lcd.print("end");
}

int readButton() 
{
    int val = analogRead(BUTTON_PIN);
    if (millis() - lastButtonTime < buttonDebounce) return BTN_NONE;
    lastButtonTime = millis();

    if (val > 900) return BTN_NONE;
    else if (val > 700) return BTN_MINUS;
    else if (val > 500) return BTN_OK;
    else if (val > 300) return BTN_PLUS;
    else return BTN_RESET;
}

void displayRecordingProgress() 
{
    lcd.clear();
    lcd.setCursor(0, 0);

    // Читаем текущие температуры
    UpdateTemperature();
    
    // Форматируем вывод температур (максимум 3 символа на датчик + пробел)
    printLcdTemps();

    lcd.setCursor(0, 1);
    lcd.print(currentIteration);
    lcd.print("/");
    lcd.print(totalIterations);

    // Дополнительно можно показать процент выполнения
    if (totalIterations > 0) {
        int percent = (currentIteration * 100) / totalIterations;
        lcd.print(" (");
        lcd.print(percent);
        lcd.print("%)");
    }
}

void UpdateTemperature() 
{
    if (millis() - lastTimeWrite >= 1000)
    {

        switch (currectTempIndex)
        {
        case 0: {
            temp1 = thermocouple1.readCelsius();
            lastTimeWrite = millis();
            break;
        }
        case 1: {
            temp2 = thermocouple2.readCelsius();
            lastTimeWrite = millis();
            break;
        }
        case 2: {
            temp3 = thermocouple3.readCelsius();
            lastTimeWrite = millis();
            break;
        }
        case 3: {
            temp4 = thermocouple4.readCelsius();
            lastTimeWrite = millis();
            break;
        }
        }
        if (currectTempIndex < 3)currectTempIndex++;
        else currectTempIndex = 0;
    }
}

void printLcdTemps()
{
    lcd.print(isnan(temp1) ? "ERR" : String(temp1, 0));
    lcd.print(" ");
    lcd.print(isnan(temp2) ? "ERR" : String(temp2, 0));
    lcd.print(" ");
    lcd.print(isnan(temp3) ? "ERR" : String(temp3, 0));
    lcd.print(" ");
    lcd.print(isnan(temp4) ? "ERR" : String(temp4, 0));
}

void initTemp() {
    // Даем время на стабилизацию питания датчиков
    digitalWrite(thermoCS1, HIGH);
    digitalWrite(thermoCS2, HIGH);
    digitalWrite(thermoCS3, HIGH);
    digitalWrite(thermoCS4, HIGH);
    delayMicroseconds(10);
    delay(2000);

    // Читаем с повторными попытками
    int attempts = 0;
    do {
        temp1 = thermocouple1.readCelsius();
        delay(250);
        temp2 = thermocouple2.readCelsius();
        delay(250);
        temp3 = thermocouple3.readCelsius();
        delay(250);
        temp4 = thermocouple4.readCelsius();
        delay(250);
        attempts++;
    } while ((isnan(temp1) || isnan(temp2) || isnan(temp3) || isnan(temp4)) && attempts < 3);
}

void updateAllTemperaturesBeforeWrite()
{
    temp1 = thermocouple1.readCelsius();
    delay(250);
    temp2 = thermocouple2.readCelsius();
    delay(250);
    temp3 = thermocouple3.readCelsius();
    delay(250);
    temp4 = thermocouple4.readCelsius();

    lastTimeWrite = millis();
    currectTempIndex = 0;
}
