
/*
внес изменения в библиотеку
ESP32-RGCH3_V1\.pio\libdeps\esp32c3_supermini\Adafruit ST7735 and ST7789 Library\Adafruit_ST7735.cpp
    //_xstart = _colstart;
    _xstart = _colstart + 2;
*/

#include <Arduino.h>
#include "si5351.h"
#include "Wire.h"
#include <Adafruit_GFX.h>         // Core graphics library
#include <Adafruit_ST7735.h>      // Hardware-specific library for ST7735
#include <SPI.h>                  // Библиотека для работы с SPI-интерфейсом

#define CLK_PIN 10 // ESP32 pin GPIO25 connected to the rotary encoder's CLK pin
#define DT_PIN  20 // ESP32 pin GPIO26 connected to the rotary encoder's DT pin
#define SW_PIN  21 // ESP32 pin GPIO27 connected to the rotary encoder's SW pin

#define CLK_A 6
#define CLK_B 7
#define CLK_C 3

uint8_t pinA = 0;             // состояние контата AB
uint8_t pinC = 0;             // состояние контата BC
uint8_t blockA = false;       // блокировка прерывания на контакте AB
uint8_t blockC = false;       // блокировка прерывания на контакте BC
uint8_t encoderResult = 0;    // Результирующая переменная
int16_t encoderCount = 0;     // Счетчик щелчков энкодера
int16_t oldencoderCount = 0;  // Счетчик старое значение

int16_t _encoderCount = 0; 

byte _select_chanel = 0;

uint32_t REQ_MAX = 100000000; // 100 MHz

byte     stepIdx      = 0;           // текущий индекс шага
uint32_t freg_select = 4000;

uint32_t freg_chanel1 = 4000;
uint32_t freg_chanel2 = 4000;
uint32_t freg_chanel3 = 4000;

                                                     
const uint32_t stepTable[] = { 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000  };         // единицы, сотни, тысячи
const byte STEP_CNT = sizeof(stepTable) / sizeof(stepTable[0]);


#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF


// Определение пинов для подключения дисплея
#define TFT_CS    5         // Пин выбора чипа
#define TFT_RST   0         // Пин сброса
#define TFT_DC    1         // Пин выбора данных/команды
#define TFT_SCLK  2         // Пин тактового сигнала
#define TFT_MOSI  4         // Пин передачи данных

// Инициализация объекта дисплея с указанием используемых пинов
//Adafruit_ST7735(int8_t cs, int8_t dc, int8_t mosi, int8_t sclk, int8_t rst);
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

Si5351 si5351;

#define timeThreshold 250               // в течении времени кнопку не опрашиваем
#define buttonPin 1                     // GPIO к которому подключена кнопка
volatile boolean buttonClick = false;   // Кнопку нажали
volatile boolean buttonClick1 = false;   // Кнопку нажали 1
volatile boolean buttonClick2 = false;   // Кнопку нажали 2
volatile boolean buttonClick3 = false;   // Кнопку нажали 3
long startTime = 0;                     // Запоминаем время нажатия
long startTime1 = 0;   
long startTime2 = 0;   
long startTime3 = 0;   

void IRAM_ATTR debounceButton()  // Прерывание если пин с HIGH на LOW
{
  if (millis() - startTime > timeThreshold)  // игнор если времени прошло мало
  {
    buttonClick = true;
    startTime = millis();
  }
}

void IRAM_ATTR debounceButtonA()  // Прерывание если пин с HIGH на LOW
{
  if (millis() - startTime1 > timeThreshold)  // игнор если времени прошло мало
  {
    buttonClick1 = true;
    startTime1 = millis();
  }
}

void IRAM_ATTR debounceButtonB()  // Прерывание если пин с HIGH на LOW
{
  if (millis() - startTime2 > timeThreshold)  // игнор если времени прошло мало
  {
    buttonClick2 = true;
    startTime2 = millis();
  }
}

void IRAM_ATTR debounceButtonC()  // Прерывание если пин с HIGH на LOW
{
  if (millis() - startTime3 > timeThreshold)  // игнор если времени прошло мало
  {
    buttonClick3 = true;
    startTime3 = millis();
  }
}

void encoderFilter () {
  encoderResult = encoderResult & B00001111;
  if (encoderResult == B1011) encoderCount ++;  // по часовой стрелке
  if (encoderResult == B0111) encoderCount --;  // против часосовой стрелки
  }

void  IRAM_ATTR pinACHANGE () {          // Изменилось состояние контакта АB
  if (blockA == true) return;
  pinA = !digitalRead(DT_PIN);
  pinC = !digitalRead(SW_PIN);  
  encoderResult  <<= 1;
  bitWrite(encoderResult, 0, pinA);
  encoderResult  <<= 1;
  bitWrite(encoderResult, 0, pinC);
  encoderFilter();  
  if (!pinA && !pinC) blockA = false; else blockA = true; 
  blockC = false;
}

void  IRAM_ATTR  pinCCHANGE () {          // Изменилось состояние контакта BC
  if (blockC == true) return;
  pinA = !digitalRead(DT_PIN);
  pinC = !digitalRead(SW_PIN);  
  encoderResult  <<= 1;
  bitWrite(encoderResult, 0, pinA);
  encoderResult  <<= 1;
  bitWrite(encoderResult, 0, pinC);
  encoderFilter(); 
  if (!pinA && !pinC) blockC = false; else blockC = true; 
  blockA = false;
}


// Преобразование 0xRRGGBB → 16-битный RGB 5-6-5
uint16_t RGB24to565(uint32_t rgb24)
{
    uint8_t  r = (rgb24 >> 16) & 0xFF;   // 8-битный красный
    uint8_t  g = (rgb24 >>  8) & 0xFF;   // 8-битный зелёный
    uint8_t  b =  rgb24        & 0xFF;   // 8-битный синий

    uint16_t r5 = r >> 3;                // 5 бит
    uint16_t g6 = g >> 2;                // 6 бит
    uint16_t b5 = b >> 3;                // 5 бит

    return (r5 << 11) | (g6 << 5) | b5;  // итоговый 0bRRRRRGGGGGGBBBBB
}

String formatWithDots(uint32_t value)
{
  /* 4294967295 → максимум 10 цифр.
     + 3 точки → 13 символов
     + '\0'    → 14, круглым берём 16. */
  char buf[16];
  uint8_t len        = 0;  // текущая длина строки в buf
  uint8_t digitBlock = 0;  // счётчик цифр в текущем блоке

  /* Записываем число задом-наперёд,
     вставляя точку после каждых трёх цифр. */
  do {
    if (digitBlock == 3) {
      buf[len++] = '.';
      digitBlock = 0;
    }
    buf[len++]  = '0' + (value % 10);
    value      /= 10;
    ++digitBlock;
  } while (value);

  /* Разворачиваем строку, чтобы получить правильный порядок. */
  for (uint8_t i = 0; i < len / 2; ++i) {
    char tmp          = buf[i];
    buf[i]            = buf[len - 1 - i];
    buf[len - 1 - i]  = tmp;
  }
  buf[len] = '\0';

  return String(buf);
}

String formatFreq(uint32_t hz)
{
    // До 1 кГц показываем целое значение в герцах
    if (hz < 1000UL)
        return String(hz) + " Hz";

    // От 1 кГц до 1 МГц — килогерцы с трёмя знаками после точки
    if (hz < 1000000UL) {
        float khz = hz / 1000.0f;
        return String(khz, 3) + " kHz";     // 45.000 kHz
    }

    // Выше 1 МГц — мегагерцы
    float mhz = hz / 1000000.0f;
    return String(mhz, 3) + " MHz";         // 1.234 MHz
}

void view_title(){
  uint16_t color_out = RGB24to565(0x1da5ff); // из #1da5ffff
  tft.fillRoundRect(2, 1, 123, 17, 3, color_out); 

  tft.setTextSize(1);
  tft.setTextColor(ST7735_BLACK);
  tft.setCursor(7, 6); 
  tft.print("FGCH3, VER:1");
}


void view_step(){
  uint16_t color_out = RGB24to565(0x1da5ff); // из #1da5ffff
  tft.drawRoundRect(2, 22, 123, 17, 3, color_out); 
}

void view_step_change(){
  
  tft.fillRoundRect(4, 24, 120, 14, 3, BLACK );
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(7, 27); 
  String formatted_stepTable = formatWithDots(stepTable[stepIdx]);
  tft.print("STEP: " + formatted_stepTable);
}

void view_panel_select(){
  uint16_t color_out = RGB24to565(0x00df82); // из #00df82ff

  tft.drawRoundRect(2, 43, 123, 44, 3, color_out); 
  tft.fillRoundRect(2, 43, 123, 17, 3, color_out );

  tft.setTextSize(1);
  tft.setTextColor(ST7735_BLACK);
  tft.setCursor(7, 47); 
  tft.print("FREQUENCY SELECT");
}

void view_select_freq(){

  tft.fillRoundRect(3, 69, 120, 16, 3, BLACK );

  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(10, 70); 
  String formatted_freg_select = formatWithDots(freg_select);
  tft.print(formatted_freg_select + " Hz");
}

void frequency_panel(){
  tft.drawRoundRect(2, 91, 123, 44, 3, WHITE); 
  tft.fillRoundRect(2, 91, 123, 17, 3, WHITE );

  tft.setTextSize(1);
  tft.setTextColor(ST7735_BLACK);
  tft.setCursor(7, 96); 
  tft.print("FREQUENCY");
}

void frequency_view(byte select_ch = 0){
  tft.fillRoundRect(3, 117, 119, 16, 0, BLACK );

  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(10, 118); 

  if(select_ch==0){
    String formatted_freg_chanel1 = formatWithDots(freg_chanel1);
    tft.print(formatted_freg_chanel1 + " Hz");
  }
  else if(select_ch==1){
    String formatted_freg_chanel2 = formatWithDots(freg_chanel2);
    tft.print(formatted_freg_chanel2 + " Hz");
  }
  else if(select_ch==2){
    String formatted_freg_chanel3 = formatWithDots(freg_chanel3);
    tft.print(formatted_freg_chanel3 + " Hz");
  }
}

void select_chanel(byte select_ch = 0){

  tft.fillRect(0, 139, 160, 40, BLACK); 

  tft.setTextSize(1);
  uint16_t color_out = RGB24to565(0xdf0025); // из #df0025ff

  if(select_ch==0){
    
    tft.fillRoundRect(2, 140, 39, 17, 3, color_out );
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(14, 145); 
    tft.print("CH1");

    tft.drawRoundRect(44, 140, 39, 17, 3, WHITE );
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(55, 145); 
    tft.print("CH2");

    tft.drawRoundRect(86, 140, 39, 17, 3, WHITE );
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(97, 145); 
    tft.print("CH3");
  } 
  else if(select_ch==1){
    tft.drawRoundRect(2, 140, 39, 17, 3, WHITE );
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(14, 145); 
    tft.print("CH1");

    tft.fillRoundRect(44, 140, 39, 17, 3, color_out );
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(55, 145); 
    tft.print("CH2");

    tft.drawRoundRect(86, 140, 39, 17, 3, WHITE );
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(97, 145); 
    tft.print("CH3");
  }
  else if(select_ch==2){
    tft.drawRoundRect(2, 140, 39, 17, 3, WHITE );
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(14, 145); 
    tft.print("CH1");

    tft.drawRoundRect(44, 140, 39, 17, 3, WHITE );
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(55, 145); 
    tft.print("CH2");

    tft.fillRoundRect(86, 140, 39, 17, 3, color_out );
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(97, 145); 
    tft.print("CH3");
  }

}

void setup() {

  Serial.begin(115200);

  pinMode(CLK_A, INPUT_PULLUP); // but 1
  pinMode(CLK_B, INPUT_PULLUP); // but 2
  pinMode(CLK_C, INPUT_PULLUP); // but 3

  attachInterrupt(digitalPinToInterrupt(CLK_A), debounceButtonA, FALLING);
  attachInterrupt(digitalPinToInterrupt(CLK_B), debounceButtonB, FALLING);
  attachInterrupt(digitalPinToInterrupt(CLK_C), debounceButtonC, FALLING);

  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(SW_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(CLK_PIN), debounceButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), pinACHANGE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SW_PIN), pinCCHANGE, CHANGE);

  // ИНИЦИАЛИЗАЦИЯ ДИСПЛЕЯ
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setRotation(0);             // Установка ориентации дисплея (3 = 270 градусов)
  tft.fillScreen(ST77XX_BLACK);   // Очистка экрана черным цветом


  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);

  si5351.set_freq(freg_chanel1 * 100UL, SI5351_CLK2);
  si5351.set_freq(freg_chanel2 * 100UL, SI5351_CLK1);
  si5351.set_freq(freg_chanel3 * 100UL, SI5351_CLK0);

  si5351.update_status();
  delay(500);

  view_title();
  view_step();
  view_step_change();
  view_panel_select();
  view_select_freq();
  select_chanel();
  frequency_panel();
  frequency_view();

}


void loop() {
  

  if (buttonClick) {
    buttonClick = false;

    Serial.println("buttonClick");
    if(_select_chanel==0){
      freg_chanel1 = freg_select;
      si5351.set_freq(freg_chanel1 * 100UL, SI5351_CLK2);
    }
    else if(_select_chanel==1){
      freg_chanel2 = freg_select;
      si5351.set_freq(freg_chanel2 * 100UL, SI5351_CLK1);
    }
    else if(_select_chanel==2){
      freg_chanel3 = freg_select;
      si5351.set_freq(freg_chanel3 * 100UL, SI5351_CLK0);
    }
    Serial.println("select_chanel > " + String(_select_chanel) + " freg_select > " + String(freg_select) );
    frequency_view(_select_chanel);


  }

  if (buttonClick1) {
    buttonClick1 = false;
    Serial.println("Resset Select");
    freg_select = stepTable[stepIdx]; 
    view_select_freq();
  }

  if (buttonClick2) {
    buttonClick2 = false;
    stepIdx = (stepIdx + 1) % STEP_CNT;           // 1 → 100 → 1000
    view_step_change();
    Serial.println("Step >" +  String(stepTable[stepIdx]));
  }

  if (buttonClick3) {
    buttonClick3 = false;
    if(_select_chanel<2){
      _select_chanel++;
    } else {
      _select_chanel=0;
    }
    select_chanel(_select_chanel);
    frequency_view(_select_chanel);

    Serial.println("Select > " + String(_select_chanel));

  }


  if (oldencoderCount != encoderCount) {
    oldencoderCount = encoderCount;
    if(_encoderCount != encoderCount) {

      if(_encoderCount > encoderCount){
        Serial.println("-");
        if(freg_select > 4000){
          freg_select = freg_select - (int32_t)stepTable[stepIdx];
        } else {
          freg_select = 4000;
        }
        
      } else {
        Serial.println("+");
        freg_select = freg_select + (int32_t)stepTable[stepIdx];
      }

      if(freg_select > (REQ_MAX + 1)) { 
        freg_select = stepTable[stepIdx]; 
      } 
      if(freg_select < 4000) {
        freg_select = 4000;
      }
    
      view_select_freq();
      Serial.println(freg_select);
      _encoderCount = encoderCount;
    }

  }


}