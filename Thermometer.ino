#include <math.h>

#define DATA_PIN    13 // пин данных (англ. data)
#define LATCH_PIN   12 // пин строба (англ. latch)
#define CLOCK_PIN   11 // пин такта (англ. clock)
#define THERMISTORPIN A0
#define TERMIST_B 4300 // Параметр конкретного типа термистора (из datasheet):
#define NUMSAMPLES 5
#define TEMPFREQ 1000
#define LDRPIN A1
#define UNDERZEROLED 6
#define ABOVEZEROLED 5

volatile unsigned long int timerDispIsr, timerTempIsr = TEMPFREQ;
unsigned long int timerDisp, timerTemp;
uint8_t i;
uint8_t lightness;
uint8_t tempDigits[3];
uint8_t commonPins[3] = {10, 9, 3};
bool belowZero = false;
byte segments[10] = {
  0b01010000, 0b01011111, 0b00110001, 0b00010101, 0b00011110,
  0b10010100, 0b10010000, 0b01011101, 0b00010000, 0b00010100
};


void setup() {
  //Serial.begin(9600);

  for (i = 0; i < 3; i++)
    pinMode(commonPins[i], OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

  /**** настройка прерывания по таймеру каждую 0,001 сек (вызов функции ISR (TIMER0_COMPA_vect)) ****/
  TCCR0A |= (1 << WGM01);              //сброс при совпадении
  OCR0A = 0xF9;                        //начало отсчета до переполнения (249)
  TIMSK0 |= (1 << OCIE0A);             //разрешить прерывание при совпадении с регистром А
  TCCR0B |= (1 << CS01) | (1 << CS00); //установить делитель частоты на 64
  sei();                               //разрешить прерывания

  /* Повышаем частоту ШИМ для пинов 9,10,11,3 меняя предделитель для таймеров 1 и 2

     Константа Делитель Частота(Гц)
     0x01 1 31250
     0x02 8 3906.25
     0x03 64 488.28125
     0x04 256 122.0703125
     0x05 1024 30.517578125

     В случае изменения Т0 необходимо учитывать, что от него зависят функции millis(), delay(), итд
  */
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
  TCCR2B = TCCR2B & 0b11111000 | 0x01;
}

void loop() {
  getTemp(TEMPFREQ);
  print7seg3dig(tempDigits, 1, belowZero, 15);
}

void print7seg3dig(uint8_t segVal[3], int dotPos, bool underZero, int interval) {
  cli();
  timerDisp = timerDispIsr;
  sei();
  if (timerDisp >= interval) {
    //cli();
    timerDispIsr = 0;
    //sei();
    for (i = 0; i < 3; i++) {
      digitalWrite(LATCH_PIN, LOW);
      // задвигаем (англ. shift out) байт-маску бит за битом, начиная с младшего (англ. Least Significant Bit first)
      if ( i == dotPos)
        shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, segments[segVal[i]] - 0b00010000);
      else
        shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, segments[segVal[i]]);
      analogWrite(commonPins[i], lightness);
      // чтобы переданный байт отразился на выходах Qx сдвигового регистра, нужно подать на пин строба высокий сигнал
      digitalWrite(LATCH_PIN, HIGH);
      delay(3);
      digitalWrite(commonPins[i], 0);
    }
    if (underZero) {
      analogWrite(UNDERZEROLED, int(lightness/2));
      digitalWrite(ABOVEZEROLED, LOW);
    }
    else {
      digitalWrite(UNDERZEROLED, LOW);
      analogWrite(ABOVEZEROLED, int(lightness/2));
    }
  }
}

void getTemp(long int interval) {
  cli();
  timerTemp = timerTempIsr;
  sei();
  if (timerTemp >= interval) {
    //cli();
    timerTempIsr = 0;
    //sei();
    int average = 0;
    for (i = 0; i < NUMSAMPLES; i++) {
      average += analogRead(THERMISTORPIN);
      delay(1);
    }
    average /= NUMSAMPLES;
    lightness = map(analogRead(LDRPIN), 0, 1023, 255, 1);
    // B parameter уравнение для расчета температуры (термистор нелинеен): http://arduino-diy.com/arduino-thermistor
    int temp = int( (1. / ( 1. / (TERMIST_B) * log( 1023. / average - 1. ) + 1. / (25. + 273.) ) - 273) * 100 );
    if (temp < 0) {
      belowZero = true;
      temp *= -1;
    }
    else
      belowZero = false;
    tempDigits[0] = (temp - temp % 1000) / 1000;
    tempDigits[1] = (temp % 1000 - temp % 100) / 100;
    tempDigits[2] = temp % 100 / 10;
  }
}

ISR (TIMER0_COMPA_vect) //функция, вызываемая прерыванием таймера каждые 0,001 сек.
{
  timerTempIsr++;
  timerDispIsr++;
}
