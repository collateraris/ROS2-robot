const int L1 = 3;
const int L2 = 2;
const int R1 = 4;
const int R2 = 5;

const int ENB = 6;
const int ENA = 1;

#include <SoftwareSerial.h>
#include "motor.h"
#include "move_case.h"

SoftwareSerial BTSerial(10, 11);

char bt_input;
unsigned long _time;

void setup()
{
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  digitalWrite(ENA, HIGH); 
  digitalWrite(ENB, HIGH); 

  setup_motor_system(L1,L2, R1, R2);

  _stop();

  BTSerial.begin(9600);

  pinMode(14, OUTPUT);
  digitalWrite(14, 1);

  //Serial.begin(9600);
  _time = micros();

}

void loop()
{
    if (BTSerial.available())
    {
        // Читаем команду и заносим ее в переменную.
        // char преобразует код символа команды в символ.
        bt_input = (char)BTSerial.read();

        // Отправляем команду в порт, чтобы можно было
        // их проверить в "Мониторе порта".
        // Serial.println(bt_input);

        // Вызов функции выбора действия по команде.
        move_case(bt_input);

        _time = micros();
    }

    if ((micros() - _time) > _move_time)
    {
        _stop();
    }

    if ((micros() - _time) >= 500)
    {
        _time = micros();
        move_case(bt_input);
    }
}
