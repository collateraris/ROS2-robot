// Переменная изменения скорости.
unsigned long _move_time;

// Выбор действий.
void move_case(char bt_input)
{
    switch (bt_input)
    {
        // Вперед.
        case 'F':
            //forward();
            backward();
            break;

        // Назад.
        case 'B':
            forward();
            //backward();
            break;

        // Влево.
        case 'L':
            left();
            break;

        // Вправо.
        case 'R':
            right();
            break;

        // Прямо и влево.
        case 'G':
            forward_left();
            break;

        // Прямо и вправо.
        case 'I':
            forward_right();
            break;

        // Назад и влево.
        case 'H':
            backward_left();
            break;

        // Назад и вправо.
        case 'J':
            backward_right();
            break;

        // Стоп.
        case 'S':
            _stop();
            break;

        // Скорость 0%.
        case '0':
            _move_time = 0;
            break;

        // Скорость 10%.
        case '1':
            _move_time = 50;
            break;

        // Скорость 20%.
        case '2':
            _move_time = 100;
            break;

        // Скорость 30%.
        case '3':
            _move_time = 150;
            break;

        // Скорость 40%.
        case '4':
            _move_time = 200;
            break;

        // Скорость 50%.
        case '5':
            _move_time = 250;
            break;

        // Скорость 60%.
        case '6':
            _move_time = 300;
            break;

        // Скорость 70%.
        case '7':
            _move_time = 350;
            break;

        // Скорость 80%.
        _move_time = 400;
        break;

        // Скорость 90%.
        case '9':
            _move_time = 450;
            break;

        // Скорость 100%.
        case 'q':
            _move_time = 500;
            break;

        case 'X':
            // switch_rejim = 1;
            break;

        case 'x':
            // switch_rejim = 0;
            break;

        case 'D':
            _stop(); // switch_rejim = 0;
            break;
    }
}