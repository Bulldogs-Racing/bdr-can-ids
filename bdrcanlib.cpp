#include "MyLibrary.h"

MyLibrary::MyLibrary(int pin) {
    _pin = pin;
}

void MyLibrary::begin() {
    pinMode(_pin, OUTPUT);
}

void MyLibrary::on() {
    digitalWrite(_pin, HIGH);
}

void MyLibrary::off() {
    digitalWrite(_pin, LOW);
}
