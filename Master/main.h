#ifndef MAIN_H
#define MAIN_H

class MSD_Controller
{
public:
  MSD_Controller(int station_1, int station_2, int station_3, int station_4, int station_5, int station_6, int station_7) : station_1(station_1), station_2(station_2), station_3(station_3), station_4(station_4), station_5(station_5), station_6(station_6), station_7(station_7){};

  void begin()
  {
    pinMode(station_1, OUTPUT);
    pinMode(station_2, OUTPUT);
    pinMode(station_3, OUTPUT);
    pinMode(station_4, OUTPUT);
    pinMode(station_5, OUTPUT);
    pinMode(station_6, OUTPUT);
    pinMode(station_7, OUTPUT);
  }

  void startStation_1()
  {
    digitalWrite(station_1, HIGH);
    delay(1000);
  }

private:
  int station_1;
  int station_2;
  int station_3;
  int station_4;
  int station_5;
  int station_6;
  int station_7;
};

#endif