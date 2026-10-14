#ifndef HC_SR04_H
#define HC_SR04_H

#include <Arduino.h>

#define CM true
#define INCH false

class HC_SR04 {
  public:
    HC_SR04(int trigger, int echo, int interrupt, int max_dist=200);

    void begin();
    void start();
    bool isFinished(){ return _finished; }
    // Сколько мс прошло с последнего start(): по нему ловим потерянный спад ECHO
    unsigned long elapsedMs(){ return millis() - _startMs; }
    // Длительность эха в см (не больше 255); 0 — замер недостоверен
    unsigned int getRange(bool units=CM);
    static HC_SR04* instance(){ return _instance; }

  private:
    static void _echo_isr();

    int _trigger, _echo, _int, _max;
    volatile unsigned long _start, _end;
    volatile bool _finished;
    volatile bool _gotRise;
    unsigned long _startMs;
    static HC_SR04* _instance;
};

#endif
