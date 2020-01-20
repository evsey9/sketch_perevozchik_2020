#include <Wire.h>
#include <Stepper.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>
#define FREQUENCY 50
#define MIN_PULSE_WIDTH 250
#define MAX_PULSE_WIDTH 2350
 
#define SERVO_SENSORS 2 // сервопривод вывода датчиков в боевую готовность
#define SERVO_PULL 1 // сервопривод затягивания кубиков
#define SERVO_PUSH 0 // сервопривод выталкивания кубиков
 
const byte stepLeftPin1 = 6;
const byte stepLeftPin2 = 7;
const byte stepLeftPin3 = 4;
const byte stepLeftPin4 = 5;
 
const byte stepRightPin1 = 8;
const byte stepRightPin2 = 9;
const byte stepRightPin3 = 10;
const byte stepRightPin4 = 11;
 
enum cube {
  BIGBLACK, // большой чёрный куб
  SMALLBLACK, // маленький чёрный куб
  BIGWHITE, // большой белый куб
  SMALLWHITE // маленький белый куб
};
// важно: НУЖНО ИЗМЕНИТЬ
const cube collectOrder[2] = {SMALLBLACK, BIGBLACK}; // массив порядка сбора кубиков ( определяется жеребьёвкой )
 
int lap = 0; // текущий круг
 
byte cubeMax = 2; // сколько разных типов кубиков надо будет собрать. НЕ ТРОГАТЬ
 
byte cubeCounts[4] = {0, 0, 0, 0}; // массив кол-ва кубиков на трассе
 
byte curCollect = 0;
 
 
const int stepForRotation = 200; // количество шагов для полного оборота колеса
const int rotationMm = 225; // количество миллиметров проедет при полном обороте колеса
 
const int stepBetweenWritingLine = 5; // количество шагов между считываниями линии
const int stepFor90Rotation = 80;//125; // количество шагов для поворота 90 градусов
const int stepFor90RotationCube = 80;//125; // количество шагов для поворота 90 градусов когда есть кубики
const int blackLineLimit = 500; // лимит после которого считаем что линия черная
 
const int blackCubeLimit = 512; // лимит после которого считаем что кубик черный!!!!!
 
const int detectCubeLimit = 70; // значение с детектора кубика при котором считаем что кубик найден
 
const int smallCubeLimit = 55; // порог значения с детектора кубика, после которого считаем, что кубик маленький
 
const int delayFor90Rotatiuon = 500; // задержка перед и после поворота на 90
 
const byte stepBetweenSensorAndMotor = 80;//90; // шагов от датчиков линии до двигателей
 
byte countBlackCube = 0; // количество черных
byte countWhiteCube = 0; // количество белых
 
byte countPulledCube = 0; // количество затянутых кубиков
 
byte turns = 2; // количество прямых поворотов
unsigned long startMillisFromDetecting = 0;
 
int millisDelayDetecting = 50; // миллисекунд задержки определения кубика
 
const int stepperspeed = 70; // скорость шаговых моторов. НЕ УВЕЛИЧИВАТЬ
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
 
VL53L0X lox;
 
Stepper rightMotor(200, stepRightPin1, stepRightPin2, stepRightPin3, stepRightPin4);
Stepper leftMotor(200, stepLeftPin1, stepLeftPin2, stepLeftPin3, stepLeftPin4);
 
void setup() {
  Wire.begin();
  //Serial.begin(9600);
  lox.setTimeout(500);
  if (!lox.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  rightMotor.setSpeed(stepperspeed);
  leftMotor.setSpeed(stepperspeed);
 
  setADPS();
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(SERVO_SENSORS, 0, pulseWidth(45)); // вывод датчиков определения кубиков в боевую готовность(чем меньше тем дальше выезжает)
  //delay(300);
  servoPullRelease(); // выдвинуть сервопривод захвата
  //delay(300);
  //servoPushRelease(); // втянуть сервопривод выталкивания
  moveFromBase(); // выехать с базы
  lox.startContinuous();
}
 
// поднятие вверх затягивающего сервопривода
void servoPullRelease()
{
  pwm.setPWM(SERVO_PULL, 0, pulseWidth(180));
}
 
// затягивание. отпускание вниз затягивающего сервопривода
void servoPullPulling()
{
  pwm.setPWM(SERVO_PULL, 0, pulseWidth(45));
}
 
// отпускание вниз выталкивающего сервопривода
void servoPushRelease()
{
  pwm.setPWM(SERVO_PUSH, 0, pulseWidth(10));
}
 
// выталкивание. поднятие вверх выталкивающего сервопривода
void servoPushPushing()
{
  pwm.setPWM(SERVO_PUSH, 0, pulseWidth(70));
}
 
// магия. Руками не трогать
void setADPS()
{
  ADCSRA |= (1 << ADPS2);                     //Биту ADPS2 присваиваем единицу - коэффициент деления 16
  ADCSRA &= ~ ((1 << ADPS1) | (1 << ADPS0));  //Битам ADPS1 и ADPS0 присваиваем нули
}
 
// переводит угол в ширину импульса (для драйвера сервоприводов)
int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.println(analog_value);
  return analog_value;
}
 
 
// поворот на 90 градусов направо
void move90Right(int steps, int forward, int afterstep = 0)
{
  for (int x = 0; x < forward; x++) { //немного поехать вперёд
    leftMotor.step(1);
    rightMotor.step(1);
  }
  delay(delayFor90Rotatiuon);
  leftMotor.setSpeed(50);
  rightMotor.setSpeed(50);
  /*for (int x = 0; x < steps; x++) { // поворот на месте
    leftMotor.step(1);
    rightMotor.step(-1);
    }*/
  for (int x = 0; x < steps; x++) { // поворот на месте
    leftMotor.step(1);
    rightMotor.step(-1);
  }
  while (analogRead(A7) < blackLineLimit) {
    leftMotor.step(1);
    rightMotor.step(-1);
  }
  delay(100);
  for (int x = 0; x < abs(afterstep); x++) { // поворот на месте
    leftMotor.step(1 * (abs(afterstep) / afterstep));
    rightMotor.step(-1 * (abs(afterstep) / afterstep));
  }
  leftMotor.setSpeed(stepperspeed);
  rightMotor.setSpeed(stepperspeed);
 
  //delay(delayFor90Rotatiuon);
}
 
 
// поворот на 90 градусов налево
void move90Left(int steps, int forward, int afterstep = 0)
{
  //dirLeft();
  for (int x = 0; x < forward; x++) { //немного поехать вперёд
    leftMotor.step(1);
    rightMotor.step(1);
  }
  leftMotor.setSpeed(50);
  rightMotor.setSpeed(50);
  delay(delayFor90Rotatiuon);
 
  /*for (int x = 0; x < steps; x++) { // поворот на месте
    leftMotor.step(-1);
    rightMotor.step(1);
  }*/
  for (int x = 0; x < steps; x++) { // поворот на месте
    leftMotor.step(-1);
    rightMotor.step(1);
  }
  while (analogRead(A1) < blackLineLimit){
    leftMotor.step(-1);
    rightMotor.step(1);
  }
  delay(100);
  for (int x = 0; x < abs(afterstep); x++) { // поворот на месте
    leftMotor.step(-1 * (abs(afterstep) / afterstep));
    rightMotor.step(1 * (abs(afterstep) / afterstep));
  }
 
  //delay(delayFor90Rotatiuon);
 
  leftMotor.setSpeed(stepperspeed);
  rightMotor.setSpeed(stepperspeed);
}
 
// выезд из базы
void moveFromBase()
{
  for (int x = 0; x < 440; x++) { // выехать вперёд и на сколько
    leftMotor.step(1);
    rightMotor.step(1);
  }
  move90Left(stepFor90Rotation + 40, 0, 0); // повернуть налево
}
 
// затягивание кубиков
void pullingCube()
{
  //delay(250);
  for (int x = 0; x < stepBetweenWritingLine * 10; x++) { // немного подвинуться назад
    leftMotor.step(-1);
    rightMotor.step(-1);
  }
  servoPullRelease();
  delay(500);
  for (int x = 0; x < stepBetweenWritingLine * 15; x++) { // немного подвинуться вперёд
    leftMotor.step(1);
    rightMotor.step(1);
  }
  servoPullPulling(); // затянуть кубик
  delay(500);
 
}
 
// выталкивание кубиков. не используется.
void pushingCube()
{
  servoPullRelease();
  delay(1000);
  servoPushPushing();
  delay(1000);
  servoPushRelease();
  delay(1000);
 
}
 
// определение цвета кубика
void cubeDetectingColor()
{
  lox.setMeasurementTimingBudget(200000); //делаем датчик более "точным"
  int colorCube = analogRead(A6); //считываем значения датчика линии
  int distanceCube = lox.readRangeSingleMillimeters(); // переменная дистанции до кубика в миллиметрах
  if (!lox.timeoutOccurred())
    distanceCube = lox.readRangeSingleMillimeters(); // записываем значение
  else
    distanceCube = 1000;
  lox.setMeasurementTimingBudget(33000); //делаем датчик опять неточным
  lox.startContinuous();
  Serial.print("colorCube ");
  Serial.println(colorCube);
  Serial.print("distanceCube: ");
  Serial.println(distanceCube);
  if (colorCube > blackCubeLimit && distanceCube < smallCubeLimit) // большой чёрный кубик
  {
    // сейчас собираем большые черные кубики, и есть у нас место?
    if (collectOrder[curCollect] == BIGBLACK && countPulledCube < 2)
    {
      // затягиваем внутрь
      pullingCube();
      countPulledCube++;
      if (lap > 0) // если второй круг, то вычитаем из кол-ва сосчитанных кубиков
        cubeCounts[BIGBLACK]--;
    }
    else {
      if (lap == 0) // если первый круг, то считаем кубик
        cubeCounts[BIGBLACK]++;
    }
 
    Serial.println("big black cube");
  }
  else if (colorCube > blackCubeLimit && distanceCube > smallCubeLimit) // маленький чёрный кубик
  {
    // кубик черный, берем или нет?
    if (collectOrder[curCollect] == SMALLBLACK && countPulledCube < 2)
    {
      // затягиваем внутрь
      pullingCube();
      countPulledCube++;
      if (lap > 0)
        cubeCounts[SMALLBLACK]--;
    }
    else {
      if (lap == 0)
        cubeCounts[SMALLBLACK]++;
    }
 
    Serial.println("small black cube");
  }
  else if (colorCube < blackCubeLimit && distanceCube < smallCubeLimit) // большой белый кубик
  {
    // кубик черный, берем или нет?
    if (collectOrder[curCollect] == BIGWHITE && countPulledCube < 2)
    {
      // затягиваем внутрь
      pullingCube();
      countPulledCube++;
      if (lap > 0)
        cubeCounts[BIGWHITE]--;
    }
    else {
      if (lap == 0)
        cubeCounts[BIGWHITE]++;
    }
 
    Serial.println("big white cube");
  }
 
  else if (colorCube < blackCubeLimit && distanceCube > smallCubeLimit) // маленький белый кубик
  {
    // кубик черный, берем или нет?
    if (collectOrder[curCollect] == SMALLWHITE && countPulledCube < 2)
    {
      // затягиваем внутрь
      pullingCube();
      countPulledCube++;
      if (lap > 0)
        cubeCounts[SMALLWHITE]--;
    }
    else {
      if (lap == 0)
        cubeCounts[SMALLWHITE]++;
    }
 
    Serial.println("small white cube");
  }
 
}
 
// определение кубика
void cubeDetecting()
{
  if (startMillisFromDetecting == 0)
  {
    int cubeDistance; // переменная дистанции до кубика в миллиметрах
    if (!lox.timeoutOccurred())
      cubeDistance = lox.readRangeContinuousMillimeters(); // записываем значение
    else
      cubeDistance = 1000;
    Serial.print(" -- ");
    Serial.println(cubeDistance);
    // не делаем проверку какое-то время
 
    //cubeDistance = 100;
    startMillisFromDetecting = millis(); //записывем когда считали значение
    if (cubeDistance < detectCubeLimit)
    {
 
 
      Serial.println("cubeDetected ");
      delay(100);
      cubeDetectingColor(); // считываем размер и цвет кубика
      for (int x = 0; x < stepBetweenWritingLine / 3; x++) {
        leftMotor.step(1);
        rightMotor.step(1);
      }
      startMillisFromDetecting = millis(); //записываем когда считали значение ещё раз, т.к. была задержка
      startMillisFromDetecting += 500; //даём побольше времени роботу чтобы проехать
    }
  }
  else
  {
    if (millis() > startMillisFromDetecting && millis() - startMillisFromDetecting > millisDelayDetecting)
    {
      //Serial.println("qwe " + (String(startMillisFromDetecting)) + " " + String(millis()));
      startMillisFromDetecting = 0;
    }
  }
}
 
 
// движение по линии
void loop() {
  cubeDetecting(); //проверяем на кубик
 
  //return;
  int outerLeft = analogRead(A0); //записываем значения датчиков линии
  int innerLeft = analogRead(A7);
  int innerRight = analogRead(A1);
  int outerRight = analogRead(A2);
 
 
 
  //  Serial.print(outerLeft);
  //  Serial.print(" ");
  //  Serial.print(innerLeft);
  //  Serial.print(" ");
  //  Serial.print(innerRight);
  //  Serial.print(" ");
  //  Serial.println(outerRight);
  //  delay(200);
 
  if (
    innerLeft > blackLineLimit &&
    outerLeft > blackLineLimit &&
    innerRight > blackLineLimit &&
    outerRight > blackLineLimit &&
    turns == 0// О, нет! Это же перекресток!
  )
  {
    lap++; // увеличиваем переменную кол-ва пройденных кругов
    turns = 2; // сбрасываем счётчик прямых поворотов
    //Serial.println("perekrestok!!!");
    if (countPulledCube > 0) // если есть затянутые кубики
    {
      //delay(1000);
      leftMotor.setSpeed(50);
      rightMotor.setSpeed(50);
      move90Right(stepFor90Rotation, stepBetweenSensorAndMotor, 10); // поворачиваемся направо
     
      for (int x = 0; x < 380; x++) { // едем вперёд
        leftMotor.step(1);
        rightMotor.step(1);
      }
      servoPullRelease(); // сбрасываем кубики
      for (int x = 0; x < 380; x++) { // едем назад
        leftMotor.step(-1);
        rightMotor.step(-1);
      }
      move90Left(stepFor90Rotation, 0, -30); // поворачиваемся налево
      for (int x = 0; x < stepBetweenSensorAndMotor / 2; x++) { // едем вперёд
        leftMotor.step(1);
        rightMotor.step(1);
      }
 
      leftMotor.setSpeed(stepperspeed);
      rightMotor.setSpeed(stepperspeed);
 
    }
    countPulledCube = 0; // сбрасываем счётчик затянутых кубиков
    // собирали кубики и их больше не осталось
    if (cubeCounts[collectOrder[curCollect]] <= 0)
    {
      if (curCollect < cubeMax - 1) {
        curCollect += 1; // начинаем собирать следующие в порядке кубики
      }
      else {
        // тут код для конца, т.к. собрали всё, что нужно
      }
    }
    else
    {
      // продолжаем собирать
    }
    //delay(1000);
 
    // едем прямо
    for (int x = 0; x < stepBetweenWritingLine * 10; x++) {
      leftMotor.step(1);
      rightMotor.step(1);
    }
 
  }
  else
  { // Фуу.. Пронесло..
    if (innerLeft > blackLineLimit)
    {
      //      if (outerLeft > blackLineLimit) {
      //        // прямой угол левый
      //        // move90Left(); // на трассе нет левых прямых углов (на всякий случай)
      //      }
      //      else
      //      {
      //        // левая на черном
      //        for (int x = 0; x < stepBetweenWritingLine; x++) {
      //          rightMotor.step(1);
      //        }
      //      }
      // левая на черном
      for (int x = 0; x < stepBetweenWritingLine / 3; x++) {
        rightMotor.step(1);
      }
    }
    else
    {
      if (innerRight > blackLineLimit)
      {
        if (outerRight > blackLineLimit and turns > 0) {
          // прямой угол правый
          move90Right(stepFor90Rotation, stepBetweenSensorAndMotor, -20);
          turns -= 1;
        }
        else
        {
          // правая на черном
          for (int x = 0; x < stepBetweenWritingLine / 3; x++) {
            leftMotor.step(1);
          }
        }
      }
      else
      {
        // едем прямо
        for (int x = 0; x < stepBetweenWritingLine / 3; x++) {
          leftMotor.step(1);
          rightMotor.step(1);
        }
      }
    }
  }
}
