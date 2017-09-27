#ifndef Mecanum_Drive_H
#define Mecanum_Drive_H

#include <Adafruit_MotorShield.h>

#define PI_4 0.78539816339

namespace MecanumDrive 
{
  // create motor shield object
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60);
  
  // create DC motors pointers
  Adafruit_DCMotor *m1;
  Adafruit_DCMotor *m2;
  Adafruit_DCMotor *m3;
  Adafruit_DCMotor *m4;

  // voltage calculations
  double maxSpeed = 1;
  double Vmax;
  double voltage1;
  double voltage2;
  double voltage3;
  double voltage4;

  // motor speed values
  uint8_t v1;
  uint8_t v2;
  uint8_t v3;
  uint8_t v4;
  
  // set orientation angle
  double orientation = 0;

  // forward declaration of namespace functions
  void initialize();
  void drive();
  void stop();
  void forward();
  void reverse();
  void strafeRight();
  void strafeLeft();
  void rotateRight();
  void rotateLeft();
  void skidSteer( float left, float right);
  void flip();

  void initialize()
  {
    AFMS.begin();
    m1 = AFMS.getMotor(4);
    m2 = AFMS.getMotor(3);
    m3 = AFMS.getMotor(1);
    m4 = AFMS.getMotor(2);
    stop();

  }

  double degToRad(int degrees)
  {
    return degrees * PI/180;
  }

  void flip()
  {
    if (orientation == 0)
      orientation = PI;
    else
      orientation = 0;
    
  }

  void getMotorValues( const double velocity, const double radians, const double rotation) 
  {
    Vmax = 0;
    voltage1 = velocity * sin( radians + orientation + PI_4 ) + rotation;
    Vmax = Vmax > abs(voltage1) ? Vmax : abs(voltage1);
    voltage2 = velocity * cos( radians + orientation + PI_4 ) - rotation;
    Vmax = Vmax > abs(voltage2) ? Vmax : abs(voltage2);
    voltage3 = velocity * cos( radians + orientation + PI_4 ) + rotation;
    Vmax = Vmax > abs(voltage3) ? Vmax : abs(voltage3);
    voltage4 = velocity * sin( radians + orientation + PI_4 ) - rotation;
    Vmax = Vmax > abs(voltage4) ? Vmax : abs(voltage4);
  }

  void normalizeValues() 
  {
    voltage1 = voltage1/Vmax;
    voltage2 = voltage2/Vmax;
    voltage3 = voltage3/Vmax;
    voltage4 = voltage4/Vmax;
  }

  void convertTo255(const double speed) 
  {
    v1 = maxSpeed * abs(voltage1) * 255;
    v2 = maxSpeed * abs(voltage2) * 255;
    v3 = maxSpeed * abs(voltage3) * 255;
    v4 = maxSpeed * abs(voltage4) * 255;
  }
  
  void setMaxSpeed(const double speed)
  {  
    if (speed < 0)
      maxSpeed = 0;
    else if (speed > 1)
      maxSpeed = 1;
    else
      maxSpeed = speed;
  }

  void drive( double velocity, int degrees, double rotation) 
  {

    // are arguments in acceptable ranges
    if (velocity > 1) velocity = 1;
    if (velocity < -1) velocity = -1;
    if (rotation > 1) rotation = 1;
    if (rotation < -1) rotation = -1;

    // convert degrees to radians
    const double radians = degToRad(degrees);

    // calculate raw voltage values
    getMotorValues( velocity, radians, rotation);

    // normalize values
    normalizeValues();

    // convert values for motor controller
    convertTo255( abs(velocity) );

    // set motor directions and write speeds
    voltage1 < 0 ? m1->run(FORWARD) : m1->run(BACKWARD); 
    voltage2 < 0 ? m2->run(BACKWARD) : m2->run(FORWARD); 
    voltage3 < 0 ? m3->run(FORWARD) : m3->run(BACKWARD); 
    voltage4 < 0 ? m4->run(BACKWARD) : m4->run(FORWARD); 

    // release motors if value is zero
    if (voltage1 == 0) m1->run(RELEASE);
    if (voltage2 == 0) m2->run(RELEASE);
    if (voltage3 == 0) m3->run(RELEASE);
    if (voltage4 == 0) m4->run(RELEASE);

    // set motor speeds
    m1->setSpeed(v1);
    m2->setSpeed(v2);
    m3->setSpeed(v3);
    m4->setSpeed(v4);

  }
  
  void skidSteer( float left, float right)
  {
    // flip left and rigt if orientation not 0
    if (orientation)
    {
      float temp = left;
      left = right;
      right = temp;
    }
  
    // make sure that float values are in range
    if (left > 1)
      left = 1;
    if (left < -1)
      left = -1;
    if (right > 1)
      right = 1;
    if (right < -1)
      right = -1;

    // set left motor directions
    if (left == 0)
    {
      m1->run(RELEASE);
      m3->run(RELEASE);
    }
    else if (left < 0)
    {
      m1->run( orientation ? BACKWARD : FORWARD );
      m3->run( orientation ? BACKWARD : FORWARD );
    }
    else
    {
      m1->run( orientation ? FORWARD : BACKWARD );
      m3->run( orientation ? FORWARD : BACKWARD );
    }

    // set left motor directions
    if (right == 0)
    {
      m2->run(RELEASE);
      m4->run(RELEASE);
    }
    else if (right < 0)
    {
      m2->run( orientation ? FORWARD : BACKWARD );
      m4->run( orientation ? FORWARD : BACKWARD );
    }
    else
    {
      m2->run( orientation ? BACKWARD : FORWARD );
      m4->run( orientation ? BACKWARD : FORWARD );
    }

    // set motor speeds
    m1->setSpeed(abs(left)*maxSpeed*255);
    m2->setSpeed(abs(right)*maxSpeed*255);
    m3->setSpeed(abs(left)*maxSpeed*255);
    m4->setSpeed(abs(right)*maxSpeed*255);
  }
  
  void stop() { drive(0,0,0); }
  void forward() { drive(1,0,0); }
  void reverse() { drive(-1,0,0); }
  void strafeRight() { drive(1,90,0); }
  void strafeLeft() { drive(1,270,0); }
  void rotateRight() { drive(0,0,1); }
  void rotateLeft() { drive(0,0,-1); }
  
  void forward(int d) 
  { 
    drive(1,0,0);
    delay(d);
    stop();
  }

  void reverse(int d) 
  {
    drive(-1,0,0);
    delay(d);
    stop();
  }

  void strafeRight(int d) 
  { 
    drive(1,90,0);
    delay(d);
    stop();
  }

  void strafeLeft(int d) 
  { 
    drive(1,270,0);
    delay(d);
    stop();
  }

  void rotateRight(int d) 
  { 
    drive(0,0,1);
    delay(d);
    stop();
  }

  void rotateLeft(int d) 
  { 
    drive(0,0,-1);
    delay(d);
    stop();
  }

}

#endif
