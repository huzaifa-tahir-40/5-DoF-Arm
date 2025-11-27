#include <ESP32Servo.h>
#include <math.h>
const float pi = 3.14159265;

// Initialize Revolute Joints
Servo F0;
Servo F1;
Servo F2;
Servo F3;
Servo F4;

// Joint Pins
int P0 = 23;
int P1 = 22;
int P2 = 21;
int P3 = 19;

// Number of Links, Lengths, and Joints
const int n = 4;
float L1 = 0; float L2 = 10; float L3 = 10;
float q1; float q2; float q3;

// Initialize Robot Parameters
// Link Twist
float alpha[n] = {0, pi/2, 0, 0};
// Link Length
float a[n] = {0, 0, L2, L3};
// Offset Distance
float d[n] = {0, L1, 0, 0};
// Joint Angles
float theta[n] = {q1, q2, q3, 0};


// Joint Angles
struct IKResult {
  long double q1;
  long double q2;
  long double q3;
};

// Inverse Kinematics
IKResult ik_hw(float x, float y, float z, float L1, float L2, float L3)
{
    IKResult result;

    // radial distance
    long double r = sqrt((long double)x*x + (long double)y*y);

    // compute cos(q3)
    long double cos3 = (r*r + (z - L1)*(z - L1) - L2*L2 - L3*L3) / (2.0L * L2 * L3);

    // clamp to avoid domain error
    if (cos3 > 1.0L)  cos3 = 1.0L;
    if (cos3 < -1.0L) cos3 = -1.0L;

    // compute sin(q3) safely
    long double sin3 = sqrtl(1.0L - cos3*cos3);

    // joint 3
    long double q3 = atan2l(sin3, cos3);

    // joint 1
    long double q1 = atan2l(y, x);

    // helper angle
    long double psi = atan2l(z - L1, r);

    // joint 2
    long double q2 = psi - atan2l(L3 * sin3, L2 + L3 * cos3);

    // --- keep inside 0–180 range ---
    if (q1 < 0) q1 += 360;        // bring to 0–360
    if (q1 > 180) q1 = 180;       // servo limit

    if (q2 < 0) q2 = 0;
    if (q2 > 180) q2 = 180;

    if (q3 < 0) q3 = 0;
    if (q3 > 180) q3 = 180;

    // assign
    result.q1 = q1;
    result.q2 = q2;
    result.q3 = q3;

    return result;
}

// rad2deg() Function
long double rad2deg(float angle)
{
  return angle * (180 / pi);
}

// Initial Position
float x = 0;
float y = 0;
float z = 15;
IKResult target;

// Linspace
/*
float linspace(float start, float end, int i) 
{
  float step = (end - start) / (num - 1);
  return start + step * i;
}
*/

// Void Setup
void setup() {
  // Make DH Table
  // buildDH();
  // Attach Servos
  F0.attach(P0);
  F1.attach(P1);
  F2.attach(P2);
  F3.attach(P3);
  // F4.attach(P4);
  // Serial Setup
  Serial.begin(115200);
  delay(1000); 
}

void loop() {

  Serial.println("Enter Coordinates:");

  Serial.println("Enter Coordinates separated by spaces (x y z):");
  
  // Wait for user input
  while (Serial.available() == 0) { delay(10); }

  // Read 3 floats from a single line
  x = Serial.parseFloat();
  y = Serial.parseFloat();
  z = Serial.parseFloat();

  // Clear any remaining input
  while (Serial.available()) Serial.read();

  Serial.print("Moving to: ");
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.println(z);

  float reach = x*x + y*y + z*z;

  if(reach > ((L2 + L3)*(L2 + L3)))
  {
    Serial.println("Error: Can't Reach This Place");
    x = 0; y = 0; z = 0;
    return;
  }

  target = ik_hw(x, y, z, L1, L2, L3);
  q1 = round(rad2deg(target.q1));
  q2 = round(rad2deg(target.q2));
  q3 = round(rad2deg(target.q3));

  Serial.println("Joint Angles: ");
  Serial.print("q1 = "); Serial.println(q1);
  Serial.print("q2 = "); Serial.println(q2);
  Serial.print("q3 = "); Serial.println(q3);

  F0.write(q1);
  F1.write(q2);
  F2.write(q3);

  char decide = false;
  Serial.print("Pick Or Drop (1 for Pick, 0 for Drop) : ");
  while (Serial.available() == 0) { delay(10); }
  decide = Serial.read();

  if (decide == '1')
  {
      close_jaw();
  }
  else if (decide == '0')
  {
      open_jaw();
  }

}

// At some position object
// At target position place

void close_jaw()
{
  F3.write(70);
}

void open_jaw()
{
  F3.write(10);
}
/*
void fk_spatial(int q1, int q2, int q3)
{

}

void make_workspace()
{
  for(int i=0; i<80; i++)
    int q1 = linspace(0, 180, i);
  for(int i=0; i<80; i++)
    int q2 = linspace(0, 180, i);
  for(int i=0; i<80; i++)
    int q3 = linspace(0, 180, i);

}*/