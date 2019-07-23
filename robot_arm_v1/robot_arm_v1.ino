// Developed by Peter Sterckx
// July 2019
// petersterckx.com

#include <Servo.h>
#define pi 3.141592653589793238462643383279
#define num_of_points 30                       // maximum number of points that can be taught

float alpha;
int x[3]; 
int y_now[3];
int y_last[3];
int angle[3];
int potPins[3];
bool teach;
int tog_teach = 2;
int tog_auto = 3;
bool toggle_now;
bool toggle_last;
int teach_points[num_of_points][4];
int pos_number;
int gripper_pin = 5;
float y;
int pos_number_i;
int open_deg = 130;
int closed_deg = 20;

Servo joint[3];     
Servo gripper;      

void setup() {
  
  gripper.attach(8);
  joint[0].attach(11);
  joint[1].attach(9);
  joint[2].attach(10);

  potPins[0] = 2;
  potPins[1] = 7;
  potPins[2] = 5;

  pinMode(tog_auto,INPUT);
  pinMode(tog_teach, INPUT);

  for (int pos_number_i = 0; pos_number_i <= num_of_points; pos_number_i++) {
    for (int i = 0; i <= 2; i++) {
      teach_points[pos_number_i][i] = 0;                           // initialize teach points array
    }
    teach_points[pos_number_i][3] = open_deg;                      // default position for the gripper is open
  }

  pos_number = 0;

  for (int i = 0; i <= 2; i++) {                                   // determine initial positions of each joint
    y_last[i] = joint[i].read();
  }
 
  alpha = .2;                                                      // set the alpha value for the moving average algorithm

  Serial.begin(9600); 
  
}

void loop() {

  for (int i = 0; i <= 2; i++) {
    x[i] = analogRead(potPins[i]);                                 // read value from potentiometer
    y_now[i] = alpha * x[i] + (1 - alpha) * y_last[i];             // apply moving average algorithm
    y_last[i] = y_now[i];                                          
    angle[i] = map(y_now[i],0,1023,0,180);                         // map potentiometer reading input to an angle from 0 to 180 degrees
    joint[i].write(angle[i]);                                      // write the value to the servo to move the axis
  }

  if (digitalRead(gripper_pin) == HIGH) {                          // open or close gripper depending on toggle state
    gripper.write(closed_deg); 
  } else if (digitalRead(gripper_pin) == LOW) {
    gripper.write(open_deg);
  }
      
  if (digitalRead(tog_teach) == HIGH) {                            // teach currently position if teach button is pressed
    toggle_now = true;
  } else if (digitalRead(tog_teach) == LOW) {
    toggle_now = false;
  }
  
  if (toggle_now and !toggle_last) {

    delay(500);

    for (int i = 0; i <= 2; i++) {                                 
      teach_points[pos_number][i] = joint[i].read();               // read current servo positions and save to teach points array
    }
  
    teach_points[pos_number][3] = gripper.read();

    pos_number = pos_number + 1;
    
    if (pos_number >= num_of_points-1) {
      pos_number = 0;
    }
  }
  
  toggle_last = toggle_now;

  while (digitalRead(tog_auto) == HIGH) {                                                 // go into automatic mode if toggle is HIGH
    for (int pos_number_i = 0; pos_number_i <= num_of_points - 1; pos_number_i++) {
      for (int i = 0; i <= 2; i++) {
        if (teach_points[pos_number_i][i] != 0) {
          MoveTo(joint[i],teach_points[pos_number_i][i]);                                 // iterate through teach points array and move joints using MoveTo function
        }
      }
      MoveTo(gripper,teach_points[pos_number_i][3]);
    }     
    
    if (digitalRead(tog_auto) == LOW) {                            // if automatic mode switched off, move to position corresponding to current orientations of potentiometers
      for (int i = 0; i <= 2; i++) {
        x[i] = analogRead(potPins[i]);
        angle[i] = map(x[i],0,1023,0,180);
        MoveTo(joint[i],angle[i]);
       } 
      MoveTo(gripper,gripper.read());
      delay(1);
    }
  }
}

void MoveTo(Servo servo, int target) {                             // function that creates smooth sinusoidal motion for one axis
  int current = servo.read();
  if (target <= 180 || target >= 0) {
    if (current < target) {
      for (float angle = current; angle < target; angle += 1) {
        y = (pi/(target-current))*angle + pi*(target-2*current)/(target-current);
        servo.write(angle);
        delay(25*(.7*sin(y)+1));                                   // use changing amount of delay to create smooth motion
      }
    }else if (current > target) {
      for (float angle = current; angle > target; angle -= 1) {
        y = (pi/(current-target))*angle + pi*(current-2*target)/(current-target);
        servo.write(angle);
        delay(25*(.7*sin(y)+1));
      }
    }
  }
}

