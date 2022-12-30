#include<ros.h>
#include<checkpoint_2/target_speed.h>
//#include<vector.h>

float left_command = 0;
float right_command = 0;

//ros
ros::NodeHandle nh;

//ros-subscriber
float supply = 2.3;
float supply_b = 5;
float pre_error_R = 0;
double acc_error_R = 0;
float pre_error = 0;
double acc_error = 0;

void messageCb(const checkpoint_2::target_speed& input_msg){
  pre_error_R = 0;
  acc_error_R = 0;
  pre_error = 0;
  acc_error = 0;
  left_command = input_msg.left_speed;
  right_command = input_msg.right_speed+supply;
}
ros::Subscriber<checkpoint_2::target_speed> sub("/target", messageCb);

//ros-publisher
checkpoint_2::target_speed output_msg;
ros::Publisher output("encoder_data", &output_msg);

//DC motor & encoder pin declare
#define EnA 5
#define EnB 6
#define In1 8
#define In2 7
#define In3 10
#define In4 11
#define Ec0A 4 //left 
#define Ec0B 2 //interrupt 0
#define Ec1A 12 //right
#define Ec1B 3 //interrupt 1

//encoder 
byte Ec0A_last;
byte Ec1A_last;
int duration_L=0;
int duration_R=0;
boolean direction_L;
boolean direction_R;

void encoderInit(){
  direction_L =  true;
  pinMode(Ec0A, INPUT);
  pinMode(Ec0B, INPUT);
  attachInterrupt(0, wheelSpeed_L, CHANGE);
  
  direction_R =  true;
  pinMode(Ec1A, INPUT);
  pinMode(Ec1B, INPUT);
  attachInterrupt(1, wheelSpeed_R, CHANGE);
}

void wheelSpeed_L(){
  int Lstate = digitalRead(Ec0A);
  if((Ec0A_last == LOW) && (Lstate == HIGH)){
    int val = digitalRead(Ec0B);
    if(val == LOW && direction_L){
      direction_L = false;
    }
    else if(val == HIGH && !direction_L){
      direction_L = true;
    }
  }
  Ec0A_last = Lstate;
  
  if(!direction_L) duration_L--;
  else duration_L++;
}

void wheelSpeed_R(){
  int Rstate = digitalRead(Ec1A);
  if((Ec1A_last == LOW) && (Rstate == HIGH)){
    int val = digitalRead(Ec1B);
    if(val == LOW && direction_R){
      direction_R = false;
    }
    else if(val == HIGH && !direction_R){
      direction_R = true;
    }
  }
  Ec1A_last = Rstate;
  
  if(!direction_R) duration_R++;
  else duration_R--;
}

void setup(){
  Serial.begin(57600);
  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  encoderInit();
  

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(output);
}

void forward(int v_left, int v_right){
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  analogWrite(EnA, v_left);
  analogWrite(EnB, v_right);
}

void freeze(){
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}

void backward(int v_left, int v_right){
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  analogWrite(EnA, v_left);
  analogWrite(EnB, v_right);
}

//pid_L
//int sat = 220; //output saturation

float pid_control(int ref_speed, int current_speed){
  float Kp = 7;
  float Ki = 0.03;
  float Kd = 0.01;
  float error = ref_speed - current_speed;
  acc_error += error*0.05;
  float output = Kp*error + Kd*(error - pre_error) + Ki*acc_error;
  pre_error = error;
  
  return output;
}
//pid_R

float pid_control_R(int ref_speed, int current_speed){
  float Kp = 7;
  float Ki = 0.05;
  float Kd = 0.01;
  float error_R = ref_speed - current_speed;
  acc_error_R += error_R*0.05;
  float output = Kp*error_R + Kd*(error_R - pre_error_R) + Ki*acc_error_R;
  pre_error_R = error_R;
  
  return output;
}

float sat(float n){
  float saturation = 200;
  if(n > saturation){
    n = saturation;
  }
  else if(n<0){
    n = 0; 
  }
  return n;
}

void activate(int current_speed_L, int current_speed_R){
 if(left_command>0 && right_command>0){
  float pwm_L = sat(left_command + pid_control(left_command, current_speed_L));
  float pwm_R = sat(right_command+ pid_control_R(right_command, current_speed_R)); 
  forward((int)(pwm_L+0.5), (int)(pwm_R+0.5));
 }
 else if(left_command==0 && (right_command-supply) ==0){
  freeze();
 } 
 else{
  float pwm_L = sat(-left_command + pid_control(-left_command, -current_speed_L));
  float pwm_R = sat(-right_command+supply_b+ pid_control_R(-right_command+supply_b, -current_speed_R)); 
   backward((int)(pwm_L+0.5), (int)(pwm_R+0.5)); 
 }
}

void publish_data(int data_L, int data_R){
  output_msg.left_speed = data_L;
  output_msg.right_speed = data_R;
  output.publish(&output_msg);
}
void loop(){
  activate(duration_L, duration_R);
  publish_data(duration_L, duration_R);
  duration_R = 0;
  duration_L = 0;
  nh.spinOnce();
  delay(50);
}

