#include<ros.h>
#include<checkpoint_3/encoderData.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Float32.h>

float left_input = 0;
float right_input = 0;

//ros
ros::NodeHandle nh;

//ros-subscriber
void messageCb(const checkpoint_3::encoderData& input_msg){
  left_input = input_msg.left_speed;
  right_input = input_msg.right_speed;
  activate(left_input, right_input);
}
ros::Subscriber<checkpoint_3::encoderData> sub("/command", messageCb);

//ros-publisher
checkpoint_3::encoderData output_msg;
std_msgs::Int32 light_msg;
std_msgs::Float32 ir_msg;
ros::Publisher output("encoder_output", &output_msg);
ros::Publisher light("light_sensor", &light_msg);
ros::Publisher ir("ir_sensor", &ir_msg);
//DC motor & encoder pin declare
#define EnA 5
#define EnB 6
#define In1 8
#define In2 7 //9 -> 7
#define In3 10
#define In4 11
#define Ec0A 4 //left 
#define Ec0B 2 //interrupt 0
#define Ec1A 12 //right
#define Ec1B 3 //interrupt 1
#define IR 9 //irsensor

#define LS A0
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
  // motor
  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  //sensor
  encoderInit();
  pinMode(LS, INPUT);
  pinMode(IR, INPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(output);
  nh.advertise(light);
  nh.advertise(ir);
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

void spinCL(int v_left, int v_right){
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  analogWrite(EnA, v_left);
  analogWrite(EnB, v_right);
}

void spinCCL(int v_left, int v_right){
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  analogWrite(EnA, v_left);
  analogWrite(EnB, v_right);
}

float sat( float input){
 if (input<0){
   return 0;
 }
 else if(input>255){
   return 255;
 }
 else{
   return input;
 } 
}

void activate(float left_command, float right_command){
 if(left_command>0 && right_command>0){
  forward(sat(left_command), sat(right_command+8.8));
 }
 else if(left_command==0 && right_command ==0){
  freeze();
 } 
 else if(left_command<0 && right_command<0){
  backward(sat(-left_command), sat(-right_command+5));
 }
 else if(left_command>0 && right_command<=0){
   spinCL(sat(left_command), sat(-right_command));
 }
 else if(left_command<=0 && right_command>0){
   spinCCL(sat(-left_command), sat(right_command));
 }
 else{
  return; 
 }
}

void publish_encoder(int data_L, int data_R){
  output_msg.left_speed = data_L;
  output_msg.right_speed = data_R;
  output.publish(&output_msg);
}

void publish_light(){
  int light_data = analogRead(LS);
  light_msg.data = light_data;
  light.publish(&light_msg);
}
int count_0 = 0;
int count_1 = 0;
int cnt = 0;
float read_ir(){
  int val;
  float rate;
  cnt++;
  val = digitalRead(IR);
  if(val) count_1++;
  if(!val) count_0++;
  rate = count_0*100/(count_0+count_1);
  return rate;
}
void publish_ir(){
  ir.publish(&ir_msg);
}
void loop(){
  //forward(0, 0);
  cnt++;
  ir_msg.data = read_ir();
  if(cnt%50==0){
  publish_encoder(duration_L, duration_R);
  publish_light();
  duration_R = 0;
  duration_L = 0;
  nh.spinOnce();

  }
  if(cnt%100==0){
     publish_ir();
     cnt = 0;
     count_0 = 0;
     count_1 = 0;
  }
  delay(1);
}

