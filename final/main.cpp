#include"mbed.h"
#include"bbcar.h"


BufferedSerial pc(USBTX,USBRX); //tx,rx
BufferedSerial uart(D1,D0); //tx,rx
Ticker servo_ticker;
PwmOut pin5(D11), pin6(D10);
BufferedSerial xbee(A1, A0);

DigitalInOut ping(D6);

BBCar car(pin5, pin6, servo_ticker);

Thread getdata;
Thread moving;
Thread dis;

Timer t;

void Getdata();
void Move();
void Distance();

int state = 0;
int num = 0;
int x_1, x_2, y_1, y_2;
int TX, TY, TZ, RX, RY, RZ;
int coun = 0;
int dataA = 0, dataL = 0;
int order;
float val;

int total_speed = 90;

int main(){
   getdata.start(Getdata);
   moving.start(Move);
   dis.start(Distance);
   order = 1;
   while(1){
      ThisThread::sleep_for(100ms);
   }
}

void Move() {
   int predataL = 0, predataA = 0;
   char buf[30];
   sprintf(buf, "start\r\n");
   xbee.write(buf, 7);
   sprintf(buf, "follow the line\r\n");
   xbee.write(buf, 17);
   while(1) {
      while(predataL != dataL && order == 1){
         predataL = dataL;
         printf("x1: %d, x2: %d, y1: %d, y2: %d\n", x_1, x_2, y_1, y_2);
         printf ("dis: %f\n", val);
         if (val > 20) {
         double ratio = (x_1 + x_2) / 300.0;
         double left_speed = total_speed * ratio;
         double right_speed = total_speed - left_speed;
         printf("left: %f, right: %f\n", left_speed, right_speed);
         car.go(left_speed, right_speed);
         } else {
            car.stop();
            sprintf(buf, "follow finish\r\n");
            xbee.write(buf, 15);
            sprintf(buf, "circle around the barrier\r\n");
            xbee.write(buf, 27);
            order = 2;
            ThisThread::sleep_for(800ms);
         }
         ThisThread::sleep_for(50ms);
      } 
      if (order == 2) {
         car.turn(60, 1);
         for (int i = 0; i < 10; i ++) {
            ThisThread::sleep_for(100ms);
         }
         car.stop();
         ThisThread::sleep_for(500ms);
         car.go(45, 100);
         for (int i = 0; i < 18; i ++) {
            ThisThread::sleep_for(300ms);
         }
         car.stop();
         ThisThread::sleep_for(800ms);
         car.turn(60, 1);
         for (int i = 0; i < 10; i ++) {
            ThisThread::sleep_for(100ms);
         }
         car.stop();
         ThisThread::sleep_for(800ms);
         car.goStraight(100);
         for (int i = 0; i < 12; i ++) {
            ThisThread::sleep_for(100ms);
         }
         car.stop();
         sprintf(buf, "circle around finish\r\n");
         xbee.write(buf, 22);
         sprintf(buf, "calibrate the location\r\n");
         xbee.write(buf, 24);
         order = 3;
         ThisThread::sleep_for(500ms);
      }
      while (predataA != dataA && order == 3) {
         predataA = dataA;
         printf("TX: %d, TY: %d, TZ: %d, RX: %d, RY: %d, RZ: %d\n", TX, TY, TZ, RX, RY, RZ);
         double angle = atan(TX *1.0 / TZ) * 54.7;
         printf ("%f\n", angle);
         if ((angle > 15) || (angle < -15)) {
            car.turnangle(angle);
         } else {
            car.stop();
            sprintf(buf, "calibrate finish\r\n");
            xbee.write(buf, 18);
            sprintf(buf, "turn 90 degree\r\n");
            xbee.write(buf, 16);
            order = 4;
            ThisThread::sleep_for(500ms);
         }
         ThisThread::sleep_for(50ms);
      }
      if (order == 4) {
         car.turn(60, 1);
         for (int i = 0; i < 10; i ++) {
            ThisThread::sleep_for(100ms);
         }
         car.stop();
         sprintf(buf, "turn finish\r\n");
         xbee.write(buf, 13);
         sprintf(buf, "follow the line\r\n");
         xbee.write(buf, 17);
         order = 5;
         ThisThread::sleep_for(1000ms);
      } 
      while(predataL != dataL && order == 5){
         predataL = dataL;
         printf("x1: %d, x2: %d, y1: %d, y2: %d\n", x_1, x_2, y_1, y_2);
         printf ("dis: %f\n", val);
         if (val > 20) {
         double ratio = (x_1 + x_2) / 300.0;
         double left_speed = total_speed * ratio;
         double right_speed = total_speed - left_speed;
         printf("left: %f, right: %f\n", left_speed, right_speed);
         car.go(left_speed, right_speed);
         } else {
            car.stop();
            sprintf(buf, "follow finish\r\n");
            xbee.write(buf, 15);
            sprintf(buf, "complete\r\n");
            xbee.write(buf, 10);
            order = 6;
            ThisThread::sleep_for(500ms);
         }
         ThisThread::sleep_for(50ms);
      } 
   }
}

void Distance() {
   pc.set_baud(9600);
   while(1) {
      ping.output();
      ping = 0; wait_us(200);
      ping = 1; wait_us(5);
      ping = 0; wait_us(5);
      ping.input();
      while(ping.read() == 0);
      t.start();
      while(ping.read() == 1);
      val = t.read();
      val = val *17700.4f;
      t.stop();
      t.reset();
      ThisThread::sleep_for(1s);
   }
}

void Getdata() {
   uart.set_baud(9600);
   while(1) {
      if(uart.readable()){
         char recv[1];
         char T[4], R[3];
         char xy[3];
         uart.read(recv, sizeof(recv));
         uart.write(recv, sizeof(recv));
         if (state == 1) {
            if (num == 0) {
               T[coun++] = recv[0];
               if (coun == 4) {
                  TX = atoi(T);
                  num++;
                  coun = 0;
               }
            } else if (num == 1) {
               T[coun++] = recv[0];
               if (coun == 4) {
                  TY = atoi(T);
                  num++;
                  coun = 0;
               }
            } else if (num == 2) {
               T[coun++] = recv[0];
               if (coun == 4) {
                  TZ = atoi(T);
                  num++;
                  coun = 0;
               }
            } else if (num == 3) {
               R[coun++] = recv[0];
               if (coun == 3) {
                  RX = atoi(R);
                  num++;
                  coun = 0;
               }
            } else if (num == 4) {
               R[coun++] = recv[0];
               if (coun == 3) {
                  RY = atoi(R);
                  num++;
                  coun = 0;
               }
            } else if (num == 5) {
               R[coun++] = recv[0];
               if (coun == 3) {
                  RZ = atoi(R);
                  num++;
                  coun = 0;
                  state = 0;
                  dataA++;
               }
            }
         }

         if (state == 2) {
            if (num == 0) {
               xy[coun++] = recv[0];
               if (coun == 3) {
                  x_1 = atoi(xy);
                  num++;
                  coun = 0;
               }
            } else if (num == 1) {
               xy[coun++] = recv[0];
               if (coun == 3) {
                  x_2 = atoi(xy);
                  num++;
                  coun = 0;
               }
            } else if (num == 2) {
               xy[coun++] = recv[0];
               if (coun == 3) {
                  y_1 = atoi(xy);
                  num++;
                  coun = 0;
               }
            } else if (num == 3) {
               xy[coun++] = recv[0];
               if (coun == 3) {
                  y_2 = atoi(xy);
                  num++;
                  coun = 0;
                  state = 0;
                  dataL++;
                  printf ("X1: %d, X2: %d, Y1: %d, Y2: %d\n", x_1, x_2, y_1, y_2);
               }
            }
         } 
         
         if (state == 0 && recv[0] == 's') {
            state = 1;
            num = 0;
            coun = 0;
         } else if (state == 0 && recv[0] == 'l') {
            state = 2;
            num = 0;
            coun = 0;
         }
      }
   }
}