#include <algorithm>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <errno.h>

#include <thread>
#include <vector>
#include <chrono>
#include <sstream>
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;
#include <mutex>
#include <condition_variable>
#include <optional>
#include <semaphore>

using std::vector;
using std::thread;

#define PID_PERIOD  2000
#define EKF_PERIOD  2000

#define SERVO_HAT_ADDRESS 0x40


#if EKF_ENABLE
#include "Ekf.h"
#include <fstream>
#endif

#if IMU_ENABLE
#include "icm20948.h"
#define DEG2RAD M_PI/180.0f
#else
#include "HIL.h"
#endif

#if PID_ENABLE
#include "pid.h"
#endif

#include "KDTree.h"
#if INV_ENABLE
#include "linearAlgebra.h"
#include "inverseKinematics.h"
#endif

#if SERVO_ENABLE
#include "PCA9685.h"
#endif

#define PORT 8081

char* parse(char line[], const char symbol[]);
char* parse_method(char line[], const char symbol[]);
char* find_token(char line[], const char symbol[], const char match[]);
int send_message(int fd, char image_path[], char head[]);
char http_header[25] = "HTTP/1.1 200 Ok\r\n";

void ekfThreadFunction();
void pidThreadFunction();
void webServerThreadFunction(int new_socket);
/*
std::condition_variable ekf_enable;
std::mutex ekf_mutex;
bool ekf_en = false;
std::condition_variable pid_enable;
std::mutex pid_mutex;
bool pid_en = false;
*/

bool ekf_en = true;
std::binary_semaphore pwm_sem{ekf_en};
std::binary_semaphore ekf_enable{ekf_en};
std::binary_semaphore pid_enable{ekf_en};
vector<double> pwm_vec= vector<double>(3); 


int main(int argc, char const *argv[])
{

    vector<thread> webServerThreads;
    int server_fd, new_socket, pid; 
    long valread;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    thread ekf_thread(ekfThreadFunction); 
    thread pid_thread(pidThreadFunction); 
   
    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("In sockets");
        exit(EXIT_FAILURE);
    }
    
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );
    
    memset(address.sin_zero, '\0', sizeof address.sin_zero);
    
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0)
    {
        perror("In bind");
        close(server_fd);
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 10) < 0)
    {
        perror("In listen");
        exit(EXIT_FAILURE);
    }
    
     
    while(1)
    {
        printf("\n+++++++ Waiting for new connection ++++++++\n\n");
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0)
        {
            perror("In accept");
            exit(EXIT_FAILURE);
        }
        
        webServerThreads.push_back(std::move(std::thread(webServerThreadFunction, new_socket))) ;   
    }
    close(server_fd);
    return 0;
}

char* parse(char line[], const char symbol[])
{
    char *copy = (char *)malloc(strlen(line) + 1);
    strcpy(copy, line);
    
    char *message;
    char * token = strtok(copy, symbol);
    int current = 0;

    while( token != NULL ) {
      
      token = strtok(NULL, " ");
      if(current == 0){
          message = token;
          if(message == NULL){
              message = "";
          }
          return message;
      }
      current = current + 1;
   }
   free(token);
   free(copy);
   return message;
}

void pidThreadFunction() {
  cout << "howdy world pid " << endl;
vector<double> pwm_des(3,0);
vector<double> pwm_meas(3,0);
#if PID_ENABLE
PIDController pid;
float kp = .1;
float ki = .2;
float kd = .3;
float lpf = 0.01f; //low pass filter coeff
float max = 2.0f;
float min = -max;
//TODO accurate dt per loop
PIDInit(&pid, kp, ki, kd, lpf, max, min);
#endif
#if SERVO_ENABLE
  PCA9685 pca("/dev/i2c-1", SERVO_HAT_ADDRESS); //TODO address
#endif 
auto T1 = Time::now();
  while (true) {
#if PID_ENABLE
    //if disabled then non busy wait
    pid_enable.acquire();
    //get updated motor_angles
    pwm_sem.acquire();
    if (pwm_vec.size()){
      pwm_des = pwm_vec;
      pwm_vec.clear();
    }
    pwm_sem.release();
    //motor_angles to pwm
    //pid to desired pwm
#if SERVO_ENABLE
    for (int i=1; i<4; i++) {
      pwm_meas[i] = pca.getPWM(i);
    }
    //This either needs to be before SERVO, or need to copy servo below
    auto T2 = Time::now();
    fsec fs = T2 - T1;
    float dt = std::chrono::duration_cast<ms>(fs).count() * 1e-3;
    for (int i=1; i<4; i++) {
      //pca.setPWM( i, PIDUpdate(&pid, pwm_des[i], pwm_meas[i], dt));
      pca.setPWM(i, pwm_des[i]);
    }
    T1 = T2;
#endif
#endif

    pid_enable.release();
    std::this_thread::sleep_for(std::chrono::milliseconds(PID_PERIOD));
    std::thread::id threadId = std::this_thread::get_id();

  std::stringstream ss;
  ss << threadId;
  std::string threadIdString = ss.str();
    
    cout << "pid iteration " << threadIdString << endl;
  }
}

void ekfThreadFunction() {
  cout << "howdy world ekf" << endl; 
  vector<double> x(7, 0);
  double y[6] = {0, 0, 0, 0, 0, 0};
  double eul[] = {0, 0, 0};
  double pwm_des[] = {0, 0, 0};
  double pwm_meas[] = {0, 0, 0};
  auto T1 = Time::now();
  double dt = EKF_PERIOD * 1e-3;
  std::fstream of("rpy.out", std::ios_base::out);
#if EKF_ENABLE 
  ekfData ekf;
  double P[7][7];
  ekf_init(&ekf, x.data(), P);
#endif 

#if IMU_ENABLE 
 ICM20948 imu;
 const IMUData* data; 
 if (! imu.initialise() ) {
    printf("Imu not detected");
 }
#else
  vector<double> true_x(7);
  double init_eul[] = {-M_PI/6.0, M_PI/8.0, M_PI/12.0};
  double w0[] = {-M_PI/180.0, 5*M_PI/180.0, 0};
  eul2Quat(init_eul[0], init_eul[1], init_eul[2], true_x.data());
  true_x[4] = w0[0];
  true_x[5] = w0[1];
  true_x[6] = w0[2];
  vector<double> eule(3);
  HIL hil(ekf, true_x);
#endif
  int i=0;
  while (true) {
   // if (i++ > 10) return;
  cout << "howdy world ekf" << endl; 
  ekf_enable.acquire();
#if IMU_ENABLE 
  data = &imu.imuDataGet();
  //accel in g
  //gyro in dps
  //mag in uT
  for (int i=0; i<3; i++) {
    y[i] = data->mAcc[i] * 9.81;
    y[3+i] = data->mMag[i] ;
    x[4+i] = data->mGyro[i] * DEG2RAD;
  }
#else
  hil.propogate(dt, y, x.data()+4);
  quat2Eul(x.data(), eule.data());
  hil.writeVector(hil.ekf_eul,eule);
  hil.writeVector(of, eule); 
#endif
#if EKF_ENABLE 
    auto T2 = Time::now();
    fsec fs = T2 - T1;
    dt = std::chrono::duration_cast<ms>(fs).count() * 1e-3;
    ekf_step(x.data(), P, &ekf, y, dt, x.data(), P);  
    T1 = T2;
#endif
#if INV_ENABLE
  quat2Eul(x.data(), eul);
  inverseKinematics(-eul[0] * 180.0f/M_PI, -eul[1]*180.0f*M_PI, pwm_des);
  for (int i=0; i<3; i++) {
    pwm_des[i] = anglesToPwm(pwm_des[i]);
  } 
  pwm_sem.acquire();
  pwm_vec = vector<double>(pwm_des, pwm_des + 3 ); 
  pwm_sem.release();
#endif

std::thread::id threadId = std::this_thread::get_id();

  std::stringstream ss;
  ss << threadId;
  std::string threadIdString = ss.str();
  vector<double> eul(3);
  quat2Eul(x.data(), eul.data());
  cout << "ekf iteration: roll, pitch " << eul[0] << " pitch " << eul[1]  << endl;
  ekf_enable.release();
  std::this_thread::sleep_for(std::chrono::milliseconds(EKF_PERIOD));
  }
  printf("EKF disabled\n");

}





char* parse_method(char line[], const char symbol[])
{
    char *copy = (char *)malloc(strlen(line) + 1);
    strcpy(copy, line);
        
    char *message;
    char * token = strtok(copy, symbol);
    int current = 0;

    while( token != NULL ) {
      
      //token = strtok(NULL, " ");
      if(current == 0){
          message = token;
          if(message == NULL){
              message = "";
          }
          return message;
      }
      current = current + 1;
   }
   free(copy);
   free(token);
   return message;
}

char* find_token(char line[], const char symbol[], const char match[])
{
    char *copy = (char *)malloc(strlen(line) + 1);
    strcpy(copy, line);
        
    char *message;
    char * token = strtok(copy, symbol);

    while( token != NULL ) {
      
      //printf("--Token: %s \n", token);
      
      if(strlen(match) <= strlen(token))
      {
          int match_char = 0;
          for(int i = 0; i < strlen(match); i++)
          {
              if(token[i] == match[i])
              {
                  match_char++;
              }
          }
          if(match_char == strlen(match)){
            message = token;
            return message;
          }
      }      
      token = strtok(NULL, symbol);
   }
   free(copy);
   free(token);
   message = "";
   return message;
}



int send_message(int fd, char image_path[], char head[]){
    /*
    char imageheader[] = 
    "HTTP/1.1 200 Ok\r\n"
    "Content-Type: image/jpeg\r\n\r\n";
    */
    struct stat stat_buf;  /* hold information about input file */

    write(fd, head, strlen(head));

    int fdimg = open(image_path, O_RDONLY);
    
    if(fdimg < 0){
        printf("Cannot Open file path : %s with error %d\n", image_path, fdimg); 
    }
     
    fstat(fdimg, &stat_buf);
    int img_total_size = stat_buf.st_size;
    int block_size = stat_buf.st_blksize;
    if(fdimg >= 0){
        ssize_t sent_size;

        while(img_total_size > 0){
              int send_bytes = ((img_total_size < block_size) ? img_total_size : block_size);
              int done_bytes = sendfile(fd, fdimg, NULL, block_size);
              img_total_size = img_total_size - done_bytes;
        }
        if(sent_size >= 0){
            printf("send file: %s \n" , image_path);
        }
        close(fdimg);
    }
}

void webServerThreadFunction(int new_socket) {

            char buffer[30000] = {0};
            size_t valread = read( new_socket , buffer, 30000);

            printf("\n buffer message: %s \n ", buffer);
            char *parse_string_method = parse_method(buffer, " ");  
            printf("Client method: %s\n", parse_string_method);


            char *parse_string = parse(buffer, " ");  
            printf("Client ask for path: %s\n", parse_string);

            char *copy = (char *)malloc(strlen(parse_string) + 1);
            strcpy(copy, parse_string);

            char *copy_head = (char *)malloc(strlen(http_header) +200);
            strcpy(copy_head, http_header);

            if(parse_string_method[0] == 'G' && parse_string_method[1] == 'E' && parse_string_method[2] == 'T'){
                if(strlen(parse_string) <= 1){
                    //case that the parse_string = "/"  --> Send index.html file
                    char path_head[500] = ".";
                    strcat(path_head, "/index.html");
                    strcat(copy_head, "Content-Type: text/html\r\n\r\n");
                    send_message(new_socket, path_head, copy_head);
                }
                if (strcmp(parse_string, "/Forward?") ==0) {
                  cout << "drive forward " << endl;
                    char path_head[500] = ".";
                    strcat(path_head, "/index.html");
                    strcat(copy_head, "Content-Type: text/html\r\n\r\n");
                    send_message(new_socket, path_head, copy_head);
                  //drive train forward
                }
                if (strcmp(parse_string, "/Backward?") ==0) {
                  cout << "drive forward " << endl;
                    char path_head[500] = ".";
                    strcat(path_head, "/index.html");
                    strcat(copy_head, "Content-Type: text/html\r\n\r\n");
                    send_message(new_socket, path_head, copy_head);
                  //drive train forward
                }
                if (strcmp(parse_string, "/Left?") ==0) {
                  cout << "drive forward " << endl;
                    char path_head[500] = ".";
                    strcat(path_head, "/index.html");
                    strcat(copy_head, "Content-Type: text/html\r\n\r\n");
                    send_message(new_socket, path_head, copy_head);
                  //drive train forward
                }
                if (strcmp(parse_string, "/Right?") ==0) {
                  cout << "drive forward " << endl;
                    char path_head[500] = ".";
                    strcat(path_head, "/index.html");
                    strcat(copy_head, "Content-Type: text/html\r\n\r\n");
                    send_message(new_socket, path_head, copy_head);
                  //drive train forward
                }
                else if (strcmp(parse_string, "/Stop?") ==0 ) {
                  cout << "enable compensation " << endl;
                  //enable ekf/pid control
                  //create new thread
                  if (ekf_en) {
		    cout << "stopping ekf " << endl;
                    ekf_enable.acquire();
                    pid_enable.acquire();
                    ekf_en^=1;
                    char path_head[500] = ".";
                    strcat(copy_head, "Content-Type: text/html\r\n\r\n");
                    strcat(path_head, "/index_start.html");
                    send_message(new_socket, path_head, copy_head);
                   
                  }
                  else {
	            cout << "starting ekf " << endl;
                    ekf_enable.release();
                    pid_enable.release();
                    ekf_en^=1;
                    char path_head[500] = ".";
                    strcat(copy_head, "Content-Type: text/html\r\n\r\n");
                    strcat(path_head, "/index.html");
                    send_message(new_socket, path_head, copy_head);
                }
                }
                else {

                    cout << "NOTHING MATCHED? " << parse_string << endl;
                }
                 
                printf("\n------------------Server sent----------------------------------------------------\n");
            }
            else if (parse_string_method[0] == 'P' && parse_string_method[1] == 'O' && parse_string_method[2] == 'S' && parse_string_method[3] == 'T'){
                char *find_string = (char*)malloc(200);
                find_string = find_token(buffer, "\r\n", "action");
                strcat(copy_head, "Content-Type: text/plain \r\n\r\n"); //\r\n\r\n
                //strcat(copy_head, "Content-Length: 12 \n");
                strcat(copy_head, "User Action: ");
                printf("find string: %s \n", find_string);
                strcat(copy_head, find_string);
                write(new_socket, copy_head, strlen(copy_head));
            }
	    /*cout << "not get or post " << parse_string_method << endl;
	    cout << "not get or post " << parse_string << endl;
	char path_head[500] = ".";
                    strcat(path_head, "/index.html");
                    strcat(copy_head, "Content-Type: text/html\r\n\r\n");
                    send_message(new_socket, path_head, copy_head);
	    */
            close(new_socket);
            free(copy);
            free(copy_head);  

}
