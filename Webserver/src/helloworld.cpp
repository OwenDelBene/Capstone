// Server side C program to demonstrate Socket programming
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
using std::vector;
using std::thread;


#if EKF_ENABLE
#include "Ekf.h"
#endif

#if IMU_ENABLE
#include "icm20948.h"
#endif


#define PORT 8081

char* parse(char line[], const char symbol[]);
char* parse_method(char line[], const char symbol[]);
char* find_token(char line[], const char symbol[], const char match[]);
int send_message(int fd, char image_path[], char head[]);
char http_header[25] = "HTTP/1.1 200 Ok\r\n";

void ekfThreadFunction(vector<char>* q);

int main(int argc, char const *argv[])
{

    vector<thread> ekfThreads;
    vector<char> ekf_q;
    int server_fd, new_socket, pid; 
    long valread;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    
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
        
        pid = fork();
        if(pid < 0){
            perror("Error on fork");
            exit(EXIT_FAILURE);
        }
        
        if(pid == 0){
            char buffer[30000] = {0};
            valread = read( new_socket , buffer, 30000);

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
                if (strcmp(parse_string, "forward") ==0) {
                  //drive train forward
                }
                else if (strcmp(parse_string, "enable") ==0 ) {
                  //enable ekf/pid control
                  //create new thread
                  if (ekfThreads.size() == 0) {
                    ekf_q.clear();
                    ekfThreads.push_back(std::move(thread(ekfThreadFunction, &ekf_q))); 
                  }
                  else {
                    ekf_q.push_back('q');
                    for (auto&x: ekfThreads) {
                        x.join();
                    }
                  }
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
            close(new_socket);
            free(copy);
            free(copy_head);  
        }
        else{
            printf(">>>>>>>>>>Parent create child with pid: %d <<<<<<<<<", pid);
            close(new_socket);
        }
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

void ekfThreadFunction(vector<char>* q) {

  double y[6] = {0, 0, 0, 0, 0, 0};
#if EKF_ENABLE 
  ekfData ekf;
  vector<double> x(7);
  double P[7][7];
  ekf_init(&ekf, x.data(), P);
#endif 
#if IMU_ENABLE 
 ICM20948 imu;
 const IMUData* data; 
 if (! imu.initialise() ) {
    printf("Imu not detected");
 }
#endif
  while (q->size() == 0) {
#if IMU_ENABLE 
  data = &imu.imuDataGet();
  //accel in g
  //gyro in dps
  //mag in uT
  for (int i=0; i<3; i++) {
    y[i] = data->mAcc[i];
    y[3+i] = data->mMag[i];
    x[4+i] = data->mGyro[i];
  }
  
#endif
#if EKF_ENABLE 
    ekf_step(x.data(), P, &ekf, y, dt, x.data(), P);  
#endif

  printf("ekf iteration\n");
  std::this_thread::sleep_for(std::chrono::seconds(1));
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







