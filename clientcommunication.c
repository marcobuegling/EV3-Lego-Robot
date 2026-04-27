#include "clientcommunication.h"

//Sleep
#define Sleep( msec ) usleep(( msec ) * 1000 )

struct robot_data message;
struct server_robot_data server_message;

int sock_fd;

//Creates socket for multiple connections
int create_socket(int domain, int type, int protocol ) {
    int sock;
    const int opt = 1;
    //create socket
    if((sock = socket(domain, type, protocol)) < 0){
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }
    //use socket for multiple connections
    setsockopt( sock, SOL_SOCKET,SO_REUSEADDR, &opt, sizeof(int));
    return sock;
}

//Connects socket to IP address
void connect_socket(int sock, char *server_ip, unsigned short port){
   struct sockaddr_in server_addr;
   struct hostent *host_info;
   unsigned long addr;

   memset(&server_addr, 0, sizeof(server_addr));
   if((addr = inet_addr(server_ip)) != INADDR_NONE){ // is regular ip address
       memcpy((char *)&server_addr.sin_addr, &addr, sizeof(addr));
   }else{
       //convert "Localhost" to 127.0.0.1
       host_info = gethostbyname(server_ip);
       if(NULL == host_info){
           perror("Unknown server");
           exit(EXIT_FAILURE);
       }
       memcpy((char *)&server_addr.sin_addr, host_info->h_addr,host_info->h_length);
   }
   server_addr.sin_family = AF_INET;
   server_addr.sin_port = htons( port );
   if(connect(sock, (struct sockaddr*) &server_addr, sizeof(server_addr)) < 0){
       perror("connect");
       exit(EXIT_FAILURE);
   }
}

//sends position update to server
void send_position(int x_coordinate, int y_coordinate, int rot, position route[]){
    message.com_code = SND_POS;
    message.coord_x = x_coordinate;
    message.coord_y = y_coordinate;
    message.rotation = rot;
    for(int i = 0; i < 32; i++){
        message.route[i] = route[i];
    }
    printf(ANSI_COLOR_CYAN "COMMUNICATION: Position update sent (%d|%d),%d\n" ANSI_COLOR_RESET, message.coord_x, message.coord_y, message.rotation);
    /*printf(ANSI_COLOR_CYAN "Path:\t" ANSI_COLOR_RESET);
    for(int i = 0; message.route[i].x != -1; i++){
        printf(ANSI_COLOR_CYAN"(%d|%d),%d->" ANSI_COLOR_RESET,message.route[i].x, message.route[i].y, message.route[i].rotation);
    }
    printf("\n");*/
    if((send(sock_fd, &message, sizeof(struct robot_data), 0)) < 0){
        perror("send");
        exit(EXIT_FAILURE);
    }
}

//requests pickup job from server
struct server_robot_data request_pickup_job(){ 
    message.com_code = REQ_JOB_PICKUP;
    if((send(sock_fd, &message, sizeof(struct robot_data), 0)) < 0){
        perror("send");
        exit(EXIT_FAILURE);
    }
    if(recv(sock_fd, &server_message, sizeof(struct server_robot_data), 0) < 0){
        perror("recv");
        exit(EXIT_FAILURE);
    }
    printf(ANSI_COLOR_CYAN "COMMUNICATION: PickUp (%d|%d), %d, %d\n" ANSI_COLOR_RESET, server_message.coord_x, server_message.coord_y, server_message.rotation, server_message.height);
    return server_message;
}

//requests dropoff job from server
struct server_robot_data request_dropoff_job(){
    message.com_code = REQ_JOB_DROPOFF;
    if((send(sock_fd, &message, sizeof(struct robot_data), 0)) < 0){
        perror("send");
        exit(EXIT_FAILURE);
    }
    if(recv(sock_fd, &server_message, sizeof(struct server_robot_data), 0) < 0){
        perror("recv");
        exit(EXIT_FAILURE);
    }
    printf(ANSI_COLOR_CYAN "COMMUNICATION: DroppOff (%d|%d), %d, %d\n" ANSI_COLOR_RESET, server_message.coord_x, server_message.coord_y, server_message.rotation, server_message.height);
    return server_message;
}

//request initial position from server
struct server_robot_data request_init_pos(){
    while(1){
        message.com_code = REQ_INIT;
        if((send(sock_fd, &message, sizeof(struct robot_data), 0)) < 0){
            perror("send");
            exit(EXIT_FAILURE);
        }
        if(recv(sock_fd, &server_message, sizeof(struct server_robot_data), 0) < 0){
            perror("recv");
            exit(EXIT_FAILURE);
        }
        if(server_message.com_code == SND_INIT && server_message.coord_x != -1 && server_message.coord_y != -1 && server_message.rotation != -1){
            printf(ANSI_COLOR_CYAN "COMMUNICATION: Recieved initial position (%d|%d), %d!\n" ANSI_COLOR_RESET, server_message.coord_x, server_message.coord_y, server_message.rotation);
            break;
        }
        Sleep(2000);
    }
    
    //printf("recieved init (%d|%d), %d\n", message.coord_x, message.coord_y, message.rotation);
    return server_message;
}

void wait_for_start(){
    message.com_code = RDY;
    if((send(sock_fd, &message, sizeof(struct robot_data), 0)) < 0){
        perror("send");
        exit(EXIT_FAILURE);
    }
    printf("Ready sent\n");
    while(1){
        if(recv(sock_fd, &server_message, sizeof(struct server_robot_data), 0) < 0){
            perror("recv");
            exit(EXIT_FAILURE);
        }
        if(server_message.com_code == START){
            break;
        }
    }
    printf("START\n");
}

//initializes client server communication
void init_communication(int group_number, unsigned short port){

    FILE *fp;
    char ip_buff[255];

    fp = fopen("./ipconfig.txt", "r");
    fgets(ip_buff, 255, (FILE*)fp);
    printf("Read %s from ipconfig\n", ip_buff);
    fclose(fp);

    sock_fd = create_socket(AF_INET, SOCK_STREAM, 0);
    connect_socket(sock_fd, ip_buff, port);
    message.group_id = group_number;
    message.com_code = INIT_GREET;
    message.coord_x = -1;
    message.coord_y = -1;
    message.rotation = -1;

    server_message.coord_x = -1;
    server_message.coord_y = -1;
    server_message.rotation = -1;

    for(int i = 0; i < 32; i++){
        message.route[i].x = -1;
        message.route[i].y = -1;
        message.route[i].rotation = -1;
    }
    if((send(sock_fd, &message, sizeof(struct robot_data), 0)) < 0){
        perror("send (init)");
        exit(EXIT_FAILURE);
    }
}
