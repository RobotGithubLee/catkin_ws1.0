#include <sys/types.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <semaphore.h>
#include <signal.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <unistd.h> /* close */
 
 
//宏定义
#define FALSE  -1
#define TRUE   0
#define SRVPORT 8888
#define CONNECT_NUM 5
#define MAX_NUM 80

#define UART_DEVICE0     "/dev/ttyUSB0"
#define UART_DEVICE1     "/dev/ttyUSB1"


static struct termios old_termios;
static int tty_fd = -1;
static int b_quit_process = 0;
static sem_t quit_sem;

static int serverSock=-1,clientSock=-1;
static int SocketEnable=0,PortEnable=0;


int tty_init(int fd, int parity, int speed)
{
	struct termios tty;

	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0) {
		printf("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 1;            // read block
	tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);

	if (tcsetattr (fd, TCSANOW, &tty) != 0) {
		printf("error %d from tcsetattr", errno);
		return -1;
	}

	if (tcgetattr(fd, &tty) != 0) {
		perror("SetupSerial 1");
		return -1;
	}

	printf("c_iflag = 0x%x\n", tty.c_iflag);
	printf("c_oflag = 0x%x\n", tty.c_oflag);
	printf("c_cflag = 0x%x\n", tty.c_cflag);
	printf("c_lflag = 0x%x\n", tty.c_lflag);

	return 0;
}

void readPort_thread_func(void *arg)
{
	int res;
	char buf[256];
	int i;

	printf("Reading...\n");

	while(1) {
		memset(buf, 0, sizeof(buf));

		printf("Read %d: \n", i++);
		res = read(tty_fd, buf, 255);
		if(res==0)
			continue;
        printf("Port read len=%d\n" ,len);
		rcv_buf[len] = '\0';
        for(i=0;i<len;i++){
             printf("%x  " ,rcv_buf[i]);
        }
        printf("\n" );
		
		if(SocketEnable){
           socketSend(rcv_buf,len);
        }	
	}
	printf("%s: exit\n", __func__);
}




/********************************************************************
* 名称：                  portSend
* 功能：                  串口发送数据
* 入口参数：              fd          :文件描述符    
*                         send_buf    :存放串口发送数据
*                         data_len    :一帧数据的个数
* 出口参数：              正确返回为1，错误返回为0
*******************************************************************/
int portSend(int fd, char *send_buf,int data_len)
{
    int len = 0; 
    len = write(fd,send_buf,data_len);
    if (len == data_len ){
		printf("Port send len=%d\n" ,len);
		for(i=0;i<len;i++){
             printf("%x  " ,send_buf[i]);
        }
        return len;
    }     
    else{
        tcflush(fd,TCOFLUSH);
        return FALSE;
    }
 }



void signal_handler(int signum)
{
	sem_post(&quit_sem);
}





void startlisten(void)
{
    int len=-1;
    char revBuf[MAX_NUM]={0};
    while(1)
    {
        clientSock=accept(serverSock,NULL,NULL);
        printf("接受到一个连接：%d \r\n", clientSock);

        SocketEnable=1;
        while(1){
			len=read(clientSock,revBuf,MAX_NUM);
            if(len<1){
                printf("read error.\n");
                printf("断开了链接：%s \r\n", "");
				break;
            }
            else{
                printf("Client receive len=%d\n",len);
				for(i=0;i<len;i++){
					printf("%x  ",revBuf[i]);
			}
				
				
                if(PortEnable){
                    len = portSend(fd,revBuf,10);
					if(len > 0)
						printf("Portsend data succes!\n");
                    else
						printf("Portsend data failed!\n");          
                }
			sleep(2);
            }   
            bzero(revBuf,sizeof(revBuf));            
        }

        SocketEnable=0;
        close(clientSock);
        printf("关闭了一个连接：%d \r\n", clientSock);
   } 
}


/********************************************************************
* 名称：                 socketSend
* 功能：                 socket 发送数据
* 入口参数：             sedBuf
                         len     length

*******************************************************************/
void socketSend(char*sedBuf,int len)
{
	int len,i;
	len=write(clientSock,sedBuf,len);

    if(len==-1){
       printf("socketSend error!\n");
    }
	else{
		printf("socketSend Len=:%d\n",len);
		for(i=0;i<len;i++){
			printf("%x  ",sedBuf[i]);
		}
		printf("\n");   
	}
}





/********************************************************************
* 名称：                 socketConnect
* 功能：                 socket Connect
* 入口参数：             void
                         

*******************************************************************/

void socket_thread_func(void *arg)
{
    struct sockaddr_in serverAddr;
    pthread_t id1;
    int ret;

    serverSock=socket(AF_INET,SOCK_STREAM,0);
    if(serverSock<0)
    {
        printf("socket creation failed\n");
        exit(-1);
    }
    printf("socket create successfully.\n");

    memset(&serverAddr,0,sizeof(serverAddr));
    serverAddr.sin_family=AF_INET;
    serverAddr.sin_port=htons((u_short) SRVPORT);
    serverAddr.sin_addr.s_addr=htons(INADDR_ANY);//inet_addr("127.0.0.1");
	
	while(1){
		if(bind(serverSock,&serverAddr,sizeof(struct sockaddr))==-1){
			printf("Bind error.\n");		
		}
		else
			break;
		sleep(10);
	}
   
    printf("Bind successful.\n");

    if(listen(serverSock,10)==-1)
    {
        printf("Listen error!\n");
    }
    printf("Start to listen!\n");

	startlisten();   
    return 0; 
}







int main(int argc, char *argv[])
{
	pthread_t socket_thread, Port_thread;
	struct sigaction sig_act;
	int i;

	printf("Start...\n");
	
	
	//打开串口
	while(1)
	{
		tty_fd = open(UART_DEVICE0, O_RDWR);
		if (tty_fd < 0) 
		{
			perror(UART_DEVICE);
		}
		else
		{
			printf("Open ttyUSB0 succes...\n");
			break;
		}
		
		tty_fd = open(UART_DEVICE1, O_RDWR);
		if (tty_fd < 0) 
		{
			perror(UART_DEVICE1);
		}
		else
		{
			printf("Open ttyUSB1 succes...\n");
			break;
		}
		sleep(2);	
	}

	tcgetattr(tty_fd, &old_termios);
    
	//初始化串口
	while(1){
		if(tty_init(tty_fd, 0, B38400) < 0) {
			printf("Set Parity Error\n");
		}
		else
			break;
	    sleep(5);
	}

	sem_init(&quit_sem, 0, 0);

	// ensure a clean shutdown if user types ctrl-c
	sig_act.sa_handler = signal_handler;
	sigemptyset(&sig_act.sa_mask);
	sig_act.sa_flags = SA_INTERRUPT;
	sigaction(SIGINT, &sig_act, NULL);

	PortEnable=1;
	//开启串口读取线程
	if(pthread_create(&port_thread, NULL, (void *)reaPort_thread_func, NULL)) {
		printf("port_thread craete failure\n");
		return -1;
	}

	//开启Socket线程
	if (pthread_create(&socket_thread, NULL, (void *)socket_thread_func, NULL)) {
		printf("write_thread craete failure\n");
		return -1;
	}

	while(1) {
		sem_wait(&quit_sem);
		break;
	}

	printf("Close...\n");

	tcsetattr(tty_fd, TCSANOW, &old_termios); // restore
	close(tty_fd);
	
	close(serverSock);
    printf("关闭了服务：%d \r\n", serverSock);

	return 0;
}
