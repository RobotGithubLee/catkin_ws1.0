#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <string.h> /* memset */
#include <unistd.h> /* close */

#define SRVPORT 8888
#define CONNECT_NUM 5
#define MAX_NUM 80
int main()
{
    int serverSock=-1,clientSock=-1;
    struct sockaddr_in serverAddr;

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
    if(bind(serverSock,&serverAddr,sizeof(struct sockaddr_in))==-1)
    {
        printf("Bind error.\n");
        exit(-1);
    }
    printf("Bind successful.\n");

    if(listen(serverSock,10)==-1)
    {
        printf("Listen error!\n");
    }
    printf("Start to listen!\n");

    char revBuf[MAX_NUM]={0};
    char sedBuf[MAX_NUM]={0};
    while(1)
    {
        clientSock=accept(serverSock,NULL,NULL);
        printf("接受到一个连接：%d \r\n", clientSock);

        while(1)
        {
            if(read(clientSock,revBuf,MAX_NUM)<1)
            {
                printf("read error.\n");
                printf("断开了链接：%s \r\n", "");
		break;
            }
            else
            {
                printf("Client:%s\n",revBuf);
            }
            if(strcmp(revBuf,"Quit")==0||strcmp(revBuf,"quit")==0)
            {
                strcpy(sedBuf,"Goodbye,my dear client!");
            }
            else
            {
                strcpy(sedBuf,"Hello Client.");
            }
            if(write(clientSock,sedBuf,sizeof(sedBuf))==-1)
            {
                printf("Send error!\n");
            }
            printf("Me(Server):%s\n",sedBuf);
            if(strcmp(revBuf,"Quit")==0||strcmp(revBuf,"quit")==0)
            {
                break;
            }
            bzero(revBuf,sizeof(revBuf));
            bzero(sedBuf,sizeof(sedBuf));
        }
        close(clientSock);
        printf("关闭了一个连接：%d \r\n", clientSock);
    }
    close(serverSock);
    printf("关闭了服务：%d \r\n", serverSock);
    return 0;
}
