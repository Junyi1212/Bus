#include "Uart_opt.h"


CUartOpt::CUartOpt()
{
	rs485_fd = 0;
	isRs485Open = false;

	pthread_mutexattr_init(&m_uartMutexAttr);
	pthread_mutexattr_settype(&m_uartMutexAttr,PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&m_uartMutex, &m_uartMutexAttr);	
}

CUartOpt::~CUartOpt()
{
	pthread_mutexattr_destroy(&m_uartMutexAttr);
	pthread_mutex_destroy(&m_uartMutex);
}

CUartOpt* CUartOpt::instance()
{
	static CUartOpt * _uartInstance = NULL;
	static pthread_mutex_t s_mutex_uart;
	
	if(NULL == _uartInstance)
	{
		pthread_mutex_lock(&s_mutex_uart);
		if(NULL == _uartInstance)
		{
			_uartInstance = new CUartOpt();
		}
		pthread_mutex_unlock(&s_mutex_uart);
	}
	
	return _uartInstance;
}

int CUartOpt::uart_open(int fd, char* port)  
{      
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);  
	if (fd < 0)  
	{  
		printf("can't open rs485:%d, %s\n", errno);  
		return -1;
	}
	printf("open uart success!\n");
#if 0	
	//恢复串口为阻塞状态                                 
	if(fcntl(fd, F_SETFL, 0) < 0)  
	{  
		printf("fcntl failed!\n");  
		return -1;  
	}       
#endif	
#if 0
	//测试是否为终端设备      
	if(0 == isatty(STDIN_FILENO))  
	{  
		printf("standard input is not a terminal device\n");  
		return -1;  
	}  
	else  
	{  
		printf("isatty success!\n");  
	}                
#endif
	return fd;  
}  

void CUartOpt::uart_close(int fd)  
{  
    close(fd);
    return;
}  

int CUartOpt::uart_set(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity)  
{       
	uint   i;  
	int   speed_arr[] = {B115200, B19200, B9600, B4800, B2400, B1200, B300};  
	int   name_arr[]  = {115200,  19200,  9600,  4800,  2400,  1200,  300};  
           
    struct termios options;  

	memset(&options, 0, sizeof(options));
     
    /* tcgetattr(fd,&options)得到与fd指向对象的相关参数，
     * 并将它们保存于options,该函数还可以测试配置是否正确，
     * 该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1. 
    */  
    if (tcgetattr(fd, &options) != 0)  
    {  
		printf("get attr err:%d\n",errno);      
		return -1;   
    }  
    
    //设置串口输入波特率和输出波特率  
    for (i= 0; i < sizeof(speed_arr) / sizeof(int); i++)  
	{  
		if (speed == name_arr[i])  
		{               
			cfsetispeed(&options, speed_arr[i]);   
			cfsetospeed(&options, speed_arr[i]);    
		}  
	}       
     
    //修改控制模式，保证程序不会占用串口  
    options.c_cflag |= CLOCAL;
    
    //修改控制模式，使得能够从串口中读取输入数据  
    options.c_cflag |= CREAD;  
    
    //设置数据流控制  
    switch(flow_ctrl)  
    {  
		case 0 ://不使用流控制  
			options.c_cflag &= ~CRTSCTS;  
			break;     
		case 1 ://使用硬件流控制  
			options.c_cflag |= CRTSCTS;  
			break;  
		case 2 ://使用软件流控制  
			options.c_cflag |= IXON | IXOFF | IXANY;  
			break;  
    }  

    //设置数据位  
    //屏蔽其他标志位  
    options.c_cflag &= ~CSIZE;  
    switch (databits)  
    {    
		case 5:  
			options.c_cflag |= CS5;  
			break;  
		case 6:  
			options.c_cflag |= CS6;  
			break;  
		case 7    :      
			options.c_cflag |= CS7;  
			break;  
		case 8:      
			options.c_cflag |= CS8;  
			break;    
		default:     
			printf("unsupported data size\n");  
			return -1;   
    }  
    
    //设置校验位  
    switch (parity)  
    {    
		case 'n':  
		case 'N': //无奇偶校验位。  
			options.c_cflag &= ~PARENB;   
			options.c_iflag &= ~INPCK;      
			break;   
		case 'o':    
		case 'O'://设置为奇校验      
			options.c_cflag |= (PARODD | PARENB);   
			options.c_iflag |= INPCK;               
			break;   
		case 'e':   
		case 'E'://设置为偶校验    
			options.c_cflag |= PARENB;         
			options.c_cflag &= ~PARODD;         
			options.c_iflag |= INPCK;        
			break;  
		case 's':  
		case 'S': //设置为空格   
			options.c_cflag &= ~PARENB;  
			options.c_cflag &= ~CSTOPB;  
			break;   
		default:    
			printf("unsupported parity\n");      
			return -1;   
    } 
    
    // 设置停止位   
    switch (stopbits)  
    {    
		case 1:     
			options.c_cflag &= ~CSTOPB;
			break;   
		case 2:     
			options.c_cflag |= CSTOPB;
			break;  
		default:     
			printf("unsupported stop bits\n");   
			return -1;  
    }  

#if 0     
 	//修改输出模式，原始数据输出  
 	options.c_oflag &= ~OPOST;  
    
  	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
     
    //设置等待时间和最小接收字符  
    options.c_cc[VTIME] = 0;//1; /* 读取一个字符等待1*(1/10)s */    
    options.c_cc[VMIN]  = 0;//1; /* 读取字符的最少个数为1 */  
     
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读  
    tcflush(fd, TCIFLUSH);  
     
    //激活配置 (将修改后的termios数据设置到串口中）  
    if (tcsetattr(fd, TCSANOW, &options) != 0)    
	{  
		printf("com set error!\n");    
		return -1;   
	}

	printf("uart set OK\n");
#endif
	cfmakeraw(&options);

	options.c_oflag &= ~OPOST;

	if (tcsetattr(fd, TCSANOW, &options) < 0)
	{	
		printf("tcsetattr %d failed:%d\n", fd, errno);
		return -1;
	} 

    return 0;   
}  

int CUartOpt::uart_init(void)  
{  
	rs485_fd = uart_open(rs485_fd, RS485_FILE_NAME);

    //设置串口数据帧格式  
    return uart_set(rs485_fd, 115200, 0, 8, 1, 'N');  
}  

int CUartOpt::uart_recv(int fd, uchar *rcv_buf)
{  
    int len = 0;
    int fs_sel;
   	uchar data  = 0;
    
    fd_set fs_read;  
     
    struct timeval time;  
     
    FD_ZERO(&fs_read);  
    FD_SET(fd, &fs_read);  
     
    time.tv_sec  =  1;  
    time.tv_usec =  0;  
     
	fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);     		 
	if(fs_sel)  
	{	
		len = read(fd, &data, 1);
	}
	else  
	{	
		printf("uart recv wrong\n");  
		return -1;  
	}	

	*rcv_buf = data;

    return len;
}  

int CUartOpt::uart_send(int fd, char *send_buf, int data_len)  
{  
	int len = 0;

	len = write(fd, send_buf, data_len);  
	if (len == data_len)  
	{
		return len;  
	}       
	else     
	{ 
		printf("uart_send, len:%d != data_len:%d\n", len, data_len);
		tcflush(fd, TCOFLUSH);  
		return 0;  
	}      
}  

int CUartOpt::DataCommuni(uchar addr, uchar *in_buf, uchar in_len, uchar *out_buf, uchar out_len)
{
	pthread_mutex_lock(&m_uartMutex);

	int ret = 0;
	uchar send_offset = 0;
	int  len = 0;
	bool  isStart = 0;
	uchar offset  = 0;

	if ((NULL == in_buf) || (NULL == out_buf))
	{
		printf("pointer is null\n");
		pthread_mutex_unlock(&m_uartMutex);
		return -1;
	}

	for(ret = 0; ret < in_len; ret++)
	{
		printf("0x%x ", in_buf[ret]);
	}
	printf("\n");

	while (send_offset < in_len)
	{
		ret = uart_send(rs485_fd, (char*)(in_buf + send_offset), 1);
		if (ret <= 0)
		{
			printf("uart_send errno:%d\n", errno);
			pthread_mutex_unlock(&m_uartMutex);
			return -1;	
		}
		send_offset++;
		usleep(3000);
	}
	
	while (out_len)
	{
		len = uart_recv(rs485_fd, out_buf + offset);
		printf("uart_recv len:%d\n", len);
		if (len != -1)
		{
			if (*out_buf == 0xA8)
			{
				isStart = 1;
			}

			if (isStart)
			{
				printf("data:0x%x\n", out_buf[offset]);			
				offset++;
				out_len -= len;
			}
		}
		else
		{
			uart_close(rs485_fd);
			uart_init();
			printf("uart wrong, reset\n");
			pthread_mutex_unlock(&m_uartMutex);
			return -1;
		}
	}

	pthread_mutex_unlock(&m_uartMutex);

	printf("DataCommuni done\n");

	return RET_SUCCESS;
}

int CUartOpt::Communi_init(void)
{
	if(true == isRs485Open)
	{
		printf("Rs485 is already Open\n");
		return 0;
	}

	int ret = uart_init();
	if(0 == ret)
	{
		isRs485Open = true;
	}

	return ret;
}

int CUartOpt::Communi_destroy(void)
{
	uart_close(rs485_fd);
	return	RET_SUCCESS;
}



