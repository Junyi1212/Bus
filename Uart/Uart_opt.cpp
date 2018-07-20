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
	//�ָ�����Ϊ����״̬                                 
	if(fcntl(fd, F_SETFL, 0) < 0)  
	{  
		printf("fcntl failed!\n");  
		return -1;  
	}       
#endif	
#if 0
	//�����Ƿ�Ϊ�ն��豸      
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
     
    /* tcgetattr(fd,&options)�õ���fdָ��������ز�����
     * �������Ǳ�����options,�ú��������Բ��������Ƿ���ȷ��
     * �ô����Ƿ���õȡ������óɹ�����������ֵΪ0��������ʧ�ܣ���������ֵΪ1. 
    */  
    if (tcgetattr(fd, &options) != 0)  
    {  
		printf("get attr err:%d\n",errno);      
		return -1;   
    }  
    
    //���ô������벨���ʺ����������  
    for (i= 0; i < sizeof(speed_arr) / sizeof(int); i++)  
	{  
		if (speed == name_arr[i])  
		{               
			cfsetispeed(&options, speed_arr[i]);   
			cfsetospeed(&options, speed_arr[i]);    
		}  
	}       
     
    //�޸Ŀ���ģʽ����֤���򲻻�ռ�ô���  
    options.c_cflag |= CLOCAL;
    
    //�޸Ŀ���ģʽ��ʹ���ܹ��Ӵ����ж�ȡ��������  
    options.c_cflag |= CREAD;  
    
    //��������������  
    switch(flow_ctrl)  
    {  
		case 0 ://��ʹ��������  
			options.c_cflag &= ~CRTSCTS;  
			break;     
		case 1 ://ʹ��Ӳ��������  
			options.c_cflag |= CRTSCTS;  
			break;  
		case 2 ://ʹ�����������  
			options.c_cflag |= IXON | IXOFF | IXANY;  
			break;  
    }  

    //��������λ  
    //����������־λ  
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
    
    //����У��λ  
    switch (parity)  
    {    
		case 'n':  
		case 'N': //����żУ��λ��  
			options.c_cflag &= ~PARENB;   
			options.c_iflag &= ~INPCK;      
			break;   
		case 'o':    
		case 'O'://����Ϊ��У��      
			options.c_cflag |= (PARODD | PARENB);   
			options.c_iflag |= INPCK;               
			break;   
		case 'e':   
		case 'E'://����ΪżУ��    
			options.c_cflag |= PARENB;         
			options.c_cflag &= ~PARODD;         
			options.c_iflag |= INPCK;        
			break;  
		case 's':  
		case 'S': //����Ϊ�ո�   
			options.c_cflag &= ~PARENB;  
			options.c_cflag &= ~CSTOPB;  
			break;   
		default:    
			printf("unsupported parity\n");      
			return -1;   
    } 
    
    // ����ֹͣλ   
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
 	//�޸����ģʽ��ԭʼ�������  
 	options.c_oflag &= ~OPOST;  
    
  	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
     
    //���õȴ�ʱ�����С�����ַ�  
    options.c_cc[VTIME] = 0;//1; /* ��ȡһ���ַ��ȴ�1*(1/10)s */    
    options.c_cc[VMIN]  = 0;//1; /* ��ȡ�ַ������ٸ���Ϊ1 */  
     
    //�����������������������ݣ����ǲ��ٶ�ȡ ˢ���յ������ݵ��ǲ���  
    tcflush(fd, TCIFLUSH);  
     
    //�������� (���޸ĺ��termios�������õ������У�  
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

    //���ô�������֡��ʽ  
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



