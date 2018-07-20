
#include "i2c_opt.h"

CI2COpt::CI2COpt()
{
	i2c_fileFd = 0;
	gpio_Int1Fd = 0;
	gpio_Int2Fd = 0;

	pthread_mutexattr_init(&m_i2cMutexAttr);
	pthread_mutexattr_settype(&m_i2cMutexAttr,PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&m_i2cMutex, &m_i2cMutexAttr);

	int ret = pthread_create(&m_threadListen, NULL, &CI2COpt::ThreadListen, this);
	if (ret != RET_SUCCESS)
	{
		printf("pthread_create err\n");
              return;
	}	
	setLooping(true);
}

CI2COpt::~CI2COpt()
{
	setLooping(false);
	pthread_mutexattr_destroy(&m_i2cMutexAttr);		
	pthread_mutex_destroy(&m_i2cMutex);

	pthread_join(m_threadListen, NULL);
	printf("~CI2COpt done\n");		
}

CI2COpt* CI2COpt::instance()
{
	static CI2COpt * _i2cInstance = NULL;
	static pthread_mutex_t s_mutex_i2c;
	
	if(NULL == _i2cInstance)
	{
		pthread_mutex_lock(&s_mutex_i2c);
		if(NULL == _i2cInstance)
		{
			_i2cInstance = new CI2COpt();
		}
		pthread_mutex_unlock(&s_mutex_i2c);
	}
	
	return _i2cInstance;
}

int CI2COpt::i2c_init(void)  
{
    if((i2c_fileFd = open(I2C_FILE_FD, O_RDWR)) < 0) 
    {  
        printf("unable to open i2c-2 file\n");  
        return -1; 
    } 

    uint gpio1 = GPIO_N_347;
    uint gpio2 = GPIO_N_348;

    gpio_export(gpio1);
    gpio_set_dir(gpio1, 0);
    gpio_set_edge(gpio1, "rising");
    gpio_Int1Fd = gpio_fd_open(gpio1);

    gpio_export(gpio2);
    gpio_set_dir(gpio2, 0);
    gpio_set_edge(gpio2, "rising");
    gpio_Int2Fd = gpio_fd_open(gpio2);	

    printf("open i2c success!\n");
    return 0;
}

/****************************************************************
 * gpio_export
 ****************************************************************/
int CI2COpt::gpio_export(uint gpio)
{
	int fd, len;
	char buf[GPIO_MAX_BUF];
 
	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		printf("gpio/export");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
 
	return 0;
}

/****************************************************************
 * gpio_unexport
 ****************************************************************/
int CI2COpt::gpio_unexport(uint gpio)
{
	int fd, len;
	char buf[GPIO_MAX_BUF];
 
	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		printf("gpio/export");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_set_dir
 ****************************************************************/
int CI2COpt::gpio_set_dir(uint gpio, uint out_flag)
{
	int fd;
	char buf[GPIO_MAX_BUF];
 
	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		printf("gpio/direction");
		return fd;
	}
 
	if (out_flag)
		write(fd, "out", 4);
	else
		write(fd, "in", 3);
 
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_set_value
 ****************************************************************/
int CI2COpt::gpio_set_value(uint gpio, uint value)
{
	int fd;
	char buf[GPIO_MAX_BUF];
 
	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		printf("gpio/set-value");
		return fd;
	}
 
	if (value)
		write(fd, "1", 2);
	else
		write(fd, "0", 2);
 
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_get_value
 ****************************************************************/
int CI2COpt::gpio_get_value(uint gpio, uint *value)
{
	int fd;
	char buf[GPIO_MAX_BUF];
	char ch;

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		printf("gpio/get-value");
		return fd;
	}
 
	read(fd, &ch, 1);

	if (ch != '0') {
		*value = 1;
	} else {
		*value = 0;
	}
 
	close(fd);
	return 0;
}


/****************************************************************
 * gpio_set_edge
 ****************************************************************/
int CI2COpt::gpio_set_edge(uint gpio, char *edge)
{
	int fd;
	char buf[GPIO_MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		printf("gpio/set-edge");
		return fd;
	}
 
	write(fd, edge, strlen(edge) + 1); 
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_fd_open
 ****************************************************************/
int CI2COpt::gpio_fd_open(uint gpio)
{
	int fd;
	char buf[GPIO_MAX_BUF];

	snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_RDONLY | O_NONBLOCK );
	if (fd < 0) {
		printf("gpio/fd_open Error\n");
	}
	return fd;
}

/****************************************************************
 * gpio_fd_close
 ****************************************************************/
int CI2COpt::gpio_fd_close(int fd)
{
	return close(fd);
}

void CI2COpt::PrintBuf(uchar* data, uint len)
{
	uint i;
	for(i=0; i<len; i++)
	{
		printf("0x%x ", data[i]);
	}
	printf("\n\n");
}

void *CI2COpt::ThreadListen(void* arg)
{
	CI2COpt* This = static_cast<CI2COpt*>(arg);

	int ret;
	uchar databuf_send1[PROTOCOL_DATA_LEN] = {0};
	uchar data_inlen1		   			  =   0;
	uchar databuf_send2[PROTOCOL_DATA_LEN] = {0};
	uchar data_inlen2		   			  =   0;	
	uchar databuf_recv[PROTOCOL_DATA_LEN] = {0};
	uchar data_outlen					  =  0;

	printf("Enter CI2COpt ThreadListen\n");

	struct pollfd fdset[2];

	char gpioValue;
	uint timeout = POLL_TIMEOUT;
	uint len;
	uint count = 0;

	while(This->Looping()) 
	{
		memset((void*)fdset, 0, sizeof(fdset));
		fdset[0].fd = This->gpio_Int1Fd;
		fdset[0].events = POLLPRI;
		fdset[1].fd = This->gpio_Int2Fd;
		fdset[1].events = POLLPRI;
		ret = poll(fdset, 2, timeout);	  

		if (ret < 0) 
		{
			printf("poll() failed!\n");
			continue;
		}
		else if(ret == 0)
		{	
			continue;		
		}
	  		
		if(fdset[0].revents & POLLPRI) 
		{
			gpioValue = 0;
			ret = lseek(fdset[0].fd, 0, SEEK_SET);
			len = read(fdset[0].fd, &gpioValue, 1);
			
			if(gpioValue == '1') //高电平
			{
				printf("GPIO347 Interrupt\n");

				//读取i2c 数据
				ret = CI2COpt::instance()->DataCommuni(databuf_send1, data_inlen1, databuf_recv, data_outlen);
				if (ret == 0)
				{
				    //Do somethin with databuf_recv
				}
				else
				{
					printf("Board1 DataCommuni err\n");	
				}	
			}
		}
	
		if(fdset[1].revents & POLLPRI) 
		{
			gpioValue = 0;
			ret = lseek(fdset[1].fd, 0, SEEK_SET);
			len = read(fdset[1].fd, &gpioValue, 1);
			
			if(gpioValue == '1') //高电平
			{
				printf("GPIO348 Interrupt\n");

				//读取i2c 数据
				ret = CI2COpt::instance()->DataCommuni(databuf_send2, data_inlen2, databuf_recv, data_outlen);
				if (ret == 0)
				{
					//Do somethin with databuf_recv
				}
				else
				{
					printf("Board2 DataCommuni err\n");	
				}
			}
		}	
	}

	XTInfo("Exit CI2COpt ThreadListen\n");
	return NULL;
}

int CI2COpt::DataCommuni(uchar *in_buf, uchar in_len, uchar *out_buf, uchar out_len)
{
    pthread_mutex_lock(&m_i2cMutex);

    struct i2c_rdwr_ioctl_data packets;  
    struct i2c_msg messages[2];  
  
    messages[0].addr  = addr;  //send addr
    messages[0].flags = 0;  
    messages[0].len   = in_len;  
    messages[0].buf   = in_buf;  

    if(out_len > 0)
    {
    	messages[1].addr  = addr;  //receive addr
    	messages[1].flags = I2C_M_RD;  	/* | I2C_M_NOSTART*/
    	messages[1].len   = out_len;  
    	messages[1].buf   = out_buf;  
    	packets.nmsgs     = 2; 
    }
    else
    {
    	packets.nmsgs     = 1;
    }
    packets.msgs      = messages;  
    
    if(ioctl(i2c_fileFd, I2C_RDWR, &packets) < 0)
    {  
        printf("unable to send data\n");  
	 pthread_mutex_unlock(&m_i2cMutex);

        return -1;  
    } 

    if(out_len > 0)
    {
    	printf("i2c recv:\n");
    	printf(out_buf, out_len);
    }

    pthread_mutex_unlock(&m_i2cMutex);
    return 0;
}

int CI2COpt::Communi_init(void)
{
	pthread_mutex_lock(&m_i2cMutex);
	int ret = i2c_init();
	pthread_mutex_unlock(&m_i2cMutex);
	return ret;
}


