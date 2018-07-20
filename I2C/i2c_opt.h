
#ifndef __I2C_OPT_H__
#define __I2C_OPT_H__


#define I2C_FILE_FD "/dev/i2c-2" 
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3000) /* 100 mseconds */

#define GPIO_MAX_BUF 64
#define GPIO_N_347	347
#define GPIO_N_348	348

#define GET_NUM_MAX 16
#define PROTOCOL_DATA_LEN 64

class CI2COpt : CHwOpt
{
public:
	CI2COpt(bool intFlag);
	~CI2COpt();
	static CI2COpt* instance();
	int DataCommuni(uchar *in_buf, uchar in_len, uchar *out_buf, uchar out_len);

private:
	static void *ThreadListen(void *arg);	
	bool Looping(){return m_threadLoop;}
	void setLooping(bool bLoop){m_threadLoop = bLoop;}	
	int i2c_init(void);  
	int gpio_export(uint gpio);
	int gpio_unexport(uint gpio);
	int gpio_set_dir(uint gpio, uint out_flag);
	int gpio_set_value(uint gpio, uint value);
	int gpio_get_value(uint gpio, uint *value);
	int gpio_set_edge(uint gpio, char *edge);
	int gpio_fd_open(uint gpio);
	int gpio_fd_close(int fd);
	void PrintBuf(uchar* data, uint len);	  
private:
	int i2c_fileFd;
	int gpio_Int1Fd;
	int gpio_Int2Fd;

	pthread_t  m_threadListen;
	bool m_threadLoop;

	//I2C ·¢ËÍ»¥³âËø
	pthread_mutex_t m_i2cMutex;
	pthread_mutexattr_t m_i2cMutexAttr;	
};



#endif

