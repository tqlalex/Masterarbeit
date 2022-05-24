#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "std_msgs/String.h"

//https://www.cnblogs.com/Braveliu/p/12219521.html
//用在C与C++混合编写的代码中，意为：如果是在C++下编译的
#ifdef __cplusplus 
extern "C"{
         //指示编译器大括号内这部分代码按C语言（而不是C++）的方式进行编译
                //https://www.cnblogs.com/xiangtingshen/p/10980055.html
#endif

#define IMU_SERIAL   "/dev/ttyACM0"
#define BAUD         (115200)
#define BUF_SIZE     1024

#ifdef __cplusplus
}
#endif

int main(int argc, char** argv)
{
    
	int rev = 0;
    //初始化节点
	ros::init(argc, argv, "serial_imu");
    //声明节点句柄
	ros::NodeHandle n;
    //声明publisher
        ros::Publisher data_pub = n.advertise<std_msgs::String>("SerialData", 1000);

    //声明串口对象
	serial::Serial sp;

    ///设置串口属性
	sp.setPort(IMU_SERIAL);     //设置接口
	sp.setBaudrate(BAUD);       //设置波特率
    //创建并设置超时时间timeout  https://www.cnblogs.com/visionfeng/p/5614066.html
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
	sp.setTimeout(to);

	///打开串口
	try
	{
    
		sp.open();
	}
	catch(serial::IOException& e)
	{
    
		ROS_ERROR_STREAM("Unable to open port.");
		return -1;
	}

    //判断串口是否打开成功
	if(sp.isOpen())
	{
    
		//https://www.codeleading.com/article/67245272933/
		ROS_INFO_STREAM(IMU_SERIAL << " is opened.");
	}
	else
	{
    
		return -1;
	}

    // alarm超时则产生SIGALRM信号。
    // https://blog.csdn.net/qq_22863733/article/details/80349120
	//alarm(1);

    //希望发布信息的速度为50Hz，要和 loop_rate.sleep() 配合使用
    // https://blog.csdn.net/weixin_51060040/article/details/122238461
    ros::Rate loop_rate(500);

	while(ros::ok())
	{
    
        //获取缓冲区内的字节数
        	std_msgs::String msg;
        	
        	std::stringstream ss;
        	
        	//ss << "hello world";
        	
        	
        	
		size_t num = sp.available();
		//std::cout << "Anfangbuff: " << num << std::endl;
		if(num!=0)
		{
    
			uint8_t buffer[BUF_SIZE]; 
	
			if(num > BUF_SIZE)
				num = BUF_SIZE;

            //读出数据
			num = sp.read(buffer, num);

			
			//std::cout << num << std::endl;
			//std::cout << "++++++++++++++++++++++++++++++++++" << std::endl;
			for (int i = 0; i < num; i++)
			{
    
                   //16进制的方式打印到屏幕
				   
                   std::cout << i << ": "<< std::hex << (buffer[i] & 0xff) << " " ;
                   
                   ss << std::hex << (buffer[i] & 0xff) << " ";
                   //std::cout << (buffer[i]) << " ";
			}
			//std::cout << "++++++++++++++++++++++++++++++++++" << std::endl;

			//std::cout << std::endl;
			msg.data = ss.str();
			data_pub.publish(msg);
		}
		
		
		
		
		
		loop_rate.sleep();
	}
    //关闭串口
	sp.close();
 
	return 0;
}

