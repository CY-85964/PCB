#include <reg52.h>
#include <math.h>
#include <stdio.h>
#include <INTRINS.H>
#include <string.h> 

#define SlaveAddress 0xA6 
#define TIMEWINDOW_MIN 40 //0.2s
#define TIMEWINDOW_MAX 400 //2s
#define REGULATION 4 //找到稳定规律所需的步数
#define INVALID 2 //失去稳定规律所需的步数

typedef unsigned char BYTE;

sbit SCL=P3^6;
sbit SDA=P3^7;

BYTE BUF[8]; 
unsigned char ADXL345_FLAG=0;
unsigned long int sampling_counter=0;
unsigned char idata bad_flag[3]={0,0,0};
unsigned int idata array0[3]={1,1,1};
unsigned int idata array1[3]={1,1,1};
unsigned int idata array2[3]={0,0,0};
unsigned int idata array3[3]={0,0,0};
unsigned int idata adresult[3]={0,0,0};
unsigned int idata max[3]={-1023,-1023,-1023};
unsigned int idata min[3]={1023,1023,1023};
unsigned long int idata dc[3]={0,0,0};
unsigned int idata vpp[3]={30,30,30};	
unsigned int idata precision[3]={5,5,5};	
unsigned long int idata old_fixed[3]={0,0,0};
unsigned long int idata new_fixed[3]={0,0,0};
unsigned long int STEPS=0; 
unsigned long int count=0; 
unsigned char count1=0;
unsigned char count2=0;
unsigned long int old_count=0;
unsigned long int new_count=0;

unsigned int Interval=0; //记录时间间隔数
unsigned char TempSteps=0; //计步缓存
unsigned char InvalidSteps=0; //无效步缓存  
unsigned char ReReg=2; //记录是否重新开始寻找规律
                                        // 2-新开始  
                                        // 1-已经开始，但还没有找到规律  
                                        // 0-已经找到规律  

void init_adxl345(void);
void single_write_adxl345(unsigned char reg_address,unsigned char reg_data); 
unsigned char single_read_adxl345(unsigned char reg_address);
void multi_read_adxl345(); 
void delay10us();
void adxl345_start(); 
void adxl345_stop(); 
void adxl345_sendack(bit ack); 
bit adxl345_recvack(); 
void adxl345_sendbyte(BYTE dat); 
BYTE adxl345_recvbyte(); 
void set52_baudrate(float frequency,long int baudrate);
void step_counter();
void TimeWindow();
void Timer1Init(void);
void Timer0Init(void);


/*******************************************************************************************************/
/*******************************************************************************************************/
//5us
void delay10us()		//@11.0592MHz
{
	unsigned char i;

	i = 2;
	while (--i);
}
/*******************************************************************************************************/
/*******************************************************************************************************/
//起始信号
void adxl345_start()
{
	SDA=1; //拉高数据线
	SCL=1; //拉高时钟线
	delay10us(); //延时
	SDA=0; //产生下降沿
	delay10us(); //延时
	SCL=0; //拉低时钟线
}
/*******************************************************************************************************/
/*******************************************************************************************************/
//终止信号
void adxl345_stop()
{
	SDA=0; //拉低数据线
	SCL=1; //拉高时钟线
	delay10us(); //延时
	SDA=1; //拉高数据线
	delay10us(); //延时
}
/*******************************************************************************************************/
/*******************************************************************************************************/
//发送应答信号 ack(0:ACK 1:NAK)
void adxl345_sendack(bit ack)
{
	SDA=ack; //写应答信号
	SCL=1; //拉高时钟线
	delay10us(); //延时
	SCL=0; //拉低时钟线
	delay10us(); //延时
}
/*******************************************************************************************************/
/*******************************************************************************************************/
//接收应答信号
bit adxl345_recvack()
{
	bit temp; 
	SCL=1; //拉高时钟线
	delay10us(); //延时
	temp=SDA; //读应答信号
	SCL=0; //拉低时钟线
	delay10us(); //延时
	return temp; //
}
/*******************************************************************************************************/
/*******************************************************************************************************/
//从IIC发送一个字节数据（本处向adxl345）
void adxl345_sendbyte(BYTE dat)
{
	BYTE i;
	for (i=0;i<8;i++) //8位计数器
	{
		dat <<= 1; //移出最高数据位
		SDA=CY; //读取移出的数据位
		SCL=1; //拉高时钟线
		delay10us(); //延时
		SCL=0; //拉低时钟线
		delay10us(); //延时 
	}
	adxl345_recvack(); //接收应答信号
}
/*******************************************************************************************************/
/*******************************************************************************************************/
//从IIC接受一个字节数据（本处单片机接收）
BYTE adxl345_recvbyte()
{
	BYTE i;
	BYTE dat=0;
	SDA=1; //拉高数据线
	for (i=0;i<8;i++) //8位计数器
	{
		dat <<= 1; 
		SCL=1; //拉高时钟线
		delay10us(); //延时 
		dat |=SDA;  //最低位置于SDA
		SCL=0; //拉低时钟线
		delay10us(); //延时
	}
	return dat;
}
/*******************************************************************************************************/
/*******************************************************************************************************/
//单字节写入（写入adxl345）
void single_write_adxl345(unsigned char reg_address,unsigned char reg_data)
{
	adxl345_start(); //起始信号
	adxl345_sendbyte(SlaveAddress); //发送设备地址+写信号
	adxl345_sendbyte(reg_address); //adxl345内部寄存器地址
	adxl345_sendbyte(reg_data); //adxl345内部寄存器数据
	adxl345_stop(); //停止信号
}
/*******************************************************************************************************/
/*******************************************************************************************************/
//单字节读取
unsigned char single_read_adxl345(unsigned char reg_address)
{
	unsigned char reg_data;
	adxl345_start();
	adxl345_sendbyte(SlaveAddress);
	adxl345_sendbyte(reg_address);
	adxl345_start();
	adxl345_sendbyte(SlaveAddress+1);
	reg_data=adxl345_recvbyte();
	adxl345_sendack(1);
	adxl345_stop();
	return reg_data;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
//连续读取adxl345地址为0x32~0x37的数据
void multi_read_adxl345(void)
{
	unsigned char i;
	adxl345_start(); //起始信号
	adxl345_sendbyte(SlaveAddress); //发送设备地址+写信号
	adxl345_sendbyte(0x32); //发送读取数据寄存器地址,从0x32开始
	adxl345_start(); //起始信号
	adxl345_sendbyte(SlaveAddress+1); //发送设备地址+读信号
	for (i=0;i<6;i++) //连续读取6个地址数据，存储在BUF中
	{
		BUF[i]=adxl345_recvbyte(); //BUF存储从adxl345对应寄存区地址接收的数据
		if (i==5)
		{
			adxl345_sendack(1); //adxl345给单片机发送，最后一个数据需要发NACK
		}
		else
		{
			adxl345_sendack(0); //回应ACK
		}
	}
	adxl345_stop(); //停止信号
}

/*******************************************************************************************************/
/*******************************************************************************************************/
//adxl345初始化
void init_adxl345()
{
	single_write_adxl345(0x31,0x0B);
	single_write_adxl345(0x2C,0x0B);
	single_write_adxl345(0x2D,0x08);
	single_write_adxl345(0x2E,0x80);
	single_write_adxl345(0x1E,0x00);
	single_write_adxl345(0x1F,0x00);
	single_write_adxl345(0x20,0x05);
}
/*******************************************************************************************************/
/*******************************************************************************************************/
//波特率
void set52_baudrate(float frequency,long int baudrate)
{	
	unsigned int itmp; 	
	unsigned char tlow,thigh;
	T2CON=0x30; 
	SCON=0x50;   
	PCON=0x00;   
	
	itmp=(int)(65536-(frequency*1000000)/(baudrate*32)); 	
	thigh=itmp/256; 	
	tlow=itmp%256; 	
	RCAP2H=thigh; 	
	RCAP2L=tlow; 	
	TH2=thigh; 	
	TL2=tlow; 	
	
	TR2 =1;     
	REN =1;	     	
	EA =1;       
	
	PS =1;       
}
/*******************************************************************************************************/
/*******************************************************************************************************/
//UART发送数据
void send_byte(unsigned long int byte)
{
	SBUF=byte;
	
	while(!TI);
	TI=0;
}

/*******************************************************************************************************/
/*******************************************************************************************************/
void Timer1Init(void)		//5毫秒@11.0592MHz
{
	TMOD &= 0x0F;		//
	TMOD |= 0x10;		//设置定时器模式
	TL1 = 0x00;		//设置定时初值
	TH1 = 0xEE;		//设置定时初值
	TF1 = 0;		//清除TF1标志
	ET1=1;
	TR1 = 1;		//定时器1开始GHB计时
}
void T1_time() interrupt 3
{
	TL1 = 0x00;		//设置定时初值
	TH1 = 0xEE;		//设置定时初值
	count2 ++;	
}

/*------------------------------------------------------------------------------------------------------------------------
*Name: 		step_counter()
*Function:实现Pedometer基本算法.
*Input:		void
*Output: 	void
*------------------------------------------------------------------------------------------------------------------------*/

/*******************************************************************************************************/
/*******************************************************************************************************/
void Timer0Init(void)		//5毫秒@11.0592MHz
{
	TMOD &= 0xF0;	 //设置定时器模式
	TMOD |= 0x01;	 //设置定时器模式
	TL0 = 0x00;		 //设置定时初值
	TH0 = 0xEE;		 //设置定时初值
	EA=1;          //打开总中断
	TF0 = 0;		   //清除TF0标志
	ET0=1;         //开定时器0的中断
	TR0 = 1;		   //定时器0开始计时
}

/*******************************************************************************************************/
/*******************************************************************************************************/
//时间窗口
void TimeWindow()
{
	old_count=new_count;
	new_count=count;
	Interval=new_count-old_count;
	
	if (ReReg==2) //如果是新开始的第一步，直接在计步缓存中加1
	{
		TempSteps++;
		ReReg=1;
		InvalidSteps=0;
	}

	else //如果不是新开始的第一步
	{
		if((Interval>=TIMEWINDOW_MIN) &&(Interval<=TIMEWINDOW_MAX)) //时间间隔在有效时间窗口内
		{
			InvalidSteps=0;
			if(ReReg==1) //如果还没有找到规律
			{
				TempSteps++; //计步缓存加1
				if (TempSteps>=REGULATION) //如果计步缓存达到所要求的规律数
				{
					ReReg=0; //已经找到规律标志
					STEPS=STEPS+TempSteps;//更新显示
					TempSteps=0;
				}
			}
			else if (ReReg==0)
			{
				STEPS++;
				TempSteps=0;
			}
		}
		else if(Interval<TIMEWINDOW_MIN) //时间间隔小于有效时间窗口下限
		{
			if (ReReg==0) //如果已经找到规律
			{
				if(InvalidSteps<255)
				{
					InvalidSteps++; //无效步缓存加1
				}
				if (InvalidSteps>=INVALID) //如果无效步达到要求数值，则重新寻找规律
				{
					InvalidSteps=0;
					ReReg=1;
					TempSteps=1;
				}
			}
			else if (ReReg==1) //如果还没有找到规律，则之前的寻找规律过程无效，重新寻找规律
			{
				InvalidSteps=0;
				ReReg=1;
				TempSteps=1;
			}
		}
		else if(Interval>TIMEWINDOW_MAX) //如果时间间隔大于时间窗口上限，计步已间断，重新寻找规律
		{
			InvalidSteps=0;
			ReReg=1;
			TempSteps=1;
		}
	}
}

/*******************************************************************************************************/
/*******************************************************************************************************/
//计步
void step_counter(void)
{
	
	unsigned char jtemp;
	unsigned char temp;
	bit i=1;
	
	//----------------------------------采样滤波-----------------------//	
	for(jtemp=0;jtemp<=2;jtemp++)//jtemp取0、1、2分别代表x、y、z轴
	{
		array3[jtemp]=array2[jtemp];
		array2[jtemp]=array1[jtemp];
		array1[jtemp]=array0[jtemp];
		array0[jtemp]=((BUF[2*jtemp]+(BUF[2*jtemp+1]<<8))<<4)>>4;
		temp=BUF[2*jtemp]+(BUF[2*jtemp+1]<<8)<<1;
		if (CY==1)
		{
			array0[jtemp]=-array0[jtemp];
		}
		
		adresult[jtemp]=array0[jtemp]+array1[jtemp]+array2[jtemp]+array3[jtemp];
		adresult[jtemp]=adresult[jtemp]>>2;
		
		if (adresult[jtemp]>max[jtemp])
		{
			max[jtemp]=adresult[jtemp];
		}
		if (adresult[jtemp]<min[jtemp])
		{
			min[jtemp]=adresult[jtemp];
		}
	}
	//--------------------------线性位移寄存器-------------------------------------
	for(jtemp=0;jtemp<=2;jtemp++)
	{
		old_fixed[jtemp]=new_fixed[jtemp];

		if(adresult[jtemp]>=new_fixed[jtemp])                         
		{   
			if((adresult[jtemp]-new_fixed[jtemp])>=precision[jtemp])
			{
				new_fixed[jtemp]=adresult[jtemp];
			} 
		}
		else if(adresult[jtemp]<new_fixed[jtemp])
		{   
			if((new_fixed[jtemp]-adresult[jtemp])>=precision[jtemp])
			{
				new_fixed[jtemp]=adresult[jtemp];
			}  
		}
	}
	//------------------------- 动态门限决判----------------------------------
	if((vpp[0]>=vpp[1])&&(vpp[0]>=vpp[2]))   //x轴最活跃
	{
		if((old_fixed[0]>=dc[0])&&(new_fixed[0]<dc[0])&&(bad_flag[0]==0))        
		{
			TimeWindow();
		}
	}
	else if((vpp[1]>=vpp[0])&&(vpp[1]>=vpp[2]))  //y轴最活跃
	{
		if(((old_fixed[1]>=dc[1])&&(new_fixed[1]<dc[1])&&(bad_flag[1]==0))||((old_fixed[1]<=dc[1])&&(new_fixed[1]>dc[1])&&(bad_flag[1]==0)))          
		{
			TimeWindow();
		}
	}
	else if((vpp[2]>=vpp[1])&&(vpp[2]>=vpp[0]))    //z轴最活跃
	{
		if((old_fixed[2]>=dc[2])&&(new_fixed[2]<dc[2])&&(bad_flag[2]==0))        
		{
			TimeWindow();
		}
	}	
	sampling_counter=sampling_counter+1;

	//----------------------------------计算动态门限和动态精度-----------------------//
	if(sampling_counter>=50)
	{                
		sampling_counter=0;			
		for(jtemp=0;jtemp<=2;jtemp++)
		{
			vpp[jtemp]=max[jtemp]-min[jtemp];
			dc[jtemp] =(min[jtemp]+max[jtemp])/2;    //dc为阈值
			max[jtemp]=-1023;
			min[jtemp]=1023;
			bad_flag[jtemp]=0;
			
			if(vpp[jtemp]>=160)
			{
				precision[jtemp]=vpp[jtemp]/32; 
			}
			else if((vpp[jtemp]>=50)&& (vpp[jtemp]<160))            
			{
				precision[jtemp]=4;
			}
			else if((vpp[jtemp]>=15) && (vpp[jtemp]<50))  
			{
				precision[jtemp]=3;
			}  			
			else
			{ 
				precision[jtemp]=2;
				bad_flag[jtemp]=1;
			}
		}
	}		
}
/*******************************************************************************************************/
/*******************************************************************************************************/
//T0中断函数
void T0_time() interrupt 1
{
	TL0 = 0x00;		
	TH0 = 0xEE;
	TF0 = 0;
	count++;	
	count1++;
	if (count>17280000)
	{
		count=0;
	}
}
/*******************************************************************************************************/
/*******************************************************************************************************/
//主函数
void main()
{
	set52_baudrate(11.0592,9600);
	init_adxl345();
	Timer0Init();
	ES=0;	
	while(1)
	{
		Timer1Init();
		multi_read_adxl345();
		step_counter();
		send_byte(STEPS);
		while(count2!=4);
		count2=0;
	}
}




