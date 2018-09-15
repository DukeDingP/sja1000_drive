//实现功能根据报文ID来判断是否接受信息
//报文ID为四个字节组成29位ID 整体右移3位

/****************** Include Files *************************/
#include "str.h"
#include "mcu.h"
#include "MCUIO.h"
#include "SJA1000RW.h"

#define SJA_BaseAdr   0X00     //基地址的选择与避免与mcu本身地址冲突



#define         REG_CONTROL       SJA_BaseAdr+0x00       //内部控制寄存器
#define         REG_COMMAND       SJA_BaseAdr+0x01       //命令寄存器
#define         REG_STATUS        SJA_BaseAdr+0x02       //状态寄存器
#define         REG_INTERRUPT     SJA_BaseAdr+0x03       //中断寄存器
#define         REG_INTENABLE     SJA_BaseAdr+0x04       //中断使能寄存器
#define         REG_BTR0          SJA_BaseAdr+0x06       //总线定时寄存器0
#define         REG_BTR1          SJA_BaseAdr+0x07       //总线定时寄存器1
#define         REG_OCR           SJA_BaseAdr+0x08       //输出控制寄存器
#define         REG_TEST          SJA_BaseAdr+0x09       //测试寄存器

#define         REG_RESVER1       SJA_BaseAdr+0x0A       //保留1
#define         REG_ARBITRATE     SJA_BaseAdr+0x0B       //仲裁丢失捕捉
#define         REG_ERRCATCH      SJA_BaseAdr+0x0C       //错误代码捕捉
#define         REG_ERRLIMIT      SJA_BaseAdr+0x0D       //错误报警限额

#define         REG_RXERR         SJA_BaseAdr+0x0E         //接收错误计数器
#define         REG_TXERR         SJA_BaseAdr+0x0F         //发送错误计数器

#define         REG_ACR1          SJA_BaseAdr+0x10       //验收代码寄存器
#define         REG_ACR2          SJA_BaseAdr+0x11       //验收代码寄存器
#define         REG_ACR3          SJA_BaseAdr+0x12       //验收代码寄存器
#define         REG_ACR4          SJA_BaseAdr+0x13       //验收代码寄存器
#define         REG_AMR1          SJA_BaseAdr+0x14       //验收屏蔽寄存器
#define         REG_AMR2          SJA_BaseAdr+0x15       //验收屏蔽寄存器
#define         REG_AMR3          SJA_BaseAdr+0x16       //验收屏蔽寄存器
#define         REG_AMR4          SJA_BaseAdr+0x17       //验收屏蔽寄存器

// 发送缓冲区寄存器
#define         REG_TXBuffer1     SJA_BaseAdr+0x10         //发送缓冲区1 包括标准 扩展 数据 远程 数据长度
#define         REG_TXBuffer2     SJA_BaseAdr+0x11         //发送缓冲区2  标准帧ID1 扩展帧ID1
#define         REG_TXBuffer3     SJA_BaseAdr+0x12         //发送缓冲区3  标准帧ID2 扩展帧ID2
#define         REG_TXBuffer4     SJA_BaseAdr+0x13         //发送缓冲区4  标准帧数据1 扩展帧ID3
#define         REG_TXBuffer5     SJA_BaseAdr+0x14         //发送缓冲区5  标准帧数据2 扩展帧ID4
#define         REG_TXBuffer6     SJA_BaseAdr+0x15         //发送缓冲区6  标准帧数据3 扩展帧数据1
#define         REG_TXBuffer7     SJA_BaseAdr+0x16         //发送缓冲区7	 标准帧数据4 扩展帧数据2
#define         REG_TXBuffer8     SJA_BaseAdr+0x17         //发送缓冲区8  标准帧数据5 扩展帧数据3
#define         REG_TXBuffer9     SJA_BaseAdr+0x18         //发送缓冲区9  标准帧数据6 扩展帧数据4  
#define         REG_TXBuffer10    SJA_BaseAdr+0x19         //发送缓冲区10  标准帧数据7 扩展帧数据5
#define         REG_TXBuffer11    SJA_BaseAdr+0x1A         //发送缓冲区11  标准帧数据8 扩展帧数据6
#define         REG_TXBuffer12    SJA_BaseAdr+0x1B         //发送缓冲区12  ********** 扩展帧数据7
#define         REG_TXBuffer13    SJA_BaseAdr+0x1C         //发送缓冲区13  ********** 扩展帧数据8

// 接收缓冲区寄存器
#define         REG_RXBuffer1       SJA_BaseAdr+0x10       //接收缓冲区1
#define         REG_RXBuffer2       SJA_BaseAdr+0x11       //接收缓冲区2
#define         REG_RXBuffer3       SJA_BaseAdr+0x12        //接收缓冲区3
#define         REG_RXBuffer4       SJA_BaseAdr+0x13       //接收缓冲区4
#define         REG_RXBuffer5       SJA_BaseAdr+0x14        //接收缓冲区5
#define         REG_RXBuffer6       SJA_BaseAdr+0x15         //接收缓冲区6
#define         REG_RXBuffer7       SJA_BaseAdr+0x16         //接收缓冲区7
#define         REG_RXBuffer8       SJA_BaseAdr+0x17         //接收缓冲区8
#define         REG_RXBuffer9       SJA_BaseAdr+0x18         //接收缓冲区9
#define         REG_RXBuffer10      SJA_BaseAdr+0x19        //接收缓冲区10
#define         REG_RXBuffer11      SJA_BaseAdr+0x1A        //接收缓冲区11
#define         REG_RXBuffer12      SJA_BaseAdr+0x1B        //接收缓冲区12
#define         REG_RXBuffer13      SJA_BaseAdr+0x1C        //接收缓冲区13

#define         REG_RXCOUNT       SJA_BaseAdr+0x1D         //RX报文计数器
#define         REG_RBSA          SJA_BaseAdr+0x1E         //接收缓冲区起始地址
#define         REG_CDR           SJA_BaseAdr+0x1F         //时钟分频寄存器



/*
功能说明:   CAN控制器SJA1000通讯波特率.SJA1000的晶振为必须为16MHZ*/

#define         BTR0_Rate_5k      0xBF          //5KBPS的预设值
#define         BTR1_Rate_5k      0xFF          //5KBPS的预设值

#define         BTR0_Rate_10k      0x31          //10KBPS的预设值
#define         BTR1_Rate_10k      0x1C          //10KBPS的预设值

#define         BTR0_Rate_20k      0x18          //20KBPS的预设值
#define         BTR1_Rate_20k      0x1C         //20KBPS的预设值

#define         BTR0_Rate_40k      0x87          //40KBPS的预设值
#define         BTR1_Rate_40k      0xFF          //40KBPS的预设值

#define         BTR0_Rate_50k      0x47          //50KBPS的预设值
#define         BTR1_Rate_50k      0x2F          //50KBPS的预设值

#define         BTR0_Rate_80k      0x83          //80KBPS的预设值
#define         BTR1_Rate_80k      0xFF          //80KBPS的预设值

#define         BTR0_Rate_100k     0x43          //100KBPS的预设值
#define         BTR1_Rate_100k     0x2f          //100KBPS的预设值

#define         BTR0_Rate_125k     0x03          //125KBPS的预设值
#define         BTR1_Rate_125k     0x1c          //125KBPS的预设值

#define         BTR0_Rate_200k     0x81          //200KBPS的预设值
#define         BTR1_Rate_200k     0xFA          //200KBPS的预设值

#define         BTR0_Rate_250k     0x01          //250KBPS的预设值
#define         BTR1_Rate_250k     0x1c          //250KBPS的预设值

#define         BTR0_Rate_400k     0x80          //400KBPS的预设值
#define         BTR1_Rate_400k     0xfa          //400KBPS的预设值

#define         BTR0_Rate_500k     0x00          //500KBPS的预设值
#define         BTR1_Rate_500k     0x1c          //500KBPS的预设值

#define         BTR0_Rate_666k     0x80          //666KBPS的预设值
#define         BTR1_Rate_666k     0xb6          //666KBPS的预设值

#define         BTR0_Rate_800k     0x00          //800KBPS的预设值
#define         BTR1_Rate_800k     0x16          //800KBPS的预设值

#define         BTR0_Rate_1000k    0x00          //1000KBPS的预设值
#define         BTR1_Rate_1000k    0x14          //1000KBPS的预设值
//BPS
#define         ByteRate_5k        5
#define         ByteRate_10k       10
#define         ByteRate_20k       20
#define         ByteRate_40k       40
#define         ByteRate_50k       50
#define         ByteRate_80k       80
#define         ByteRate_100k      100
#define         ByteRate_125k      125
#define         ByteRate_200k      200
#define         ByteRate_400k      400
#define         ByteRate_500k      500
#define         ByteRate_800k      800
#define         ByteRate_1000k     1000

void user_interrupt(void){}

unsigned char RECOK;
 
int i=0;

int ENTER_RSTMODE()
{
	char a,b;
	a=WR_SJA_REG(REG_CONTROL ,0x09);//进入复位模式，单滤波模式
	b=RD_SJA_REG(REG_CONTROL);
	puts(xtoa(b));
	putch('\n');
	a&=0x1;
	b&=0x1;
	if (a==b)
	{
		puts("reset mode single filter mode set ....ready");
		putch('\n');
		return 0;
	}
	else
	return 1;
}

int TEST_COMMUNI_REG()
{
	char a,b;
	a=WR_SJA_REG(REG_TEST ,0xAA);
	b=RD_SJA_REG(REG_TEST);
	puts(xtoa(b));
	putch('\n');
	if(a==b)
	{
		puts("read and write test ....ready");
		putch('\n');
		return 0;
	}
	else 
	return 1;
}

int SET_CLOCK_REG()
{
	char a,b;
	a=WR_SJA_REG(REG_CDR ,0Xc8); //Pelican模式 禁能外部时钟引脚 Intel模式  ????
	b=RD_SJA_REG(REG_CDR); 
	puts(xtoa(b));
	putch('\n');
	if(a==b)
	{
		puts("pelican intel mode set ....ready");
		putch('\n');
		return 0;
	}
	else 
	return 1;
}

/****************** 过滤器逻辑  *************************/
/*
CAN设备设置接收过滤器结构体
参数: IdMask，Mask
???? IdFilter，Filter
*参数说明:                                                              *
*    BCAN_ACR(0-3):存放验收代码寄存器（ACR）的参数设置                  *
*    BCAN_AMR(0-3):存放接收屏蔽寄存器（AMR）的参数设置                  *
*返回值:                                                                *
*           0 ;设置成功                                                 *
*           1 ;设置失败                                                 *
*说明:设置CAN节点的通讯对象，允许接收的报文,是由AMR和ACR共同决定的. 
是否接收数据按照如下规律:
Mask Filter RevID Receive
0???  x????  x?? ? yes
1???  0????  0???  yes
1???  0????  1???  no
1???  1????  0???  no
1???  1????  1???  yes
*/
/****************** 过滤器逻辑 *************************/


int SET_ACCEPT_FILTER(char BCAN_ACR0,char BCAN_ACR1,char BCAN_ACR2,char BCAN_ACR3,char BCAN_AMR0,char BCAN_AMR1,char BCAN_AMR2,char BCAN_AMR3)
{
	char a,b,c,d,e,f,g;
	WR_SJA_REG(REG_TXBuffer1,BCAN_ACR0);
	WR_SJA_REG(REG_TXBuffer2,BCAN_ACR1);
	WR_SJA_REG(REG_TXBuffer3,BCAN_ACR2);
	a=WR_SJA_REG(REG_TXBuffer4,BCAN_ACR3);
	b=RD_SJA_REG(REG_TXBuffer4); 
	e= RD_SJA_REG(REG_TXBuffer1);
	f= RD_SJA_REG(REG_TXBuffer2);
	g= RD_SJA_REG(REG_TXBuffer3);	
	puts(xtoa(e));
	putch('\n');
	puts(xtoa(f));
	putch('\n');
	puts(xtoa(g));
	putch('\n');
	puts(xtoa(b));
	putch('\n');
	if(a!=b)
	return 1;
	WR_SJA_REG(REG_TXBuffer5,BCAN_AMR0);
	WR_SJA_REG(REG_TXBuffer6,BCAN_AMR1);
	WR_SJA_REG(REG_TXBuffer7,BCAN_AMR2);
	c=WR_SJA_REG(REG_TXBuffer8,BCAN_AMR3);
	d=RD_SJA_REG(REG_TXBuffer8);
	a= RD_SJA_REG(REG_TXBuffer5);
	b= RD_SJA_REG(REG_TXBuffer6);
	e= RD_SJA_REG(REG_TXBuffer7);
	puts(xtoa(a));
	putch('\n');
	puts(xtoa(b));
	putch('\n');
	puts(xtoa(e));
	putch('\n');
	puts(xtoa(c));
	putch('\n');
	puts(xtoa(d));
	putch('\n');
	if(c!=d)
	return 1;
	puts("ACR AMR set ....ready");
	putch('\n');
	return 0;
	
}
/************************************************************************
;*函数原型:  bit BCAN_SET_BANDRATE(unsigned char CAN_ByteRate)          *
;*返回值:                                                               *
;*           0 ;波特率设置成功                                          *
;*           1 ;波特率设置失败                                          *
;*                                                                      * 
;*说明:设置CAN控制器SJA1000通讯波特率.SJA1000的晶振必须为16MHZ,         *
;*     其它晶体的频率的值的波特率，需自己计算 。该子程序只能用于        *
;*     复位模式                                                         *  
;************************************************************************/ 
int SET_BAUDRATE(unsigned char CAN_ByteRate)
{
	int a,b,c,d;
	char BR_Num= CAN_ByteRate;
	char BTR0_num;
	char BTR1_num;
	switch (BR_Num)
	 {
	   case ByteRate_5k:
			BTR0_num=0xBF;
			BTR1_num=0xFF;
			break;
	   case ByteRate_10k:
			BTR0_num=0x31;
			BTR1_num=0x1C;
			break;
	   case ByteRate_20k:
			BTR0_num=0x18;
			BTR1_num=0x1C;
			break;
	   case ByteRate_40k  :
			BTR0_num=0x87;
			BTR1_num=0xff;
			break;
	   case ByteRate_50k:
			//BTR0_num=0x47;
			//BTR1_num=0x2f;
			BTR0_num=0x0e;
			BTR1_num=0x1c;
			break;
	   case ByteRate_80k  :
			//BTR0_num=0x83;
			//BTR1_num=0xff;
			BTR0_num=0x49;                      //ok
			BTR1_num=0x1b;
			break;
	   case ByteRate_100k  :
			//BTR0_num=0x43;
			//BTR1_num=0x2f;
			BTR0_num=0x04;                       //ok
			BTR1_num=0x1C;
			break;
	   case ByteRate_125k  :                      //SJA1000的晶振为必须为16MHZ,波特率设置为125kpbs
			BTR0_num=0x04;
			BTR1_num=0x1c;
			//BTR0_num=0x44;                        //ok
			//BTR1_num=0x1f;
			break;
	   case ByteRate_200k  ://24MHZ
		 //   BTR0_num=0xc5;  //
		  //  BTR1_num=0xa5;
			BTR0_num=0x43;                        //ok
			BTR1_num=0x1b;
			break;
	   /* case ByteRate_200k  ://24MHZ
			BTR0_num=0x81;
			BTR1_num=0xFA;
			break;*/
	 /*  case ByteRate_400k  :
			BTR0_num=0x80;

			BTR1_num=0xfa;
			break;
	   case ByteRate_500k  :
			BTR0_num=0x01;
			BTR1_num=0x1c;
			break;
	   case ByteRate_800k  :
			BTR0_num=0x00;
			BTR1_num=0x16;
			break;
	   case ByteRate_1000k  :
			BTR0_num=0x00;
			BTR1_num=0x14;
			break;*/
	   default :
			return 1;
			break;
	 }
	 a=WR_SJA_REG(REG_BTR0 ,BTR0_num);
	 b=RD_SJA_REG(REG_BTR0);
	 
	 	if(a!=b)
	 return 1;
	 

	 c=WR_SJA_REG(REG_BTR1 ,BTR1_num);
	 d = RD_SJA_REG(REG_BTR1);
	
	 if(c==d)
	 { 
		puts("baudrate set .... ready");
	 	putch('\n');
	    return 0;
	}
	else 
	return 1;
	
}



/****************************************************
**函数原型：   bit Sja_1000_Init(void)
**功    能：   初始化SJA10000
**入口参数:    无 
**返 回 值:     
      0： 初始化成功 
      1： 复位失败 
      2:  测试sja1000失败 
      3： 设置失败 
      4： 设置验收滤波器失败
      5： 设置波特率失败     
*****************************************************/
int Sja1000_Init()
{
	int s;
	int a;
	int b;
	
	s=ENTER_RSTMODE();  //置位复位请求和单滤波模式
	if (s==1) 
	return 1;
	
	s=TEST_COMMUNI_REG(); //写入测试寄存器
	if (s==1) 
	return 2;
	
	s=SET_CLOCK_REG();//Pelican模式 禁能外部时钟引脚 Intel模式
	if (s==1)
	return 3;

	s=SET_ACCEPT_FILTER(0x10,0xff,0xff,0xff,0xff,0xff,0xff,0xff);//屏蔽寄存器，都设为无关，接收所有报文 
	//当屏蔽位为1，不滤波，屏蔽为位0就表示可以导通相同ID
	//s=BCAN_SET_OBJECT(0x55,0xe0,0xaa,0xa1,0x00,0x00,0xff,0xff);//标准帧格前两个数据字节来存放识别码 扩展帧全波识别码 手册P32 单个滤波器配置
	if (s==1) 
	return 4;
	
	s=SET_BAUDRATE(ByteRate_40k);
	if (s==1)
	return 5;

	WR_SJA_REG(REG_OCR,0x1A);  //输出控制寄存器--正常输出模式
	WR_SJA_REG(REG_INTENABLE,0x1D);//设置中断，接受和发送中断
	//唤醒中断使能 数据溢出中断使能 错误报警中断使能 发送中断使能 接收中断使能

	a=RD_SJA_REG(REG_CONTROL);
	a&=0xfe;
	WR_SJA_REG(REG_CONTROL,a);//退出复位模式
	a&=0x01;
	if(a==0)
	{
		b = RD_SJA_REG(REG_CONTROL);
		puts(xtoa(b));

		puts("quit the RESET MODE....READY");
		putch('\n');
		return 0;
	}
	return 6;

	

	

	//MemoryWrite(SYS_CTL0_REG,0X1);   //开总中断
	
	return 0;
}


void main(void)
{
	unsigned char ss,length,key,int0;
	unsigned char a,b,c,d,e,t,k;
	unsigned int i,f,g=0,h;


	//RT_GDR_BIT_ON(4);


	//MemoryWrite(SYS_CTL0_REG,0X0);//关总中断

	ss=Sja1000_Init();
	if(ss!=0)
	{
	
		puts("STEP ");
		puts(" ");
		puts(xtoa(ss));
		puts(" ");
		puts("FAIL");
		putch('\n');
		puts("Initial Fail");
	}

	else
		puts("Initial Success");
		putch('\n');

//MemoryOr32(SYS_CTL0_REG,0x1);//打开中断	
	while(1)
	{
		/*a=RD_SJA_REG(REG_STATUS);
		puts(xtoa(a));
		puts("1");
		putch('\n');*/
		a = RD_SJA_REG(REG_INTERRUPT);
		//puts(xtoa(a));
		//puts("2");
		//putch('\n');
		if(a&0x01)
		{
			b=RD_SJA_REG(REG_RXBuffer1);
			length=b&0x0f;
			if ((b & 0x40) != 0x40) //数据帧   = 为远程帧
			{
				h = 0;
				for (i = 0;i < 4;i++)
					{

						f = RD_SJA_REG(REG_RXBuffer5 - i);
						g = f << (8 * i);
						h += g;

					}
				h = h >> 3;
				puts("ID: ");
				puts(xtoa(h));
				putch('\n');
				for (i = 0;i < length;i++)   //数据 扩展帧从buffer6 标准帧从buffer4
					{
						puts("current: ");
						c = RD_SJA_REG(REG_RXBuffer6 + i);
						puts(xtoa(c));
						puts(" ");
					}
				putch('\n');

			}	
			 WR_SJA_REG(REG_COMMAND,0x04);//释放缓冲区*/			
		}
	
	
//判断输入input为高电平	
 // WR_SJA_REG(REG_COMMAND,0x04);//释放缓冲区*/

		key=RT_SEG_Read(0);
 
		if(key==0)
		{
			e=RD_SJA_REG(REG_STATUS);  	//delay10ms
			if((e&0x04)==0x04)//cpu可以向发送缓冲区写信息
			{
				 WR_SJA_REG(REG_TXBuffer1,0x1);
				 WR_SJA_REG(REG_TXBuffer2,0x0);
				 WR_SJA_REG(REG_TXBuffer3,0x1);
				 WR_SJA_REG(REG_TXBuffer4,0x1);
	 
				 puts("i am in ");

				WR_SJA_REG(REG_COMMAND,0x03);  //发送命令
			    if((e&0x20)==1){};
			 }
		 }

		if(key==0)
		{
			for(i=0;i<128;i++)
			{
				puts(xtoa(i));
				puts("---");
				f=RD_SJA_REG(SJA_BaseAdr+i);
				puts(xtoa(f));
				putch('\n');
			}	
		}
	}
}



	
		


