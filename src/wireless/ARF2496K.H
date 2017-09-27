/**************************************************************
########���޹�˾
�ļ�����drv_ARF2498.h
�汾��1.0
���ߣ�����ΰ lee
ʱ�䣺2015.01.01
�޸���Ϣ����
**************************************************************/
#ifndef  __ARF2498_H
#define  __ARF2498_H

#define ARF2496K_CHANNEL  0x44  //ͨ����
#define DUIMA_CHANNEL  0x44  //ͨ����

#define HOST_ADDR 0x02  // RF������ַ
#define SLAVE_ADDR 0x01  // RF�ӻ���ַ

#define HOST_ID 0  // RF�������
#define REMOTE_ID 1  // RF�ӻ����
#define DOCK_ID 2  // RF�ӻ����
#define VWALL_ID 3  // RF�ӻ����

//���涨����Ҫ�õ��Ķ���
#define ARF2498_STMODE	 	0X00 	//оƬ�ڴ���ģʽ
#define ARF2498_RXMODE	 	0X80 	//оƬ�ڽ���ģʽ
#define ARF2498_TXMODE	 	0XA0   //оƬ�ڷ���ģʽ
#define ARF2498_TXACKMODE	 	0XC0   //оƬ�ڷ��ͣ�Ӧ��ģʽ
#define ARF2498_REUSETX 		0XD0   //���·���

#define ARF2498_CHANNEL  		0x44 	//ͨ����  2440 MHZ

#define RFREG_Rx_Chn  0x00
#define RFREG_State  0x01
#define RFREG_Int  0x02
#define RFREG_PWR  0x23
#define RF_IRQ_CLR  0x3f

#define  RF_STANDBY	0	// ����ģʽ
#define  RF_SLEEP		1	// ˯��ģʽ
#define  RF_SEND		2	// ����ģʽ
#define  RF_RECV		3	// ����ģʽ

typedef struct{
	u8 mode;			// ����ģʽ
	u8 resend_en;	// �ظ�����ʹ��
	u8 ask_en;		// �������Ƿ���������Ӧ��
}RF_DATA;

extern void  ReadDeviceAddress(void);
extern bool QueryARF2496KInceptSign(void);  //��ѯ����оƬ�Ƿ��յ����ݣ��յ��򷵻�TRUE�����򷵻�FALSE���������ݺ󽫱�־���
extern void ReadRF(u8 *readdata,u8 readnumber);//�ӻ����ж���RF���ݣ����Ϊ16�ֽڣ�
extern void SendRF(u8 *senddata,u8 sendnumber);//�������ݣ����Ϊ16�ֽ�
extern void SetPartnershipAddr(void);//����Ϊ��Ե�ַ
extern void DisableRFIncept(void);//оƬ�������״̬�����������߽���
extern void EnableRFIncept(void);//ʹ�����߽���
extern void InitARF2496k(void);//��ʼ������ģ��оƬ
extern void RFIrq(void);
extern void EnterRXMode(void);
extern void Set_rfchannel(u8 ch);
extern void SetDestinationAddress(void);	//����ΪĿ���ַ
extern void PartnershipRF(void); //���Ͷ�����Ϣ
extern void Syn_DockRF(void) ;

#endif