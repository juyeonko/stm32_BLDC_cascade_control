#ifndef PID_CONTROL_TEST_H_
#define PID_CONTROL_TEST_H_

/* Part of PID Struct */

/** Long ���� PID ����� ������ ����ü */

typedef struct _LONGPID{
	double nowValue;		//!< ���� ��
	double pastValue;		//!< ���� ��
	
	double nowError;		//!< ���� ������
	double pastError;		//!< ���� ������
	double target;		//!< ��ǥ��
	
	double errorSum;		//!< ���� ������
	double errorSumLimit;	//!< ���� ������ ���� (0�� ��� ���� ����)
	double errorDiff;		//!< ���� �̺а�

	double nowOutput;		//!< ���� ��°�
	double pastOutput;	//!< ���� ��°�
	double outputLimit;	//!< ��� ���Ѱ�
	
	long underOfPoint;	//!< kP, kI, kD�� �������� ���� ������ (underOfPoint=1000, kP=1�̸� P�̵氪 = 0.001)
	
	long kP;			//!< P(���)�̵氪
	long kI;			//!< I(����)�̵氪
	long kD;			//!< D(�̺�)�̵氪

}LPID;

enum {pos, vel, cur};

/* Part of PID Function */

/** Long �̵氪�� ����ϴ� PID ����� �Լ� */
void PID_Control(LPID* dst, 	//!< ���� ��� PID ������ ����ü
		double target, 				//!< ��ǥ ��
		double input					//!< ���� ��
		);


/** Long �̵氪�� ����ϴ� PID ����ü �ʱ�ȭ �Լ� */
void PID_Control_Long_Initialize(LPID* dst , int select //!< ���� ��� PID ������ ����ü
		);


#endif /* PID_CONTROL_TEST_H_ */