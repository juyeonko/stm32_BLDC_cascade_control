#ifndef PID_CONTROL_TEST_H_
#define PID_CONTROL_TEST_H_

/* Part of PID Struct */

/** Long 지원 PID 제어기 데이터 구조체 */

typedef struct _LONGPID{
	double nowValue;		//!< 현재 값
	double pastValue;		//!< 이전 값
	
	double nowError;		//!< 현재 에러값
	double pastError;		//!< 이전 에러값
	double target;		//!< 목표값
	
	double errorSum;		//!< 에러 누적값
	double errorSumLimit;	//!< 에러 누적값 제한 (0일 경우 제한 없음)
	double errorDiff;		//!< 에러 미분값

	double nowOutput;		//!< 현재 출력값
	double pastOutput;	//!< 이전 출력값
	double outputLimit;	//!< 출력 제한값
	
	long underOfPoint;	//!< kP, kI, kD에 공통으로 들어가는 나눗셈 (underOfPoint=1000, kP=1이면 P이득값 = 0.001)
	
	long kP;			//!< P(비례)이득값
	long kI;			//!< I(적분)이득값
	long kD;			//!< D(미분)이득값

}LPID;

enum {pos, vel, cur};

/* Part of PID Function */

/** Long 이득값을 사용하는 PID 제어기 함수 */
void PID_Control(LPID* dst, 	//!< 제어 대상 PID 데이터 구조체
		double target, 				//!< 목표 값
		double input					//!< 현재 값
		);


/** Long 이득값을 사용하는 PID 구조체 초기화 함수 */
void PID_Control_Long_Initialize(LPID* dst , int select //!< 제어 대상 PID 데이터 구조체
		);


#endif /* PID_CONTROL_TEST_H_ */