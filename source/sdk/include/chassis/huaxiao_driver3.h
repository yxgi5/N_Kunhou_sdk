#ifndef _HUAXIAO_DRIVER3_CHENG_H_
#define _HUAXIAO_DRIVER3_CHENG_H_

#include "TimerDiff.h"
#include "chassis/driver_base.h"

class cTransferDevice;

class huaxiao_driver3 : Driver_base
{

public:
	huaxiao_driver3();
	~huaxiao_driver3();

protected:

private://virtual
	void setPara(std::string name ,std::string value);

	bool init_driver();
	void close_driver();
	void setRPM(int id,F32 v);
	//void setAngle(int id,F32 rad_angle);
	F32 GetDiffPos(U8 id );
	F32 getAngle( int id );
	bool open_transfer_device();

	F32 getSpeed(int id);
	F32 getDiffAngle(int id);

private:
	std::string port_;
	cTransferDevice* pTransDevice_;
	F32 reduction_ratio_;
	U32 counts_;

	U32 one_wheel_counts_;

	bool first_odo_;

	void init_customer_para();

	void GetSpeedLR();
	void SendData( U8* s_data,U16 len );
	void ReadData(  U8* r_data,int &len,const int &need  );

	void CreatSpeedL(S32 &SpeedL);
	void CreatSpeedR(S32 &SpeedR);
	void Send_Speed(void);
	void Rec_Counts(void);

	F32 left_speed_;
	S32 left_speed_1;
	S32 left_speed_2;
	S32 left_speed_3;
	S32 left_speed_4;
	F32 right_speed_;
	S32 right_speed_1;
	S32 right_speed_2;
	S32 right_speed_3;
	S32 right_speed_4;

	S32 left_pos_;
	S32 last_left_pos_;
	S32 deta_left_pos_;


	S32 right_pos_;
	S32 last_right_pos_;
	S32 deta_right_pos_;

	U8 send_data[19];
	U8 ch_read[23];

	cTimerDiff dt_;

	F32 c_left_;
	F32 c_right_;

};

//




#endif /* _HUAXIAO_DRIVER3_CHENG_H_ */
