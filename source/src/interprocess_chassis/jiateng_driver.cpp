#include "math.h"
#include "iostream"
#include "Comm.h"


//#include "SPort.h"
#include "TransferDevice.h"
#include "chassis/jiateng_driver.h"

#define LEN_MAX 1024

jiateng_driver::jiateng_driver():pTransDevice_(0),counts_(60),reduction_ratio_(4.3),one_wheel_counts_(0),first_odo_(true)
{
	port_ = "";
	one_wheel_counts_ = reduction_ratio_*counts_;

	init_customer_para();
}

jiateng_driver::~jiateng_driver()
{

}

void jiateng_driver::setPara( std::string name ,std::string value )
{

	if (name == "port")
	{
		port_ = value;
	}else if(name == "counts"){
		cComm::ConvertToNum(counts_,value);
		cComm::RangeIt(counts_,(U32)0,(U32)100000);
		std::cout<<"counts_:"<<counts_<<std::endl;
	}else if(name == "reduction"){
		cComm::ConvertToNum(reduction_ratio_,value);
		cComm::RangeIt(reduction_ratio_,(F32)0,(F32)200.0);
		std::cout<<"reduction_ratio_:"<<reduction_ratio_<<std::endl;
	}else{
		std::cout<<"err para:"<<name<<" value:"<<value<<std::endl;
	}

	one_wheel_counts_ = reduction_ratio_*counts_;
	std::cout<<"one_wheel_counts_:"<<one_wheel_counts_<<std::endl;
}

void jiateng_driver::setRPM( int id,F32 v )
{

	//set left driver speed
	if (id == 0)
	{
		left_speed_ = v ;
	}else{
		right_speed_ = v ;
	}
}
F32 jiateng_driver::getSpeed( int id )
{
	if (id == 0)
	{
		return c_left_;
	}else{
		return c_right_;
	}
}

F32 jiateng_driver::getDiffAngle( int id )
{
	
	F32 dangle = 0;

	dangle = GetDiffPos(id);
	dangle /= one_wheel_counts_;

	return dangle;

}

bool jiateng_driver::open_transfer_device()
{

	//sport_ = new SerialPort();

	cConnPara conn_para;
	conn_para.m_iType = cConnPara::SERIALPORT ;
	conn_para.setDeviceName(port_);
	conn_para.nBaud = 115200;     
	conn_para.chParity = 'N';
	conn_para.nDataBits = 8;  
	conn_para.nStopBits = 1;  

	pTransDevice_ = cTransferDevice::GetInstance(conn_para);
	U8 wait = 10;
	//if (sport_->Init(port_,115200))
	while(wait--)
	{
		if(pTransDevice_ && pTransDevice_->m_Status.IsOpen()){
			std::cout<<"open_transfer_device over!!!!!"<<std::endl;
			return true;
		}
		SLEEP(500);
	}
	return false;

}
bool jiateng_driver::init_driver(){

	memset((U8*)&download_cmd_,0,sizeof(DOWNLOAD));
	download_cmd_.head0_ = 0x5a;
	download_cmd_.head1_= 0xa5;
	download_cmd_.cmd_ = 0x01;
	download_cmd_.add_ = 0x00;
	download_cmd_.len_ = 0x0c;

	memset((U8*)&upload_cmd_,0,sizeof(UPLOAD));
	


	return true;
}
void jiateng_driver::close_driver()
{
	if (pTransDevice_)
	{
		pTransDevice_->Close();
		SDelete(pTransDevice_);
	}
}
/////////////////////////////////////////////////////////////


void jiateng_driver::GetSpeedLR(){

// 	c_left_ = left_current_v / reduction_ratio_;
// 	c_right_ = (-right_current_v) / reduction_ratio_;
	
	F64 dt = dt_.GetTime();
	dt /= 1000000;
	dt_.Begin();

	if(dt < 2.0){
//		c_left_ =  F32(deta_left_pos_) / one_wheel_counts_ / dt * 60;
//		c_right_ = F32(deta_right_pos_) / one_wheel_counts_ / dt * 60;
		c_left_ = left_speed_;
		c_right_ = -right_speed_;
		//std::cout<<"c_left_:"<<c_left_<<" c_right_:"<<c_right_<<std::endl;
	}
}
U16 jiateng_driver::CRC16(U8 *p,U32 len)
{
	U8 i;
	int j;

	U16 uiCRC = 0xffff;

	for(j=0;j<len;j++)
	{
		uiCRC ^= (*p);
		p++;
		for( i=8; i != 0; i--)
		{
			if( uiCRC & 1 ){
				uiCRC >>= 1;
				uiCRC ^= 0xa001;
			}
			else{
				uiCRC>>=1;
			}
		}
	}
	return(uiCRC);
}
bool jiateng_driver::SetSpeedLR( F32 left,F32 right )
{

	download_cmd_.left_wheel_rpm_ = left;
	download_cmd_.right_wheel_rpm_ = -right;
	
	//5A A5 01 00 0C  01 02 03 04  05 06 07 08  09 0A 0B 0C  57 72
	

	int struct_len = 19;
	U8 send_data[19] = {0};
	send_data[0] = download_cmd_.head0_;
	send_data[1] = download_cmd_.head1_;
	send_data[2] = download_cmd_.cmd_;
	send_data[3] = download_cmd_.add_;
	send_data[4] = download_cmd_.len_;

 	memcpy( (send_data + 5),&(download_cmd_.left_wheel_rpm_),4);
 	memcpy( (send_data + 9),&(download_cmd_.right_wheel_rpm_),4);
 	memcpy( (send_data + 13),&(download_cmd_.back_),4);
	
	U16 crc = CRC16( send_data,struct_len - 2 );
	download_cmd_.crc0_ = (crc & 0x00ff)>>0;
	download_cmd_.crc1_ = (crc & 0xff00)>>8;

	send_data[17] = download_cmd_.crc0_;
	send_data[18] = download_cmd_.crc1_;

	SendData(send_data,struct_len);

	unsigned char ch_read[100];
	int len = 0;
	int len_read = 19;
	ReadData(ch_read,len,len_read,50);

	if( len == len_read ) 
	{
		std::string str = cComm::ByteToHexString(ch_read,len);
		//std::cout<<"Read:"<<str<<std::endl;
		upload_cmd_.head0_ = ch_read[0];
		upload_cmd_.head1_ = ch_read[1];
		upload_cmd_.cmd_ = ch_read[2];
		upload_cmd_.add_ = ch_read[3];
		upload_cmd_.len_ = ch_read[4];

 		memcpy(&(upload_cmd_.left_wheel_counts_),(ch_read + 5),4);
 		memcpy(&(upload_cmd_.right_wheel_counts_),(ch_read + 9),4);
 		memcpy(&(upload_cmd_.back_),(ch_read + 13),4);
		upload_cmd_.crc0_ = ch_read[17];
		upload_cmd_.crc1_ = ch_read[18];

		if( (upload_cmd_.head0_ != 0x5a ) || (upload_cmd_.head1_ != 0xa5) || (upload_cmd_.cmd_ != 0x01))
		{
			std::cout<<"Error Upload data!  head0:"<<upload_cmd_.head0_ <<" upload_cmd_.head1_ :"<<upload_cmd_.head1_ <<" upload_cmd_.cmd_ :"<<upload_cmd_.cmd_ <<std::endl;
			return false;
		}
		if( (upload_cmd_.add_ != 0x00 ) || (upload_cmd_.len_ != 0x0c) ){
			std::cout<<"Error Upload data!  add_:"<<upload_cmd_.add_ <<" upload_cmd_.len_ :"<<upload_cmd_.len_ <<std::endl;
			return false;
		}
		U16 crc = CRC16( (U8*)&ch_read,len - 2 );

		if ( ( upload_cmd_.crc0_ != ((crc & 0x00ff)>>0) ) || ( upload_cmd_.crc1_ != ((crc & 0xff00)>>8) ) )
		{
			std::cout<<"Error Upload data!  crc0_:"<<int((crc & 0x00ff)>>0)<<" upload_cmd_.crc1_ :"<<int((crc & 0xff0)>>8) <<std::endl;
			return false;
		}
		
		left_pos_ = upload_cmd_.left_wheel_counts_;
		right_pos_ = upload_cmd_.right_wheel_counts_;


		return true;
	}else{
		std::cout<<"Error Upload data! len:"<<len<<" need:19"<<std::endl;
		return false;
	}

	

}

void jiateng_driver::SendData( U8* s_data,U16 len )
{
	if (pTransDevice_)
	{
//		std::stringstream ss;
//		ss<<"send data len:"<<len<<" data:"<<cComm::ByteToHexString(s_data,len);
//		std::cout<<ss.str()<<std::endl;

		pTransDevice_->WriteData(s_data,len);

	}
}

void jiateng_driver::ReadData( U8* r_data,int &len,int need,int timeout )
{
	if (pTransDevice_)
	{
		pTransDevice_->ReadData(r_data,len,need,timeout);

	}
}

void jiateng_driver::init_customer_para()
{
	left_speed_ = 0;
	right_speed_ = 0;
	
	left_pos_ = 0;
	last_left_pos_ = 0;
	deta_left_pos_ = 0;


	right_pos_= 0;		    //pos
	last_right_pos_ = 0;
	deta_right_pos_ = 0;


	c_left_ = 0;
	c_right_ = 0;

}

F32 jiateng_driver::GetDiffPos( U8 id )
{
	if (id == 0)
	{
		//std::cout<<"set left_speed_:"<<left_speed_<<" right_speed_:"<<right_speed_<<std::endl;
		SetSpeedLR(left_speed_,right_speed_);

		if (first_odo_)
		{

			last_left_pos_ = left_pos_;
			last_right_pos_ = right_pos_;
			if ((abs(last_left_pos_) > 0 ) && (abs(last_right_pos_) > 0))
			{
				first_odo_ = false;
				std::cout<<"first left_pos_:"<<left_pos_<<" first right_pos_:"<<right_pos_<<std::endl;
			}

			return 0;
		}

		deta_left_pos_ = (left_pos_ - last_left_pos_)  ;
		deta_right_pos_ = (right_pos_ - last_right_pos_);


//		std::cout<<"deta_left_pos_:"<<deta_left_pos_<<" deta_right_pos_:"<<deta_right_pos_<<std::endl;
//		std::cout<<"left_pos_:"<<left_pos_<<" right_pos_:"<<right_pos_<<std::endl;

		if(abs(deta_left_pos_) > 200000){
			std::cout<<"err deta_left_pos_:"<<deta_left_pos_<<" left_pos_:"<<left_pos_<<" last_left_pos_:"<<last_left_pos_<<std::endl;
		}
		if(abs(deta_right_pos_) > 200000){
			std::cout<<"err deta_right_pos_:"<<deta_right_pos_<<" right_pos_:"<<right_pos_<<" last_right_pos_:"<<last_right_pos_<<std::endl;
		}

		GetSpeedLR();

		last_left_pos_ = left_pos_;
		last_right_pos_ = right_pos_;


		return deta_left_pos_;

	}else{
		return deta_right_pos_;
	}









}


