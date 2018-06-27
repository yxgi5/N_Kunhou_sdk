#ifndef _POS_LASER_MAP_WANGHONGTAO_2017_12_12_
#define _POS_LASER_MAP_WANGHONGTAO_2017_12_12_

#include <map>

#include "Comm.h"
#include "RobotStruct.h"
#include "buffer_con.hpp"
#include "Geometry.h"
#include "bag/bag.h"

#include "laser_filter/laser_filter.h"

//#define CLOSED_DIFF_XY (0.2)   //+-0.2m
#define CLOSED_DIFF_XY (1.5)   //+-1.5m
#define CLOSED_DIFF_ANGLE (0.28) //+-0.28rad = +-16.04deg

struct Point2s
{
	 Point2s()
	{
		x_ = 0 ;
		y_ = 0 ;
		a_ = 0;

		file_name_ = "tmp.bag";
	};
	 Point2s(double x , double y,double a ,std::string file_name)
    {
        x_ = x;
        y_ = y;
        a_ = a;

        file_name_ = file_name;
    };
	 Point2s(const Point2s &p2s)
    {
        this->x_ = p2s.x_ ;
        this->y_ = p2s.y_ ;
        this->a_ = p2s.a_ ;

        this->file_name_ = p2s.file_name_ ;
    };
    bool operator < (const Point2s &p2s) const
    {
        if ( fabs( this->x_ - p2s.x_) < CLOSED_DIFF_XY )
        {
        	if ( fabs( this->y_ - p2s.y_) < CLOSED_DIFF_XY ){

        		if ( fabs( VecPosition::normalizeAngleRad( this->a_ - p2s.a_ )) < CLOSED_DIFF_ANGLE ){
        			return false;
        		}else{
        			return this->a_ < p2s.a_ ;
        		}

        	}else{
        		return this->y_ < p2s.y_ ;
        	}
        }else{
        	return this->x_ < p2s.x_ ;
        }

    };

    double x_ ;
    double y_ ;
    double a_ ;

    std::string file_name_;
};


class pos_laser_map{

public:

	pos_laser_map();
	~pos_laser_map();


	void load_all();
	void save(const std::string file_name,const SBAG &bag);
	void del( const std::string file_name);


	void put(const SBAG &bag);
	int get( std::multimap< Point2s , SBAG> &res, const SPos &pos);

	void debug_print();
	SBAG debug_all_print();

private:
	laser_filter l_filter_;
	bool b_first_frame_;
	void load_bag( std::string path );
	void put( const std::string file_name,  const SBAG &bag);

	std::multimap< Point2s , SBAG> mp ;
	boost::mutex map_data_lock_;
public:
	SBAG ref_bag_;
	SBAG cur_bag_;
};



#endif//_POS_LASER_MAP_WANGHONGTAO_2017_12_12_
