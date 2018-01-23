#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#include <vector>

using namespace std;

// Lane Info.
enum{
  LEFT=0,
  CENTER,
  RIGHT
};
const int LANE_WIDTH = 4;

const int SEARCH_CAR_FRONT = 30;
const int SEARCH_CAR_REAR = 10;
const int SPEED_REDUCTION_RANGE = 20;//20
const int SPEED_REDUCTION_VIECLE_NUM = 1;

const int CONTINUOUS_GAP_COST_RANGE = 50;

const int LANE_CHANGE_COST = 20;
const int GAP_COST_WEIGHT = 20;
const int LANE_CHANGABLE_SPEED = 30;
const int LANE_CHANGABLE_COST = 250;
const int LANE_CHANGE_COUTINUOUS_COUNT = 10;
const int LANE_CHANGE_INTERVAL = 50;

//const double MAX_VELOCITY = 49.75;
const double CONV_COEFF_MPH = 2.23694;
const double BASE_ACCEL_VAL = 0.224;
const double SPEED_COEFF = 0.95;



#endif //__CONSTANTS_H__
