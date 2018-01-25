#ifndef __TRAJETORY_H__
#define __TRAJETORY_H__

#include <vector>
#include <math.h>
#include "json.hpp"
#include "constants.hpp"
#include "utils.hpp"


using namespace std;
using json = nlohmann::json;

// return speed coef
double CalcSpeedCoeff(int viecle_sum)
{
  double sp_coeff = 1;

  if(viecle_sum  >= SPEED_REDUCTION_VIECLE_NUM)
      sp_coeff = 0.95;
  else
    sp_coeff = 1;

  return sp_coeff;
}

// calculate cost and select best line
int CalcCostAndSelectBestLane(json sensor_fusion, double cost_array[3], int lane, int prev_size, double distance[3], double car_s)
{
  vector<int> lane_candidate;
  int best_lane;

  if (lane == LEFT)
    lane_candidate = {LEFT,CENTER};
  else if (lane == CENTER)
    lane_candidate = {LEFT,CENTER,RIGHT};
  else
    lane_candidate = {CENTER,RIGHT};

  ////////////////////////////////
  int cnt = 0;
  double best_cost=100000;
  for (int check_lane: lane_candidate) {
    double cost = 0;

    double vx = sensor_fusion[check_lane][3];
    double vy = sensor_fusion[check_lane][4];
    double check_speed = sqrt(vx*vx+vy*vy);
    double check_car_s = sensor_fusion[check_lane][5];
    check_car_s+=((double)prev_size*.02*check_speed);

    // continuous cost of gap
    double gap = 0;
    gap = Normalise(fabs(30/(distance[check_lane]))) * GAP_COST_WEIGHT;

    // lane change cost
    double lane_ch=0;
    if (check_lane != lane)
      lane_ch = LANE_CHANGE_COST;//10;

    // nearest viecle gap
    auto ids = GetLaneViecleIDS(check_lane,sensor_fusion);
    double nearest_gap;
    nearest_gap = GetNearestGap(ids, sensor_fusion, .02*prev_size, car_s);

    // Discrete Gap Cost
    double near=0;
    double costLut[5]={50, 100, 200, 300, 400};

    if(nearest_gap < 5)
      near = costLut[4];
    else if(nearest_gap < 10)
      near = costLut[3];
    else if(nearest_gap < 20)
      near = costLut[2];
    else if(nearest_gap < 25)//25
      near = costLut[1];
    else if(nearest_gap < 50)
      near = costLut[0];

    cost = gap + lane_ch + near;
    cost_array[check_lane] = cost;

    if (cost < best_cost) {
      best_lane = check_lane;
      best_cost = cost;
    }
//    printf("Lane[%d] Cost:%f %f %f %f %f\n", check_lane, cost, gap, lane_ch, near, distance[check_lane]);

    if(cnt == 0)
      printf("COST DISPLAY:\n");
    printf("Lane[%d] Cost:%3.1f gap:%3.1f lane_ch:%3.1f near_cost(Discretion):%d\n", check_lane, cost, gap, lane_ch, (int)near);
    cnt++;

  }

  return best_lane;
}

// Finaly, judge best lane
void JudgementBestLane(int *lane, int *best_lane, bool *isLaneChange, int *lane_ch_cnt, int *frame, double ref_vel, double cost_array[])
{
  int curr_lane = *lane;
  int selected_lane = *best_lane;
  string lanepos[3] = {"LEFT", "CENTER","RIGHT"};
  string state_array[3] = {"KEEP", "LCH_PREPARING","CHANGING"};


  if(*lane != *best_lane && *isLaneChange == false){
    if(ref_vel > LANE_CHANGABLE_SPEED){
      (*lane_ch_cnt)++;

      // Lane change reliability
      if(*lane_ch_cnt > LANE_CHANGE_COUTINUOUS_COUNT && cost_array[*best_lane] < LANE_CHANGABLE_COST){
        *isLaneChange = true;
        *lane = *best_lane;
      }
    }
  }
  else{
    *best_lane = *lane;
    *lane_ch_cnt = 0;
  }

  int state;
  if(*lane_ch_cnt == 0)
    state = 0;
  else if(*isLaneChange == true)
    state = 2;
  else
    state = 1;


  printf("LANE SELECTION INFO.:\n");
  printf("Lane Change State: %s\n", state_array[state].c_str());
  printf("Lane Current. Selected(CNT:%d). Final: %s %s %s!!\n", *lane_ch_cnt, lanepos[curr_lane].c_str(), lanepos[selected_lane].c_str(), lanepos[*best_lane].c_str());


  // lane chage interval
  if(*frame == LANE_CHANGE_INTERVAL){
    *isLaneChange = false;
    *frame = 0;
  }

  // interval reset
  if(*isLaneChange == true)
    (*frame)++;
  ///////////////////////////////

  return;
}

// calculate acceleration according to velocity and gap, in addition  lane chage
double CalcAcceleration(bool too_close, int lane, int best_lane, double ref_vel, double max_vel, double closest_speed, double ego_dist, bool isLaneChange)
{
  double accVal;
  double Acceleration[2] = {BASE_ACCEL_VAL*0.5, BASE_ACCEL_VAL*3};
  double Deceleration[2] = {BASE_ACCEL_VAL*4, BASE_ACCEL_VAL*0.5};

  if (too_close) {
    // Speed up acceleration calculation
    double act=0;
    if(ego_dist > 0){
      if(ego_dist < 10)
        act = Deceleration[0];
      else if(ego_dist < 20)
        act = ((ego_dist - 10) * Deceleration[1] + (20 - ego_dist) * Deceleration[0])/10;
      else
        act = Deceleration[1];

      accVal = act;
    }

    // Speed-down acceleration calculation
    if (best_lane == lane && ref_vel > closest_speed){
      double diff = ref_vel - closest_speed;
      double tmp_a;
      if(diff > /*20*/40)
        tmp_a = Acceleration[1];
      else
        tmp_a = (diff * Acceleration[1] + (40 - diff) * Acceleration[0])/40;

      if(tmp_a < accVal)
        tmp_a = accVal;

      accVal = -tmp_a;
    }
  } else {
    if (ref_vel < max_vel)
    {
      double diff = max_vel - ref_vel;
      double tmp_a;
      if(diff > /*20*/40)
        tmp_a = Acceleration[1];
      else
        tmp_a = (diff * Acceleration[1] + (40 - diff) * Acceleration[0])/40;

      accVal = tmp_a;

    }
//    else{
//      accVal = -BASE_ACCEL_VAL;
//    }
  }

  if(isLaneChange == true)
    accVal -= 0.224;

  return accVal;
}

#endif //__TRAJETORY_H__
