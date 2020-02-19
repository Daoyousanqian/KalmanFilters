#include "Kalman.h"
/**
 *@function: - 
 *@kalmanFilter  structure
 *@init_x initial state
 *@init_p posteriori estimate covariance intial value
 */
void kalmanFilter_init(KalmanStructTypedef *kalmanFilter, float init_x, float init_p,float predict_q,float newMeasured_q)
{
    kalmanFilter->x = init_x;// initial state
    kalmanFilter->p = init_p;// posteriori estimate covariance intial value
    kalmanFilter->A = 1;
    kalmanFilter->H = 1;
    kalmanFilter->q = predict_q;// predict noise deviation
    kalmanFilter->r = newMeasured_q;// measure noise deviation
}

/**
 *@function: - 卡尔曼滤波器
 *@kalmanFilter:卡尔曼结构体
 *@newMeasured；测量值
 *返回滤波后的值
 */
float kalmanFilter_filter(KalmanStructTypedef *kalmanFilter, float newMeasured)
{
    /* Predict */
    kalmanFilter->x = kalmanFilter->A * kalmanFilter->x;//%x priori state updated by the posteriori of last time
    kalmanFilter->p = kalmanFilter->A * kalmanFilter->A * kalmanFilter->p + kalmanFilter->q;  /*计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Correct */
    kalmanFilter->gain = kalmanFilter->p * kalmanFilter->H / (kalmanFilter->p * kalmanFilter->H * kalmanFilter->H + kalmanFilter->r);
    kalmanFilter->x = kalmanFilter->x + kalmanFilter->gain * (newMeasured - kalmanFilter->H * kalmanFilter->x);//利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出
    kalmanFilter->p = (1 - kalmanFilter->gain * kalmanFilter->H) * kalmanFilter->p;//%计算后验均方差

    return kalmanFilter->x;//得到现时刻的最优估计
}

void main(){   // add main function 
	
	
	
	
	
	
	
}
