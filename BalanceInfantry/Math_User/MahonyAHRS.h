//
// Created by 45441 on 2023/7/6.
//

#ifndef INFANTRYGIMBALC_MAHONYAHRS_H
#define INFANTRYGIMBALC_MAHONYAHRS_H
#ifdef __cplusplus
extern "C" {
#endif
//C
extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
#ifdef __cplusplus
}
#endif
//C++

#endif //INFANTRYGIMBALC_MAHONYAHRS_H
