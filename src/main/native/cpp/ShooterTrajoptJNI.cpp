// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include <jni.h>
#include <wpi/array.h>
#include <wpi/jni_util.h>

#include "ShooterTrajopt.h"
#include "jni_md.h"

extern "C" {
/*
 * Class:     frc_robot_jni_ShooterTrajoptJNI
 * Method:    calculateTrajectory
 * Signature: ([D????)V
 */
JNIEXPORT void JNICALL
Java_frc_robot_jni_ShooterTrajoptJNI_calculateTrajectory
  (JNIEnv* env, jclass, jdoubleArray javatraj, double x, double y, double vel_x,
   double vel_y)
{
  wpi::array traj = calculate_trajectory(x, y, vel_x, vel_y);
  env->SetDoubleArrayRegion(javatraj, 0, 3, traj.data());
}
}  // extern "C"
