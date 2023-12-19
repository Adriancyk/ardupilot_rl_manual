// mode_rl_zigzag.cpp
// This file contains the main code for running RL_ZigZag with a Jetson TX2 as the upper computer.
// Copyright 2023 Fanxin Wang, all rights reserved.

// Dr. Fanxin Wang, 2023-2024
// Email: fanxinw2@illinois.edu
// Advanced Controls Research Laboratory
// Department of Mechanical Science and Engineering
// University of Illinois Urbana-Champaign
// Champaign, IL 61820, USA

#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Motors/AP_Motors_Class.h> // for sending motor speed
// #include "ACRL_trajectories.h"         // small libraries for trajectories at ACRL

#if MODE_RL_ZIGZAG_ENABLED == ENABLED

/*
 * Init and run calls for Jetson TX2 flight mode (copied from adaptive)
 */

// Init function: this function will be called everytime the FC enters the JetsonTX2 mode.
bool ModeRL_ZigZag::init(bool ignore_checks)
{
    fsm.cur_state = DISARMED;
    fsm.prev_state = STATES_NULL;
    fsm.inputsigs.land_r = 0;
    
    initTimeOffset = AP_HAL::micros() * 0.000001f; // initTimeOffset has the unit of second

    gcs().send_text(MAV_SEVERITY_INFO, "TX2 mode initialization is done.");
    return true;
}

void ModeRL_ZigZag::run()
{
    // static uint32_t initialTime = 0; // store previous system time

    if (!motors->armed())
    {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }
    else if (copter.ap.throttle_zero)
    {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }
    else
    {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state())
    {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower)
        {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // ===================================================
    // start custom code by ACRL

    // initiate the trajectory variables
    static Vector3f targetPos = {0,0,0};
    static Vector3f targetVel = {0,0,0};
    static Vector3f targetAcc = {0,0,0};
    
    static Vector3f targetRPY = {0,0,0};
    static Vector3f targetRPY_dot = {0,0,0};
    static Vector3f targetRPY_ddot = {0,0,0};

    // load current time
    float now = AP_HAL::micros() * 0.000001f - initTimeOffset;

    determine_states(now); // change current state and previous state based on input signals

    // when state changes, do the corresponding tasks and release ready signal on finish
    // when state does not change, keep track of time and fill in command vectors (pos, vel, acc, jerk, snap, yaw, yaw_rate, yaw_acc)
    
    switch(fsm.cur_state){
        case ARMED:
            if(fsm.cur_state != fsm.prev_state){
                gcs().send_text(MAV_SEVERITY_INFO, "Current State: ARMED\n");
                

                copter.mode_RL_zigzag.fsm.target_duration = g.disarm_delay; // the disarm delay is the same as system configures
                // log state and entrance time
                AP::logger().Write("L1AS","state,time","Bf",
                                    (uint8_t)ARMED,
                                    (double)now);
            }
            else
            {
                // manually load the arm motor cmd to replace the comment-outed arducopter's own motor control in copter.cpp
                if (motors->armed()) // only command the motor PWM when the vehicle is armed.
                {
                    motors->rc_write(0, 1000 + 100); // manual set motor speed to 1100 for arming
                    motors->rc_write(1, 1000 + 100); 
                    motors->rc_write(2, 1000 + 100);
                    motors->rc_write(3, 1000 + 100);
                }
            }           
            break;
        case DISARMED:
            if(fsm.cur_state != fsm.prev_state){
                gcs().send_text(MAV_SEVERITY_INFO, "Current State: DISARMED\n");
                copter.mode_RL_zigzag.fsm.begin = now;
                gcs().send_text(MAV_SEVERITY_INFO, "Current state start time is %.3f\n",copter.mode_RL_zigzag.fsm.begin);

                // log state and entrance time
                AP::logger().Write("L1AS","state,time","Bf",
                                    (uint8_t)DISARMED,
                                    (double)now);
            }
            else
            {
                // manually command 0 speed for the rotors
                if (!motors->armed()) // only command the motor PWM when the vehicle is armed.
                {
                    motors->rc_write(0, 1000); // manual set motor speed to 1000 for disarm
                    motors->rc_write(1, 1000); 
                    motors->rc_write(2, 1000);
                    motors->rc_write(3, 1000);
                }
            }
            break;
        case TAKEOFF:
            if(fsm.cur_state != fsm.prev_state){
                // first time entering TAKEOFF
                gcs().send_text(MAV_SEVERITY_INFO, "Current State: TAKEOFF\n");
                printf("Height: %f\n", copter.mode_RL_zigzag.fsm.params[0]);
                copter.mode_RL_zigzag.fsm.begin = now;
                gcs().send_text(MAV_SEVERITY_INFO, "Current state start time is %.3f\n",copter.mode_RL_zigzag.fsm.begin);

                // save the duration of takeoff, which doubles the takeoff height
                copter.mode_RL_zigzag.fsm.target_duration = 2 * fsm.params[0]; 
                // save the current position when entering the takeoff state
                int locAvailable = ahrs.get_relative_position_NED_origin(currentPosition); // save current position
                if(locAvailable)
                {
                    gcs().send_text(MAV_SEVERITY_INFO, "Current pose is %.3f %.3f %.3f\n",currentPosition[0],currentPosition[1],currentPosition[2]);
                }
                // ahrs.get_velocity_NED(currentVelocity);
                // save current yaw
                currentRPY[2] = ahrs.get_yaw(); 
                currentYaw = currentRPY[2];

                gcs().send_text(MAV_SEVERITY_INFO, "Taking off to %.3f m in %.2f s\n",fsm.params[0], copter.mode_RL_zigzag.fsm.target_duration);
                
                // log state and entrance time
                AP::logger().Write("L1AS","state,time","Bf",
                                    (uint8_t)TAKEOFF,
                                    (double)now);
            }
            else // continuing with TAKEOFF procedure
            {
                // Vector3f takeOffDestination = {currentPosition[0],currentPosition[1],-fsm.params[0]}; // the third element should be in the down direction
                // Vector2f takeOffYawVec = {cosf(currentYaw),sinf(currentYaw)};

                float timeInThisRun = now - fsm.begin;
                int thrust_p_takeoff = int((copter.mode_RL_zigzag.fsm.target_duration*1000000.0-timeInThisRun)*0.00001); //5s takeoff, max thrust = 5*10^6*0.00001
                motors->rc_write(0, 1040+thrust_p_takeoff); // manual set motor speed to 1000 for disarm
                motors->rc_write(1, 1040+thrust_p_takeoff); 
                motors->rc_write(2, 1040+thrust_p_takeoff);
                motors->rc_write(3, 1040+thrust_p_takeoff);
                int locAvailable = ahrs.get_relative_position_NED_origin(currentPosition); // save current position
                if(locAvailable)
                {
                    gcs().send_text(MAV_SEVERITY_INFO, "Current pose is %.3f %.3f %.3f\n",currentPosition[0],currentPosition[1],currentPosition[2]);
                }
                currentRPY[2] = ahrs.get_yaw(); 
                currentYaw = currentRPY[2];
                gcs().send_text(MAV_SEVERITY_INFO, "Current State: TAKEOFF\n");
                gcs().send_text(MAV_SEVERITY_INFO, "Current height %.3f m \n",currentPosition[1]);
                
                // log state and entrance time
                AP::logger().Write("L1AS","state,time","Bf",
                                    (uint8_t)TAKEOFF,
                                    (double)now);
                
            }
            break;
        case HOVER:
            if(fsm.cur_state != fsm.prev_state){
                // first time entering HOVER
                gcs().send_text(MAV_SEVERITY_INFO, "Current State: HOVER\n");
                copter.mode_RL_zigzag.fsm.begin = now;
                gcs().send_text(MAV_SEVERITY_INFO, "Current state start time is %.3f\n",copter.mode_RL_zigzag.fsm.begin);
                // save the duration of hover, which is set to arbitrarily large.
                copter.mode_RL_zigzag.fsm.target_duration = 10000; // seconds

                // save the current position when entering the hover state
                int locAvailable = ahrs.get_relative_position_NED_origin(currentPosition); // save current position
                if(locAvailable)
                {
                    gcs().send_text(MAV_SEVERITY_INFO, "Current pose is %.3f %.3f %.3f\n",currentPosition[0],currentPosition[1],currentPosition[2]);
                }
                // save current yaw
                currentRPY[2] = ahrs.get_yaw(); 
                currentYaw = currentRPY[2];

                // log state and entrance time
                AP::logger().Write("L1AS","state,time","Bf",
                                    (uint8_t)HOVER,
                                    (double)now);
            }
            else
            {
                // continuing with hover state
                targetPos = currentPosition;
                targetVel = (Vector3f) {0,0,0};
                targetAcc = (Vector3f) {0,0,0};
                targetRPY = (Vector3f) {0,0,currentYaw};
                targetRPY_dot = (Vector3f) {0,0,0};
                targetRPY_ddot = (Vector3f) {0,0,0};
            }
            break;
        case LAND:
            if(fsm.cur_state != fsm.prev_state){
                gcs().send_text(MAV_SEVERITY_INFO, "Current State: LAND\n");
                copter.mode_RL_zigzag.fsm.begin = now;
                gcs().send_text(MAV_SEVERITY_INFO, "Current state start time is %.3f\n",copter.mode_RL_zigzag.fsm.begin);

                // save the current position when entering the land state
                int locAvailable = ahrs.get_relative_position_NED_origin(currentPosition); // save current position
                if(locAvailable)
                {
                    gcs().send_text(MAV_SEVERITY_INFO, "Current pose is %.3f %.3f %.3f\n",currentPosition[0],currentPosition[1],currentPosition[2]);
                }
                int velAvailable = ahrs.get_velocity_NED(currentVelocity); // save current velocity
                
                if(velAvailable)
                {
                    gcs().send_text(MAV_SEVERITY_INFO, "Current velocity is %.3f %.3f %.3f\n",currentVelocity[0],currentVelocity[1],currentVelocity[2]);
                }
                
                currentRPY[2] = ahrs.get_yaw(); 
                currentYaw = currentRPY[2];

                // log state and entrance time
                AP::logger().Write("L1AS","state,time","Bf",
                                    (uint8_t)LAND,
                                    (double)now);
            }
            else // continuing with LAND procedure
            {
                float timeInThisRun = now - fsm.begin;
            
                if (currentPosition[2] >= -0.3) // if the initial altitude upon entering land mode is within 30 cm, then set landComplete to 1 to overwrite the motor throttle to 1.
                {   
                    if (!fsm.inputsigs.land_r)
                    {
                        fsm.inputsigs.land_r = 1;
                        gcs().send_text(MAV_SEVERITY_INFO, "Quadrotor is on the ground. Motor commands set to minimum.");
                    }       
                }
                else 
                {
                    int thrust_p_landing = int((copter.mode_RL_zigzag.fsm.target_duration*1000000.0-timeInThisRun)*0.00001); //5landing p control
                    motors->rc_write(0, 1050-thrust_p_landing); // manual set motor speed to 1000 for disarm
                    motors->rc_write(1, 1050-thrust_p_landing); 
                    motors->rc_write(2, 1050-thrust_p_landing);
                    motors->rc_write(3, 1050-thrust_p_landing);
                    gcs().send_text(MAV_SEVERITY_INFO, "Quadrotor is landing. Motor throttle decreasing.");
                } 
            }
            break;
        case TRAJECTORY:
            if(fsm.cur_state != fsm.prev_state){
                gcs().send_text(MAV_SEVERITY_INFO, "Current State: TRAJECTORY\n");              
                
                copter.mode_RL_zigzag.fsm.begin = now;
                gcs().send_text(MAV_SEVERITY_INFO, "Current state start time is %.3f\n",copter.mode_RL_zigzag.fsm.begin);
                copter.mode_RL_zigzag.fsm.target_duration = 10000; // seconds
                // log state and entrance time
                AP::logger().Write("L1AS","state,time","Bf",
                                    (uint8_t)TRAJECTORY,
                                    (double)now);
            }
            else{
                float timeInThisRun = now - fsm.begin;     
                
                // automatically switch to hover if duration is longer than commanded
                if(!refuseTraj){
                    currentYaw = ahrs.get_yaw(); // save current yaw
                    gcs().send_text(MAV_SEVERITY_INFO, "Quadrotor is following cart.");
                    //gcs receive vicon
                    //ardupilot process ekf
                    //gcs send x,xd,xdd,rpy,rpyd,rpydd
                    //gcs receive u1,u2,u3,u4
                    if(timeInThisRun >= fsm.target_duration ){
                        fsm.inputsigs.trajectory_r = 1;
                        gcs().send_text(MAV_SEVERITY_INFO, "Trajectory finished. Switching to hover.");
                    }
                    motors->rc_write(0, 1050); // manual set motor speed to 1000 for disarm
                    motors->rc_write(1, 1050); 
                    motors->rc_write(2, 1050);
                    motors->rc_write(3, 1050);

                    //gcs send u1,u2,u3,u4                             
                }
            }

            break;
        case LOAD_TRAJECTORY:
            // haven't decided yet
            break;
        case STATES_NULL: 
            break;
    }

    float duration = now - fsm.begin;
    // printf("duration %f\n", duration);
    // temperary code to imitate real callback from controls
    if(duration >= fsm.target_duration){
           // gcs().send_text(MAV_SEVERITY_INFO, "Time: %f\n", ((float)(clock() - fsm.begin))/CLOCKS_PER_SEC);
        switch(fsm.cur_state){
            case ARMED:
                break;
            case DISARMED:
                break;
            case TAKEOFF:
                fsm.inputsigs.takeoff_r = 1;
                break;
            case HOVER:
                break;
            case LAND:
                fsm.inputsigs.land_r = 1;
                break;
            case TRAJECTORY:
                fsm.inputsigs.trajectory_r = 1;
                break;
            case LOAD_TRAJECTORY:
                break;
            case STATES_NULL: 
                break;
        }
    }

    // only applies the controllers when the vechile is in the following states: TAKEOFF, HOVER, LAND, TRAJECTORY, LOAD_TRAJECTORY.
    if ((fsm.cur_state==TAKEOFF) || (fsm.cur_state==HOVER) || (fsm.cur_state==LAND) || (fsm.cur_state==TRAJECTORY) || (fsm.cur_state==LOAD_TRAJECTORY))
    {   
        VectorN<float, 4> thrustMomentCmd;
        thrustMomentCmd[0] = 1000.0;
        thrustMomentCmd[1] = 1000.0;
        thrustMomentCmd[2] = 1000.0;
        thrustMomentCmd[3] = 1000.0;

        AP::logger().Write("L1AB", "thrust,mx,my,mz", "ffff",
                           (double)thrustMomentCmd[0],
                           (double)thrustMomentCmd[1],
                           (double)thrustMomentCmd[2],
                           (double)thrustMomentCmd[3]);

        #if (USE_BATT_COMP)
            float BattVolt = 0;
            BattVolt = copter.battery.voltage(); // current battery voltage
            AP::logger().Write("L1BT", "batt", "f", BattVolt);
        #endif

        // L1 adaptive augmentation
        VectorN<float, 4> L1thrustMomentCmd;
        // L1thrustMomentCmd = ACRL_methods.L1AdaptiveAugmentation(thrustMomentCmd);

        // // motor mixing
        VectorN<float, 4> motorPWM;
        #if (USE_BATT_COMP)
            motorPWM = motorMixingBattComp(thrustMomentCmd + L1thrustMomentCmd, BattVolt);
        #else
            motorPWM = motorMixing(thrustMomentCmd + L1thrustMomentCmd);
        #endif
        
        // motorPWM saturation
        if (motorPWM[0] < 0) {motorPWM[0] = 0;}
        else if (motorPWM[0] > 100) {motorPWM[0] = 100;}
        if (motorPWM[1] < 0) {motorPWM[1] = 0;}
        else if (motorPWM[1] > 100) {motorPWM[1] = 100;}
        if (motorPWM[2] < 0) {motorPWM[2] = 0;}
        else if (motorPWM[2] > 100) {motorPWM[2] = 100;}
        if (motorPWM[3] < 0) {motorPWM[3] = 0;}
        else if (motorPWM[3] > 100) {motorPWM[3] = 100;}

        // disarm the vehicle by setting PWM to 1 when landing is completed
        if (fsm.inputsigs.land_r)
        {
            motorPWM[0] = 1;
            motorPWM[1] = 1;
            motorPWM[2] = 1;
            motorPWM[3] = 1;
        }

        if (motors->armed()) // only command the motor PWM when the vehicle is armed.
        {
            motors->rc_write(0, 1000 + 10 * motorPWM[0]); // manual set motor speed: PWM_MIN/MAX has been forced to 1000/2000
            motors->rc_write(1, 1000 + 10 * motorPWM[1]); // rc_write is called from <AP_Motors/AP_Motors_Class.h>
            motors->rc_write(2, 1000 + 10 * motorPWM[2]);
            motors->rc_write(3, 1000 + 10 * motorPWM[3]);
        }
    

        // // logging
        Vector3f statePos;

        int locAvailable = ahrs.get_relative_position_NED_origin(statePos);
        if (!locAvailable)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "Location unavailable.");
        }

        AP::logger().Write("L1AC", "currentT,thisRunT,xxd,yyd,zzd,xx,yy,zz,m1,m2,m3,m4", "ffffffffffff",
                           (double)0,
                           (double)0,
                           (double)targetPos.x,
                           (double)targetPos.y,
                           (double)targetPos.z,
                           (double)statePos.x,
                           (double)statePos.y,
                           (double)statePos.z,
                           (double)motorPWM[0],
                           (double)motorPWM[1],
                           (double)motorPWM[2],
                           (double)motorPWM[3]);
    }
    // end sample code for testing L1AC
    // ===================================================

    // end custom code by ACRL
    // ===================================================
}

// determine the value of fsm.cur_state and fsm.prev_state based on control signals
void ModeRL_ZigZag::determine_states(float currentTime){
    fsm.inputsigs.arm_r = motors->armed();
    fsm.inputsigs.disarm_r = !motors->armed();
    // double now = AP_HAL::micros();;
    // double duration = 0.000001f * (now - fsm.begin);
    float duration = currentTime - fsm.begin;

    switch(fsm.cur_state){
        case ARMED:
            if(fsm.inputsigs.takeoff){
                fsm.cur_state = TAKEOFF;
                fsm.prev_state = ARMED;
                fsm.inputsigs.takeoff = 0;
            }
            else if(fsm.inputsigs.disarm_r){
                fsm.cur_state = DISARMED;
                fsm.prev_state = ARMED;
                fsm.inputsigs.disarm_r = 0;
            }
            else{
                fsm.prev_state = fsm.cur_state;
                clear_sigs();
            }
            break;
        case DISARMED:
            if(fsm.inputsigs.arm_r){
                fsm.cur_state = ARMED;
                fsm.prev_state = DISARMED;
                fsm.inputsigs.arm_r = 0;
            }
            else{
                fsm.prev_state = fsm.cur_state;
                clear_sigs();
            }
            break;
        case TAKEOFF:
            if(fsm.inputsigs.takeoff_r){
                fsm.cur_state = HOVER;
                fsm.prev_state = TAKEOFF;
                fsm.inputsigs.takeoff_r = 0;
            }
            else{
                fsm.prev_state = fsm.cur_state;
                clear_sigs();
            }
            break;
        case HOVER:
            if(fsm.inputsigs.land){
                fsm.cur_state = LAND;
                fsm.prev_state = HOVER;
                fsm.inputsigs.land = 0;
            }
            else if(fsm.inputsigs.new_trajectory){
                fsm.cur_state = LOAD_TRAJECTORY;
                fsm.prev_state = HOVER;
                fsm.inputsigs.new_trajectory = 0;
            }
            else{
                fsm.prev_state = fsm.cur_state;
                clear_sigs();
            }
            break;
        case LAND:
            if(fsm.inputsigs.land_r){
                fsm.cur_state = DISARMED;
                fsm.prev_state = LAND;
                fsm.inputsigs.land_r = 0;
            }
            else{
                fsm.prev_state = fsm.cur_state;
                clear_sigs();
            }
            break;
        case TRAJECTORY:
            if(fsm.inputsigs.pause){        
                fsm.cur_state = HOVER;
                fsm.prev_state = TRAJECTORY;
                fsm.inputsigs.pause = 0;
            }
            else if(fsm.target_duration < duration){    // switch to HOVER state when the target duration has reached
                fsm.cur_state = HOVER;
                fsm.prev_state = TRAJECTORY;
            }
            else if(fsm.inputsigs.trajectory_r){
                fsm.cur_state = HOVER;
                fsm.prev_state = TRAJECTORY;
                fsm.inputsigs.trajectory_r = 0;
            }
            else if(fsm.inputsigs.new_trajectory){
                fsm.cur_state = LOAD_TRAJECTORY;
                fsm.prev_state = TRAJECTORY;
                fsm.inputsigs.new_trajectory = 0;
            }
            else{
                fsm.prev_state = fsm.cur_state;
                clear_sigs();
            }
            break;
        case LOAD_TRAJECTORY:
            // load trajectory
            fsm.cur_state = TRAJECTORY;
            fsm.prev_state = LOAD_TRAJECTORY;
            break;
        case STATES_NULL:
            break;  
    }
}

// clear all control signals
void ModeRL_ZigZag::clear_sigs(){
    fsm.inputsigs.arm_r = 0;
    fsm.inputsigs.disarm_r = 0;
    fsm.inputsigs.takeoff = 0;
    fsm.inputsigs.takeoff_r = 0;
    fsm.inputsigs.land = 0;
    fsm.inputsigs.land_r = 0;
    fsm.inputsigs.pause = 0;
    fsm.inputsigs.new_trajectory = 0;
    fsm.inputsigs.trajectory_r = 0;
}



//motor mixing
//determined by battery comp

#if (USE_BATT_COMP)
VectorN<float, 4> ModeRL_ZigZag::motorMixingBattComp(VectorN<float, 4> thrustMomentCmd, float BattVolt)
{
    #if (REAL_OR_SITL!=0)
        float TestVolt = 16.6; // battery voltage when testing for motormixing curve
        float BattCompScale = (float) (TestVolt / BattVolt);
    #endif

    VectorN<float, 4> w;
#if (REAL_OR_SITL==0)       // SITL
    const float L = 0.25; // for x layout
    const float D = 0.25;
    const float a_F = 0.0014597;
    const float b_F = 0.043693;
    const float a_M = 0.000011667;
    const float b_M = 0.0059137;
#elif (REAL_OR_SITL==1) // parameters for real drone
    const float L = 0.175; // longer distance between adjacent motors
    const float D = 0.131; // shorter distance between adjacent motors
    const float a_F = 0.0009251;
    const float b_F = 0.021145;
    const float a_M = 0.00001211;
    const float b_M = 0.0009864;
#endif
#if (REAL_OR_SITL==0 || REAL_OR_SITL==1)
    // solve for linearizing point
    float w0 = (-b_F + sqrtF(b_F * b_F + a_F * thrustMomentCmd[0])) / 2 / a_F;

    float c_F = 2 * a_F * w0 + b_F;
    float c_M = 2 * a_M * w0 + b_M;

    float thrust_biased = 2 * thrustMomentCmd[0] - 4 * b_F * w0;
    float M1 = thrustMomentCmd[1];
    float M2 = thrustMomentCmd[2];
    float M3 = thrustMomentCmd[3];

    // motor mixing for x layout
    const float c_F4_inv = 1 / (4 * c_F);
    const float c_FL_inv = 1 / (2 * L * c_F);
    const float c_FD_inv = 1 / (2 * D * c_F);
    const float c_M4_inv = 1 / (4 * c_M);

    w[0] = c_F4_inv * thrust_biased - c_FL_inv * M1 + c_FD_inv * M2 + c_M4_inv * M3;
    w[1] = c_F4_inv * thrust_biased + c_FL_inv * M1 - c_FD_inv * M2 + c_M4_inv * M3;
    w[2] = c_F4_inv * thrust_biased + c_FL_inv * M1 + c_FD_inv * M2 - c_M4_inv * M3;
    w[3] = c_F4_inv * thrust_biased - c_FL_inv * M1 - c_FD_inv * M2 - c_M4_inv * M3;

    // 2nd shot on solving for motor speed
    // output: VectorN<float, 4> new motor speed
    // input: a_F, b_F, a_M, b_M, w, L, D, thrustMomentCmd

    // motor speed after the second iteration
    VectorN<float, 4> w2;
    w2 = iterativeMotorMixing(w, thrustMomentCmd, a_F, b_F, a_M, b_M, L, D);

    // motor speed after the third iteration
    VectorN<float, 4> w3;
    w3 = iterativeMotorMixing(w2, thrustMomentCmd, a_F, b_F, a_M, b_M, L, D);
    return w3;
    // logging
    // AP::logger().Write("L1A1", "m1,m2,m3,m4", "ffff",
    //                      (double)w[0],
    //                      (double)w[1],
    //                      (double)w[2],
    //                      (double)w[3]);
    // AP::logger().Write("L1A2", "m1,m2,m3,m4", "ffff",
    //                      (double)w2[0],
    //                      (double)w2[1],
    //                      (double)w2[2],
    //                      (double)w2[3]);
    // AP::logger().Write("L1A3", "m1,m2,m3,m4", "ffff",
    //                      (double)w3[0],
    //                      (double)w3[1],
    //                      (double)w3[2],
    //                      (double)w3[3]);
#elif (REAL_OR_SITL==2) //for holybro
    const float a_F_1 =0.001993;
    const float b_F_1 =-0.006193;
    const float a_F_2 =0.0002735;
    const float b_F_2 =0.1589;
    const float c_F_2 =-3.5860;
    float trust = thrustMomentCmd[0];
    float M1 = thrustMomentCmd[1];
    float M2 = thrustMomentCmd[2];
    float M3 = thrustMomentCmd[3];
    float f1 = 0.25*trust+(-1.4286)*M1+(1.4286)*M2+(10.1446)*M3;
    float f2 = 0.25*trust+(1.4286)*M1+(-1.4286)*M2+(10.1446)*M3;
    float f3 = 0.25*trust+(1.4286)*M1+(1.4286)*M2+(-10.1446)*M3;
    float f4 = 0.25*trust+(-1.4286)*M1+(-1.4286)*M2+(-10.1446)*M3;
    w[0] = (-b_F_1+sqrtf(b_F_1*b_F_1+4*a_F_1*f1))/(2*a_F_1);
    if (f1<0)
    {
        w[0] = 0;
    }
    if (f1>5.0428)
    {
        w[0]=(-b_F_2+sqrt(b_F_2*b_F_2+4*a_F_2*(f1-c_F_2)))/(2*a_F_2);
    }
    w[1] = (-b_F_1+sqrtf(b_F_1*b_F_1+4*a_F_1*f2))/(2*a_F_1);
    if (f2<0)
    {
        w[1] = 0;
    }
    if (f2>5.0428)
    {
        w[1]=(-b_F_2+sqrt(b_F_2*b_F_2+4*a_F_2*(f2-c_F_2)))/(2*a_F_2);
    }
    w[2] = (-b_F_1+sqrtf(b_F_1*b_F_1+4*a_F_1*f3))/(2*a_F_1);
    if (f3<0)
    {
        w[2] = 0;
    }
    if (f3>5.0428)
    {
        w[2]=(-b_F_2+sqrt(b_F_2*b_F_2+4*a_F_2*(f3-c_F_2)))/(2*a_F_2);
    }
    w[3] = (-b_F_1+sqrtf(b_F_1*b_F_1+4*a_F_1*f4))/(2*a_F_1);
    if (f4<0)
    {
        w[3] = 0;
    }
    if (f4>5.0428)
    {
        w[3]=(-b_F_2+sqrt(b_F_2*b_F_2+4*a_F_2*(f4-c_F_2)))/(2*a_F_2);
    }

    w = w * BattCompScale;
    return w;
#endif
}
#else
VectorN<float, 4> ModeRL_ZigZag::motorMixing(VectorN<float, 4> thrustMomentCmd)
{
    VectorN<float, 4> w;
#if (REAL_OR_SITL==0)       // SITL
    const float L = 0.25; // for x layout
    const float D = 0.25;
    const float a_F = 0.0014597;
    const float b_F = 0.043693;
    const float a_M = 0.000011667;
    const float b_M = 0.0059137;
#elif (REAL_OR_SITL==1) // parameters for real drone
    const float L = 0.175; // longer distance between adjacent motors
    const float D = 0.131; // shorter distance between adjacent motors
    const float a_F = 0.0009251;
    const float b_F = 0.021145;
    const float a_M = 0.00001211;
    const float b_M = 0.0009864;
#endif
#if (REAL_OR_SITL==0 || REAL_OR_SITL==1)
    // solve for linearizing point
    float w0 = (-b_F + sqrtF(b_F * b_F + a_F * thrustMomentCmd[0])) / 2 / a_F;

    float c_F = 2 * a_F * w0 + b_F;
    float c_M = 2 * a_M * w0 + b_M;

    float thrust_biased = 2 * thrustMomentCmd[0] - 4 * b_F * w0;
    float M1 = thrustMomentCmd[1];
    float M2 = thrustMomentCmd[2];
    float M3 = thrustMomentCmd[3];

    // motor mixing for x layout
    const float c_F4_inv = 1 / (4 * c_F);
    const float c_FL_inv = 1 / (2 * L * c_F);
    const float c_FD_inv = 1 / (2 * D * c_F);
    const float c_M4_inv = 1 / (4 * c_M);

    w[0] = c_F4_inv * thrust_biased - c_FL_inv * M1 + c_FD_inv * M2 + c_M4_inv * M3;
    w[1] = c_F4_inv * thrust_biased + c_FL_inv * M1 - c_FD_inv * M2 + c_M4_inv * M3;
    w[2] = c_F4_inv * thrust_biased + c_FL_inv * M1 + c_FD_inv * M2 - c_M4_inv * M3;
    w[3] = c_F4_inv * thrust_biased - c_FL_inv * M1 - c_FD_inv * M2 - c_M4_inv * M3;

    // 2nd shot on solving for motor speed
    // output: VectorN<float, 4> new motor speed
    // input: a_F, b_F, a_M, b_M, w, L, D, thrustMomentCmd

    // motor speed after the second iteration
    VectorN<float, 4> w2;
    w2 = iterativeMotorMixing(w, thrustMomentCmd, a_F, b_F, a_M, b_M, L, D);

    // motor speed after the third iteration
    VectorN<float, 4> w3;
    w3 = iterativeMotorMixing(w2, thrustMomentCmd, a_F, b_F, a_M, b_M, L, D);
    return w3;
    // logging
    // AP::logger().Write("L1A1", "m1,m2,m3,m4", "ffff",
    //                      (double)w[0],
    //                      (double)w[1],
    //                      (double)w[2],
    //                      (double)w[3]);
    // AP::logger().Write("L1A2", "m1,m2,m3,m4", "ffff",
    //                      (double)w2[0],
    //                      (double)w2[1],
    //                      (double)w2[2],
    //                      (double)w2[3]);
    // AP::logger().Write("L1A3", "m1,m2,m3,m4", "ffff",
    //                      (double)w3[0],
    //                      (double)w3[1],
    //                      (double)w3[2],
    //                      (double)w3[3]);
#elif (REAL_OR_SITL==2) //for holybro
    const float a_F_1 =0.001993;
    const float b_F_1 =-0.006193;
    const float a_F_2 =0.0002735;
    const float b_F_2 =0.1589;
    const float c_F_2 =-3.5860;
    float trust = thrustMomentCmd[0];
    float M1 = thrustMomentCmd[1];
    float M2 = thrustMomentCmd[2];
    float M3 = thrustMomentCmd[3];
    float f1 = 0.25*trust+(-1.4286)*M1+(1.4286)*M2+(10.1446)*M3;
    float f2 = 0.25*trust+(1.4286)*M1+(-1.4286)*M2+(10.1446)*M3;
    float f3 = 0.25*trust+(1.4286)*M1+(1.4286)*M2+(-10.1446)*M3;
    float f4 = 0.25*trust+(-1.4286)*M1+(-1.4286)*M2+(-10.1446)*M3;
    w[0] = (-b_F_1+sqrtf(b_F_1*b_F_1+4*a_F_1*f1))/(2*a_F_1);
    if (f1<0)
    {
        w[0] = 0;
    }
    if (f1>5.0428)
    {
        w[0]=(-b_F_2+sqrt(b_F_2*b_F_2+4*a_F_2*(f1-c_F_2)))/(2*a_F_2);
    }
    w[1] = (-b_F_1+sqrtf(b_F_1*b_F_1+4*a_F_1*f2))/(2*a_F_1);
    if (f2<0)
    {
        w[1] = 0;
    }
    if (f2>5.0428)
    {
        w[1]=(-b_F_2+sqrt(b_F_2*b_F_2+4*a_F_2*(f2-c_F_2)))/(2*a_F_2);
    }
    w[2] = (-b_F_1+sqrtf(b_F_1*b_F_1+4*a_F_1*f3))/(2*a_F_1);
    if (f3<0)
    {
        w[2] = 0;
    }
    if (f3>5.0428)
    {
        w[2]=(-b_F_2+sqrt(b_F_2*b_F_2+4*a_F_2*(f3-c_F_2)))/(2*a_F_2);
    }
    w[3] = (-b_F_1+sqrtf(b_F_1*b_F_1+4*a_F_1*f4))/(2*a_F_1);
    if (f4<0)
    {
        w[3] = 0;
    }
    if (f4>5.0428)
    {
        w[3]=(-b_F_2+sqrt(b_F_2*b_F_2+4*a_F_2*(f4-c_F_2)))/(2*a_F_2);
    }
    return w;
#endif
}
#endif


//motor iter mixxing and mat_inv method



VectorN<float, 4> ModeRL_ZigZag::iterativeMotorMixing(VectorN<float, 4> w_input, VectorN<float, 4> thrustMomentCmd, float a_F, float b_F, float a_M, float b_M, float L, float D)
{
    // The function iterativeMotorMixing computes the motor speed to achieve the desired thrustMoment command
    // input:
    // VectorN<float, 4> w_input -- initial guess of the motor speed (linearizing point)
    // VectorN<float, 4> thrustMomentCmd -- desired thrust and moment command
    // float a_F -- 2nd-order coefficient for motor's thrust-speed curve
    // float b_F -- 1st-order coefficient for motor's thrust-speed curve
    // float a_M -- 2nd-order coefficient for motor's torque-speed curve
    // float b_M -- 1st-order coefficient for motor's torque-speed curve
    // float L -- longer distance between adjacent motors
    // float D -- shorter distance between adjacent motors

    // output:
    // VectorN<float, 4> w_new new motor speed

    VectorN<float, 4> w_new; // new motor speed

    float w1_square = w_input[0] * w_input[0];
    float w2_square = w_input[1] * w_input[1];
    float w3_square = w_input[2] * w_input[2];
    float w4_square = w_input[3] * w_input[3];

    float c_F1 = -a_F * w1_square;
    float c_F2 = -a_F * w2_square;
    float c_F3 = -a_F * w3_square;
    float c_F4 = -a_F * w4_square;

    float c_M1 = -a_M * w1_square;
    float c_M2 = -a_M * w2_square;
    float c_M3 = -a_M * w3_square;
    float c_M4 = -a_M * w4_square;

    float d_F1 = 2 * a_F * w_input[0] + b_F;
    float d_F2 = 2 * a_F * w_input[1] + b_F;
    float d_F3 = 2 * a_F * w_input[2] + b_F;
    float d_F4 = 2 * a_F * w_input[3] + b_F;

    float d_M1 = 2 * a_M * w_input[0] + b_M;
    float d_M2 = 2 * a_M * w_input[1] + b_M;
    float d_M3 = 2 * a_M * w_input[2] + b_M;
    float d_M4 = 2 * a_M * w_input[3] + b_M;

    VectorN<float, 4> coefficientRow1;
    VectorN<float, 4> coefficientRow2;
    VectorN<float, 4> coefficientRow3;
    VectorN<float, 4> coefficientRow4;

    coefficientRow1[0] = d_F1;
    coefficientRow1[1] = d_F2;
    coefficientRow1[2] = d_F3;
    coefficientRow1[3] = d_F4;

    coefficientRow2[0] = -d_F1;
    coefficientRow2[1] = d_F2;
    coefficientRow2[2] = d_F3;
    coefficientRow2[3] = -d_F4;

    coefficientRow3[0] = d_F1;
    coefficientRow3[1] = -d_F2;
    coefficientRow3[2] = d_F3;
    coefficientRow3[3] = -d_F4;

    coefficientRow4[0] = d_M1;
    coefficientRow4[1] = d_M2;
    coefficientRow4[2] = -d_M3;
    coefficientRow4[3] = -d_M4;

    VectorN<float, 16> coefficientMatrixInv = mat4Inv(coefficientRow1, coefficientRow2, coefficientRow3, coefficientRow4);

    VectorN<float, 4> coefficientInvRow1;
    VectorN<float, 4> coefficientInvRow2;
    VectorN<float, 4> coefficientInvRow3;
    VectorN<float, 4> coefficientInvRow4;

    coefficientInvRow1[0] = coefficientMatrixInv[0];
    coefficientInvRow1[1] = coefficientMatrixInv[1];
    coefficientInvRow1[2] = coefficientMatrixInv[2];
    coefficientInvRow1[3] = coefficientMatrixInv[3];

    coefficientInvRow2[0] = coefficientMatrixInv[4];
    coefficientInvRow2[1] = coefficientMatrixInv[5];
    coefficientInvRow2[2] = coefficientMatrixInv[6];
    coefficientInvRow2[3] = coefficientMatrixInv[7];

    coefficientInvRow3[0] = coefficientMatrixInv[8];
    coefficientInvRow3[1] = coefficientMatrixInv[9];
    coefficientInvRow3[2] = coefficientMatrixInv[10];
    coefficientInvRow3[3] = coefficientMatrixInv[11];

    coefficientInvRow4[0] = coefficientMatrixInv[12];
    coefficientInvRow4[1] = coefficientMatrixInv[13];
    coefficientInvRow4[2] = coefficientMatrixInv[14];
    coefficientInvRow4[3] = coefficientMatrixInv[15];

    VectorN<float, 4> shiftedCmd;
    shiftedCmd[0] = thrustMomentCmd[0] - c_F1 - c_F2 - c_F3 - c_F4;
    shiftedCmd[1] = 2 * thrustMomentCmd[1] / L + c_F1 - c_F2 - c_F3 + c_F4;
    shiftedCmd[2] = 2 * thrustMomentCmd[2] / D - c_F1 + c_F2 - c_F3 + c_F4;
    shiftedCmd[3] = thrustMomentCmd[3] - c_M1 - c_M2 + c_M3 + c_M4;

    w_new[0] = coefficientInvRow1 * shiftedCmd;
    w_new[1] = coefficientInvRow2 * shiftedCmd;
    w_new[2] = coefficientInvRow3 * shiftedCmd;
    w_new[3] = coefficientInvRow4 * shiftedCmd;

    return w_new;
}

VectorN<float, 16> ModeRL_ZigZag::mat4Inv(VectorN<float, 4> coefficientRow1, VectorN<float, 4> coefficientRow2, VectorN<float, 4> coefficientRow3, VectorN<float, 4> coefficientRow4)
{
    // inverse of a 4x4 matrix
    // modified from https://stackoverflow.com/a/44446912
    float A2323 = coefficientRow3[2] * coefficientRow4[3] - coefficientRow3[3] * coefficientRow4[2];
    float A1323 = coefficientRow3[1] * coefficientRow4[3] - coefficientRow3[3] * coefficientRow4[1];
    float A1223 = coefficientRow3[1] * coefficientRow4[2] - coefficientRow3[2] * coefficientRow4[1];
    float A0323 = coefficientRow3[0] * coefficientRow4[3] - coefficientRow3[3] * coefficientRow4[0];
    float A0223 = coefficientRow3[0] * coefficientRow4[2] - coefficientRow3[2] * coefficientRow4[0];
    float A0123 = coefficientRow3[0] * coefficientRow4[1] - coefficientRow3[1] * coefficientRow4[0];
    float A2313 = coefficientRow2[2] * coefficientRow4[3] - coefficientRow2[3] * coefficientRow4[2];
    float A1313 = coefficientRow2[1] * coefficientRow4[3] - coefficientRow2[3] * coefficientRow4[1];
    float A1213 = coefficientRow2[1] * coefficientRow4[2] - coefficientRow2[2] * coefficientRow4[1];
    float A2312 = coefficientRow2[2] * coefficientRow3[3] - coefficientRow2[3] * coefficientRow3[2];
    float A1312 = coefficientRow2[1] * coefficientRow3[3] - coefficientRow2[3] * coefficientRow3[1];
    float A1212 = coefficientRow2[1] * coefficientRow3[2] - coefficientRow2[2] * coefficientRow3[1];
    float A0313 = coefficientRow2[0] * coefficientRow4[3] - coefficientRow2[3] * coefficientRow4[0];
    float A0213 = coefficientRow2[0] * coefficientRow4[2] - coefficientRow2[2] * coefficientRow4[0];
    float A0312 = coefficientRow2[0] * coefficientRow3[3] - coefficientRow2[3] * coefficientRow3[0];
    float A0212 = coefficientRow2[0] * coefficientRow3[2] - coefficientRow2[2] * coefficientRow3[0];
    float A0113 = coefficientRow2[0] * coefficientRow4[1] - coefficientRow2[1] * coefficientRow4[0];
    float A0112 = coefficientRow2[0] * coefficientRow3[1] - coefficientRow2[1] * coefficientRow3[0];

    float det = coefficientRow1[0] * (coefficientRow2[1] * A2323 - coefficientRow2[2] * A1323 + coefficientRow2[3] * A1223) - coefficientRow1[1] * (coefficientRow2[0] * A2323 - coefficientRow2[2] * A0323 + coefficientRow2[3] * A0223) + coefficientRow1[2] * (coefficientRow2[0] * A1323 - coefficientRow2[1] * A0323 + coefficientRow2[3] * A0123) - coefficientRow1[3] * (coefficientRow2[0] * A1223 - coefficientRow2[1] * A0223 + coefficientRow2[2] * A0123);
    det = 1 / det;

    VectorN<float, 16> inv;
    inv[0] = det * (coefficientRow2[1] * A2323 - coefficientRow2[2] * A1323 + coefficientRow2[3] * A1223);
    inv[1] = det * -(coefficientRow1[1] * A2323 - coefficientRow1[2] * A1323 + coefficientRow1[3] * A1223);
    inv[2] = det * (coefficientRow1[1] * A2313 - coefficientRow1[2] * A1313 + coefficientRow1[3] * A1213);
    inv[3] = det * -(coefficientRow1[1] * A2312 - coefficientRow1[2] * A1312 + coefficientRow1[3] * A1212);
    inv[4] = det * -(coefficientRow2[0] * A2323 - coefficientRow2[2] * A0323 + coefficientRow2[3] * A0223);
    inv[5] = det * (coefficientRow1[0] * A2323 - coefficientRow1[2] * A0323 + coefficientRow1[3] * A0223);
    inv[6] = det * -(coefficientRow1[0] * A2313 - coefficientRow1[2] * A0313 + coefficientRow1[3] * A0213);
    inv[7] = det * (coefficientRow1[0] * A2312 - coefficientRow1[2] * A0312 + coefficientRow1[3] * A0212);
    inv[8] = det * (coefficientRow2[0] * A1323 - coefficientRow2[1] * A0323 + coefficientRow2[3] * A0123);
    inv[9] = det * -(coefficientRow1[0] * A1323 - coefficientRow1[1] * A0323 + coefficientRow1[3] * A0123);
    inv[10] = det * (coefficientRow1[0] * A1313 - coefficientRow1[1] * A0313 + coefficientRow1[3] * A0113);
    inv[11] = det * -(coefficientRow1[0] * A1312 - coefficientRow1[1] * A0312 + coefficientRow1[3] * A0112);
    inv[12] = det * -(coefficientRow2[0] * A1223 - coefficientRow2[1] * A0223 + coefficientRow2[2] * A0123);
    inv[13] = det * (coefficientRow1[0] * A1223 - coefficientRow1[1] * A0223 + coefficientRow1[2] * A0123);
    inv[14] = det * -(coefficientRow1[0] * A1213 - coefficientRow1[1] * A0213 + coefficientRow1[2] * A0113);
    inv[15] = det * (coefficientRow1[0] * A1212 - coefficientRow1[1] * A0212 + coefficientRow1[2] * A0112);

    return inv;
}


#endif