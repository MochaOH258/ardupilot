#include "Sub.h"

/*
 * Init and run calls for guided flight mode
 */
// guided_init - initialise guided controller
bool ModeCustom2::init(bool ignore_checks)
{
    if (!sub.position_ok() && !ignore_checks) {
        return false;
    }

    GeneratePath();
    path_num = 0;

    // start in position control mode
    guided_pos_control_start();
    return true;
}

void ModeCustom2::GeneratePath(){
    float square_cm = 3000.0;

    sub.wp_nav.get_wp_stopping_point(path[0]);
    path[1] = path[0] + Vector3f(1.0f, 0, 0) * square_cm;
    path[2] = path[1] + Vector3f(0, 1.0f, 0) * square_cm;
    path[3] = path[2] + Vector3f(-1.0f, 0, 0) * square_cm;
    path[4] = path[0];
}

// get_default_auto_yaw_mode - returns auto_yaw_mode based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
autopilot_yaw_mode ModeCustom2::get_default_auto_yaw_mode(bool rtl) const
{
    switch (g.wp_yaw_behavior) {

    case WP_YAW_BEHAVIOR_NONE:
        return AUTO_YAW_HOLD;
        break;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
        if (rtl) {
            return AUTO_YAW_HOLD;
        } else {
            return AUTO_YAW_LOOK_AT_NEXT_WP;
        }
        break;

    case WP_YAW_BEHAVIOR_LOOK_AHEAD:
        return AUTO_YAW_LOOK_AHEAD;
        break;

    case WP_YAW_BEHAVIOR_CORRECT_XTRACK:
        return AUTO_YAW_CORRECT_XTRACK;
        break;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
    default:
        return AUTO_YAW_LOOK_AT_NEXT_WP;
        break;
    }
}


// initialise guided mode's position controller
void ModeCustom2::guided_pos_control_start()
{
    // initialise waypoint controller
    sub.wp_nav.wp_and_spline_init();

    

    // no need to check return status because terrain data is not used
    sub.wp_nav.set_wp_destination(path[0], false);

    // initialise yaw
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void ModeCustom2::run()
{
    if (path_num <= 4){
        if (sub.wp_nav.reached_wp_destination()){
            path_num++;
            sub.wp_nav.set_wp_destination(path[path_num], false);
        }
    }
    
    guided_pos_control_run();
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void ModeCustom2::guided_pos_control_run()
{
    // if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.wp_nav.wp_and_spline_init();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!sub.failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        } else{
            if (sub.yaw_rate_only){
                set_auto_yaw_mode(AUTO_YAW_RATE);
            } else{
                set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
            }
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    position_control->update_z_controller();

    // call attitude controller
    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch & yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_LOOK_AT_HEADING) {
        // roll, pitch from pilot, yaw & yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_slew_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_RATE) {
        // roll, pitch from pilot, yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        // roll, pitch from pilot, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), true);
    }
}

// set_auto_yaw_mode - sets the yaw mode for auto
void ModeCustom2::set_auto_yaw_mode(autopilot_yaw_mode yaw_mode)
{
    // return immediately if no change
    if (sub.auto_yaw_mode == yaw_mode) {
        return;
    }
    sub.auto_yaw_mode = yaw_mode;

    // perform initialisation
    switch (sub.auto_yaw_mode) {
    
    case AUTO_YAW_HOLD:
        // pilot controls the heading
        break;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
        // wpnav will initialise heading when wpnav's set_destination method is called
        break;

    case AUTO_YAW_ROI:
        // point towards a location held in yaw_look_at_WP
        sub.yaw_look_at_WP_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading
        // caller should set the yaw_look_at_heading
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        sub.yaw_look_ahead_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // initial_armed_bearing will be set during arming so no init required
        break;
    
    case AUTO_YAW_RATE:
        // set target yaw rate to yaw_look_at_heading_slew
        break;
    }
}

// get_auto_heading - returns target heading depending upon auto_yaw_mode
// 100hz update rate
float ModeCustom2::get_auto_heading()
{
    switch (sub.auto_yaw_mode) {

    case AUTO_YAW_ROI:
        // point towards a location held in roi_WP
        return sub.get_roi_yaw();
        break;

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
        return sub.yaw_look_at_heading;
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        return sub.get_look_ahead_yaw();
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // changes yaw to be same as when quad was armed
        return sub.initial_armed_bearing;
        break;

    case AUTO_YAW_CORRECT_XTRACK: {
        // TODO return current yaw if not in appropriate mode
        // Bearing of current track (centidegrees)
        float track_bearing = get_bearing_cd(sub.wp_nav.get_wp_origin().xy(), sub.wp_nav.get_wp_destination().xy());

        // Bearing from current position towards intermediate position target (centidegrees)
        const Vector2f target_vel_xy{position_control->get_vel_target_cms().x, position_control->get_vel_target_cms().y};
        float angle_error = 0.0f;
        if (target_vel_xy.length() >= position_control->get_max_speed_xy_cms() * 0.1f) {
            const float desired_angle_cd = degrees(target_vel_xy.angle()) * 100.0f;
            angle_error = wrap_180_cd(desired_angle_cd - track_bearing);
        }
        float angle_limited = constrain_float(angle_error, -g.xtrack_angle_limit * 100.0f, g.xtrack_angle_limit * 100.0f);
        return wrap_360_cd(track_bearing + angle_limited);
    }
    break;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
    default:
        // point towards next waypoint.
        // we don't use wp_bearing because we don't want the vehicle to turn too much during flight
        return sub.wp_nav.get_yaw();
        break;
    }
}

