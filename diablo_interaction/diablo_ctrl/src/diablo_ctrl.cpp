#include <iostream>
#include "diablo_ctrl.hpp"

using namespace std;

void diabloCtrlNode::heart_beat_loop(void){
    RCLCPP_INFO(this->get_logger(), "start.");
    while (this->thd_loop_mark_)
    {
        if (onSend)
        {
            if(!pMovementCtrl->in_control())
            {
                usleep(180000);
                continue;
            }
            
            if(!ctrl_msg_.mode_mark){
                pMovementCtrl->ctrl_data.forward = ctrl_msg_.value.forward;
                pMovementCtrl->ctrl_data.left = ctrl_msg_.value.left;
                pMovementCtrl->ctrl_data.up = ctrl_msg_.value.up;
                pMovementCtrl->ctrl_data.roll = ctrl_msg_.value.roll;
                pMovementCtrl->ctrl_data.pitch = ctrl_msg_.value.pitch;
                pMovementCtrl->ctrl_data.leg_split = ctrl_msg_.value.leg_split;
                uint8_t result = pMovementCtrl->SendMovementCtrlCmd();
                if(result)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "Heartbeat SendMovementCtrlCmd failed: result=%u ctrl_mode=%u robot_mode=%u error=%u warning=%u",
                        result,
                        pTelemetry->status.ctrl_mode,
                        pTelemetry->status.robot_mode,
                        pTelemetry->status.error,
                        pTelemetry->status.warning);
                }
                
            }else{
                // Transform and mode commands are one-shot requests. Replaying
                // them in the heartbeat can flood the serial link when the
                // controller rejects a posture change.
                usleep(180000);
                continue;
            }
            usleep(180000);
        }
    }
}

void diabloCtrlNode::run_(void){
    this->thd_loop_mark_ = true;
    this->thread_ = std::make_shared<std::thread>(&diabloCtrlNode::heart_beat_loop,this);
}

void diabloCtrlNode::Motion_callback(const motion_msgs::msg::MotionCtrl::SharedPtr msg)
{
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "MotionCmd callback entered: mode_mark=%d forward=%.3f left=%.3f up=%.3f in_control=%d",
        msg->mode_mark,
        msg->value.forward,
        msg->value.left,
        msg->value.up,
        pMovementCtrl->in_control());
    onSend = false;
    if(!pMovementCtrl->in_control())
    {
        uint8_t result = pMovementCtrl->obtain_control();
        if(!pMovementCtrl->in_control())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "drop MotionCmd because movement control is not obtained: result=%u ctrl_mode=%u robot_mode=%u error=%u warning=%u",
                result,
                pTelemetry->status.ctrl_mode,
                pTelemetry->status.robot_mode,
                pTelemetry->status.error,
                pTelemetry->status.warning);
            onSend = true;
            return;
        }
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "MotionCmd received: mode_mark=%d forward=%.3f left=%.3f up=%.3f stand=%d",
        msg->mode_mark,
        msg->value.forward,
        msg->value.left,
        msg->value.up,
        msg->mode.stand_mode);

    ctrl_msg_.mode = msg->mode;
    ctrl_msg_.mode_mark = msg->mode_mark;
    ctrl_msg_.value = msg->value;
    
    if(!msg->mode_mark){
        pMovementCtrl->ctrl_data.forward = msg->value.forward;
        pMovementCtrl->ctrl_data.left = msg->value.left;
        pMovementCtrl->ctrl_data.up = msg->value.up;
        pMovementCtrl->ctrl_data.roll = msg->value.roll;
        pMovementCtrl->ctrl_data.pitch = msg->value.pitch;
        pMovementCtrl->ctrl_data.leg_split = msg->value.leg_split;
        uint8_t result = pMovementCtrl->SendMovementCtrlCmd();
        if(result)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "SendMovementCtrlCmd failed: result=%u ctrl_mode=%u robot_mode=%u error=%u warning=%u",
                result,
                pTelemetry->status.ctrl_mode,
                pTelemetry->status.robot_mode,
                pTelemetry->status.error,
                pTelemetry->status.warning);
        }
        else if(msg->value.forward != 0.0 || msg->value.left != 0.0)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "SendMovementCtrlCmd ok: forward=%.3f left=%.3f up=%.3f ctrl_mode=%u robot_mode=%u",
                msg->value.forward,
                msg->value.left,
                msg->value.up,
                pTelemetry->status.ctrl_mode,
                pTelemetry->status.robot_mode);
        }
    }else{
 
        uint8_t transform_result = 0;
        if(msg->mode.stand_mode)
            transform_result = pMovementCtrl->SendTransformUpCmd();
        else{
            transform_result = pMovementCtrl->SendTransformDownCmd();
        }
        if(transform_result)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Transform command failed: result=%u stand=%d ctrl_mode=%u robot_mode=%u error=%u warning=%u",
                transform_result,
                msg->mode.stand_mode,
                pTelemetry->status.ctrl_mode,
                pTelemetry->status.robot_mode,
                pTelemetry->status.error,
                pTelemetry->status.warning);
        }
        
        // RCLCPP_INFO(this->get_logger(), "try to jump.");
        if(pTelemetry->status.robot_mode == 3){
            pMovementCtrl->SendJumpCmd(msg->mode.jump_mode);
        }
        pMovementCtrl->SendDanceCmd(msg->mode.split_mode);
        // else{
        //     pMovementCtrl->SendJumpCmd(0);
        // }
        pMovementCtrl->ctrl_mode_data.height_ctrl_mode = msg->mode.height_ctrl_mode;
        pMovementCtrl->ctrl_mode_data.pitch_ctrl_mode = msg->mode.pitch_ctrl_mode;
        pMovementCtrl->ctrl_mode_data.roll_ctrl_mode = msg->mode.roll_ctrl_mode;
        uint8_t mode_result = pMovementCtrl->SendMovementModeCtrlCmd();
        if(mode_result)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "SendMovementModeCtrlCmd failed: result=%u ctrl_mode=%u robot_mode=%u error=%u warning=%u",
                mode_result,
                pTelemetry->status.ctrl_mode,
                pTelemetry->status.robot_mode,
                pTelemetry->status.error,
                pTelemetry->status.warning);
        }
    }
    onSend = true;
}



diabloCtrlNode::~diabloCtrlNode()
{
    RCLCPP_INFO(this->get_logger(), "done.");
    this->thd_loop_mark_ = false;
    thread_->join();
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<diabloCtrlNode>("diablo_ctrl_node");

    DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init("/dev/ttyAMA0")) return -1;

    DIABLO::OSDK::Vehicle vehicle(&Hal);                     
    if(vehicle.init()) return -1;

    if(vehicle.telemetry->activate())
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to activate Diablo telemetry over serial. Check robot power, serial wiring, and /dev/ttyAMA0.");
        return -1;
    }

    diablo_imu_publisher imuPublisher(node,&vehicle);
    imuPublisher.imu_pub_init();

    diablo_battery_publisher batteryPublisher(node,&vehicle);
    batteryPublisher.battery_pub_init();

	diablo_motors_publisher motorsPublisher(node,&vehicle);
    motorsPublisher.motors_pub_init();

    diablo_body_state_publisher bodyStatePublisher(node,&vehicle);
    bodyStatePublisher.body_pub_init();

    if(vehicle.telemetry->configUpdate())
    {
        RCLCPP_WARN(node->get_logger(), "Diablo telemetry topic configuration did not receive an ACK after retries; motion control will continue, but sensor topics may be stale.");
    }

    // vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_POWER);
    // vehicle.telemetry->setMaxSpeed(1.0);
    node->pMovementCtrl = vehicle.movement_ctrl;
    node->pTelemetry = vehicle.telemetry;
    node->run_();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
