#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>


ros::NodeHandle nh;
float starting_lin_vel = 0;
float starting_angular_vel = 0;
ros::Publisher publisher = nh.advertise<std_msgs::Float32MultiArray>("velocity_commands", 10);
PID_CONTROLLER pid_loop;

class PID_CONTROLLER {

    public:

        PID_CONTROLLER() {

            previous_error_lin = 0;
            previous_error_ang = 0;
            integral_lin = 0;
            integral_ang = 0;
            kP = 0.5;
            kI = 0.1;
            kD = 0.4;
        }

        void set_lingoal(float goal) {
            lin_setpoint = goal;
        }
    
        void set_angular_goal(float goal) {
            angular_setpoint = goal;
        }

        void execute_pid_loop() {

            while (output1 != lin_setpoint && output2 != angular_setpoint) {


                if (current_value_lin != lin_setpoint) {
                    double error_lin = lin_setpoint - current_value_lin;
                    double p_term_lin = kP * error_lin;
                    integral_lin += error_lin;
                    double i_term_lin = kI * integral_lin;
                    double d_term_lin = kD * (error_lin - previous_error_lin);
                    previous_error_lin = error_lin;
                    output1 = p_term_lin + i_term_lin + d_term_lin;

                }

                if (current_value_ang != angular_setpoint) {
                    
                    double error_ang = angular_setpoint - current_value_ang;
                    double p_term_ang = kP * error_ang;
                    integral_ang += error_ang;
                    double i_term_ang = kI * integral_ang;
                    double d_term_ang = kD * (error_ang - previous_error_ang);
                    previous_error_ang = error_ang;
                    output2 = p_term_ang + i_term_ang + d_term_ang;

                }

                vel[0] = output1;
                vel[1] = output2;

                publisher.publish(vel);
            } 



            current_value_lin = lin_setpoint;
            current_value_ang = angular_setpoint;
            
        }
    private:

        float kD;
        float kP;
        float kI;
        float previous_error_lin;
        float previous_error_ang;
        float integral_lin;
        float integral_ang;
        float lin_setpoint;
        float angular_setpoint;
        float current_value_lin;
        float current_value_ang;
        float output1;
        float output2;
        std::vector<float> vel;



};


void velocity_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

    pid_loop.set_lingoal(msg->data[0]);
    pid_loop.set_angular_goal(msg->data[1]);
    pid_loop.execute_pid_loop();



}
int main(int argc, char **argv) {


    ros::init(argc, argv, "PID Control algorithm");
    ros::Subscriber velocity = nh.subscribe("velocity_commands", 10, velocity_callback);
    ros::spin();

}
