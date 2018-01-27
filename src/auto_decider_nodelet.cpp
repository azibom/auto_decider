#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include <auto_decider/parsian_robot_status.h>
#include <auto_decider/parsian_robot.h>
#include <auto_decider/parsian_world_model.h>
#include <auto_decider/vector2D.h>
#include <auto_decider/result.h>
# define buffer_size  10
# define threshold_y 5
# define threshold_x 5
auto_decider::result msg3;
int robot_status,robot_x,robot_y,ball_y,ball_x,vision_sensor,decide_array[100],sum,decide_1,counter_1=1;
bool robot_sensor=0,decide=1,final_decide=1;
namespace auto_decider
{
class Decider : public nodelet::Nodelet
{
private:
    virtual void onInit()
    {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        pub = private_nh.advertise<auto_decider::result>("result",5);
        sub1 = private_nh.subscribe("auto_decider/parsian_robot_status",5, &Decider::function_1, this);
        sub2 = private_nh.subscribe("auto_decider/parsian_robot",5, &Decider::function_2, this);
        sub3 = private_nh.subscribe("auto_decider/parsian_world_model",5, &Decider::function_3, this);
    }
    void function_1(const auto_decider::parsian_robot_status msg)
    {
        robot_status=msg.status-1;
        robot_sensor=msg.spinCatchBall;
        if(msg.battery<=3||msg.kickFault==0||msg.chipFault==0||msg.m1Fault==0||msg.m1Fault==0||msg.m2Fault==0||msg.m3Fault==0||msg.En4Fault==0||msg.En1Fault==0||msg.En2Fault==0||msg.En3Fault==0||msg.En4Fault==0)
            decide=0;
    }
    void function_2(const auto_decider::parsian_robot msg1)
    {
        robot_x=msg1.pos.x;
        robot_y=msg1.pos.y;
    }

    void function_3(const auto_decider::parsian_world_model msg2)
    {
        ball_x=msg2.ball.pos.x;
        ball_y=msg2.ball.pos.y;
        if(abs(ball_x-robot_x)<threshold_x && abs(ball_y-robot_y)<threshold_y )
            vision_sensor=1;
        else
            vision_sensor=0;
        if(vision_sensor==0 && robot_sensor==1)
            decide=0;
        if(counter_1> buffer_size ){
            decide_1=decide;
            sum=0;
            for(int aa=1;aa<=buffer_size;aa++)
                sum=decide_array[aa]+sum;
            if(sum<=buffer_size/2)
                final_decide=0;
            else
                final_decide=1;
            for(int counter_2=1;counter_2<buffer_size;counter_2++)
                decide_array[counter_2]=decide_array[counter_2+1];
            decide_array[buffer_size]=decide_1;
        }
        else
        {
            decide_array[counter_1]=decide;
            counter_1++;
        }

        msg3.r[robot_status] = final_decide;
        pub.publish(msg3);
        ROS_INFO("Message published ");
        decide=1;
    }
    ros::Publisher pub;
    ros::Subscriber sub1,sub2,sub3;
};
}
PLUGINLIB_DECLARE_CLASS(auto_decider,Decider,auto_decider::Decider, nodelet::Nodelet);

