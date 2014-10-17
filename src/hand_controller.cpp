#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

class hand_controller{
	public:
	ros::NodeHandle n;
    ros::Publisher Twist_publisher;
    geometry_msgs::Twist msg;
	ros::Subscriber Hand_subscriber;

	hand_controller(double x, double y){
		n = ros::NodeHandle("~");
		Hand_subscriber = n.subscribe("/hand_tracker/direction", 1, &hand_controller::directioncallbacker,this);
		Twist_publisher= n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 100);
		error[0]=0;
		error[1]=0;
		aimed[0]=x;
		aimed[1]=y;
		current[0]=x;
		current[1]=y;
		speed=0;
		angel=0;
		speedparameter = 1;
		angularparameter= 5;
        if(n.hasParam("SpeedParamter")){
            n.getParam("SpeedParameter",speedparameter);
		}
		if(n.hasParam("angularParamter")){
            n.getParam("angularParameter",angularparameter);
		}
		if(n.hasParam("Xtarget")){
			n.getParam("Xtarget",aimed[0]);
		}
		if(n.hasParam("Ytarget")){
			n.getParam("Ytarget",aimed[1]);
		}
        ROS_INFO_ONCE("Parameters are speed: %f angular:  %f Xtarget: %f  Ytarget %f", speedparameter, angularparameter, aimed[0], aimed[1]);
	}
    void call(){
		error[0]= current[0] - aimed[0];
		error[1]= current[1] - aimed[1];
		speed=speedparameter*error[0];
		angel=angularparameter*error[1];
		msg.linear.x=speed;
    	msg.linear.y=0;
    	msg.linear.z=0;
    	msg.angular.x=0;
    	msg.angular.y=0;
    	msg.angular.z=angel;
    	Twist_publisher.publish(msg);
    //ROS_INFO("Speed [%f] Angular [%f]", speed, angel);
	}
	void directioncallbacker(geometry_msgs::Point msgP){
		current[0]=msgP.x;
		current[1]=msgP.y;
		if(std::abs(current[0])<0.000001 && std::abs(current[1])<0.000001){
				current[0]=aimed[0];
				current[1]=aimed[1];
                //ROS_INFO("ARE WE IN HERE");
		}
		
	}
	private:
	double speedparameter;
	double angularparameter;
	double current [2];
	double error [2]; 
	double aimed [2];
	double speed, angel;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hand_controller");
	double xvalue=0.6;
	double yvalue=0;
    hand_controller hand_controller_node(xvalue,yvalue);
    ros::Rate loop_rate(10.0);

    while(hand_controller_node.n.ok())
    {
    	
    	hand_controller_node.call();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
