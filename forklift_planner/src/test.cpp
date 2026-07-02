#include <ros/ros.h>

class DemoNode
{
public:         //对外的函数接口，允许外部函数创建，构造函数应该属于public
    DemoNode()
    {
        ROS_INFO("构造函数执行");

        // 注册一个 0.1s 定时器，参数分别为：定时器周期、回调函数的地址，当前对象的指针
        timer_ = nh_.createTimer(ros::Duration(0.1),&DemoNode::tick,this);
    }

private:    // 内部的函数
    void tick(const ros::TimerEvent&)
    {
        ROS_INFO("tick执行一次");
    }

private:    // 内部数据
    //都在构造函数里被用过了，而且必须作为成员变量保存下来。
    ros::NodeHandle nh_;        //作为 ROS 节点句柄(NodeHandle)，
                                //用来创建 Publisher/Subscriber/Timer/Service 等 ROS 资源
    ros::Timer timer_;          //保存定时器对象，确保定时器不会被析构
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_node");
    DemoNode node;  //此时会创建一个DemoNode类的名为node的对象，同时调用对应构造函数DemoNode，
    //构造函数内部注册ROS Timer，构造函数结束，注意此时tick()还没有执行

    //启用ROS回调循环，当Timer每隔0.1s到达时候，ROS->node.tick()执行
    ros::spin();
    return 0;
}
