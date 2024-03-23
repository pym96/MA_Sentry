process[velocity_smoother_ema-1]: started with pid [12814]
[ERROR] [1711206976.444113546]: Client [/rostopic_11537_1711206883903] wants topic /cmd_vel to have datatype/md5sum [geometry_msgs/TwistStamped/98d34b0043a2093cf9d9345ab6eef12e], but our version has [geometry_msgs/Twist/9f195f881246fdfa2798d1d3eebca84a]. Dropping connection.
[ERROR] [1711206976.640208152]: Client [/vehicleSimulator] wants topic /cmd_vel to have datatype/md5sum [geometry_msgs/TwistStamped/98d34b0043a2093cf9d9345ab6eef12e], but our version has [geometry_msgs/Twist/9f195f881246fdfa2798d1d3eebca84a]. Dropping connection.

```

#include <tf/tf.h>

template <typename T>
void LaserMapping::SetPosestamp(T &out) {
    out.pose.position.x = state_point_.pos(0);
    out.pose.position.y = state_point_.pos(1);
    out.pose.position.z = state_point_.pos(2);

    // 获得当前四元数
    tf::Quaternion q(
        state_point_.rot.coeffs()[0],
        state_point_.rot.coeffs()[1],
        state_point_.rot.coeffs()[2],
        state_point_.rot.coeffs()[3]);

    // 四元数转换为欧拉角
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 取pitch和yaw的反值
    pitch = -pitch;
    yaw = -yaw;

    // 将修改后的欧拉角转换回四元数
    tf::Quaternion q_new = tf::createQuaternionFromRPY(roll, pitch, yaw);

    // 更新out对象的方向
    out.pose.orientation.x = q_new.x();
    out.pose.orientation.y = q_new.y();
    out.pose.orientation.z = q_new.z();
    out.pose.orientation.w = q_new.w();
}


```
