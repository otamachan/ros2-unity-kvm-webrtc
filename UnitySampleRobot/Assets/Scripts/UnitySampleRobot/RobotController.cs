using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ROS2
{
public class RobotController : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<geometry_msgs.msg.Twist> chatter_sub;

    private float speed = 0.0F;
    private float angleSpeed = 0.0F;
    private DateTime lastTime;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        lastTime = DateTime.Now;
    }

    void Update()
    {
        if (ros2Node == null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("dummy_robot_controller");
            chatter_sub = ros2Node.CreateSubscription<geometry_msgs.msg.Twist>("cmd_vel", Callback);
        }
        if ((DateTime.Now - lastTime).TotalMilliseconds > 500)
        {
            // timeout
            speed = 0.0F;
            angleSpeed = 0.0F;
        }
        transform.position = transform.position + new Vector3(0.0F, 0.0F, speed * Time.deltaTime);
        transform.Rotate(0.0F, angleSpeed * Time.deltaTime, 0.0F, Space.Self);
    }

    void Callback(geometry_msgs.msg.Twist msg)
    {
        lastTime = DateTime.Now;
        speed = (float)msg.Linear.X;
        angleSpeed = -Mathf.Rad2Deg * (float)msg.Angular.Z;
    }
    
}
}  // namespace ROS2