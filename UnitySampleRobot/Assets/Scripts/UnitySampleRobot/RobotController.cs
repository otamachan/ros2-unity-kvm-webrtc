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
    private ISubscription<geometry_msgs.msg.Twist> cmdVelSubsciber;

    geometry_msgs.msg.Twist lastMsg;
    private DateTime lastTime;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        lastTime = DateTime.Now;
        lastMsg = new geometry_msgs.msg.Twist();
    }

    void Update()
    {
        if (ros2Node == null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("dummy_robot_controller");
            cmdVelSubsciber = ros2Node.CreateSubscription<geometry_msgs.msg.Twist>("cmd_vel", Callback);
        }
        if ((DateTime.Now - lastTime).TotalMilliseconds > 500)
        {
            // timeout
            lastMsg.Linear.X = 0;
            lastMsg.Linear.Y = 0;
            lastMsg.Linear.Z = 0;
            lastMsg.Angular.X = 0;
            lastMsg.Angular.Y = 0;
            lastMsg.Angular.Z = 0;
        }
        transform.Translate(-(float)lastMsg.Linear.Y * Time.deltaTime, (float)lastMsg.Linear.Z * Time.deltaTime, (float)lastMsg.Linear.X * Time.deltaTime, Space.Self);
        transform.Rotate((float)lastMsg.Angular.Y * Mathf.Rad2Deg * Time.deltaTime, -(float)lastMsg.Angular.Z * Mathf.Rad2Deg * Time.deltaTime, 0.0F, Space.Self);
    }

    void Callback(geometry_msgs.msg.Twist msg)
    {
        lastTime = DateTime.Now;
        lastMsg = msg;
    }
    
}
}  // namespace ROS2
