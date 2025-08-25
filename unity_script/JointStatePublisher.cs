using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class JointStatePublisher : MonoBehaviour
{
    public string topicName = "/unity_joint_states";
    public ArticulationBody[] jointArticulationBodies;

    ROSConnection ros;
    JointStateMsg jointStateMsg;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(topicName);

        jointStateMsg = new JointStateMsg();
        jointStateMsg.name = new string[jointArticulationBodies.Length];
        jointStateMsg.position = new double[jointArticulationBodies.Length];
        jointStateMsg.velocity = new double[jointArticulationBodies.Length];
        jointStateMsg.effort = new double[jointArticulationBodies.Length];

        for (int i = 0; i < jointArticulationBodies.Length; i++)
        {
            jointStateMsg.name[i] = "panda_joint" + (i + 1);  // panda_joint1 ~ panda_joint7
        }
    }

    void FixedUpdate()
    {
        for (int i = 0; i < jointArticulationBodies.Length; i++)
        {
            jointStateMsg.position[i] = jointArticulationBodies[i].jointPosition[0];  // radians
            jointStateMsg.velocity[i] = 0.0;  // optional
            jointStateMsg.effort[i] = 0.0;    // optional
        }

        ros.Publish(topicName, jointStateMsg);
    }
}
