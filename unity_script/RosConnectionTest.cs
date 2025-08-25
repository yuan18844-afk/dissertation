using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RosConnectionTest : MonoBehaviour
{
    void Start()
    {
        Debug.Log("[RosConnectionTest] Script Start()");
        var ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("/joint_states", OnJointState);
    }

    void OnJointState(JointStateMsg msg)
    {
        Debug.Log($"[RosConnectionTest] Received joint_states: {msg.name[0]} = {msg.position[0]:F2} rad");
    }
}
