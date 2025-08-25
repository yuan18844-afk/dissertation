using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;

public class FrankaJointStateSubscriber : MonoBehaviour
{
    [Header("ROS Setting")]
    public string jointStateTopic = "/franka_state_controller/joint_states";

    [Header("Joint")]
    public ArticulationBody[] jointArticulationBodies;

    [Header("debug")]
    public bool debugLog = false;

    private ROSConnection ros;


    private float[] targetJointAngles = new float[7];
    private float[] currentJointAngles = new float[7];
    private bool newDataReceived = false;


    private float minUpdateInterval = 1f / 60f;
    private float lastUpdateTime = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(jointStateTopic, OnJointStatesReceived);
        Debug.Log($"[JointStateSubscriber] subscriber topic: {jointStateTopic}");
    }

    void OnJointStatesReceived(JointStateMsg msg)
    {
        if (Time.time - lastUpdateTime < minUpdateInterval)
            return;

        for (int i = 0; i < msg.name.Length; i++)
        {
            int jointIndex = GetJointIndexByName(msg.name[i]);
            if (jointIndex >= 0 && jointIndex < jointArticulationBodies.Length)
            {
                targetJointAngles[jointIndex] = (float)(msg.position[i] * Mathf.Rad2Deg); // 缓存目标角度
                newDataReceived = true;
            }
        }

        lastUpdateTime = Time.time;
    }

    void FixedUpdate()
    {
        if (!newDataReceived) return;

        for (int i = 0; i < jointArticulationBodies.Length; i++)
        {
            float angleError = Mathf.Abs(currentJointAngles[i] - targetJointAngles[i]);


            if (angleError > 0.05f)
            {
                currentJointAngles[i] = Mathf.Lerp(currentJointAngles[i], targetJointAngles[i], 0.2f);
            }

            var joint = jointArticulationBodies[i];
            var drive = joint.xDrive;
            drive.target = currentJointAngles[i];
            joint.xDrive = drive;

            if (debugLog)
                Debug.Log($"[JointState] panda_joint{i + 1} -> {currentJointAngles[i]:F2}°");
        }
    }

    int GetJointIndexByName(string jointName)
    {
        if (jointName.StartsWith("panda_joint"))
        {
            string numStr = jointName.Replace("panda_joint", "");
            if (int.TryParse(numStr, out int idx))
                return idx - 1;
        }
        return -1;
    }
}
