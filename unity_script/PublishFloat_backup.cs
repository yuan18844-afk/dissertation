using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class PublishFloat64 : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float64Msg>("/test_float64");
        InvokeRepeating(nameof(PublishMessage), 1f, 1f);
    }

    void PublishMessage()
    {
        Float64Msg msg = new Float64Msg(0.5);
        ros.Publish("/test_float64", msg);
        Debug.Log("✅ Repeated publish to /test_float64");
    }
}
