using UnityEngine;
using System.IO;

public class FPSLogger : MonoBehaviour
{
    private float deltaTime = 0.0f;
    private string filePath;

    void Start()
    {

        filePath = Application.dataPath + "/fps_log.csv";
        File.WriteAllText(filePath, "Time(s),FPS\n");
    }

    void Update()
    {
        deltaTime += (Time.unscaledDeltaTime - deltaTime) * 0.1f;
        float fps = 1.0f / deltaTime;


        string logLine = Time.time.ToString("F2") + "," + fps.ToString("F2") + "\n";
        File.AppendAllText(filePath, logLine);
    }
}
