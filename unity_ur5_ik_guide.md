# Unity UR5 Inverse Kinematics Setup Guide

A complete guide to setting up UR5 robot simulation with IK in Unity for your VLA project.

---

## Overview

```
OpenVLA (Python)                    Unity (C#)
      │                                  │
      │  action = [Δx,Δy,Δz,...]        │
      │                                  │
      └──────── TCP Socket ─────────────→│
                                         │
                                    ┌────┴────┐
                                    │   IK    │
                                    │ Solver  │
                                    └────┬────┘
                                         │
                                    joint angles
                                         │
                                         ▼
                                   ┌──────────┐
                                   │   UR5    │
                                   │  Robot   │
                                   └──────────┘
```

---

## Part 1: Unity Project Setup

### Step 1: Create New Unity Project

1. Open Unity Hub
2. Create new project with **3D (URP)** or **3D** template
3. Use Unity **2021.3 LTS** or newer

### Step 2: Install Required Packages

Open **Window → Package Manager** and install:

```
Via Package Manager (Add package from git URL):
- https://github.com/Unity-Technologies/URDF-Importer.git
- https://github.com/Unity-Technologies/ROS-TCP-Connector.git (optional, for ROS)
```

Or add to `Packages/manifest.json`:

```json
{
  "dependencies": {
    "com.unity.robotics.urdf-importer": "https://github.com/Unity-Technologies/URDF-Importer.git",
    "com.unity.robotics.ros-tcp-connector": "https://github.com/Unity-Technologies/ROS-TCP-Connector.git"
  }
}
```

### Step 3: Configure Physics

1. Go to **Edit → Project Settings → Physics**
2. Set **Solver Type** to `Temporal Gauss Seidel`
3. Set **Default Solver Iterations** to `20`
4. Set **Default Solver Velocity Iterations** to `20`

---

## Part 2: Import UR5 Robot

### Step 1: Get UR5 URDF

Download the official UR5 URDF:

```bash
# Option 1: From Universal Robots GitHub
git clone https://github.com/ros-industrial/universal_robot.git
# URDF is in: universal_robot/ur_description/urdf/ur5.urdf.xacro

# Option 2: Direct download (pre-converted)
# https://github.com/qian256/ur5_unity
```

If you have a `.xacro` file, convert it to `.urdf`:

```bash
# Requires ROS installed
rosrun xacro xacro --inorder -o ur5.urdf ur5.urdf.xacro
```

### Step 2: Import URDF into Unity

1. Copy the `ur5.urdf` file and mesh folders to `Assets/URDF/ur5/`
2. In Unity, right-click on `ur5.urdf`
3. Select **Import Robot from URDF**
4. Settings:
   - **Axis Type**: `Z Axis` (ROS standard)
   - **Mesh Decomposer**: `VHACD` (recommended)
   - Click **Import**

### Step 3: Verify Import

After import, you should see:

```
ur5 (GameObject)
├── base_link
│   └── shoulder_link
│       └── upper_arm_link
│           └── forearm_link
│               └── wrist_1_link
│                   └── wrist_2_link
│                       └── wrist_3_link
│                           └── ee_link (end effector)
```

Each joint link should have an **ArticulationBody** component.

---

## Part 3: UR5 Analytical IK Solver (C#)

Create a new C# script `UR5IKSolver.cs`:

```csharp
using UnityEngine;
using System;
using System.Collections.Generic;

/// <summary>
/// Analytical Inverse Kinematics solver for UR5 robot.
/// Based on "Analytic IK for Universal Robots UR-5/UR-10" by Kelsey P. Hawkins
/// </summary>
public class UR5IKSolver : MonoBehaviour
{
    // UR5 DH Parameters (in meters)
    private const float d1 = 0.089159f;
    private const float a2 = -0.42500f;
    private const float a3 = -0.39225f;
    private const float d4 = 0.10915f;
    private const float d5 = 0.09465f;
    private const float d6 = 0.0823f;

    /// <summary>
    /// Solve IK for target position and rotation
    /// </summary>
    /// <param name="targetPosition">Target end-effector position in world space</param>
    /// <param name="targetRotation">Target end-effector rotation in world space</param>
    /// <param name="currentAngles">Current joint angles (for selecting closest solution)</param>
    /// <returns>Array of 6 joint angles in radians, or null if no solution</returns>
    public float[] SolveIK(Vector3 targetPosition, Quaternion targetRotation, float[] currentAngles = null)
    {
        // Convert Unity coordinates to robot base frame
        // Unity: Y-up, Z-forward
        // Robot: Z-up, X-forward (typically)
        Vector3 pos = TransformToRobotFrame(targetPosition);
        Matrix4x4 rotMatrix = Matrix4x4.Rotate(targetRotation);

        // Get all possible solutions
        List<float[]> solutions = GetAllSolutions(pos, rotMatrix);

        if (solutions.Count == 0)
        {
            Debug.LogWarning("No IK solution found - target may be unreachable");
            return null;
        }

        // Select best solution (closest to current angles)
        return SelectBestSolution(solutions, currentAngles);
    }

    /// <summary>
    /// Solve IK using delta actions (like OpenVLA output)
    /// </summary>
    public float[] SolveIKDelta(Vector3 currentPos, Quaternion currentRot, 
                                 float[] deltaAction, float[] currentAngles)
    {
        // deltaAction = [Δx, Δy, Δz, Δroll, Δpitch, Δyaw]
        Vector3 deltaPos = new Vector3(deltaAction[0], deltaAction[1], deltaAction[2]);
        Vector3 deltaEuler = new Vector3(deltaAction[3], deltaAction[4], deltaAction[5]) * Mathf.Rad2Deg;

        Vector3 targetPos = currentPos + deltaPos;
        Quaternion deltaRot = Quaternion.Euler(deltaEuler);
        Quaternion targetRot = currentRot * deltaRot;

        return SolveIK(targetPos, targetRot, currentAngles);
    }

    private Vector3 TransformToRobotFrame(Vector3 unityPos)
    {
        // Adjust based on how your robot is oriented in Unity
        // This assumes robot base is at origin with Z-up in robot frame
        return new Vector3(unityPos.x, unityPos.z, unityPos.y);
    }

    private List<float[]> GetAllSolutions(Vector3 pos, Matrix4x4 rotMatrix)
    {
        List<float[]> solutions = new List<float[]>();

        float x = pos.x;
        float y = pos.y;
        float z = pos.z;

        // Extract rotation matrix columns
        Vector3 r13 = new Vector3(rotMatrix.m02, rotMatrix.m12, rotMatrix.m22);
        
        // Wrist center position
        Vector3 wristCenter = pos - d6 * r13;
        float xc = wristCenter.x;
        float yc = wristCenter.y;
        float zc = wristCenter.z;

        // Joint 1: Two solutions (shoulder left/right)
        float[] theta1Options = GetTheta1Options(xc, yc);

        foreach (float theta1 in theta1Options)
        {
            // Joint 5: Two solutions (wrist up/down)
            float[] theta5Options = GetTheta5Options(theta1, rotMatrix);

            foreach (float theta5 in theta5Options)
            {
                // Joint 6
                float theta6 = GetTheta6(theta1, theta5, rotMatrix);

                // Joints 2, 3, 4
                float[][] theta234Options = GetTheta234Options(theta1, wristCenter);

                foreach (float[] theta234 in theta234Options)
                {
                    float[] solution = new float[6];
                    solution[0] = theta1;
                    solution[1] = theta234[0];
                    solution[2] = theta234[1];
                    solution[3] = theta234[2];
                    solution[4] = theta5;
                    solution[5] = theta6;

                    // Validate solution
                    if (IsValidSolution(solution))
                    {
                        solutions.Add(solution);
                    }
                }
            }
        }

        return solutions;
    }

    private float[] GetTheta1Options(float xc, float yc)
    {
        List<float> options = new List<float>();
        float r = Mathf.Sqrt(xc * xc + yc * yc);

        if (r < 1e-6f)
        {
            options.Add(0f);
            return options.ToArray();
        }

        float alpha = Mathf.Atan2(yc, xc);

        if (Mathf.Abs(d4 / r) <= 1f)
        {
            float beta = Mathf.Asin(Mathf.Clamp(d4 / r, -1f, 1f));
            options.Add(alpha - beta + Mathf.PI);
            options.Add(alpha + beta);
        }
        else
        {
            options.Add(alpha);
        }

        return options.ToArray();
    }

    private float[] GetTheta5Options(float theta1, Matrix4x4 rotMatrix)
    {
        List<float> options = new List<float>();
        
        float c1 = Mathf.Cos(theta1);
        float s1 = Mathf.Sin(theta1);

        // P5_6_y calculation
        float p5_6_y = -s1 * rotMatrix.m02 + c1 * rotMatrix.m12;

        if (Mathf.Abs(p5_6_y) <= 1f)
        {
            float theta5 = Mathf.Acos(Mathf.Clamp(p5_6_y, -1f, 1f));
            options.Add(theta5);
            options.Add(-theta5);
        }

        return options.ToArray();
    }

    private float GetTheta6(float theta1, float theta5, Matrix4x4 rotMatrix)
    {
        float c1 = Mathf.Cos(theta1);
        float s1 = Mathf.Sin(theta1);
        float s5 = Mathf.Sin(theta5);

        if (Mathf.Abs(s5) < 1e-6f)
        {
            return 0f; // Singularity
        }

        float x = (s1 * rotMatrix.m00 - c1 * rotMatrix.m10) / s5;
        float y = (s1 * rotMatrix.m01 - c1 * rotMatrix.m11) / s5;

        return Mathf.Atan2(y, x);
    }

    private float[][] GetTheta234Options(float theta1, Vector3 wristCenter)
    {
        List<float[]> options = new List<float[]>();

        float c1 = Mathf.Cos(theta1);
        float s1 = Mathf.Sin(theta1);

        // Transform to joint 1 frame
        float px = c1 * wristCenter.x + s1 * wristCenter.y;
        float py = -s1 * wristCenter.x + c1 * wristCenter.y - d4;
        float pz = wristCenter.z - d1;

        // Distance from joint 2 to wrist center
        float D = Mathf.Sqrt(px * px + pz * pz);

        float L2 = Mathf.Abs(a2);
        float L3 = Mathf.Abs(a3);

        // Check reachability
        if (D > L2 + L3 || D < Mathf.Abs(L2 - L3))
        {
            return options.ToArray();
        }

        // Joint 3: Two solutions (elbow up/down)
        float cosTheta3 = (D * D - L2 * L2 - L3 * L3) / (2 * L2 * L3);
        cosTheta3 = Mathf.Clamp(cosTheta3, -1f, 1f);

        float[] theta3Options = new float[] { Mathf.Acos(cosTheta3), -Mathf.Acos(cosTheta3) };

        foreach (float theta3 in theta3Options)
        {
            // Joint 2
            float beta = Mathf.Atan2(pz, px);
            float psi = Mathf.Atan2(L3 * Mathf.Sin(theta3), L2 + L3 * Mathf.Cos(theta3));
            float theta2 = beta - psi;

            // Joint 4 (simplified - you may need to refine this based on orientation)
            float theta4 = 0f; // Placeholder - compute based on remaining rotation

            options.Add(new float[] { theta2, theta3, theta4 });
        }

        return options.ToArray();
    }

    private bool IsValidSolution(float[] solution)
    {
        // UR5 joint limits (in radians)
        float[] minLimits = { -2 * Mathf.PI, -2 * Mathf.PI, -Mathf.PI, -2 * Mathf.PI, -2 * Mathf.PI, -2 * Mathf.PI };
        float[] maxLimits = { 2 * Mathf.PI, 2 * Mathf.PI, Mathf.PI, 2 * Mathf.PI, 2 * Mathf.PI, 2 * Mathf.PI };

        for (int i = 0; i < 6; i++)
        {
            if (float.IsNaN(solution[i]) || float.IsInfinity(solution[i]))
                return false;

            if (solution[i] < minLimits[i] || solution[i] > maxLimits[i])
                return false;
        }

        return true;
    }

    private float[] SelectBestSolution(List<float[]> solutions, float[] currentAngles)
    {
        if (currentAngles == null || currentAngles.Length != 6)
        {
            return solutions[0];
        }

        float minDistance = float.MaxValue;
        float[] bestSolution = solutions[0];

        foreach (float[] solution in solutions)
        {
            float distance = 0f;
            for (int i = 0; i < 6; i++)
            {
                float diff = NormalizeAngle(solution[i] - currentAngles[i]);
                distance += diff * diff;
            }

            if (distance < minDistance)
            {
                minDistance = distance;
                bestSolution = solution;
            }
        }

        return bestSolution;
    }

    private float NormalizeAngle(float angle)
    {
        while (angle > Mathf.PI) angle -= 2 * Mathf.PI;
        while (angle < -Mathf.PI) angle += 2 * Mathf.PI;
        return angle;
    }
}
```

---

## Part 4: UR5 Robot Controller

Create `UR5Controller.cs`:

```csharp
using UnityEngine;
using System.Collections;

public class UR5Controller : MonoBehaviour
{
    [Header("Robot Configuration")]
    public ArticulationBody[] joints = new ArticulationBody[6];
    public Transform endEffector;

    [Header("IK Settings")]
    public UR5IKSolver ikSolver;
    public float moveDuration = 0.5f;
    public float stiffness = 100000f;
    public float damping = 10000f;

    [Header("Gripper")]
    public ArticulationBody gripperJoint;
    public float gripperOpenAngle = 0f;
    public float gripperCloseAngle = 0.04f;

    private float[] currentJointAngles = new float[6];
    private Coroutine moveCoroutine;

    void Start()
    {
        // Auto-find joints if not assigned
        if (joints[0] == null)
        {
            FindJoints();
        }

        // Configure joint drives
        ConfigureJointDrives();

        // Initialize current angles
        UpdateCurrentAngles();

        // Get IK solver
        if (ikSolver == null)
        {
            ikSolver = GetComponent<UR5IKSolver>();
            if (ikSolver == null)
            {
                ikSolver = gameObject.AddComponent<UR5IKSolver>();
            }
        }
    }

    void FindJoints()
    {
        string[] jointNames = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };

        ArticulationBody[] allBodies = GetComponentsInChildren<ArticulationBody>();

        for (int i = 0; i < jointNames.Length; i++)
        {
            foreach (ArticulationBody body in allBodies)
            {
                if (body.name.Contains(jointNames[i]) || 
                    body.transform.parent?.name.Contains(jointNames[i]) == true)
                {
                    joints[i] = body;
                    break;
                }
            }
        }

        // Find end effector
        Transform[] allTransforms = GetComponentsInChildren<Transform>();
        foreach (Transform t in allTransforms)
        {
            if (t.name.Contains("ee_link") || t.name.Contains("tool0"))
            {
                endEffector = t;
                break;
            }
        }
    }

    void ConfigureJointDrives()
    {
        foreach (ArticulationBody joint in joints)
        {
            if (joint == null) continue;

            ArticulationDrive drive = joint.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = float.MaxValue;
            joint.xDrive = drive;
        }
    }

    void UpdateCurrentAngles()
    {
        for (int i = 0; i < 6; i++)
        {
            if (joints[i] != null)
            {
                currentJointAngles[i] = joints[i].jointPosition[0];
            }
        }
    }

    /// <summary>
    /// Move to target position and rotation using IK
    /// </summary>
    public void MoveToTarget(Vector3 targetPosition, Quaternion targetRotation)
    {
        float[] targetAngles = ikSolver.SolveIK(targetPosition, targetRotation, currentJointAngles);

        if (targetAngles != null)
        {
            if (moveCoroutine != null)
            {
                StopCoroutine(moveCoroutine);
            }
            moveCoroutine = StartCoroutine(MoveToAngles(targetAngles));
        }
    }

    /// <summary>
    /// Apply delta action from VLA model
    /// </summary>
    public void ApplyDeltaAction(float[] deltaAction)
    {
        // deltaAction = [Δx, Δy, Δz, Δroll, Δpitch, Δyaw, gripper]
        if (deltaAction.Length < 7)
        {
            Debug.LogError("Delta action must have 7 elements");
            return;
        }

        // Get current end effector pose
        Vector3 currentPos = endEffector.position;
        Quaternion currentRot = endEffector.rotation;

        // Solve IK with delta
        float[] poseDelta = new float[6];
        System.Array.Copy(deltaAction, poseDelta, 6);

        float[] targetAngles = ikSolver.SolveIKDelta(currentPos, currentRot, poseDelta, currentJointAngles);

        if (targetAngles != null)
        {
            if (moveCoroutine != null)
            {
                StopCoroutine(moveCoroutine);
            }
            moveCoroutine = StartCoroutine(MoveToAngles(targetAngles));
        }

        // Control gripper
        float gripperCmd = deltaAction[6];
        SetGripper(gripperCmd > 0.5f);
    }

    /// <summary>
    /// Set joint angles directly (in radians)
    /// </summary>
    public void SetJointAngles(float[] angles)
    {
        for (int i = 0; i < Mathf.Min(6, angles.Length); i++)
        {
            if (joints[i] != null)
            {
                ArticulationDrive drive = joints[i].xDrive;
                drive.target = angles[i] * Mathf.Rad2Deg;
                joints[i].xDrive = drive;
            }
        }
        currentJointAngles = (float[])angles.Clone();
    }

    /// <summary>
    /// Set joint angles instantly (teleport)
    /// </summary>
    public void SetJointAnglesInstant(float[] angles)
    {
        for (int i = 0; i < Mathf.Min(6, angles.Length); i++)
        {
            if (joints[i] != null)
            {
                var positions = new ArticulationReducedSpace(angles[i]);
                joints[i].jointPosition = positions;
            }
        }
        currentJointAngles = (float[])angles.Clone();
    }

    private IEnumerator MoveToAngles(float[] targetAngles)
    {
        float[] startAngles = (float[])currentJointAngles.Clone();
        float elapsed = 0f;

        while (elapsed < moveDuration)
        {
            elapsed += Time.deltaTime;
            float t = Mathf.Clamp01(elapsed / moveDuration);
            t = t * t * (3f - 2f * t); // Smoothstep

            float[] interpolated = new float[6];
            for (int i = 0; i < 6; i++)
            {
                interpolated[i] = Mathf.LerpAngle(startAngles[i] * Mathf.Rad2Deg, 
                                                   targetAngles[i] * Mathf.Rad2Deg, t) * Mathf.Deg2Rad;
            }

            SetJointAngles(interpolated);
            yield return null;
        }

        SetJointAngles(targetAngles);
    }

    /// <summary>
    /// Control gripper
    /// </summary>
    public void SetGripper(bool close)
    {
        if (gripperJoint == null) return;

        ArticulationDrive drive = gripperJoint.xDrive;
        drive.target = (close ? gripperCloseAngle : gripperOpenAngle) * Mathf.Rad2Deg;
        gripperJoint.xDrive = drive;
    }

    /// <summary>
    /// Get current end effector pose
    /// </summary>
    public (Vector3 position, Quaternion rotation) GetEndEffectorPose()
    {
        if (endEffector != null)
        {
            return (endEffector.position, endEffector.rotation);
        }
        return (Vector3.zero, Quaternion.identity);
    }

    /// <summary>
    /// Get current joint angles in radians
    /// </summary>
    public float[] GetJointAngles()
    {
        UpdateCurrentAngles();
        return (float[])currentJointAngles.Clone();
    }
}
```

---

## Part 5: Python ↔ Unity Communication

### Unity Side: TCP Server

Create `VLAServer.cs`:

```csharp
using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Newtonsoft.Json;

public class VLAServer : MonoBehaviour
{
    [Header("Server Settings")]
    public int port = 5000;
    public UR5Controller robotController;

    [Header("Camera")]
    public Camera robotCamera;
    public int imageWidth = 640;
    public int imageHeight = 480;

    private TcpListener server;
    private Thread serverThread;
    private bool isRunning = false;
    private ActionCommand pendingCommand = null;
    private readonly object commandLock = new object();

    [Serializable]
    public class ActionCommand
    {
        public float[] action;  // [Δx, Δy, Δz, Δroll, Δpitch, Δyaw, gripper]
    }

    [Serializable]
    public class StateResponse
    {
        public float[] joint_angles;
        public float[] ee_position;
        public float[] ee_rotation;
        public string image_base64;
    }

    void Start()
    {
        StartServer();
    }

    void OnDestroy()
    {
        StopServer();
    }

    void Update()
    {
        // Process pending commands on main thread
        lock (commandLock)
        {
            if (pendingCommand != null)
            {
                robotController.ApplyDeltaAction(pendingCommand.action);
                pendingCommand = null;
            }
        }
    }

    void StartServer()
    {
        isRunning = true;
        serverThread = new Thread(ServerLoop);
        serverThread.Start();
        Debug.Log($"VLA Server started on port {port}");
    }

    void StopServer()
    {
        isRunning = false;
        server?.Stop();
        serverThread?.Join();
    }

    void ServerLoop()
    {
        server = new TcpListener(IPAddress.Any, port);
        server.Start();

        while (isRunning)
        {
            try
            {
                if (server.Pending())
                {
                    TcpClient client = server.AcceptTcpClient();
                    HandleClient(client);
                }
                Thread.Sleep(10);
            }
            catch (Exception e)
            {
                Debug.LogError($"Server error: {e.Message}");
            }
        }
    }

    void HandleClient(TcpClient client)
    {
        NetworkStream stream = client.GetStream();
        byte[] buffer = new byte[4096];

        try
        {
            int bytesRead = stream.Read(buffer, 0, buffer.Length);
            string request = Encoding.UTF8.GetString(buffer, 0, bytesRead);

            string response = ProcessRequest(request);
            byte[] responseBytes = Encoding.UTF8.GetBytes(response);
            stream.Write(responseBytes, 0, responseBytes.Length);
        }
        finally
        {
            client.Close();
        }
    }

    string ProcessRequest(string request)
    {
        try
        {
            if (request.StartsWith("GET_STATE"))
            {
                return GetState();
            }
            else if (request.StartsWith("ACTION:"))
            {
                string actionJson = request.Substring(7);
                ActionCommand cmd = JsonConvert.DeserializeObject<ActionCommand>(actionJson);

                lock (commandLock)
                {
                    pendingCommand = cmd;
                }

                return "{\"status\": \"ok\"}";
            }
            else if (request.StartsWith("GET_IMAGE"))
            {
                return GetImage();
            }
        }
        catch (Exception e)
        {
            return $"{{\"error\": \"{e.Message}\"}}";
        }

        return "{\"error\": \"unknown command\"}";
    }

    string GetState()
    {
        // This needs to be called from main thread via Update()
        var (pos, rot) = robotController.GetEndEffectorPose();
        float[] angles = robotController.GetJointAngles();

        StateResponse state = new StateResponse
        {
            joint_angles = angles,
            ee_position = new float[] { pos.x, pos.y, pos.z },
            ee_rotation = new float[] { rot.eulerAngles.x, rot.eulerAngles.y, rot.eulerAngles.z }
        };

        return JsonConvert.SerializeObject(state);
    }

    string GetImage()
    {
        // Capture camera image (simplified - actual implementation needs main thread)
        // For full implementation, use RenderTexture
        return "{\"image_base64\": \"...\"}";
    }
}
```

### Python Side: Client

Create `unity_client.py`:

```python
import socket
import json
import numpy as np
from PIL import Image
import base64
import io

class UnityRobotClient:
    def __init__(self, host='localhost', port=5000):
        self.host = host
        self.port = port
    
    def _send_command(self, command):
        """Send command and receive response"""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.host, self.port))
            s.sendall(command.encode())
            response = s.recv(65536).decode()
            return json.loads(response)
    
    def get_state(self):
        """Get current robot state"""
        return self._send_command("GET_STATE")
    
    def get_image(self):
        """Get camera image as numpy array"""
        response = self._send_command("GET_IMAGE")
        if 'image_base64' in response:
            img_data = base64.b64decode(response['image_base64'])
            img = Image.open(io.BytesIO(img_data))
            return np.array(img)
        return None
    
    def send_action(self, action):
        """
        Send action to robot
        action: [Δx, Δy, Δz, Δroll, Δpitch, Δyaw, gripper]
        """
        action_list = action.tolist() if isinstance(action, np.ndarray) else action
        command = f"ACTION:{json.dumps({'action': action_list})}"
        return self._send_command(command)
    
    def step(self, action):
        """Execute action and return new state"""
        self.send_action(action)
        return self.get_state()


# Example usage with OpenVLA
if __name__ == "__main__":
    from transformers import AutoModelForVision2Seq, AutoProcessor
    from PIL import Image
    import torch

    # Initialize Unity client
    robot = UnityRobotClient(host='localhost', port=5000)

    # Load OpenVLA
    processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
    vla = AutoModelForVision2Seq.from_pretrained(
        "openvla/openvla-7b",
        torch_dtype=torch.bfloat16,
        trust_remote_code=True
    ).to("cuda:0")

    # Control loop
    instruction = "pick up the red block"
    prompt = f"In: What action should the robot take to {instruction}?\nOut:"

    for step in range(100):
        # Get image from Unity
        image_np = robot.get_image()
        image = Image.fromarray(image_np)

        # Get action from OpenVLA
        inputs = processor(prompt, image).to("cuda:0", dtype=torch.bfloat16)
        action = vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)

        # Send to Unity
        result = robot.step(action)
        print(f"Step {step}: ee_pos = {result['ee_position']}")
```

---

## Part 6: Complete Setup Checklist

### Unity Setup
- [ ] Create new Unity project (2021.3+)
- [ ] Install URDF Importer package
- [ ] Configure physics settings
- [ ] Import UR5 URDF
- [ ] Add `UR5IKSolver.cs` script
- [ ] Add `UR5Controller.cs` script
- [ ] Add `VLAServer.cs` script
- [ ] Configure joint references in Inspector
- [ ] Add camera for robot view

### Python Setup
- [ ] Install dependencies: `pip install torch transformers pillow numpy`
- [ ] Download OpenVLA model
- [ ] Create `unity_client.py`
- [ ] Test connection to Unity

### Testing
1. Run Unity scene
2. Run Python script
3. Verify robot moves in response to actions

---

## Troubleshooting

### Robot doesn't move
- Check ArticulationBody drive settings (stiffness/damping)
- Verify joint references are assigned
- Check console for IK errors

### IK solutions not found
- Target may be outside workspace
- Check coordinate frame transformation
- Verify robot orientation in scene

### Communication issues
- Ensure firewall allows port 5000
- Check IP address if using separate machines
- Verify JSON formatting

---

## Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [UR5 Unity IK](https://github.com/astemeric/ur5_unity_ik)
- [OpenVLA](https://github.com/openvla/openvla)
- [Universal Robots URDF](https://github.com/ros-industrial/universal_robot)
