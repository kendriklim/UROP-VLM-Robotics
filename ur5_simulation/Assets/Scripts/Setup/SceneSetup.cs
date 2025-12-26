using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

using Unity.Robotics.UrdfImporter.Control; //for ManualController

[System.Serializable]
public class RobotArms
{
    public GameObject robot;
    public GameObject[] targets;
}

public class SceneSetup : MonoBehaviour
{
    [Header("Scene References")]
    [SerializeField]
    private RobotArms[] robotArms;
    [HideInInspector]
    public GameObject[] robots;
    private List<GameObject[]> robotTargetsList = new List<GameObject[]>();

    [Header("Agentic AI")]
    public bool allRobotsActive = false;
    public bool pythonSocket = false;
    
    [HideInInspector]
    public AgentRobotController agentRobotController;
    [HideInInspector]
    public AgentTrajectoryController agentTrajectoryController;
    [HideInInspector]
    public AgentTrajectory agentTrajectory;
    [HideInInspector]
    public PythonSocketSetup pythonSocketSetup;


    [HideInInspector]
    public int selectedRobotIndex;
    
    [HideInInspector]
    public UnifiedRobotController unifiedRobotController;
    [HideInInspector]
    public SuctionController suctionController;
    [HideInInspector]
    public CSVTrajectoryController csvTrajectoryController;
    [HideInInspector]
    public CSVTrajectoryExample csvTrajectoryExample;

    void Start()
    {
        if (robotArms.Length == 0 || robotArms == null)
        {
            Debug.LogError("Robots not assigned in the inspector.");
            return;
        }

        // foreach (GameObject target in targets)
        // {
        //     Collider blockCollider = target.GetComponent<Collider>();
        //     Debug.Log(target.name + " Original Position: " + blockCollider.bounds.center);
        // }

        int robotCount = robotArms.Length;
        robots = new GameObject[robotCount];
        int robotIdx = 0;

        foreach (RobotArms robotArm in robotArms)
        {
            robots[robotIdx] = robotArm.robot;
            robotTargetsList.Add(robotArm.targets);
            robotIdx += 1;
        }

        if (allRobotsActive)
            {
                agentRobotController = this.gameObject.AddComponent<AgentRobotController>();
                agentTrajectoryController = this.gameObject.AddComponent<AgentTrajectoryController>();
                agentTrajectory = this.gameObject.AddComponent<AgentTrajectory>();

                agentTrajectoryController.robots = robots;
                agentTrajectory.robots = robots;
            }

        selectedRobotIndex = 0; // Default to first robot
        int idx = 0;
        foreach (GameObject robot in robots)
        {     
            RobotSetup(robot, idx);
            idx += 1;
        }


        if (pythonSocket)
        {
            if (allRobotsActive) pythonSocketSetup = this.gameObject.AddComponent<PythonSocketSetup>();
            else pythonSocket = false;
        }
    }

    void RobotSetup(GameObject robot, int idx)
    {
        Debug.Log($"RobotSetup: {robot.name}");

        GameObject[] targets = robotTargetsList[idx];
        foreach (GameObject target in targets)
        {
            Collider blockCollider = target.GetComponent<Collider>();
            Debug.Log(target.name + " Original Position: " + blockCollider.bounds.center);
        }

        SuctionController suctionController = robot.GetComponent<SuctionController>();
        if (suctionController != null) suctionController.targetBlocks = targets;

        ManualController manualController = robot.GetComponent<ManualController>();
        PickAndPlaceController pickAndPlaceController = robot.GetComponent<PickAndPlaceController>();
        UnifiedRobotController unifiedController = robot.GetComponent<UnifiedRobotController>();
        CSVTrajectoryController csvController = robot.GetComponent<CSVTrajectoryController>();
        CSVTrajectoryExample csvExample = robot.GetComponent<CSVTrajectoryExample>();

        if (allRobotsActive)
        {
            if (manualController != null) manualController.enabled = false;
            if (pickAndPlaceController != null) pickAndPlaceController.enabled = false;
            if (unifiedController != null) unifiedController.enabled = false;
            if (csvController != null) csvController.enabled = false;
            if (csvExample != null) csvExample.enabled = false;
        }
        else
        {
            if(idx == selectedRobotIndex){
                ToggleRobotOn(robot);
            }
            else{
                ToggleRobotOff(robot);
            }
        }
    }

    void Update()
    {
        if (allRobotsActive) return;

        bool selectRobot = Input.GetKeyDown(KeyCode.P);
        if (selectRobot)
        {
            selectedRobotIndex = (selectedRobotIndex + 1) % robots.Length;
			ToggleRobotOff(robots[(selectedRobotIndex + 1) % robots.Length]);
	        ToggleRobotOn(robots[selectedRobotIndex]);
        }
    }

    void ToggleRobotOff(GameObject robot)
    {
        if (robot != null)
        {
            //disable all controllers in robot
            ManualController manualController = robot.GetComponent<ManualController>();
            if (manualController != null) manualController.enabled = false;

            PickAndPlaceController pickAndPlaceController = robot.GetComponent<PickAndPlaceController>();
            if (pickAndPlaceController != null) pickAndPlaceController.enabled = false;

            UnifiedRobotController unifiedController = robot.GetComponent<UnifiedRobotController>();
            if (unifiedController != null) unifiedController.enabled = false;
            
            SuctionController suctionController = robot.GetComponent<SuctionController>();
            if (suctionController != null) suctionController.enabled = false;
            
            CSVTrajectoryController csvController = robot.GetComponent<CSVTrajectoryController>();
            if (csvController != null) csvController.enabled = false;
            
            CSVTrajectoryExample csvExample = robot.GetComponent<CSVTrajectoryExample>();
            if (csvExample != null) csvExample.enabled = false;
        }
    }

    void ToggleRobotOn(GameObject robot)
    {
		print("toggled robot on " + robot);
        if (robot != null)
        {
            //enable all controllers in robot
            UnifiedRobotController unifiedController = robot.GetComponent<UnifiedRobotController>();
            if (unifiedController != null) unifiedController.enabled = true;
            
            SuctionController suctionController = robot.GetComponent<SuctionController>();
            if (suctionController != null) suctionController.enabled = true;
            
            CSVTrajectoryController csvController = robot.GetComponent<CSVTrajectoryController>();
            if (csvController != null) csvController.enabled = true;
            
            CSVTrajectoryExample csvExample = robot.GetComponent<CSVTrajectoryExample>();
            if (csvExample != null) csvExample.enabled = true;
        }
    }

    void OnGUI()
    {
        if (allRobotsActive) return;

        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = 16;
        style.normal.textColor = Color.white;

        string robotName = robots[selectedRobotIndex].name;
        GUI.Label(new Rect(Screen.width - 250, 10, 300, 30), "Selected Robot: " + robotName, style);
        GUI.Label(new Rect(Screen.width - 250, 35, 300, 20), "Press 'P' to switch", style);
    }

}
