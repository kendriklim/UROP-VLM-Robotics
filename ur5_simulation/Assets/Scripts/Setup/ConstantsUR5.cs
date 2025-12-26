using UnityEngine;
using System;
using System.Collections.Generic;

//using Preliy.Flange;

public static class ConstantsUR5
{
    public static readonly Dictionary<string, string> JointNames = new Dictionary<string, string>()
    {
        {"shoulder_link", "base_joint"},
        {"upper_arm_link", "shoulder_joint"},
        {"forearm_link", "elbow_joint"},
        {"wrist_1_link", "wrist_1_joint"},
        {"wrist_2_link", "wrist_2_joint"},
        {"wrist_3_link", "wrist_3_joint"},
        {"ee_link", "end_effector"}
    };

    public const int JointStiffness = 10000;
    public const int JointDamping = 100;
    public const int JointFriction = 10;
    public const int AngularDamping = 10;
    public const int ForceLimit = 1000;
    public const float JointSpeed = 15f; //15 deg/s
    public const float JointSlowSpeed = 1f; //1 deg/s
    public const float JointSuperSlowSpeed = 0.01f; //0.01 deg/s
    public const float JointAcceleration = 30f; //20 deg/s^2
    public const float JointSlowAcceleration = 2f; //2 deg/s^2
    public const float JointSuperSlowAcceleration = 0.01f; //0.01 deg/s^2
    public const float JointVelocityLimit = 1f; // theoretical max is 3.15
    public const float SmallestJointAngle = -359f;
    public const float LargestJointAngle = 359f;

    public const int JointCount = 6;
    public const int BaseIndex = 3;
    public const int EndEffectorIndex = 11;
    public const int CoordinateRecordLength = JointCount + 3; //no of joints + end effector + block + platform

    public static readonly float[] StableStartingRotations = new float[] //rotations in radians
    {
        0f,     // Base rotation
        -0.8f,  // Shoulder (lowered to better reach cubes)
        1.0f,   // Elbow (adjusted to position arm lower)
        -1.5f,  // Wrist 1 (adjusted to position suction cup better)
        -1.57f, // Wrist 2
        0f      // Wrist 3
    };

    public struct DenavitHartenbergParameters
    {
        public float a;
        public float d;
        public float alpha;
        public float theta;

        public DenavitHartenbergParameters(float aDH = 0, float dDH = 0, float alphaDH = 0, float thetaDH = 0)
        {
            a = aDH;
            d = dDH;
            alpha = alphaDH;
            theta = thetaDH;
        }
    }

    private const float halfPi = 1.57079632f;

    public static readonly DenavitHartenbergParameters BaseDH = new DenavitHartenbergParameters();
    public static readonly DenavitHartenbergParameters BaseJointDH = new DenavitHartenbergParameters(0, 0.089159f, halfPi);
    public static readonly DenavitHartenbergParameters ShoulderJointDH = new DenavitHartenbergParameters(-0.425f);
    public static readonly DenavitHartenbergParameters ElbowJointDH = new DenavitHartenbergParameters(-0.39225f);
    public static readonly DenavitHartenbergParameters Wrist1JointDH = new DenavitHartenbergParameters(0, 0.10915f, halfPi);
    public static readonly DenavitHartenbergParameters Wrist2JointDH = new DenavitHartenbergParameters(0, 0.09465f, -halfPi);
    public static readonly DenavitHartenbergParameters Wrist3JointDH = new DenavitHartenbergParameters(0, 0.0823f);
    public static readonly DenavitHartenbergParameters EndEffectorDH = new DenavitHartenbergParameters();

    /*
    public FrameConfig(float alpha, float a, float d, float theta, string name = null)
    {
        _name = name;
        _alpha = alpha;
        _a = a;
        _d = d;
        _theta = theta;
    }
    */

    // private const float halfPi = 1.57079632f;

    // public static readonly FrameConfig BaseFrameConfig = FrameConfig.Default;
    // public static readonly FrameConfig BaseJointFrameConfig = new FrameConfig(halfPi, 0, 0.089159f, 0);
    // public static readonly FrameConfig ShoulderJointFrameConfig = new FrameConfig(0, -0.425f, 0, 0);
    // public static readonly FrameConfig ElbowJointFrameConfig = new FrameConfig(0, -0.39225f, 0, 0);
    // public static readonly FrameConfig Wrist1JointFrameConfig = new FrameConfig(halfPi, 0, 0.10915f, 0);
    // public static readonly FrameConfig Wrist2JointFrameConfig = new FrameConfig(-halfPi, 0, 0.09465f, 0);
    // public static readonly FrameConfig Wrist3JointFrameConfig = new FrameConfig(0, 0, 0.0823f, 0);
    // public static readonly FrameConfig EndEffectorFrameConfig = FrameConfig.Default;

    /*
    public JointConfig(TransformJoint.JointType type, Vector2 limits, float offset, float factor, float speedMax, float accMax, string name = null)
    {
        _name = name;
        _type = type;
        _limits = limits;
        _offset = offset;
        _factor = factor;
        _speedMax = speedMax;
        _accMax = accMax;
    }
    */

    //public static readonly JointConfig defaultJointConfig = new JointConfig(TransformJoint.JointType.Rotation, new Vector2(-360, 360), 0, 1f, 15f, 10f);

}
