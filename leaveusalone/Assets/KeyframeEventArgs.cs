using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class KeyframeEventArgs : ScriptableObject
{
    public UdpMsg udpMsg;
    public PantiltRobot target;
    public Keyframe keyframe;

    public KeyframeEventArgs()
    {
            
    }
}

