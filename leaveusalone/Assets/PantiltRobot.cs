using JetBrains.Annotations;
using System.Collections;
using System.Collections.Generic;
using System.Net.Sockets;
using UnityEngine;
using MessagePack;
using System;
using TMPro;
using UnityEditor;
using System.Linq;
using System.Drawing.Text;
using MessagePack.Resolvers;
using System.Net.NetworkInformation;

[MessagePackObject]
public class UdpMsg
{

    [Key(0)]
    public byte cmd;
    [Key(1)]
    public float[] data;

    public UdpMsg() { }

    public UdpMsg(byte cmd, float[] data)
    {
        this.cmd = cmd;
        this.data = data;
    }
}


[ExecuteAlways]
public class PantiltRobot : MonoBehaviour
{
    public float pan = 0;
    public float tilt = 0;

    public string udp_address = "pantilt_bot_1";
    public int udp_port = 1234;
    //public bool liveSend = false;

    public GameObject lookatTarget;

    public float lookAtAccel = 20.0f;

    private readonly UdpClient udp_client = new();

    private Transform eye;
    private Transform pan_tr;
    private Transform tilt_tr;

    [System.NonSerialized]
    public bool isLiveTracking = false;



    public enum UdpCommands
    {
        Home = 0,
        MovePanTilt = 1,
        PT_Acceleration = 2,
        MovePan_Accel = 3,
        MoveTilt_Accel = 4,
    }

    // Start is called before the first frame update
    void Start()
    {
        // ProcessCurves();
    }

    // Update is called once per frame
    void Update()
    {
        if (pan_tr == null)
            pan_tr = gameObject.transform.Find("spine:1");
        if (tilt_tr == null)
            tilt_tr = gameObject.transform.Find("spine:1/camera:1/camera");
        if (eye == null)
            eye = gameObject.transform.Find("spine:1/camera:1/camera/cam:1/Camera");

        if (lookatTarget != null)
        {
            var target = lookatTarget.transform.position;

            if ((target - eye.position).magnitude > 0.0)
            {
                var lookAt = Matrix4x4.LookAt(eye.position, target, Vector3.up);

                var euler = lookAt.rotation.eulerAngles;

                tilt = -euler[0];
                if (tilt < -180)
                    tilt += 360;
                pan = euler[1];
                if (pan > 180)
                    pan -= 360;
                // print($"{euler} -> {pan} {tilt}");
            }
        }

        pan_tr.localEulerAngles = new Vector3(0, 0, pan);
        tilt_tr.localEulerAngles = new Vector3(tilt, 0, 0);
    }

    public void StartLiveTracking()
    {
        // if (lookatTarget == null)
        //     return;
        
        // var anim = transform.parent.GetComponent<Animation>();
        // anim.Stop();

        isLiveTracking = true;
        InvokeRepeating(nameof(LiveTrackingMoveUDP), 0.0f, 0.2f);
    }

    public void StopLiveTracking()
    {
        CancelInvoke(nameof(LiveTrackingMoveUDP));
        
        // var anim = GetComponent<Animation>();
        // anim.Play();

        isLiveTracking = false;
    }

    public void LiveTrackingMoveUDP()
    {
        MoveUDP(lookAtAccel);
    }

    public void MoveUDP(float accel = 20)
    {
        // pan = pan_tr.localEulerAngles.z;
        // tilt = tilt_tr.localEulerAngles.x;

        // var tilt_t = tilt;
        // if (udp_address == "pantilt-robot_b.hq.bitwaescherei.net") { tilt_t = -tilt_t; }
        var msg = new UdpMsg((byte)UdpCommands.PT_Acceleration, new float[] { pan, tilt, accel, accel });

        var dgram = MessagePackSerializer.Serialize(msg);
        var json = MessagePackSerializer.ConvertToJson(dgram);
        print(json);
        udp_client.Send(dgram, dgram.Length, udp_address, udp_port);
    }

    public void Home()
    {
        var msg = new UdpMsg((byte)UdpCommands.Home, new float[] {0, 0});
        var dgram = MessagePackSerializer.Serialize(msg);
        print(MessagePackSerializer.ConvertToJson(dgram));
        udp_client.Send(dgram, dgram.Length, udp_address, udp_port);
    }
    
    public void SendUdpMsg(UdpMsg msg)
    {
        var dgram = MessagePackSerializer.Serialize(msg);
        var json = MessagePackSerializer.ConvertToJson(dgram);
        print(json);
        udp_client.Send(dgram, dgram.Length, udp_address, udp_port);
    }

    // public void ProcessCurves()
    // {
    //     var animation = gameObject.GetComponent<Animation>();
    //     var clips = AnimationUtility.GetAnimationClips(gameObject);

    //     var bindings = AnimationUtility.GetCurveBindings(clips[0]);

    //     var clip = clips[0];

    //     var pan_binding = bindings.Single(b => b.propertyName == "pan");
    //     var tilt_binding = bindings.Single(b => b.propertyName == "tilt");

    //     var events = new List<AnimationEvent>();
    //     AddAnimationEvents(clip, pan_binding, UdpCommands.MovePan_Accel, events);
    //     AddAnimationEvents(clip, tilt_binding, UdpCommands.MoveTilt_Accel, events);
    //     AnimationUtility.SetAnimationEvents(clip, events.ToArray());
    // }

    // public void AddAnimationEvents(AnimationClip clip, EditorCurveBinding curveBinding, UdpCommands cmd, List<AnimationEvent> allEvents)
    // {
    //     var curve = AnimationUtility.GetEditorCurve(clip, curveBinding);
    //     for (int i = 1; i < curve.keys.Length; i++)
    //     {
    //         var c = curve.keys[i - 1];
    //         var n = curve.keys[i];

    //         float dt = n.time - c.time;
    //         float dp = n.value - c.value;

    //         float acc = Math.Abs(dp) / (dt * dt / 4);
    //         // float vmax = acc * dt / 2;

    //         var eventArg = ScriptableObject.CreateInstance<KeyframeEventArg>();

    //         var pos = n.value;
    //         if (cmd == UdpCommands.MoveTilt_Accel && udp_address == "pantilt-robot_b.hq.bitwaescherei.net") { pos = -pos; }
    //         // if (cmd == UdpCommands.MoveTilt_Accel) { pos = -pos; }
    //         eventArg.udpMsg = new UdpMsg((byte)cmd, new float[] { pos, acc });
    //         eventArg.keyframe = c;
    //         eventArg.target = this;
    //         var evt = new AnimationEvent
    //         {
    //             functionName = "OnKeyframe",
    //             objectReferenceParameter = eventArg,
    //             time = curve.keys[i - 1].time
    //         };
    //         allEvents.Add(evt);
    //     }
    // }
}



#if UNITY_EDITOR
[CustomEditor(typeof(PantiltRobot)), CanEditMultipleObjects]
public class PantiltEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        var target = (PantiltRobot)this.target;

        if (GUILayout.Button("Move UDP"))
        {
            target.MoveUDP();
        }
        if (GUILayout.Button("Home axes"))
        {
            target.Home();
        }

        if (!target.isLiveTracking)
        {
            if (GUILayout.Button("Start Live Move"))
            {
                target.StartLiveTracking();
            }
        }
        else
        {
            if (GUILayout.Button("STOP Live Move"))
            {
                target.StopLiveTracking();
            }
        }
    }
}
#endif