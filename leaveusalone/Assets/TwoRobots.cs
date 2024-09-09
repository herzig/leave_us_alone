using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEditor;
using UnityEngine;
using static PantiltRobot;

[ExecuteAlways]
public class TwoRobots : MonoBehaviour
{
    public PantiltRobot camA;
    public PantiltRobot camB;

    public float aPan;
    public float aTilt;
    public float bPan;
    public float bTilt;

    public GameObject target;

    private Task resumeTask = null;
    private CancellationTokenSource cancelSource = null;

    public bool IsTracking { get { return isTracking; } }

    private bool isTracking = false;


    // Start is called before the first frame update
    void Start()
    {
#if UNITY_EDITOR
        ProcessCurves();
#endif
    }

    public void StartTracking()
    {
        var anim = GetComponent<Animator>();
        anim.Play("stop", 0);
        anim.speed = 0;

        if (resumeTask != null) // cancel resume (to animation) task if it is currently running
        {
            cancelSource.Cancel();
            Debug.Log("wait for previous task...");
            try
            {
                resumeTask.Wait(); // should be very quick if not instant
            }
            catch (Exception e)
            {
                Debug.Log($"done waiting EX: {e.Message}");
            }
            finally
            {
                cancelSource.Dispose();
            }
        }

        camA.lookatTarget = target;
        camA.StartTargetTracking();
        // camA.liveSend = true;
        camB.lookatTarget = target;
        camB.StartTargetTracking();
        //camB.liveSend = true;
        isTracking = true;
    }

    public void StopTracking()
    {
        cancelSource = new CancellationTokenSource();
        isTracking = false;
        resumeTask = ResumeAnimation(cancelSource.Token);
    }

    // Update is called once per frame
    void Update()
    {
        if (target == null)
            target = transform.Find("target").gameObject;

        if (Input.GetKeyDown(KeyCode.Space))
        {
            StartTracking();
        }
        if (Input.GetKeyUp(KeyCode.Space))
        {
            StopTracking();
        }

        if (camA != null && camA.isActiveAndEnabled)
        {
            if (camA.lookatTarget == null)
            {
                camA.pan = aPan;
                camA.tilt = aTilt;
            }
            else
            {
                aPan = camA.pan;
                aTilt = camA.tilt;
            }
        }

        if (camB != null && camB.isActiveAndEnabled)
        {
            if (camB.lookatTarget == null)
            {
                camB.pan = bPan;
                camB.tilt = bTilt;
            }
            else
            {
                bPan = camB.pan;
                bTilt = camB.tilt;
            }
        }
    }

    public void ReadPanTilt(PantiltRobot child)
    {
        if (child == camA)
        {
            aPan = child.pan;
            aTilt = child.tilt;
        }
        else if (child == camB)
        {
            bPan = child.pan;
            bTilt = child.tilt;
        }

    }

    private async Task ResumeAnimation(CancellationToken cancel)
    {
        var a = camA.StopTargetTracking(cancel);
        var b = camB.StopTargetTracking(cancel);

        await Task.WhenAll(a, b);

        if (!cancel.IsCancellationRequested)
        {
            var anim = GetComponent<Animator>();
            anim.Play("aaluege", 0);
            anim.speed = 1;
            resumeTask = null;
            Debug.Log($"all done & resumed animation");
        }
        else
        {
            Debug.Log($"cancelled");
        }

    }

    public void MoveUDP()
    {
        if (camA != null && camA.isActiveAndEnabled)
        {
            camA.MoveUDP();
        }
        if (camB != null && camB.isActiveAndEnabled)
        {
            camB.MoveUDP();
        }
    }

    public void OnKeyframe(KeyframeEventArgs e)
    {
        if (e.target.isActiveAndEnabled)
        {
            e.target.SendUdpMsg(e.udpMsg);
        }
    }

    public class KeyframeEventArgs : ScriptableObject
    {
        public UdpMsg udpMsg;
        public PantiltRobot target;
        public Keyframe keyframe;
    }

#if UNITY_EDITOR
    //     public void ProcessCurvesAnimator()
    //     {
    //         var animator = GetComponent<Animator>();
    //         var clipInfos = animator.GetCurrentAnimatorClipInfo(0);
    // 
    //         var state = animator.GetCurrentAnimatorStateInfo(0);
    //         print(state.fullPathHash);
    // 
    //         foreach (var clipInfo in clipInfos)
    //         {
    //             print(clipInfo.clip.name);
    // 
    //             var bindings = AnimationUtility.GetCurveBindings(clipInfo.clip);
    // 
    //             foreach (var binding in bindings)
    //             {
    // 
    //                 var events = new List<AnimationEvent>();
    //                 if (binding.propertyName == "pan") {
    //                    AddAnimationEvents(clipInfo.clip, binding, UdpCommands.MovePan_Accel, events);
    //                 }
    //                 if (binding.propertyName == "tilt") {
    //                     AddAnimationEvents(clipInfo.clip, binding, UdpCommands.MoveTilt_Accel, events);
    //                 }
    //                 AnimationUtility.SetAnimationEvents(clipInfo.clip, events.ToArray());
    //             }
    // 
    //             // var pan_binding = bindings.Single(b => b.path == "pan");
    //             // var pan_binding = bindings.Single(b => b.propertyName == "pan");
    //             // var tilt_binding = bindings.Single(b => b.propertyName == "tilt");
    //         }
    // 
    // 
    // 
    //     }

    public void ProcessCurves()
    {
        //var animation = GetComponent<Animation>();

        var clips = AnimationUtility.GetAnimationClips(gameObject);

        foreach (var clip in clips)
        {
            var bindings = AnimationUtility.GetCurveBindings(clip);

            var events = new List<AnimationEvent>();
            foreach (var binding in bindings)
            {
                if (binding.propertyName == "aPan")
                {
                    AddAnimationEvents(clip, binding, camA, UdpCommands.MovePan_Accel, events);
                }
                else if (binding.propertyName == "bPan")
                {
                    AddAnimationEvents(clip, binding, camB, UdpCommands.MovePan_Accel, events);
                }
                else if (binding.propertyName == "aTilt")
                {
                    AddAnimationEvents(clip, binding, camA, UdpCommands.MoveTilt_Accel, events);
                }
                else if (binding.propertyName == "bTilt")
                {
                    AddAnimationEvents(clip, binding, camB, UdpCommands.MoveTilt_Accel, events);
                }
            }
            AnimationUtility.SetAnimationEvents(clip, events.ToArray());

            // var pan_binding = bindings.Single(b => b.path == "pan");
            // var pan_binding = bindings.Single(b => b.propertyName == "pan");
            // var tilt_binding = bindings.Single(b => b.propertyName == "tilt");
        }


    }

    public void AddAnimationEvents(AnimationClip clip, EditorCurveBinding curveBinding, PantiltRobot robot, UdpCommands cmd, List<AnimationEvent> allEvents)
    {
        print($"add animation events: {curveBinding.path} -> {curveBinding.propertyName}");

        var curve = AnimationUtility.GetEditorCurve(clip, curveBinding);
        for (int i = 1; i < curve.keys.Length; i++)
        {
            var current = curve.keys[i - 1];
            var next = curve.keys[i];

            float dt = next.time - current.time;
            float dp = next.value - current.value;

            float acc = Math.Abs(dp) / (dt * dt / 4);
            // float vmax = acc * dt / 2;

            var eventArgs = ScriptableObject.CreateInstance<KeyframeEventArgs>();

            var pos = next.value;
            // if (cmd == UdpCommands.MoveTilt_Accel && udp_address == "pantilt-robot_b.hq.bitwaescherei.net") { pos = -pos; }
            // if (cmd == UdpCommands.MoveTilt_Accel) { pos = -pos; }
            eventArgs.udpMsg = new UdpMsg((byte)cmd, new float[] { pos, acc });
            eventArgs.keyframe = current;
            eventArgs.target = robot;
            var evt = new AnimationEvent
            {
                functionName = "OnKeyframe",
                objectReferenceParameter = eventArgs,
                time = curve.keys[i - 1].time
            };
            allEvents.Add(evt);
        }
    }
#endif
}

#if UNITY_EDITOR
[CustomEditor(typeof(TwoRobots))]
public class RobotsEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        var target = (TwoRobots)this.target;

        if (!target.IsTracking)
        {
            if (GUILayout.Button("START tracking"))
            {
                target.StartTracking();
            }
        }
        else
        {
            if (GUILayout.Button("STOP tracking"))
            {
                target.StopTracking();
            }
        }
        if (GUILayout.Button("Move UDP"))
        {
            target.MoveUDP();
        }
        if (GUILayout.Button("Process Curves"))
        {
            target.ProcessCurves();
        }
    }
}
#endif
