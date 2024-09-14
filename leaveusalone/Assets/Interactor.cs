using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Windows.Kinect;

public class Interactor : MonoBehaviour
{

    public BodySourceManager bodyManager;
    public TwoRobots cams;
    public BoxCollider interactiveBox;
    public float smoothTime = 0.05f;
    public JointType focusJoint = JointType.Head;
    public float deadband_dist = 0.1f;

    private GameObject Target { get { return cams.target; } }


    private ulong currentTrackingId;

    private Vector3 currentVelocity = Vector3.zero;


    void Start()
    {
    }

    void Update()
    {
        var bodies = bodyManager.GetData();
        if (bodies == null) return;


        var trackedBodies = new List<Body>(bodies.Length);
        var trackedPositions = new List<Vector3>(bodies.Length);

        for (int i = 0; i < bodies.Length; i++) 
        {
            var b = bodies[i];

            if (!b.IsTracked) continue;

            var joint = b.Joints[focusJoint];
            var pos = BodySourceView.GetVector3FromJoint(joint);
            pos = bodyManager.transform.localToWorldMatrix.MultiplyPoint(pos);

            if (joint.TrackingState == TrackingState.Tracked && interactiveBox.bounds.Contains(pos))
            {
                trackedPositions.Add(pos);
                trackedBodies.Add(b);
            }
        }

        if (trackedBodies.Count > 0)
        {

            // print($"body {b.TrackingId} head: {head.Position.X} {head.Position.Y} {head.Position.Z}");
            //var pos = new Vector3(head.Position.X, head.Position.Y, head.Position.Z);
            //target.transform.position = pos;

            var newTarget = trackedPositions[0];

            var diff = cams.target.transform.position - newTarget;
            if (diff.magnitude > deadband_dist)
            {
                cams.target.transform.position = Vector3.SmoothDamp(cams.target.transform.position, trackedPositions[0], ref currentVelocity, smoothTime);
            }

            if (!cams.IsTracking)
                cams.StartTracking();
        }
        else
        {
            if (cams.IsTracking)
                cams.StopTracking();
        }

        
    }
}
