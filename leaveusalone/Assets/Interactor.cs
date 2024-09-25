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
    public float deadband_dist = 0.15f;

    private GameObject Target { get { return cams.target; } }


    public int currentTrackingIndex = 0;
    public float trackingTime = 0;
    public float trackingDuration = 8;


    private Vector3 currentVelocity = Vector3.zero;


    void Start()
    {
        trackingTime = 0;
        currentTrackingIndex = 0;
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
            trackingTime += Time.deltaTime;
            if (trackingTime > trackingDuration)
            {
                currentTrackingIndex = Mathf.Min(currentTrackingIndex + 1, trackedBodies.Count - 1);
                trackingTime = 0;
            }

            if (trackedPositions.Count -1 < currentTrackingIndex)
            {
                currentTrackingIndex = 0;
            }

            var newTarget = trackedPositions[currentTrackingIndex];

            var diff = cams.target.transform.position - newTarget;
            if (diff.magnitude > deadband_dist)
            {
                cams.target.transform.position = Vector3.SmoothDamp(cams.target.transform.position, newTarget, ref currentVelocity, smoothTime);
            }

            if (!cams.IsTracking)
            {
                cams.StartTracking();
                trackingTime = 0;
            }
        }
        else
        {
            if (cams.IsTracking)
            {
                cams.StopTracking();
                trackingTime = 0;
            }
        }

        
    }
}
