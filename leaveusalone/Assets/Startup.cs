using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;
using UnityEngine.SceneManagement;

public class Startup : MonoBehaviour
{
    public float timeUntilAutoStart = 10;
    public float timeForHome = 30;
    public float timeForBase = 5;


    public float secondsLeft = 0;

    public TwoRobots cams;

    public string state = "wait";
    

    void Start()
    {
        state = "start";
        StartCoroutine(Delay(timeUntilAutoStart, CamHomingAndStart));
    }

    IEnumerator Delay(float seconds, Action next)
    {
        secondsLeft = seconds;
        do
        {
            yield return new WaitForSeconds(1);
        } while (--secondsLeft > 0);

        next();
    }

    void CamHomingAndStart()
    {
        if (cams == null)
            return;

        cams.camA.Home();
        cams.camB.Home();

        state = "homing";
        StartCoroutine(Delay(timeForHome, LoadNextScene));
    }

    void LoadNextScene()
    {
        cams.camA.pan = cams.camA.lastAnimatedPanTilt[0];
        cams.camA.tilt = cams.camA.lastAnimatedPanTilt[1];
        cams.camB.pan = cams.camB.lastAnimatedPanTilt[0];
        cams.camB.tilt = cams.camB.lastAnimatedPanTilt[1];
        cams.ReadPanTilt(cams.camA);
        cams.ReadPanTilt(cams.camB);
              
        cams.MoveUDP();

        state = "go to base";
        StartCoroutine(Delay(timeForBase, () => SceneManager.LoadScene("run")));
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            StopAllCoroutines();
            SceneManager.LoadScene("run");
        }
    }

    void OnGUI()
    {
        GUIStyle style = new() { fontSize = 30};
        GUI.Label(new Rect(Screen.width*0.5f, Screen.height-100, 100, 30), secondsLeft.ToString() + " " + state, style);     
    }
}
