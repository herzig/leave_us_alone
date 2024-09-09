using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CamTex : MonoBehaviour
{

    public int cameraId;

    public WebCamTexture texture;

    // Start is called before the first frame update
    void Start()
    {
        WebCamDevice[] devices = WebCamTexture.devices;

        for(int i = 0; i < devices.Length; i++)
        {
            print("Webcam available: " + devices[i].name);
        }

        WebCamTexture tex = new WebCamTexture(devices[cameraId].name);
        texture = tex;
    }

    // Update is called once per frame
    void Update()
    {
    }
}
