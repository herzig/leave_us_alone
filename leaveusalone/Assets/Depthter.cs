using Assets;
using Doji.AI.Depth;
using Doji.AI.Depth.Samples;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Sentis;
using Unity.Sentis.Layers;
using UnityEngine;

public class Depthter : MonoBehaviour
{
    public WebCamTexture webcam;
    public RenderTexture depth;

    public int cameraId = 1;

    public DepthPointRenderer pointRendererPrefab;
    private DepthPointRenderer pointRenderer;

    private Midas midas;
    private FaceBlazer faceblaze;

    public Texture faceTargetTexture;

    Vector2Int resolution = new Vector2Int(256, 256);

    [SerializeField, Range(0, 1)] public float iouThreshold = 0.3f;
    [SerializeField, Range(0, 1)] public float scoreThreshold = 0.3f;

    void Start()
    {
        midas = new Midas(ModelType.midas_v21_small_256);
        //midas = new Midas(ModelType.dpt_swin2_tiny_256);
        GetComponentsInChildren<Renderer>().Single(r => r.name == "depth").sharedMaterial.mainTexture = midas.Result;

        WebCamDevice[] devices = WebCamTexture.devices;

        // for debugging purposes, prints available devices to the console
        for(int i = 0; i < devices.Length; i++)
        {
            print("Webcam available: " + devices[i].name);
        }

        WebCamTexture tex = new WebCamTexture(devices[cameraId].name, resolution.x, resolution.y);
        webcam = tex;

        this.GetComponentsInChildren<Renderer>().Single(r => r.name == "webcam").sharedMaterial.mainTexture = tex;

        tex.Play();

        depth = midas.Result;
        pointRenderer = Instantiate(pointRendererPrefab);
        pointRenderer.Depth = midas.Result;
        pointRenderer.Source = webcam;
        pointRenderer.enabled = false;

        faceblaze = new FaceBlazer(resolution);
        faceTargetTexture = faceblaze.targetTexture;
    }

    // Update is called once per frame
    void Update()
    {
        // if (midas == null)
        // {
        //     midas = new Midas(ModelType.midas_v21_small_256);
        //     GetComponentsInChildren<Renderer>().Single(r => r.name == "depth").sharedMaterial.mainTexture = midas.Result;
        // }

        // if (webcam == null)
        // {
        //     WebCamDevice[] devices = WebCamTexture.devices;

        //     Renderer renderer = this.GetComponentInChildren<Renderer>();

        //     // assuming the first available WebCam is desired
        //     WebCamTexture tex = new WebCamTexture(devices[cameraId].name);
        //     webcam = tex;

        //     GetComponentsInChildren<Renderer>().Single(r => r.name == "webcam").sharedMaterial.mainTexture = tex;

        //     tex.Play();
        // }

        faceblaze.iouThreshold = iouThreshold;
        faceblaze.scoreThreshold = scoreThreshold;

        if (webcam != null && webcam.didUpdateThisFrame)
        {
            midas.EstimateDepth(webcam);
            pointRenderer.Source = webcam;
            var (min, max) = midas.GetMinMax();
            pointRenderer.MinPred = min;
            pointRenderer.MaxPred = max;

            var boxes = faceblaze.ExecuteML(webcam);

            var d = new Texture2D(depth.width, depth.height);

            //for (int i = 0; i < boxes.Length; i++)
            {
                int i = 0;
                //var b = boxes[i];
                var b = new FaceBlazer.BoundingBox();

                RenderTexture.active = midas.Result;

                d.ReadPixels(new Rect(0, 0, depth.width, depth.height), 0, 0);

                RenderTexture.active = null;

                b.centerX = 0.5f * resolution.x;
                b.centerY = 0.5f * resolution.y;

                var facedepth = d.GetPixels((int)b.centerX-2, (int)b.centerY-2, 4, 4);

                float meanDepth = 0;
                for (int j = 0; j < facedepth.Length; j++)
                {
                    meanDepth += facedepth[j].r;
                }
                meanDepth /= facedepth.Length;
                meanDepth = (meanDepth - min) / (max - min);
                meanDepth = (pointRenderer.MaxDepth - pointRenderer.MinDepth) * meanDepth + pointRenderer.MinDepth;

                print($"{b.centerX}, {b.centerY} {meanDepth}");

                GameObject sphere;
                if (pointRenderer.transform.childCount < i + 1)
                { 
                    sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    sphere.transform.parent = pointRenderer.transform;
                    sphere.transform.localScale = new Vector3(0.25f, 0.25f, 0.25f);
                }
                else
                {
                    sphere = pointRenderer.transform.GetChild(i).gameObject;
                }

                sphere.transform.localPosition = new Vector3(b.centerX/resolution.x, b.centerY/resolution.y, meanDepth);

            }


        }
    }
    private void OnDestroy() 
    {
        midas?.Dispose();
        faceblaze?.Dispose();
    }
}
