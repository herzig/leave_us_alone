using UnityEngine;
using Unity.Sentis;
using UnityEngine.Video;
using UnityEngine.UI;
using Lays = Unity.Sentis.Layers;

public class RunBlazeFace : MonoBehaviour
{
    //Drag a link to a raw image here:
    public RawImage previewUI = null;

    // Put your bounding box sprite image here
    public Sprite faceTexture;

    // 6 optional sprite images (left eye, right eye, nose, mouth, left ear, right ear)
    public Sprite[] markerTextures;

    public string videoName = "chatting.mp4";

    // 
    public Texture2D inputImage;

    public InputType inputType = InputType.Video;

    Vector2Int resolution = new Vector2Int(640, 640);
    WebCamTexture webcam;
    VideoPlayer video;

    const BackendType backend = BackendType.GPUCompute;

    RenderTexture targetTexture;
    public enum InputType { Image, Video, Webcam };
   
    
    //Some adjustable parameters for the model
    [SerializeField, Range(0, 1)] float iouThreshold = 0.5f;
    [SerializeField, Range(0, 1)] float scoreThreshold = 0.5f;
    int maxOutputBoxes = 64;

    IWorker worker;

    //Holds input image size
    int size = 128;

    Ops ops;
    ITensorAllocator allocator;

    Model model;

    //webcam device name:
    const int webcamId = 1;

    bool closing = false;

    public struct BoundingBox
    {
        public float centerX;
        public float centerY;
        public float width;
        public float height;
    }

    void Start()
    {
        allocator = new TensorCachingAllocator();

        //(Note: if using a webcam on mobile get permissions here first)

        targetTexture = new RenderTexture(resolution.x, resolution.y, 0);

        SetupInput();

        SetupModel();

        SetupEngine();
    }

    void SetupInput()
    {
        switch (inputType)
        {
            case InputType.Webcam:
                {
                    var camName = WebCamTexture.devices[webcamId].name;
                    webcam = new WebCamTexture(camName, resolution.x, resolution.y);
                    webcam.requestedFPS = 30;
                    webcam.Play();
                    print(camName);
                    break;
                }
            case InputType.Video:
                {
                    video = gameObject.AddComponent<VideoPlayer>();//new VideoPlayer();
                    video.renderMode = VideoRenderMode.APIOnly;
                    video.source = VideoSource.Url;
                    video.url = Application.streamingAssetsPath + "/"+videoName;
                    video.isLooping = true;
                    video.Play();
                    break;
                }
            default:
                {
                    Graphics.Blit(inputImage, targetTexture);
                }
                break;
        }
    }

    void Update()
    {
        if (inputType == InputType.Webcam)
        {
            // Format video input
            if (!webcam.didUpdateThisFrame) return;

            var aspect1 = (float)webcam.width / webcam.height;
            var aspect2 = (float)resolution.x / resolution.y;
            var gap = aspect2 / aspect1;

            var vflip = webcam.videoVerticallyMirrored;
            var scale = new Vector2(gap, vflip ? -1 : 1);
            var offset = new Vector2((1 - gap) / 2, vflip ? 1 : 0);

            Graphics.Blit(webcam, targetTexture, scale, offset);
        }
        if (inputType == InputType.Video)
        {
            var aspect1 = (float)video.width / video.height;
            var aspect2 = (float)resolution.x / resolution.y;
            var gap = aspect2 / aspect1;

            var vflip = false;
            var scale = new Vector2(gap, vflip ? -1 : 1);
            var offset = new Vector2((1 - gap) / 2, vflip ? 1 : 0);
            Graphics.Blit(video.texture, targetTexture, scale, offset);
        }
        if (inputType == InputType.Image)
        {
            Graphics.Blit(inputImage, targetTexture);
        }

        if (Input.GetKeyDown(KeyCode.Escape))
        {
            closing = true;
            Application.Quit();
        }

        if (Input.GetKeyDown(KeyCode.P))
        {
            previewUI.enabled = !previewUI.enabled;
        }
    }


    void LateUpdate()
    {
        if (!closing)
        {
            RunInference(targetTexture);
        }
    }

    //Calculate the centers of the grid squares for two 16x16 grids and six 8x8 grids
    //The positions of the faces are given relative to these "anchor points"
    float[] GetGridBoxCoords()
    {
        var offsets = new float[896 * 4];
        int n = 0;
        AddGrid(offsets, 16, 2, 8, ref n);
        AddGrid(offsets, 8, 6, 16, ref n);
        return offsets;
    }

    void AddGrid(float[] offsets, int rows, int repeats, int cellWidth, ref int n)
    {
        for (int j = 0; j < repeats * rows * rows; j++)
        {
            offsets[n++] = cellWidth * ((j / repeats) % rows - (rows - 1) * 0.5f);
            offsets[n++] = cellWidth * ((j / repeats / rows) - (rows - 1) * 0.5f);
            n += 2;
        }
    }

    void SetupModel()
    {
        float[] offsets = GetGridBoxCoords();

        model = ModelLoader.Load(Application.streamingAssetsPath + "/blazeface.sentis");

        //We need to add extra layers to the model in order to aggregate the box predicions:

        size = model.inputs[0].shape.ToTensorShape()[1]; // Input tensor width

        model.AddConstant(new Lays.Constant("0", new int[] { 0 }));
        model.AddConstant(new Lays.Constant("2", new int[] { 2 }));
        model.AddConstant(new Lays.Constant("4", new int[] { 4 }));
        model.AddConstant(new Lays.Constant("offsets", new TensorFloat(new TensorShape(1, 896, 4), offsets)));

        model.AddLayer(new Lays.Slice("boxes", "regressors", "0", "4", "2"));
        model.AddLayer(new Lays.Transpose("scores", "classificators", new int[] { 0, 2, 1 }));
        model.AddLayer(new Lays.Add("boxCoords", "boxes", "offsets"));
        model.AddOutput("boxCoords");

        model.AddConstant(new Lays.Constant("maxOutputBoxes", new int[] { maxOutputBoxes }));
        model.AddConstant(new Lays.Constant("iouThreshold", new float[] { iouThreshold }));
        model.AddConstant(new Lays.Constant("scoreThreshold", new float[] { scoreThreshold }));

        model.AddLayer(new Lays.NonMaxSuppression("NMS", "boxCoords", "scores",
            "maxOutputBoxes", "iouThreshold", "scoreThreshold",
            centerPointBox: Lays.CenterPointBox.Center
        ));
        model.AddOutput("NMS");
    }
    public void SetupEngine()
    {
        worker = WorkerFactory.CreateWorker(backend, model);
        ops = WorkerFactory.CreateOps(backend, allocator);
    }

    void DrawFaces(TensorFloat index3, TensorFloat regressors, int NMAX, Vector2 scale)
    {
        for (int n = 0; n < NMAX; n++)
        {
            //Draw bounding box of face
            var box = new BoundingBox
            {
                centerX = index3[0, n, 0] * scale.x,
                centerY = index3[0, n, 1] * scale.y,
                width = index3[0, n, 2] * scale.x,
                height = index3[0, n, 3] * scale.y
            };
            DrawBox(box, faceTexture);
            if (regressors == null) continue;
            
            //Draw markers for eyes, ears, nose, mouth:
            for (int j = 0; j < 6; j++)
            {
                var marker = new BoundingBox
                {
                    centerX = box.centerX + (regressors[0, n, 4 + j * 2] - regressors[0, n, 0]) * scale.x,
                    centerY = box.centerY + (regressors[0, n, 4 + j * 2 + 1] - regressors[0, n, 1]) * scale.y,
                    width = 8.0f * scale.x,
                    height = 8.0f * scale.y,
                };
                DrawBox(marker, j < markerTextures.Length ? markerTextures[j] : faceTexture);
            }
        }
    }

    void ExecuteML(Texture source)
    {
        var transform = new TextureTransform();
        transform.SetDimensions(size, size, 3);
        transform.SetTensorLayout(0, 3, 1, 2);
        using var image0 = TextureConverter.ToTensor(source, transform);

        // Pre-process the image to make input in range (-1..1)
        using var image = ops.Mad(image0, 2f, -1f);

        worker.Execute(image);

        using var boxCoords = worker.PeekOutput("boxCoords") as TensorFloat; //face coords
        using var regressors = worker.PeekOutput("regressors") as TensorFloat; //contains markers

        var NM1 = worker.PeekOutput("NMS") as TensorInt;

        using var boxCoords2 = ops.Reshape(boxCoords, boxCoords.shape.Unsqueeze(0));
        using var output = ops.GatherND(boxCoords2, NM1, 0);

        using var regressors2 = ops.Reshape(regressors,regressors.shape.Unsqueeze(0));
        using var markersOutput = ops.GatherND(regressors2, NM1, 0);

        output.MakeReadable();
        markersOutput.MakeReadable();

        ClearAnnotations();

        Vector2 markerScale = previewUI.rectTransform.rect.size / size;

        DrawFaces(output, markersOutput, output.shape[0], markerScale);

    }

    void RunInference(Texture input)
    {
        // Face detection
        ExecuteML(input);

        previewUI.texture = input;
    }

    public void DrawBox(BoundingBox box, Sprite sprite)
    {
        var panel = new GameObject("ObjectBox");
        panel.AddComponent<CanvasRenderer>();
        panel.AddComponent<Image>();
        panel.transform.SetParent(previewUI.transform, false);

        var img = panel.GetComponent<Image>();
        img.color = Color.white;
        img.sprite = sprite;
        img.type = Image.Type.Sliced;

        panel.transform.localPosition = new Vector3(box.centerX, -box.centerY);
        RectTransform rt = panel.GetComponent<RectTransform>();
        rt.sizeDelta = new Vector2(box.width, box.height);
    }
    public void ClearAnnotations()
    {
        foreach (Transform child in previewUI.transform)
        {
            Destroy(child.gameObject);
        }
    }

    void CleanUp()
    {
        closing = true;
        ops?.Dispose();
        allocator?.Dispose();
        if (webcam) Destroy(webcam);
        if (video) Destroy(video);
        RenderTexture.active = null;
        targetTexture.Release();
        worker?.Dispose();
        worker = null;
    }

    void OnDestroy()
    {
        CleanUp();
    }

}