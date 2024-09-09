using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;
using Unity.Sentis;
using UnityEngine.Video;
using UnityEngine.UI;
using Lays = Unity.Sentis.Layers;
using System.Drawing.Printing;

namespace Assets
{
    class FaceBlazer : IDisposable
    {
        public struct BoundingBox
        {
            public float centerX;
            public float centerY;
            public float width;
            public float height;
        }

        const BackendType backend = BackendType.GPUCompute;
        Model model;
        IWorker worker;

        //Holds input image size
        int size = 128;

        Ops ops;
        ITensorAllocator allocator;

        public float iouThreshold = 0.3f;
        public float scoreThreshold = 0.3f;
        int maxOutputBoxes = 64;

        Vector2Int resolution;

        public RenderTexture targetTexture;


        public FaceBlazer(Vector2Int resolution)
        {
            this.resolution = resolution;
            allocator = new TensorCachingAllocator();
            targetTexture = new RenderTexture(resolution.x, resolution.y, 0);
            SetupModel();
            SetupEngine();
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


        void SetupEngine()
        {
            worker = WorkerFactory.CreateWorker(backend, model);
            ops = WorkerFactory.CreateOps(backend, allocator);
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

        public BoundingBox[] ExecuteML(WebCamTexture webcam)
        {
            var aspect1 = (float)webcam.width / webcam.height;
            var aspect2 = (float)resolution.x / resolution.y;
            var gap = aspect2 / aspect1;

            var vflip = webcam.videoVerticallyMirrored;
            var scale = new Vector2(gap, vflip ? -1 : 1);
            var offset = new Vector2((1 - gap) / 2, vflip ? 1 : 0);

            Graphics.Blit(webcam, targetTexture, scale, offset);
            return ExecuteML(targetTexture);
        }

        public BoundingBox[] ExecuteML(Texture source)
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

            using var regressors2 = ops.Reshape(regressors, regressors.shape.Unsqueeze(0));
            using var markersOutput = ops.GatherND(regressors2, NM1, 0);

            output.MakeReadable();
            markersOutput.MakeReadable();

            Debug.Log($"exec {output.shape}");

            //ClearAnnotations();

            Vector2 scale = new Vector2(source.width, source.height) / size; // previewUI.rectTransform.rect.size / size;
             
            //DrawFaces(output, markersOutput, output.shape[0], markerScale);
            int NMAX = output.shape[0];
            var index3 = output;

            BoundingBox[] result = new BoundingBox[NMAX];
            for (int n = 0; n < NMAX; n++)
            {
                result[n] = new BoundingBox
                {
                    centerX = index3[0, n, 0] * scale.x + source.width * 0.5f,
                    centerY = index3[0, n, 1] * scale.y + source.height * 0.5f,
                    width = index3[0, n, 2] * scale.x,
                    height = index3[0, n, 3] * scale.y
                };
            }
            return result;
        }

        public void Dispose()
        {
            //closing = true;
            ops?.Dispose();
            allocator?.Dispose();
            // if (webcam) Destroy(webcam);
            // if (video) Destroy(video);
            // RenderTexture.active = null;
            // targetTexture.Release();
            worker?.Dispose();
            worker = null;
        }
    }
}
