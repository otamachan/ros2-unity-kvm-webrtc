using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.Experimental.Rendering;

namespace ROS2 {
public class CameraPublisher : CustomPass
{
    public int Width = 640;
    public int Height = 480;
    public string TopicName = "image_raw";
    public string FrameId = "camera";
    public float PublishRate = 20F;
    public bool SavePicture = false;
    public string PictureFileName = "/tmp/camera";

    private Camera camera = null;
    private RenderTexture cameraRenderTexture;
    private RenderTexture colorRenderTexture;
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<sensor_msgs.msg.Image> colorPublisher;
    private sensor_msgs.msg.Image colorImage;
    private float deltaTime = 0.0F;
    private bool preSavePngFile = false;

    protected override void Setup(ScriptableRenderContext renderContext, CommandBuffer cmd)
    {
        // find camera which owns this CustomPass
        CustomPassVolume[] customPassVolumes = GameObject.FindObjectsOfType<CustomPassVolume>();
        foreach (var customPassVolume in customPassVolumes)
        {
            foreach (var customPass in customPassVolume.customPasses)
            {
                if (customPass == this)
                {
                    camera = customPassVolume.GetComponent<Camera>();
                    break;
                }
            }
        }
        if (camera == null || !Application.isPlaying)
        {
            return;
        }
        // create textures
        cameraRenderTexture = new RenderTexture(Width, Height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
        cameraRenderTexture.enableRandomWrite = true;
        colorRenderTexture = new RenderTexture(Width, Height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
        colorRenderTexture.enableRandomWrite = true;
        colorRenderTexture.autoGenerateMips = false;

        camera.targetTexture = cameraRenderTexture;

        ros2Unity = camera.GetComponent<ROS2UnityComponent>();

        colorImage = new sensor_msgs.msg.Image()
        {
            Width = (uint)Width,
            Height = (uint)Height,
            Encoding = "rgb8",
            Is_bigendian = 0,
            Step = (uint)Width * 3,
            Data = new byte[Width * Height * 3]
        };
        colorImage.SetHeaderFrame(FrameId);
    }
    protected override void Execute(CustomPassContext ctx)
    {
        if (camera == null || camera != ctx.hdCamera.camera || !Application.isPlaying)
        {
            return;
        }
        System.Action<AsyncGPUReadbackRequest> colorAction = (AsyncGPUReadbackRequest req) =>
        {
            if (ros2Unity.Ok())
            {
                if (ros2Node == null)
                {
                    ros2Node = ros2Unity.CreateNode("camera_publisher");
                    //colorPublisher = ros2Node.CreateSensorPublisher<sensor_msgs.msg.Image>("image_raw");
                    colorPublisher = ros2Node.CreatePublisher<sensor_msgs.msg.Image>(TopicName);
                }
                var msgWithHeader = colorImage as MessageWithHeader;
                ros2Node.clock.UpdateROSTimestamp(ref msgWithHeader);
                var buffer = req.GetData<byte>();
                buffer.CopyTo(colorImage.Data);
                colorPublisher.Publish(colorImage);
            }
            if (!preSavePngFile && SavePicture)
            {
                SavePngFile(cameraRenderTexture);
            }
            preSavePngFile = SavePicture;
        };
        float delta = 1.0f / PublishRate;
        deltaTime += Time.deltaTime;
        while (deltaTime > delta) {
            deltaTime -= delta;
            var scale = RTHandles.rtHandleProperties.rtHandleScale;
            ctx.cmd.Blit(ctx.cameraColorBuffer, colorRenderTexture, new Vector2(scale.x, -scale.y), new Vector2(0.0f, scale.y), 0, 0);
            ctx.cmd.RequestAsyncReadback(colorRenderTexture, 0, TextureFormat.RGB24, colorAction);
        }
    }
    void SavePngFile(RenderTexture rt)
    {
        Texture2D tex = new Texture2D(Width, Height, TextureFormat.RGB24, false);
        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, Width, Height), 0, 0);
        tex.Apply();
        // Encode texture into PNG
        byte[] bytes = tex.EncodeToPNG();
        UnityEngine.Object.Destroy(tex);
        //Write to a file in the project folder
        System.IO.File.WriteAllBytes(PictureFileName + ".png", bytes);
    }
    protected override void Cleanup()
    {
        if (camera == null)
        {
            return;
        }
        cameraRenderTexture?.Release();
        colorRenderTexture?.Release();
    }
}
}  // namespace ROS2