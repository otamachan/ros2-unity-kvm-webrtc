#include <atomic>
#include <iostream>
#include <queue>

#include <NvCodec/NvEncoder/NvEncoderCuda.h>
#include <Samples.h>
#include <Utils/Logger.h>
#include <Utils/NvCodecUtils.h>
#include <Utils/NvEncoderCLIOptions.h>
#include <cuda.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

simplelogger::Logger* logger =
    simplelogger::LoggerFactory::CreateConsoleLogger();
extern PSampleConfiguration gSampleConfiguration;

struct EncodedFrame {
  std::vector<std::vector<uint8_t>> packets;
  int width;
  int height;
};

template <class T>
class ThreadSafeQueue {
 public:
  ThreadSafeQueue(void) : queue_(), mutex_(), condition_() {}
  ~ThreadSafeQueue(void) {}
  void Enqueue(T&& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(std::move(value));
    if (queue_.size() > 10) {
      std::cerr << "EncodedFrame queue got full. abandoned" << std::endl;
      queue_.pop();
    }
    condition_.notify_one();
  }
  T Dequeue(void) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (queue_.empty()) {
      condition_.wait(lock);
    }
    T val = queue_.front();
    queue_.pop();
    return val;
  }

 private:
  std::queue<T> queue_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
};

class KVSClient {
 public:
  static KVSClient& GetInstance() {
    if (!instance) {
      instance = new KVSClient;
    }
    return *instance;
  }
  bool isReady() const { return ready.load(); }
  void QueueEncodedFrame(EncodedFrame&& frame) {
    frameQueue.Enqueue(std::move(frame));
  }
  bool Init(PCHAR channel_name) {
    STATUS retStatus = STATUS_SUCCESS;
    PCHAR pChannelName;
    signalingClientMetrics.version = SIGNALING_CLIENT_METRICS_CURRENT_VERSION;

    SET_INSTRUMENTED_ALLOCATORS();

    // do trickleIce by default
    printf("[KVS Master] Using trickleICE by default\n");

#ifdef IOT_CORE_ENABLE_CREDENTIALS
    CHK_ERR((pChannelName = getenv(IOT_CORE_THING_NAME)) != NULL,
            STATUS_INVALID_OPERATION, "AWS_IOT_CORE_THING_NAME must be set");
#else
    pChannelName = channel_name;
#endif

    retStatus = createSampleConfiguration(pChannelName,
                                          SIGNALING_CHANNEL_ROLE_TYPE_MASTER,
                                          TRUE, TRUE, &pSampleConfiguration);
    if (retStatus != STATUS_SUCCESS) {
      printf(
          "[KVS Master] createSampleConfiguration(): operation returned status "
          "code: 0x%08x \n",
          retStatus);
      goto CleanUp;
    }

    printf("[KVS Master] Created signaling channel %s\n", pChannelName);

    if (pSampleConfiguration->enableFileLogging) {
      retStatus = createFileLogger(
          FILE_LOGGING_BUFFER_SIZE, MAX_NUMBER_OF_LOG_FILES,
          (PCHAR)FILE_LOGGER_LOG_FILE_DIRECTORY_PATH, TRUE, TRUE, NULL);
      if (retStatus != STATUS_SUCCESS) {
        printf(
            "[KVS Master] createFileLogger(): operation returned status code: "
            "0x%08x \n",
            retStatus);
        pSampleConfiguration->enableFileLogging = FALSE;
      }
    }

    // Set the audio and video handlers
    // pSampleConfiguration->audioSource = sendAudioPackets;
    pSampleConfiguration->videoSource = KVSClient::sendVideoPackets;
    // pSampleConfiguration->receiveAudioVideoSource = sampleReceiveVideoFrame;
    pSampleConfiguration->onDataChannel = onDataChannel;
    pSampleConfiguration->mediaType = SAMPLE_STREAMING_AUDIO_VIDEO;
    printf("[KVS Master] Finished setting audio and video handlers\n");

    // Initialize KVS WebRTC. This must be done before anything else, and must
    // only be done once.
    retStatus = initKvsWebRtc();
    if (retStatus != STATUS_SUCCESS) {
      printf(
          "[KVS Master] initKvsWebRtc(): operation returned status code: "
          "0x%08x "
          "\n",
          retStatus);
      goto CleanUp;
    }
    printf("[KVS Master] KVS WebRTC initialization completed successfully\n");

    pSampleConfiguration->signalingClientCallbacks.messageReceivedFn =
        signalingMessageReceived;

    strcpy(pSampleConfiguration->clientInfo.clientId, SAMPLE_MASTER_CLIENT_ID);

    retStatus = createSignalingClientSync(
        &pSampleConfiguration->clientInfo, &pSampleConfiguration->channelInfo,
        &pSampleConfiguration->signalingClientCallbacks,
        pSampleConfiguration->pCredentialProvider,
        &pSampleConfiguration->signalingClientHandle);
    if (retStatus != STATUS_SUCCESS) {
      printf(
          "[KVS Master] createSignalingClientSync(): operation returned status "
          "code: 0x%08x \n",
          retStatus);
      goto CleanUp;
    }
    printf("[KVS Master] Signaling client created successfully\n");

    // Enable the processing of the messages
    retStatus =
        signalingClientFetchSync(pSampleConfiguration->signalingClientHandle);
    if (retStatus != STATUS_SUCCESS) {
      printf(
          "[KVS Master] signalingClientFetchSync(): operation returned status "
          "code: 0x%08x \n",
          retStatus);
      goto CleanUp;
    }

    retStatus =
        signalingClientConnectSync(pSampleConfiguration->signalingClientHandle);
    if (retStatus != STATUS_SUCCESS) {
      printf(
          "[KVS Master] signalingClientConnectSync(): operation returned "
          "status "
          "code: 0x%08x \n",
          retStatus);
      goto CleanUp;
    }
    printf("[KVS Master] Signaling client connection to socket established\n");

    gSampleConfiguration = pSampleConfiguration;

    printf("[KVS Master] Channel %s set up done \n", pChannelName);
    return true;
  CleanUp:
    return false;
  }
  void Run() {
    cleanupThread = std::thread([this] {
      STATUS retStatus = STATUS_SUCCESS;
      retStatus = sessionCleanupWait(pSampleConfiguration);
      if (retStatus != STATUS_SUCCESS) {
        printf(
            "[KVS Master] sessionCleanupWait(): operation returned status "
            "code: 0x%08x \n",
            retStatus);
      }
      printf("[KVS Master] Streaming session terminated\n");
    });
  }
  void CleanUp() {
    printf("[KVS Master] Cleaning up....\n");
    if (cleanupThread.joinable()) {
      ATOMIC_STORE_BOOL(&pSampleConfiguration->interrupted, TRUE);
      cleanupThread.join();
    }
    STATUS retStatus = STATUS_SUCCESS;
    if (pSampleConfiguration != NULL) {
      // Kick of the termination sequence
      ATOMIC_STORE_BOOL(&pSampleConfiguration->appTerminateFlag, TRUE);

      if (IS_VALID_MUTEX_VALUE(
              pSampleConfiguration->sampleConfigurationObjLock)) {
        MUTEX_LOCK(pSampleConfiguration->sampleConfigurationObjLock);
      }

      // Cancel the media thread
      if (pSampleConfiguration->mediaThreadStarted) {
        DLOGD("Canceling media thread");
        THREAD_CANCEL(pSampleConfiguration->mediaSenderTid);
      }

      if (IS_VALID_MUTEX_VALUE(
              pSampleConfiguration->sampleConfigurationObjLock)) {
        MUTEX_UNLOCK(pSampleConfiguration->sampleConfigurationObjLock);
      }

      if (pSampleConfiguration->mediaSenderTid != INVALID_TID_VALUE) {
        THREAD_JOIN(pSampleConfiguration->mediaSenderTid, NULL);
      }

      if (pSampleConfiguration->enableFileLogging) {
        freeFileLogger();
      }
      retStatus = signalingClientGetMetrics(
          pSampleConfiguration->signalingClientHandle, &signalingClientMetrics);
      if (retStatus == STATUS_SUCCESS) {
        logSignalingClientStats(&signalingClientMetrics);
      } else {
        printf(
            "[KVS Master] signalingClientGetMetrics() operation returned "
            "status code: 0x%08x\n",
            retStatus);
      }
      retStatus =
          freeSignalingClient(&pSampleConfiguration->signalingClientHandle);
      if (retStatus != STATUS_SUCCESS) {
        printf(
            "[KVS Master] freeSignalingClient(): operation returned status "
            "code: 0x%08x",
            retStatus);
      }

      retStatus = freeSampleConfiguration(&pSampleConfiguration);
      if (retStatus != STATUS_SUCCESS) {
        printf(
            "[KVS Master] freeSampleConfiguration(): operation returned status "
            "code: 0x%08x",
            retStatus);
      }
    }
    printf("[KVS Master] Cleanup done\n");

    RESET_INSTRUMENTED_ALLOCATORS();

    delete instance;
    instance = nullptr;
  }
  static PVOID sendVideoPackets(PVOID args) {
    STATUS retStatus = STATUS_SUCCESS;
    PSampleConfiguration pSampleConfiguration = (PSampleConfiguration)args;
    RtcEncoderStats encoderStats;
    Frame frame;
    UINT32 frameSize;
    STATUS status;
    UINT32 i;
    KVSClient& kvs_client = GetInstance();
    MEMSET(&encoderStats, 0x00, SIZEOF(RtcEncoderStats));

    if (pSampleConfiguration == NULL) {
      printf(
          "[KVS Master] sendVideoPackets(): operation returned status code: "
          "0x%08x \n",
          STATUS_NULL_ARG);
      goto CleanUp;
    }

    frame.presentationTs = 0;

    kvs_client.ready.store(true);
    printf("[KVS Master] sendVideoPackets(): started\n");
    while (!ATOMIC_LOAD_BOOL(&pSampleConfiguration->appTerminateFlag)) {
      const EncodedFrame encoded_frame = kvs_client.frameQueue.Dequeue();
      for (const auto& packet : encoded_frame.packets) {
        frameSize = packet.size();
        // Re-alloc if needed
        if (frameSize > pSampleConfiguration->videoBufferSize) {
          pSampleConfiguration->pVideoFrameBuffer = (PBYTE)MEMREALLOC(
              pSampleConfiguration->pVideoFrameBuffer, frameSize);
          if (pSampleConfiguration->pVideoFrameBuffer == NULL) {
            printf(
                "[KVS Master] Video frame Buffer reallocation failed...%s "
                "(code "
                "%d)\n",
                strerror(errno), errno);
            printf(
                "[KVS Master] MEMREALLOC(): operation returned status code: "
                "0x%08x "
                "\n",
                STATUS_NOT_ENOUGH_MEMORY);
            goto CleanUp;
          }

          pSampleConfiguration->videoBufferSize = frameSize;
        }

        frame.frameData = pSampleConfiguration->pVideoFrameBuffer;
        frame.size = frameSize;
        memcpy(frame.frameData, packet.data(), packet.size());

        encoderStats.width = encoded_frame.width;
        encoderStats.height = encoded_frame.height;
        encoderStats.targetBitrate = 262000;
        frame.presentationTs += SAMPLE_VIDEO_FRAME_DURATION;

        MUTEX_LOCK(pSampleConfiguration->streamingSessionListReadLock);
        for (i = 0; i < pSampleConfiguration->streamingSessionCount; ++i) {
          status =
              writeFrame(pSampleConfiguration->sampleStreamingSessionList[i]
                             ->pVideoRtcRtpTransceiver,
                         &frame);
          encoderStats.encodeTimeMsec =
              4;  // update encode time to an arbitrary
                  // number to demonstrate stats update
          updateEncoderStats(pSampleConfiguration->sampleStreamingSessionList[i]
                                 ->pVideoRtcRtpTransceiver,
                             &encoderStats);
          if (status != STATUS_SRTP_NOT_READY_YET) {
            if (status != STATUS_SUCCESS) {
#ifdef VERBOSE
              printf("writeFrame() failed with 0x%08x\n", status);
#endif
            }
          }
        }
        MUTEX_UNLOCK(pSampleConfiguration->streamingSessionListReadLock);
      }
    }
  CleanUp:
    CHK_LOG_ERR(retStatus);
    return (PVOID)(ULONG_PTR)retStatus;
  }

 private:
  PSampleConfiguration pSampleConfiguration = NULL;
  SignalingClientMetrics signalingClientMetrics;
  ThreadSafeQueue<EncodedFrame> frameQueue;
  std::atomic<bool> ready{false};
  std::thread cleanupThread;

  static KVSClient* instance;
};

KVSClient* KVSClient::instance = nullptr;

class CudaEncoder {
 public:
  void Init() {
    int iGpu = 0;
    ck(cuInit(0));
    int nGpu = 0;
    ck(cuDeviceGetCount(&nGpu));
    if (iGpu < 0 || iGpu >= nGpu) {
      std::cout << "GPU ordinal out of range. Should be within [" << 0 << ", "
                << nGpu - 1 << "]" << std::endl;
      exit(EXIT_FAILURE);
    }
    CUdevice cuDevice = 0;
    ck(cuDeviceGet(&cuDevice, iGpu));
    char szDeviceName[80];
    ck(cuDeviceGetName(szDeviceName, sizeof(szDeviceName), cuDevice));
    std::cout << "GPU in use: " << szDeviceName << std::endl;
    ck(cuCtxCreate(&cuContext, 0, cuDevice));
  }
  void InitEncoder(int nWidth, int nHeight, NV_ENC_BUFFER_FORMAT eFormat) {
    NvEncoderInitParam encodeCLIOptions = NvEncoderInitParam("");
    enc = std::make_unique<NvEncoderCuda>(cuContext, nWidth, nHeight, eFormat);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    NV_ENC_INITIALIZE_PARAMS initializeParams = {NV_ENC_INITIALIZE_PARAMS_VER};
    NV_ENC_CONFIG encodeConfig = {NV_ENC_CONFIG_VER};
#pragma GCC diagnostic pop
    initializeParams.encodeConfig = &encodeConfig;
    enc->CreateDefaultEncoderParams(&initializeParams,
                                    encodeCLIOptions.GetEncodeGUID(),
                                    encodeCLIOptions.GetPresetGUID());
    // send IDR periodically
    initializeParams.encodeConfig->gopLength = 60;
    initializeParams.encodeConfig->encodeCodecConfig.h264Config.idrPeriod = 60;
    initializeParams.encodeConfig->encodeCodecConfig.h264Config.repeatSPSPPS =
        1;
    encodeCLIOptions.SetInitParams(&initializeParams, eFormat);
    // std::cout << NvEncoderInitParam().FullParamToString(&initializeParams);
    enc->CreateEncoder(&initializeParams);

    int nFrameSize = enc->GetFrameSize();
    pHostFrame.resize(nFrameSize);
  }
  EncodedFrame Encode(int nWidth, int nHeight,
                      const std::vector<uint8_t>& image) {
    if (!enc) {
      InitEncoder(nWidth, nHeight, NV_ENC_BUFFER_FORMAT_ARGB);
    }
    int nFrameSize = enc->GetFrameSize();
    const uint8_t* rgb = image.data();
    uint8_t* argb = pHostFrame.data();
    for (int i = 0; i < nFrameSize / 4; ++i) {
      argb[0] = rgb[2];
      argb[1] = rgb[1];
      argb[2] = rgb[0];
      rgb += 3;
      argb += 4;
    }
    EncodedFrame encoded_frame;
    encoded_frame.width = nWidth;
    encoded_frame.height = nHeight;
    const NvEncInputFrame* encoderInputFrame = enc->GetNextInputFrame();
    NvEncoderCuda::CopyToDeviceFrame(
        cuContext, pHostFrame.data(), 0,
        (CUdeviceptr)encoderInputFrame->inputPtr, (int)encoderInputFrame->pitch,
        enc->GetEncodeWidth(), enc->GetEncodeHeight(), CU_MEMORYTYPE_HOST,
        encoderInputFrame->bufferFormat, encoderInputFrame->chromaOffsets,
        encoderInputFrame->numChromaPlanes);
    enc->EncodeFrame(encoded_frame.packets);
    return encoded_frame;
  }

 private:
  CUcontext cuContext = NULL;
  std::unique_ptr<NvEncoderCuda> enc;
  std::vector<uint8_t> pHostFrame;
};

class KVSWebRtcForwarder : public rclcpp::Node {
 public:
  KVSWebRtcForwarder(KVSClient& kvs_client,
                     const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : KVSWebRtcForwarder(kvs_client, "", options) {}
  KVSWebRtcForwarder(KVSClient& kvs_client, const std::string& name_space,
                     const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("kvs_webrtc_forwarder", name_space, options),
        kvs_client_{kvs_client} {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::QoS(10),
        std::bind(&KVSWebRtcForwarder::ImageCallback, this,
                  std::placeholders::_1));
    encoder_.Init();
  }

  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!kvs_client_.isReady()) {
      return;
    }
    EncodedFrame encoded_frame =
        encoder_.Encode(msg->width, msg->height, msg->data);
    kvs_client_.QueueEncodedFrame(std::move(encoded_frame));
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  CudaEncoder encoder_;
  KVSClient& kvs_client_;
};

int main(int argc, char* argv[]) {
  KVSClient& kvs_client = KVSClient::GetInstance();
  if (kvs_client.Init(SAMPLE_CHANNEL_NAME)) {
    kvs_client.Run();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KVSWebRtcForwarder>(kvs_client));
    rclcpp::shutdown();
  }
  kvs_client.CleanUp();
  return 0;
}
