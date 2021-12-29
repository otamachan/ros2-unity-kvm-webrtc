var joy = new JoyStick('joyDiv');
var joy2 = new JoyStick('joyDiv2', {internalFillColor: '#AA0000', externalStrokeColor: '#880000'});
var viewer = {};

function getRandomClientId() {
    return Math.random()
        .toString(36)
        .substring(2)
        .toUpperCase();
}

function joyControl() {
    viewer.dataChannel.send(JSON.stringify({x: joy.GetY() / 50.0, y: -joy2.GetX() / 50.0, z: joy2.GetY() / 50.0, yaw: -joy.GetX() / 100.0}));
}

async function connect() {
    viewer.remoteView = document.getElementById('remote_view');
    const region = document.getElementById('region').value;
    const accessKeyId = document.getElementById('awsAccessKey').value;
    const secretAccessKey = document.getElementById('awsSecretAccessKey').value;
    const kinesisVideoClient = new AWS.KinesisVideo({
      region,
      accessKeyId,
      secretAccessKey,
    });
    const channel = document.getElementById('channel').value;
    const describeSignalingChannelResponse = await kinesisVideoClient
        .describeSignalingChannel({
            ChannelName: channel,
        })
        .promise();
    const channelARN = describeSignalingChannelResponse.ChannelInfo.ChannelARN;
    console.log('[VIEWER] Channel ARN: ', channelARN);

    const getSignalingChannelEndpointResponse = await kinesisVideoClient
        .getSignalingChannelEndpoint({
            ChannelARN: channelARN,
            SingleMasterChannelEndpointConfiguration: {
                Protocols: ['WSS', 'HTTPS'],
                Role: KVSWebRTC.Role.VIEWER,
            },
        })
        .promise();
    const endpointsByProtocol = getSignalingChannelEndpointResponse.ResourceEndpointList.reduce((endpoints, endpoint) => {
        endpoints[endpoint.Protocol] = endpoint.ResourceEndpoint;
        return endpoints;
    }, {});
    console.log('[VIEWER] Endpoints: ', endpointsByProtocol);

    const kinesisVideoSignalingChannelsClient = new AWS.KinesisVideoSignalingChannels({
        region: region,
        accessKeyId: accessKeyId,
        secretAccessKey: secretAccessKey,
        endpoint: endpointsByProtocol.HTTPS,
        correctClockSkew: true,
    });

    // Get ICE server configuration
    const getIceServerConfigResponse = await kinesisVideoSignalingChannelsClient
        .getIceServerConfig({
            ChannelARN: channelARN,
        })
        .promise();
    const iceServers = [];
    iceServers.push({ urls: `stun:stun.kinesisvideo.${region}.amazonaws.com:443` });
    getIceServerConfigResponse.IceServerList.forEach(iceServer =>
        iceServers.push({
            urls: iceServer.Uris,
            username: iceServer.Username,
            credential: iceServer.Password,
        }),
    );
    console.log('[VIEWER] ICE servers: ', iceServers);

    // Create Signaling Client
    viewer.signalingClient = new KVSWebRTC.SignalingClient({
        channelARN,
        channelEndpoint: endpointsByProtocol.WSS,
        clientId: getRandomClientId(),
        role: KVSWebRTC.Role.VIEWER,
        region: region,
        credentials: {
            accessKeyId: accessKeyId,
            secretAccessKey: secretAccessKey,
        },
        systemClockOffset: kinesisVideoClient.config.systemClockOffset,
    });

    const constraints = {
        video: false,
        audio: false,
    };
    const configuration = {
        iceServers,
        iceTransportPolicy: 'all',
    };
    viewer.peerConnection = new RTCPeerConnection(configuration);    
    viewer.dataChannel = viewer.peerConnection.createDataChannel('kvsDataChannel');
    viewer.dataChannel.onopen = event => {
        console.log('[VIEWER] Data channel opend');
	viewer.controlInterval = setInterval(() => joyControl(), 100);
    };

    viewer.signalingClient.on('open', async () => {
        console.log('[VIEWER] Connected to signaling service');

        // Create an SDP offer to send to the master
        console.log('[VIEWER] Creating SDP offer');
        await viewer.peerConnection.setLocalDescription(
            await viewer.peerConnection.createOffer({
                offerToReceiveAudio: true,
                offerToReceiveVideo: true,
            }),
        );

        // When trickle ICE is enabled, send the offer now and then send ICE candidates as they are generated. Otherwise wait on the ICE candidates.
        console.log('[VIEWER] Sending SDP offer');
        viewer.signalingClient.sendSdpOffer(viewer.peerConnection.localDescription);
        console.log('[VIEWER] Generating ICE candidates');
    });

    viewer.signalingClient.on('sdpAnswer', async answer => {
        // Add the SDP answer to the peer connection
        console.log('[VIEWER] Received SDP answer');
        await viewer.peerConnection.setRemoteDescription(answer);
    });

    viewer.signalingClient.on('iceCandidate', candidate => {
        // Add the ICE candidate received from the MASTER to the peer connection
        console.log('[VIEWER] Received ICE candidate');
        viewer.peerConnection.addIceCandidate(candidate);
    });

    viewer.signalingClient.on('close', () => {
        console.log('[VIEWER] Disconnected from signaling channel');
    });

    viewer.signalingClient.on('error', error => {
        console.error('[VIEWER] Signaling client error: ', error);
    });

    // Send any ICE candidates to the other peer
    viewer.peerConnection.addEventListener('icecandidate', ({ candidate }) => {
        if (candidate) {
            console.log('[VIEWER] Generated ICE candidate');

            // When trickle ICE is enabled, send the ICE candidates as they are generated.
            console.log('[VIEWER] Sending ICE candidate');
            viewer.signalingClient.sendIceCandidate(candidate);
        } else {
            console.log('[VIEWER] All ICE candidates have been generated');
        }
    });

    // As remote tracks are received, add them to the remote view
    viewer.peerConnection.addEventListener('track', event => {
        console.log('[VIEWER] Received remote track');
        if (viewer.remoteView.srcObject) {
            return;
        }
        viewer.remoteStream = event.streams[0];
        viewer.remoteView.srcObject = viewer.remoteStream;
	viewer.remoteView.play().catch(err => console.error(err));
    });

    console.log('[VIEWER] Starting viewer connection');
    viewer.signalingClient.open();
}

function disconnect() {
    console.log('[VIEWER] Stopping viewer connection');
    if (viewer.signalingClient) {
        viewer.signalingClient.close();
        viewer.signalingClient = null;
    }

    if (viewer.peerConnection) {
        viewer.peerConnection.close();
        viewer.peerConnection = null;
    }

    if (viewer.remoteStream) {
        viewer.remoteStream.getTracks().forEach(track => track.stop());
        viewer.remoteStream = null;
    }

    if (viewer.controlInterval) {
        clearInterval(viewer.controlInterval);
        viewer.controlInterval = null;
    }

    if (viewer.remoteView) {
        viewer.remoteView.srcObject = null;
    }

    if (viewer.dataChannel) {
        viewer.dataChannel = null;
    }
}
