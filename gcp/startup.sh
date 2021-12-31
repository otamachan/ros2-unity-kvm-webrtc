#!/bin/bash

CONTAINER_NAME=gcp-ros2-xserver
CONTAINER_IMAGE=otamachan/gcp-ros2-xserver
UNITY_SIM_ZIP=unity-ros2-sample.zip
UNITY_SIM_ZIP_ID=1CqufHDrNqtIsJhbnjjVhDIY8HNupKxBR

# mkdir
mkdir -p /opt/sim
cd /opt/sim

# git clone
git clone --recursive https://github.com/otamachan/ros2-unity-kvm-webrtc.git

# download unity app
curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=${UNITY_SIM_ZIP_ID}" > /dev/null
CODE="$(awk '/_warning_/ {print $NF}' /tmp/cookie)"  
curl -Lb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=${CODE}&id=${UNITY_SIM_ZIP_ID}" -o ${UNITY_SIM_ZIP}
unzip ${UNITY_SIM_ZIP}

# run xserver
docker run --gpus all --name ${CONTAINER_NAME} -d -it --rm -v /opt/sim:/sim -p 8081:8081 ${CONTAINER_IMAGE}

cat << EOF > run.sh
#!/bin/bash
CONTAINER_NAME=${CONTAINER_NAME}
# build ROS2 node
sudo docker exec -w /sim -it \${CONTAINER_NAME} bash -c "mkdir -p ws/src; ln -sf ../../ros2-unity-kvm-webrtc/ros2_kvm_webrtc_sample/ ws/src/ros2_kvm_webrtc_sample; cd ws; colcon build"
# run Unity Simulator
sudo docker exec \${CONTAINER_NAME} /sim/unity-ros2-sample/unity-ros2-sample.x86_64 &&
# run ROS2/WebRTC proxy
sudo docker exec -w /sim -it -e AWS_ACCESS_KEY_ID=\${AWS_ACCESS_KEY_ID} -e AWS_SECRET_ACCESS_KEY=\${AWS_SECRET_ACCESS_KEY} AWS_DEFAULT_REGION=ap-northeast-1 \${CONTAINER_NAME} bash -c "source ws/install/setup.bash; ros2 run ros2_kvm_webrtc_sample ros2_kvm_webrtc_sample"
EOF

chmod +x run.sh
