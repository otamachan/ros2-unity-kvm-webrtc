#!/bin/bash
export IMAGE_FAMILY="common-cu110-ubuntu-2004"
export ZONE="asia-northeast1-a"
# export ZONE="southamerica-east1-c"
export INSTANCE_NAME="ros2-unity-xserver"
export INSTANCE_TYPE="n1-standard-4"
gcloud compute instances create $INSTANCE_NAME \
       --zone=$ZONE \
       --image-family=$IMAGE_FAMILY \
       --image-project=deeplearning-platform-release \
       --maintenance-policy=TERMINATE \
       --accelerator="type=nvidia-tesla-t4-vws,count=1" \
       --machine-type=$INSTANCE_TYPE \
       --boot-disk-size=120GB \
       --metadata="install-nvidia-driver=True" \
       --metadata-from-file="startup-script=startup.sh" \
       --preemptible
