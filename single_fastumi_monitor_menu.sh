#!/usr/bin/env bash
XVISIO_SERIALS="$1"
bash rviz/scripts/generate_configs.sh $XVISIO_SERIALS
echo "generate rviz config success"
echo "install dependencies"
conda run -n fastumi pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
echo "install dependencies finish"
conda run -n fastumi python3 pose_to_markers.py $XVISIO_SERIALS &
echo "wait start pose_to_markers.py"
sleep 2
echo "open menu"
bash run_rostopic_menu.sh $XVISIO_SERIALS
