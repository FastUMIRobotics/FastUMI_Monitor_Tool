#!/usr/bin/env bash
XVISIO_SERIALS=($(rostopic list 2>/dev/null | grep -o '/xv_sdk/[^/]*' | cut -d'/' -f3 | grep -v -E '^(parameter_descriptions|parameter_updates|new_device)$' | sort -u))
bash rviz/scripts/generate_configs.sh $XVISIO_SERIALS
echo "generate rviz config success"
export xv_serial_number=$XVISIO_SERIALS
python3 pose_to_markers.py $XVISIO_SERIALS &
echo "wait start pose_to_markers.py"
sleep 2
echo "open menu"
bash run_rostopic_menu.sh
