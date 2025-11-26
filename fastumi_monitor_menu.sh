#!/usr/bin/env bash
XVISIO_SERIALS=$(rostopic list 2>/dev/null | grep -o '/xv_sdk/[^/]*' | cut -d'/' -f3 | grep -v -E '^(parameter_descriptions|parameter_updates|new_device)$' | sort -u)
echo $XVISIO_SERIALS

#cat -A <<< "$XVISIO_SERIALS"

mapfile -t XVISIO_SERIAL_ARRAY <<< "$XVISIO_SERIALS"


# 遍历数组
for item in ${XVISIO_SERIAL_ARRAY[@]}; do
    echo "当前序列号: $item"
    gnome-terminal -- bash -c "bash single_fastumi_monitor_menu.sh $item"
done


