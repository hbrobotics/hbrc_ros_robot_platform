#!/usr/bin/bash
log_file=/tmp/usbip_bind.log
echo "/usr/local/bin/usbipd_host.servce.sh started" > "$log_file"
/usr/bin/usbip list -p -l 2>&1 >> "$log_file"
st_link_bus_id="$(/usr/bin/usbip list -p -l)"
echo "st_link_bus_id1=$st_link_bus_id" >> "$log_file"
st_link_bus_id=`echo $st_link_bus_id | /usr/bin/grep 0483`
echo "st_link_bus_id2=$st_link_bus_id" >> "$log_file"
st_link_bus_id=`echo $st_link_bus_id | /usr/bin/sed s,busid=,,`
echo "st_link_bus_id3=$st_link_bus_id" >> "$log_file"
st_link_bus_id=`echo $st_link_bus_id | /usr/bin/sed s,#.*,,`
echo "st_link_bus_id4=$st_link_bus_id" >> "$log_file"
if [ "$st_link_bus_id" ]
then
    echo "attempting bind" >> "$log_file"
    /usr/bin/usbip bind -b "$st_link_bus_id" 2>&1 >> "$log_file"
    echo "bind attempted" >> "$log_file"
fi
echo "/usr/local/bin/usbipd.servce.sh ended" >> "$log_file"
   
