cmds=(  "ros2 launch agv_bringup base.launch.py"
	"ros2 launch agv_bringup sllidar.launch.py "
	"ros2 launch agv_bringup navigation2.launch.py "
	"ros2 launch agv_bringup collision_monitor.launch.py "
	 )


for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2 
done
