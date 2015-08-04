controlit_models_dir=`rospack find controlit_models`

# Only include directories that contain models needed by Gazebo
sub_directories=(dreamer trikey controlit_models_common)
#sub_directories=`find $controlit_models_dir/.. -mindepth 1 -maxdepth 1 -type d \( ! -iname ".git" \)`

for d in ${sub_directories[@]}; do
    current_path="$controlit_models_dir/../$d/models"
	# echo "Adding to GAZEBO_MODEL_PATH: $current_path"
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$current_path
done
