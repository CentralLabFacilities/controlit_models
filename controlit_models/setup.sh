controlit_models_dir=`rospack find controlit_models`
config_directories=`find $controlit_models_dir/.. -mindepth 1 -maxdepth 1 -type d \( ! -iname ".git" \)`

for d in $config_directories; do
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$d/models
    # export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$d/lib
done
