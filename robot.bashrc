alias sb='source ~/.bashrc; echo \"Reloaded bashrc\"'
alias ros='source /opt/ros/humble/setup.bash; echo "Activated ROS2 humble"'
alias torch='export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(python3 -c "import torch; print(torch.utils.cmake_prefix_path)"); echo "Activate Pytorch"'