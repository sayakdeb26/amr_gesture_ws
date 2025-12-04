import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Launch arguments
    ui_port = int(LaunchConfiguration('ui_port').perform(context))
    auto_approve = LaunchConfiguration('auto_approve').perform(context).lower() == 'true'
    
    # ZED Camera Node
    # We assume the standard zed_wrapper is installed.
    # We remap the ZED's RGB output to /image_raw so the rest of the pipeline works unchanged.
    
    try:
        zed_wrapper_share = get_package_share_directory('zed_wrapper')
        zed_launch_file = os.path.join(zed_wrapper_share, 'launch', 'zed_camera.launch.py')
        
        zed_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch_file),
            launch_arguments={
                'camera_model': 'zed2i'
            }.items()
        )
        
        # Relay ZED topic to /image_raw
        zed_topic_relay = Node(
            package='topic_tools',
            executable='relay',
            name='zed_to_pipeline_relay',
            arguments=['/zed/zed_node/rgb/image_rect_color', '/image_raw'],
            output='screen'
        )
        
    except Exception as e:
        print(f"Warning: zed_wrapper not found. This launch file requires 'zed_wrapper' package. Error: {e}")
        zed_node = None
        zed_topic_relay = None

    # --- Existing Pipeline Nodes (Copied from pipeline.launch.py) ---

    # 2. Simplifier
    simplifier_node = Node(
        package='frame_simplifier_pkg',
        executable='simplifier_node',
        name='simplifier_node',
        parameters=[{
            'input_topic': '/image_raw',
            'output_topic': '/frames/simplified',
            'output_width': 320,
            'output_height': 240,
            'target_fps': 10.0
        }]
    )

    # 3. LSTM
    lstm_node = Node(
        package='gesture_classifiers_pkg',
        executable='lstm_node',
        name='lstm_node',
        parameters=[{
            'min_frames': 30,
            'conf_threshold': 0.80,
            'model_path': '/home/sayak/amr_gesture_ws/models/lstm/jester20b_12cls/final_jester_model.onnx'
        }],
        remappings=[('/camera/image_raw', '/image_raw')]
    )

    # 4. Recorder
    recorder_node = Node(
        package='vlm_recorder_pkg',
        executable='recorder_node',
        name='recorder_node',
        parameters=[{
            'input_topic': '/frames/simplified',
            'save_dir': '/home/sayak/amr_gesture_ws/data/runtime_clips'
        }]
    )

    # 5. Bridge
    bridge_node = Node(
        package='vlm_bridge_pkg',
        executable='bridge_node',
        name='bridge_node',
        parameters=[{
            'confirm_timeout_s': 20.0,
            'wait_clip_timeout_s': 5.0,
            'default_clip': ''
        }]
    )

    # 6. Central DB
    db_node = Node(
        package='central_db_pkg',
        executable='central_db_node',
        name='central_db_node'
    )

    # 7. UI Kiosk
    ui_node = Node(
        package='ui_kiosk_pkg',
        executable='ui_kiosk_node',
        name='ui_kiosk_node',
        parameters=[{
            'host': '0.0.0.0',
            'port': ui_port,
            'auto_approve': auto_approve,
            'deadline_s': 20.0
        }]
    )

    # 8. Telemetry
    telemetry_node = Node(
        package='amr_telemetry_pkg',
        executable='telemetry_node',
        name='telemetry_node'
    )

    # 9. VLM (Real or Dummy) - Using rosgpu_isolated venv
    vlm_node = Node(
        package='vlm_videollava_pkg',
        executable='/home/sayak/amr_gesture_ws/src/vlm_videollava_pkg/vlm_videollava_pkg/video_llava_node_wrapper.sh',
        name='video_llava_node',
        output='screen'
    )
    
    nodes = [
        simplifier_node,
        lstm_node,
        recorder_node,
        bridge_node,
        db_node,
        ui_node,
        telemetry_node,
        vlm_node
    ]
    
    if zed_node:
        nodes.insert(0, zed_node)
    if zed_topic_relay:
        nodes.insert(1, zed_topic_relay)
        
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ui_port', default_value='8008'),
        DeclareLaunchArgument('auto_approve', default_value='false'),
        OpaqueFunction(function=launch_setup)
    ])
