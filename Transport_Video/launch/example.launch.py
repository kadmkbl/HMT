from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import ruamel.yaml

ruamel_yaml = ruamel.yaml.YAML(typ="safe")
ruamel_yaml.default_flow_style = False


def generate_launch_description():
    ld = LaunchDescription()

    # example_config_file, example_namespace = prompt_for_example_namepace()
    workspace = get_package_share_directory("video_io").split("/install")[0]

    example_config_file = f"{workspace}/src/video_io/config/example.yaml"
    with open(example_config_file, "r") as yaml_file:
        config_dict = ruamel_yaml.load(yaml_file)

    print("\nPick an example :")
    input_dict = {}
    for idx, (namespace, namespace_dict) in enumerate(config_dict.items(), start=1):
        example_description = namespace_dict.get("example_name", "")
        if idx == 1:
            print(f"   {idx}. {namespace}, {example_description} (default)")
        else:
            print(f"   {idx}. {namespace}, {example_description}")
        input_dict[idx] = namespace

    input_prompt = input("\nselect index for an example:")

    print(f"\ninput selected:\n    {input_prompt}")
    if input_prompt == "":
        selected_input = 1
    else:
        try:
            selected_input = int(input_prompt)
        except:
            print("Error with input")
            exit()

    example_namespace = input_dict[selected_input]

    nodes_to_launch = config_dict[example_namespace]["nodes_to_launch"]

    for node in nodes_to_launch:

        try:
            params = config_dict[example_namespace][node["name"]]["ros__parameters"]
        except:
            params = {}

        if node["executable"] == "rqt_image_view":
            args = f"/{example_namespace}/video_player/image"
        else:
            args = []

        node_to_add = Node(
            namespace=example_namespace,
            package=node["package"],
            executable=node["executable"],
            name=node["name"],
            parameters=[params],
            arguments=[args],
        )
        ld.add_action(node_to_add)

    return ld