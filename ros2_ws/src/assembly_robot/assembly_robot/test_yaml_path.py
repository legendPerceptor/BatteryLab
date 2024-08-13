import yaml
import rclpy
from pathlib import Path
from ament_index_python.packages import get_package_share_path

def main():
    rclpy.init()
    position_file = Path(get_package_share_path("assembly_robot")) / "yaml" / "well_positions.yaml"

    with open(position_file, "r") as f:
        try:
            constant_positions = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print("Cannot load the YAML file with error: ", e)
    
    print(constant_positions)
    assemble_post = constant_positions["AessemblePost"]
    print(assemble_post["rail_pos"])
    print(assemble_post["cartesian"])
    rclpy.shutdown()

if __name__ == '__main__':
    main()