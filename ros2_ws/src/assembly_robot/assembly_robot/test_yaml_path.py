import yaml
from pathlib import Path

def main():
    
    position_file = Path(__file__).parent / ".." / "resource" / "well_positions.yaml"

    with open(position_file, "r") as f:
        try:
            constant_positions = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print("Cannot load the YAML file with error: ", e)
    
    print(constant_positions)
    assemble_post = constant_positions["AessemblePost"]
    print(assemble_post["rail_pos"])
    print(assemble_post["cartesian"])

if __name__ == '__main__':
    main()