from typing import List
import yaml
import sys
import os
import pprint

# Generate a markdown table of all the parts in YAML config file
# Usege: python tools/parts_list_generator.py config/irsl/irsl_assembler_config.yaml


class RobotPart:
    def __init__(
        self,
        label: str = "-",
        type: str = "-",
        description: str = "-",
        print_file: str = "-",
    ):
        self.label = label
        self.type = type
        self.description = description
        self.print_file = print_file

    def __str__(self):
        return f"{self.label} ({self.type}): {self.description} <{self.print_file}>"

    __repr__ = __str__


def load_yaml(path: str) -> dict:
    try:
        with open(path) as file:
            config = yaml.safe_load(file)
            return config
    except Exception as e:
        print(f"Failed to load {path}: {e}")
        raise e


def parse_panel_settings(config: dict) -> dict:
    panels: dict = {}

    if "PanelSettings" in config:
        if "tab_list" in config["PanelSettings"]:
            for tab in config["PanelSettings"]["tab_list"]:
                panels[tab["name"]] = tab["parts"]

            print(f"Loaded {len(panels)} panels...")
            return panels

    raise Exception("Failed to parse panel settings")


def parse_parts_settings(config: dict) -> dict:
    parts: dict = {}

    if "PartsSettings" in config:
        for part_dict in config["PartsSettings"]:
            part = RobotPart()
            if "type" in part_dict:
                part.type = part_dict["type"]
            if "class" in part_dict:
                part.label = part_dict["class"]
            if "description" in part_dict:
                part.description = part_dict["description"]
            if "print-file" in part_dict:
                part.print_file = part_dict["print-file"]
            parts[part.type] = part

        print(f"Loaded {len(parts)} parts...")
        return parts

    raise Exception("Failed to parse parts settings")


def generate_markdown_table(panels: dict, parts: dict) -> str:
    print("Generating a markdown file...")

    file_name = "docs/parts_list.md"
    os.makedirs(os.path.dirname(file_name), exist_ok=True)

    with open("docs/parts_list.md", "w") as file:
        file.write("# Parts List\n\n")

        for panel, parts_list in panels.items():
            file.write(f"## {panel}\n")
            file.write("| Name | Description | 3D print file |\n")
            file.write("| --- | --- | --- |\n")
            for part_type in parts_list:
                part: RobotPart = parts[part_type]
                file.write(
                    f"| **{part.label}** [{part.type}] | {part.description} | {part.print_file} |\n"
                )
            file.write("\n")

        print("🚀 Successfully generated markdown table in docs/parts_list.md!")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: parts_list_generator.py <path/to/config.yaml>")
        sys.exit(1)

    config = load_yaml(sys.argv[1])
    panels = parse_panel_settings(config)
    parts = parse_parts_settings(config)

    generate_markdown_table(panels, parts)
