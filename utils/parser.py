import yaml

def parser(file_name):
    with open(file_name, 'r') as file:
        configs = yaml.safe_load(file)
        return configs  # Return the parsed data
