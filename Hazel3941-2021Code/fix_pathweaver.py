import json
from pathlib import Path

deploy_path = Path("src/main/deploy/output")

for path_path in deploy_path.iterdir():
    print(f"Fixing {path_path.name}...")
    with path_path.open() as path_file:
        path_json = json.load(path_file)

        print(f"Original file contains {len(path_json)} steps.")

        new_path = []
        init_x = path_json[0]["pose"]["translation"]["x"]
        init_y = path_json[0]["pose"]["translation"]["y"]

        for step in path_json:
            new_step = step

            # Recenter
            new_step["pose"]["translation"]["x"] -= init_x
            new_step["pose"]["translation"]["y"] -= init_y

            # Invert y
            new_step["pose"]["translation"]["y"] *= -1

            new_path.append(new_step)
        
        print(f"Built new path containing {len(new_path)} steps!")

    print(f"Writing {path_path.name}...")
    with path_path.open("w") as path_file:
        json.dump(new_path, path_file)