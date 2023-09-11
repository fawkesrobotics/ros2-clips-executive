def process_line(line, mode, objects):
    # Strip parentheses and split words based on the mode
    line = line.strip("()\n")
    if mode == ":objects":
        # Split on " - " first and then on " "
        entries = [entry.strip() for entry in line.split(" - ")]
        obj_type = entries[1]
        obj_names = [entry.strip() for entry in entries[0].split(" ")]
        objects.setdefault("objects", {}).setdefault(obj_type, []).extend(obj_names) 
    elif mode == ":init":
        # Split on " "
        entries = line.split()
        if entries:
            fact_name = entries[0]
            parameters = entries[1:]
            objects.setdefault("facts", []).append({"fact_name": fact_name, "parameters": parameters})


def read_file(filename):
    mode = None
    objects = {}

    with open(filename, "r") as file:
        for line in file:
            if ")\n" == line:
                mode = None
            
            if mode is None:
                if ":objects" in line:
                    mode = ":objects"
                elif ":init" in line:
                    mode = ":init"
            else:
                process_line(line, mode, objects)


    return objects


# Example usage
filename = "instance-1.pddl"
objects = read_file(filename)
print(objects["objects"])
print(objects["facts"])
