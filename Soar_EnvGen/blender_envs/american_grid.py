import bpy
import random
import math 
import mathutils


# def create_building(location, width, depth, height):
#     """
#     Creates a building at the specified location with the given dimensions.
#     """
#     bpy.ops.mesh.primitive_cube_add(size=1, enter_editmode=False, align='WORLD', location=location)
#     building = bpy.context.active_object
#     building.scale = (width, depth, height)
#     return building

# def create_street_grid(building_count, block_size, street_width):
#     """
#     Creates a grid of buildings separated by streets.
    
#     :param building_count: Number of buildings to create
#     :param block_size: The size of each block (building + street)
#     :param street_width: The width of the streets
#     """
#     buildings_per_row = round((building_count ** 0.5))

#     for i in range(building_count):
#         row = i // buildings_per_row
#         col = i % buildings_per_row

#         # Calculate location based on row and column in the grid
#         location_x = (col * block_size) + street_width
#         location_y = (row * block_size) + street_width

#         # Define the size of the building
#         building_width = block_size - street_width
#         building_depth = block_size - street_width
#         building_height = random.uniform(25, 60)

#         # Create the building at the calculated location
#         create_building((location_x, location_y, building_height / 2), building_width / 2, building_depth / 2, building_height)

# # Clear existing meshes (optional)
# bpy.ops.object.select_all(action='DESELECT')
# bpy.ops.object.select_by_type(type='MESH')
# bpy.ops.object.delete()

# # Parameters for the urban environment
# building_count = 30
# block_size = 20  # Size of a block including street
# street_width = 1  # Width of the streets

# # Create the urban environment
# create_street_grid(building_count, block_size, street_width)


def create_building(location, width, depth, height, collection):
    """
    Creates a building at the specified location with the given dimensions.
    The building is oriented to stand on the XZ plane with height extending into the negative Y-direction.
    """
    bpy.ops.mesh.primitive_cube_add(size=1, enter_editmode=False, align='WORLD', location=(0, 0, 0))  # Place at origin temporarily
    building = bpy.context.active_object
    building.scale = (width, depth, height)
    
    # Adjust the building's location considering the height in the negative Y-direction
    # adjusted_location = (location[0], location[1] - height, location[2])
    adjusted_location = (location[0], location[1], location[2])
    building.location = adjusted_location

    # Link the building to the specified collection and unlink from the scene's default collection
    collection.objects.link(building)
    bpy.context.collection.objects.unlink(building)
    return building

def check_overlap(building, buildings):
    """
    Checks if the given building overlaps with any existing building in the buildings list.
    """
    for existing_building in buildings:
        if building.bound_box and existing_building.bound_box:
            # Calculate bounding boxes in world coordinates
            building_bbox = [building.matrix_world @ mathutils.Vector(corner) for corner in building.bound_box]
            existing_bbox = [existing_building.matrix_world @ mathutils.Vector(corner) for corner in existing_building.bound_box]
            
            # Check for overlap
            if (min([v.x for v in building_bbox]) < max([v.x for v in existing_bbox]) and
                max([v.x for v in building_bbox]) > min([v.x for v in existing_bbox]) and
                min([v.z for v in building_bbox]) < max([v.z for v in existing_bbox]) and
                max([v.z for v in building_bbox]) > min([v.z for v in existing_bbox])):
                return True
    return False

def create_street_grid(building_count, block_size, street_width, width_limits, depth_limits, height_limits):
    """
    Creates a grid of buildings on the XZ plane, separated by streets, with heights extending into the negative Y-direction.
    """
    buildings_per_row = round((building_count ** 0.5))
    
    # Create a new collection for the buildings
    buildings_collection = bpy.data.collections.new("XZ Plane Buildings")
    bpy.context.scene.collection.children.link(buildings_collection)

    buildings = []
    for i in range(building_count):
        row = i // buildings_per_row
        col = i % buildings_per_row

        # Calculate location on the XZ plane, with Y for height
        location_x = (col * block_size) + street_width + 20
        location_z = (row * block_size) + street_width +45
        # The Y-coordinate represents the "height" extending in the negative direction
        location_y = 0  # Adjust this based on how you want to position them vertically

        # Define the size of the building
        # building_width = block_size - street_width
        # building_depth = block_size - street_width
        # building_width = random.uniform(block_size / 2, block_size - street_width)
        # building_depth = random.uniform(block_size / 2, block_size - street_width)
        building_width = random.uniform(*width_limits)
        building_depth = random.uniform(*depth_limits)
        
        building_height = random.uniform(25, 60)

        # Create the building and add it to the collection
        # create_building((location_x, -building_height / 2, location_z), building_width / 2, building_height, building_depth / 2, buildings_collection)
        building = create_building((location_x, -building_height / 2, location_z), building_width / 2, building_height, building_depth / 2, buildings_collection)

        placed = False
        attempts = 0
        while not placed and attempts < 100:  # Try to place the building 100 times before giving up
            attempts += 1
            # Calculate location on the XZ plane
            location_x = (col * block_size) + street_width + 20 + random.uniform(-block_size/2, block_size/2)
            location_z = (row * block_size) + street_width + 45 + random.uniform(-block_size/2, block_size/2)
            location_y = 0  # Adjust this based on how you want to position them vertically

            # Create the building and check for overlap
            building = create_building((location_x, -building_height / 2, location_z), building_width, building_depth, building_height, buildings_collection)
            if not check_overlap(building, buildings):
                buildings.append(building)
                placed = True
            else:
                # Delete the overlapping building and try again
                bpy.data.objects.remove(building, do_unlink=True)

        if not placed:
            print(f"Failed to place building {i} after {attempts} attempts.")


# Clear existing meshes (optional)
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.select_by_type(type='MESH')
bpy.ops.object.delete()

# Parameters for the urban environment
building_count = 30
block_size = 20  # Size of a block including streets
street_width = 10  # Width of the streets

# Limits for building dimensions (width, depth, height)
width_limits = (10, 60)  # Example: min 5, max 30
depth_limits = (10, 30)  # Example: min 5, max 30
height_limits = (25, 70)  # Example: min 25, max 60

# Create the urban environment
create_street_grid(building_count, block_size, street_width, width_limits, depth_limits, height_limits)

# Create the urban environment


# Save the Blender file
filepath = "blender_envs/blend_files/"  # Specify the desired filepath and filename
filename = "buildings_testing_shapes.blend"
bpy.ops.wm.save_as_mainfile(filepath=filepath+filename)