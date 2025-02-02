import bpy
import os
import random
import mathutils

def import_building(filepath, building_name):
    """
    Imports a building from a Blender file.
    """
    with bpy.data.libraries.load(filepath, link=False) as (data_from, data_to):
        if building_name in data_from.objects:
            data_to.objects = [building_name]
    
    for obj in data_to.objects:
        if obj is not None:
            bpy.context.collection.objects.link(obj)
            return obj
    return None

# def check_overlap(obj, other_objs, buffer=20.0):
def check_overlap(obj, other_objs):
    """
    Checks if the object overlaps with any objects in the list.
    """
    # Calculate the bounding box of the object in local coordinates
    obj_bbox_local = [mathutils.Vector(corner) for corner in obj.bound_box]
    obj_min_local = mathutils.Vector((min(v.x for v in obj_bbox_local), min(v.y for v in obj_bbox_local), min(v.z for v in obj_bbox_local)))
    obj_max_local = mathutils.Vector((max(v.x for v in obj_bbox_local), max(v.y for v in obj_bbox_local), max(v.z for v in obj_bbox_local)))
    
    # Translate the bounding box to the object's location
    obj_min_world = obj.location + obj_min_local
    obj_max_world = obj.location + obj_max_local

    print(f"Checking building at location: {obj.location}, Bounding Box World Coordinates: Min: {obj_min_world}, Max: {obj_max_world}")

    for other_obj in other_objs:
        other_bbox_local = [mathutils.Vector(corner) for corner in other_obj.bound_box]
        other_min_local = mathutils.Vector((min(v.x for v in other_bbox_local), min(v.y for v in other_bbox_local), min(v.z for v in other_bbox_local)))
        other_max_local = mathutils.Vector((max(v.x for v in other_bbox_local), max(v.y for v in other_bbox_local), max(v.z for v in other_bbox_local)))

        # Translate the bounding box to the other object's location
        other_min_world = other_obj.location + other_min_local
        other_max_world = other_obj.location + other_max_local

        print(f"Comparing with building at location: {other_obj.location}, Bounding Box World Coordinates: Min: {other_min_world}, Max: {other_max_world}")

        if (obj_min_world.x < other_max_world.x and obj_max_world.x > other_min_world.x and
            obj_min_world.y < other_max_world.y and obj_max_world.y > other_min_world.y):
            print("Overlap detected.")
            return True

    print("No overlap detected.")
    return False

def apply_texture(obj, texture_path):
    """
    Applies a texture to an object.
    """
    # Ensure the object has a UV map
    if len(obj.data.uv_layers) == 0:
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.uv.smart_project()
        bpy.ops.object.mode_set(mode='OBJECT')
    
    # Create a new material with nodes
    mat = bpy.data.materials.new(name="Material")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    
    # Add image texture node
    tex_image = mat.node_tree.nodes.new('ShaderNodeTexImage')
    tex_image.image = bpy.data.images.load(texture_path)
    tex_image.projection = 'BOX'
    mat.node_tree.links.new(bsdf.inputs['Base Color'], tex_image.outputs['Color'])
    
    # Add texture mapping and coordinate nodes for scaling
    mapping = mat.node_tree.nodes.new('ShaderNodeMapping')
    tex_coord = mat.node_tree.nodes.new('ShaderNodeTexCoord')

    # Set the mapping type to box
    tex_coord.outputs['Generated'].default_value = (1.0, 1.0, 1.0)

    # Set the scale of the texture
    mapping.inputs['Scale'].default_value[0] = 1.0  # Adjust these values to scale the texture properly
    mapping.inputs['Scale'].default_value[1] = 1.0
    mapping.inputs['Scale'].default_value[2] = 1.0
    
    # Link texture coordinate to mapping node, and mapping node to texture
    mat.node_tree.links.new(mapping.inputs['Vector'], tex_coord.outputs['Generated'])
    mat.node_tree.links.new(tex_image.inputs['Vector'], mapping.outputs['Vector'])

    # Assign the material to the object
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)

    # Ensure the texture is visible in the viewport
    obj.active_material = mat
    bpy.context.view_layer.update()

def create_random_field(building_folder, building_texture_folder, field_size, building_count):
    """
    Creates a random field of buildings.
    """
    building_files = [os.path.join(building_folder, f) for f in os.listdir(building_folder) if f.endswith('.blend')]
    building_textures = [os.path.join(building_texture_folder, f) for f in os.listdir(building_texture_folder) if f.endswith('.jpg')]
    placed_buildings = []

    for _ in range(building_count):
        building_file = random.choice(building_files)
        building_name = os.path.splitext(os.path.basename(building_file))[0]
        imported_building = import_building(building_file, building_name)
        
        if imported_building:
            for attempt in range(100):
                random_location = (
                    random.uniform((-field_size / 2)+field_size/6, (field_size / 2)-field_size/6),
                    random.uniform((-field_size / 2)+field_size/6, (field_size / 2)-field_size/6),
                    0  # Keep the Z coordinate at 0
                )
                imported_building.location = random_location
                print(f"building name : {building_name}")
                if not check_overlap(imported_building, placed_buildings):
                    placed_buildings.append(imported_building)
                    # Apply random texture to the building
                    random_texture = random.choice(building_textures)
                    apply_texture(imported_building, random_texture)
                    break
            else:
                # If we failed to place the building after 100 attempts, remove it
                bpy.data.objects.remove(imported_building)

def add_ground_plane(size, texture_folder):
    """
    Adds a ground plane of the given size on the XZ plane and applies a random texture.
    """
    bpy.ops.mesh.primitive_plane_add(size=size, enter_editmode=False, align='WORLD', rotation=(0, 0, 0))
    ground_plane = bpy.context.object
    ground_plane.name = "Ground Plane"
    
    # Apply random texture to the ground plane
    plane_textures = [os.path.join(texture_folder, f) for f in os.listdir(texture_folder) if f.endswith('.png')]
    if plane_textures:
        random_texture = random.choice(plane_textures)
        apply_texture(ground_plane, random_texture)

def create_random_field_of_buildings(building_folder, building_texture_folder, plane_texture_folder, field_size, building_count, output_filepath):
    """
    Main function to create a random field of buildings.
    """
    # Ensure the output directory exists
    output_dir = os.path.dirname(output_filepath)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Add the ground plane
    add_ground_plane(field_size, plane_texture_folder)
    
    # Create the random field of buildings
    create_random_field(building_folder, building_texture_folder, field_size, building_count)

    # Set viewport shading to Material Preview to see textures
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D':
                    space.shading.type = 'MATERIAL'
                    break

    # Save the Blender file
    bpy.ops.file.pack_all() # packs all external data 
    bpy.ops.wm.save_as_mainfile(filepath=output_filepath)

# Paths to the folders
building_folder = "/home/kjell/Documents/Repositories/Soar_EnvGen/blender_envs/building_assets_files"  # Replace with the path to your building assets folder
building_texture_folder = "/home/kjell/Documents/Repositories/Soar_EnvGen/blender_envs/building_assets_files/textures/building_textures"
plane_texture_folder = "/home/kjell/Documents/Repositories/Soar_EnvGen/blender_envs/building_assets_files/textures/ground_planes"
field_size = 100  # Size of the field (e.g., 200x200 units)
building_count = 4  # Number of buildings to place in the field
# output_filepath = "/home/kjell/Documents/Repositories/Soar_EnvGen/blender_envs/random_field_new.blend"  # Replace with the desired save path
output_filepath = "/home/kjell/Documents/Repositories/Soar_EnvGen/blender_envs/random_field_flow_test.blend"  # Replace with the desired save path

# Clear existing meshes (optional)
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.select_by_type(type='MESH')
bpy.ops.object.delete()

# Run the main function
create_random_field_of_buildings(building_folder, building_texture_folder, plane_texture_folder, field_size, building_count, output_filepath)
