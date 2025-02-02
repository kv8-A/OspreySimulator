import bpy
import os
import mathutils

def import_obj(filepath):
    """
    Imports the .obj file.
    """
    bpy.ops.import_scene.obj(filepath=filepath)

def rotate_obj():
    """
    Rotates the imported mesh so that the floors are on the xz plane and the tops are aligned with the negative y-axis.
    """
    imported_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
    if not imported_objects:
        print("No mesh objects found.")
        return

    imported_object = imported_objects[0]
    bpy.context.view_layer.objects.active = imported_object
    imported_object.select_set(True)

    # Rotate the object 90 degrees around the x-axis
    bpy.ops.transform.rotate(value=1.5708, orient_axis='X')  # -1.5708 radians is -90 degrees


def separate_buildings():
    """
    Separates the imported mesh by loose parts (individual buildings).
    """
    # Ensure we're in Object mode
    if bpy.context.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')

    # Find the imported mesh object
    imported_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
    if not imported_objects:
        print("No mesh objects found.")
        return
    
    imported_object = imported_objects[0]
    bpy.context.view_layer.objects.active = imported_object
    imported_object.select_set(True)

    # Switch to edit mode and separate by loose parts
    try:
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.separate(type='LOOSE')
        bpy.ops.object.mode_set(mode='OBJECT')
    except Exception as e:
        print(f"Error during separation: {e}")

def set_origin_to_geometry():
    """
    Sets the origin of each building to the center of its geometry.
    """
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH':
            bpy.context.view_layer.objects.active = obj
            obj.select_set(True)
            bpy.ops.object.origin_set(type='ORIGIN_CENTER_OF_MASS', center='BOUNDS')
            obj.select_set(False)


def set_origin_to_bottom_right():
    """
    Sets the origin of each building to the right bottom corner of its geometry.
    """
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH':
            bpy.context.view_layer.objects.active = obj
            obj.select_set(True)
            # Calculate the bounding box corners
            bbox = [obj.matrix_world @ mathutils.Vector(corner) for corner in obj.bound_box]
            # Determine if the Y-axis is positive or negative
            y_is_negative = any(corner.y < 0 for corner in bbox)
            if y_is_negative:
                bottom_right_corner = max(bbox, key=lambda v: (v.x, v.y))
            else:
                bottom_right_corner = min(bbox, key=lambda v: (v.x, v.y))
            
            # Set the origin to the bottom right corner
            bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')
            bpy.context.scene.cursor.location = bottom_right_corner
            bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
            obj.select_set(False)


def save_buildings_to_collections():
    """
    Moves each separated building to its own collection.
    """
    scene_collection = bpy.context.scene.collection
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH':
            collection_name = f'Building_{obj.name}'
            new_collection = bpy.data.collections.new(collection_name)
            bpy.context.scene.collection.children.link(new_collection)
            new_collection.objects.link(obj)
            if obj.name in scene_collection.objects:
                scene_collection.objects.unlink(obj)

def save_buildings_to_files(output_folder):
    """
    Saves each separated building to its own Blender file in the specified folder.
    """
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH':
            bpy.ops.object.select_all(action='DESELECT')
            obj.select_set(True)
            bpy.context.view_layer.objects.active = obj
            filepath = os.path.join(output_folder, f'{obj.name}.blend')
            bpy.ops.wm.save_as_mainfile(filepath=filepath)


def main(filepath):
    """
    Main function to import, separate, and save buildings.
    """
    import_obj(filepath)
    rotate_obj()
    separate_buildings()
    # set_origin_to_geometry()
    set_origin_to_bottom_right()
    save_buildings_to_collections()
    save_buildings_to_files("blender_envs/building_assets_files")

# Path to your .obj file
filepath = "blender_envs/blend_files/buildings_tutorial.obj"  # Replace with the path to your .obj file

# Clear existing meshes (optional)
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.select_by_type(type='MESH')
bpy.ops.object.delete()

# Run the main function
main(filepath)

# Save the Blender file
output_filepath = "blender_envs/building_spliter.blend"  # Replace with the desired save path
bpy.ops.wm.save_as_mainfile(filepath=output_filepath)
