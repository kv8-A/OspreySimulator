import bpy

def create_textured_cube(texture_path):
    """
    Creates a cube object and applies a texture to all its faces.
    """
    # Clear existing objects
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    
    # Create a new cube
    bpy.ops.mesh.primitive_cube_add(size=2)
    cube = bpy.context.active_object

    # Ensure the object is active and in object mode
    bpy.context.view_layer.objects.active = cube
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')

    # Apply Smart UV Project for UV mapping
    bpy.ops.uv.smart_project(angle_limit=66, island_margin=0.02)
    bpy.ops.object.mode_set(mode='OBJECT')

    # Create a new material
    mat = bpy.data.materials.new(name="CubeMaterial")
    mat.use_nodes = True

    # Get the Principled BSDF node
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if not bsdf:
        bsdf = mat.node_tree.nodes.new(type="ShaderNodeBsdfPrincipled")
    
    # Add image texture node
    tex_image = mat.node_tree.nodes.new('ShaderNodeTexImage')
    
    try:
        tex_image.image = bpy.data.images.load(texture_path)
        print(f"Successfully loaded texture: {texture_path}")
    except Exception as e:
        print(f"Failed to load texture: {texture_path}. Error: {e}")
        return

    # Add texture coordinate and mapping nodes
    tex_coord = mat.node_tree.nodes.new('ShaderNodeTexCoord')
    mapping = mat.node_tree.nodes.new('ShaderNodeMapping')
    
    # Link nodes
    mat.node_tree.links.new(mapping.inputs['Vector'], tex_coord.outputs['Generated'])
    mat.node_tree.links.new(tex_image.inputs['Vector'], mapping.outputs['Vector'])
    mat.node_tree.links.new(bsdf.inputs['Base Color'], tex_image.outputs['Color'])

    # Assign the material to the cube
    if cube.data.materials:
        cube.data.materials[0] = mat
    else:
        cube.data.materials.append(mat)

    # Ensure the texture is visible in the viewport
    cube.active_material = mat
    bpy.context.view_layer.update()

# Path to your texture image
texture_path = "blender_envs/BuildingsHighRise0294.jpg"

# Clear existing meshes (optional)
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.select_by_type(type='MESH')
bpy.ops.object.delete()

# Create and texture the cube
create_textured_cube(texture_path)

# Save the Blender file (optional)
output_filepath = "blender_envs/textured_cube.blend"
bpy.ops.wm.save_as_mainfile(filepath=output_filepath)
