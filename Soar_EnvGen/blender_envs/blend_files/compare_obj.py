import pandas as pd

def read_obj(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
    vertices = []
    faces = []
    objects = []
    materials = []
    
    for line in lines:
        if line.startswith('v '):
            vertices.append(line.strip())
        elif line.startswith('f '):
            faces.append(line.strip())
        elif line.startswith('o '):
            objects.append(line.strip())
        elif line.startswith('usemtl'):
            materials.append(line.strip())
    
    return {
        'vertices': vertices,
        'faces': faces,
        'objects': objects,
        'materials': materials
    }

# Paths to the uploaded files
file_1_path = 'blender_envs/blend_files/buildings_american_grid.obj'
file_1_path = 'blender_envs/blend_files/backup_files/buildings_tutorial.obj'
file_2_path = 'blender_envs/blend_files/random_field.obj'
# Read and analyze both files
file_1_data = read_obj(file_1_path)
file_2_data = read_obj(file_2_path)

# Convert the data to pandas DataFrames for better readability
file_1_df = pd.DataFrame({
    'Vertices': pd.Series(file_1_data['vertices']),
    'Faces': pd.Series(file_1_data['faces']),
    'Objects': pd.Series(file_1_data['objects']),
    'Materials': pd.Series(file_1_data['materials'])
})

file_2_df = pd.DataFrame({
    'Vertices': pd.Series(file_2_data['vertices']),
    'Faces': pd.Series(file_2_data['faces']),
    'Objects': pd.Series(file_2_data['objects']),
    'Materials': pd.Series(file_2_data['materials'])
})

# Display the data
print("Buildings American Grid Data")
print(file_1_df)

print("\nRandom Field Data")
print(file_2_df)


