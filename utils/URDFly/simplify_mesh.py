import trimesh
import os
import numpy as np

def create_detailed_approximation(mesh_files, maxConvexHulls=64):
    """
    Create a detailed approximation of multiple mesh files using convex decomposition.
    
    Args:
        mesh_files (list): List of paths to STL mesh files to approximate
        detail_level (int, optional): Level of detail for the approximation (higher = more detailed)
                                     This controls the maximum number of convex hulls in the decomposition.
    
    Returns:
        list: List of paths to the saved approximated mesh files
    """
    approximated_files = []
    
    for mesh_file in mesh_files:
        # 1. Load the STL mesh
        mesh = trimesh.load(mesh_file)
        
        # 2. Print original mesh information
        print(f"Original mesh '{os.path.basename(mesh_file)}' has {mesh.faces.shape[0]} triangular faces")
        
        try:
            # 3. Create a detailed approximation using convex decomposition (V-HACD algorithm)
            # This breaks the mesh into multiple convex pieces for better detail preservation
            convex_pieces = mesh.convex_decomposition(
                maxConvexHulls=maxConvexHulls,         # Maximum number of convex hulls to generate

            )
            
            # If convex_pieces is a list, combine them into a single mesh
            if isinstance(convex_pieces, list):
                # Combine all pieces into a single mesh
                vertices = []
                faces = []
                face_offset = 0
                
                for piece in convex_pieces:
                    vertices.append(piece.vertices)
                    # Offset the face indices to account for the combined vertices
                    faces.append(piece.faces + face_offset)
                    face_offset += len(piece.vertices)
                
                # Create a new mesh from the combined pieces
                if vertices and faces:
                    vertices = np.vstack(vertices)
                    faces = np.vstack(faces)
                    approximated_mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
                else:
                    # Fallback to convex hull if decomposition fails to produce valid pieces
                    print("Warning: Convex decomposition produced no valid pieces. Falling back to convex hull.")
                    approximated_mesh = mesh.convex_hull
            else:
                # If it's already a single mesh, use it directly
                approximated_mesh = convex_pieces
                
        except Exception as e:
            # Fallback to convex hull if decomposition fails
            print(f"Warning: Convex decomposition failed with error: {e}. Falling back to convex hull.")
            approximated_mesh = mesh.convex_hull
        
        # 4. Print approximated mesh information
        print(f"Approximated mesh has {approximated_mesh.faces.shape[0]} triangular faces")
        
        # 5. Generate output filename by adding "_approx" before the extension
        file_path, file_ext = os.path.splitext(mesh_file)
        output_file = f"{file_path}_approx{file_ext}"
        
        # 6. Export the approximated mesh
        approximated_mesh.export(output_file)
        print(f"Saved approximated mesh to '{output_file}'")
        
        # 7. Add the output file path to the result list
        approximated_files.append(output_file)
    
    return approximated_files


# Example usage:
if __name__ == "__main__":
    # Example with a single file
    mesh_files = ['original_model.stl']
    
    # For more detail, increase the detail_level parameter (default is 8)
    # Higher values create more detailed approximations but take longer to compute
    approximated_files = create_detailed_approximation(mesh_files, detail_level=12)
    print(f"Approximated mesh files: {approximated_files}")
    
    # Example with multiple files
    # mesh_files = ['model1.stl', 'model2.stl', 'model3.stl']
    # approximated_files = create_detailed_approximation(mesh_files)
    # print(f"Approximated mesh files: {approximated_files}")
    
    # Example with different detail levels
    # Low detail (faster)
    # approximated_files = create_detailed_approximation(mesh_files, detail_level=4)
    # Medium detail
    # approximated_files = create_detailed_approximation(mesh_files, detail_level=8)
    # High detail (slower)
    # approximated_files = create_detailed_approximation(mesh_files, detail_level=16)
