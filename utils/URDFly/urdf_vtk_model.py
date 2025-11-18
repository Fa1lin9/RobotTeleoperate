import vtk
import os

class URDFModel:
    """Class to represent a URDF link with its mesh and transformation"""

    def __init__(self, name, mesh_file, mesh_transform, link_frame, color=None, axis_text=None):
        self.name = name
        self.mesh_file = mesh_file
        self.mesh_transform = mesh_transform  # 4x4 transformation matrix
        self.link_frame = link_frame

        # Create source for geometry
        if mesh_file is None:
            # Create a sphere source if no mesh file is provided
            self.source = vtk.vtkSphereSource()
            self.source.SetRadius(0.01)  # Small sphere with radius 0.02
            self.source.SetPhiResolution(20)
            self.source.SetThetaResolution(20)
            self.source.Update()
            color = [1.0, 0, 0]
        else:
            # Get file extension to determine the appropriate reader
            _, file_extension = os.path.splitext(mesh_file)
            file_extension = file_extension.lower()
            
            if file_extension == '.obj':
                # Create the OBJ reader for mesh file
                self.source = vtk.vtkOBJReader()
                self.source.SetFileName(mesh_file)
                self.source.Update()
            else:
                # Default to STL reader for mesh file (or any other extension)
                self.source = vtk.vtkSTLReader()
                self.source.SetFileName(mesh_file)
                self.source.Update()

        # Create mapper and actor
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputConnection(self.source.GetOutputPort())

        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)

        # Set color for the actor
        if color is None:
            self.set_random_color()
        else:
            self.actor.GetProperty().SetColor(color[0], color[1], color[2])
            if len(color) > 3:
                self.actor.GetProperty().SetOpacity(color[3])

        # Store original color for highlighting/unhighlighting
        self.original_color = self.actor.GetProperty().GetColor()

        # Flag to track if model is highlighted
        self.is_highlighted = False

        # Initialize transparency (1.0 = fully opaque, 0.0 = fully transparent)
        self.transparency = 1.0

        # Apply the transformation
        self.apply_transform(self.mesh_transform)

        if link_frame is not None:

            # Create coordinate axes for this link
            self.axes_actor, self.text_actor = self.create_axes_actor(self.link_frame, axis_text=axis_text)
        
        else:
            self.axes_actor = None
            self.text_actor = None


    def set_random_color(self):
        """Set a random color for the model"""
        import random

        r = random.random()
        g = random.random()
        b = random.random()
        self.actor.GetProperty().SetColor(r, g, b)
        # Store the original color
        self.original_color = (r, g, b)

    def highlight(self):
        """Highlight the model by changing its appearance"""
        if not self.is_highlighted:
            # Store current color if not already stored
            if not hasattr(self, "original_color"):
                self.original_color = self.actor.GetProperty().GetColor()

            # Set highlight properties
            self.actor.GetProperty().SetColor(1.0, 1.0, 0.0)  # Yellow highlight
            self.actor.GetProperty().SetEdgeVisibility(True)
            self.actor.GetProperty().SetEdgeColor(1.0, 1.0, 1.0)  # White edges
            self.actor.GetProperty().SetLineWidth(2.0)
            self.actor.GetProperty().SetSpecular(0.6)
            self.actor.GetProperty().SetSpecularPower(30)
            self.is_highlighted = True

    def unhighlight(self):
        """Remove highlighting from the model"""
        if self.is_highlighted:
            # Restore original color and properties
            self.actor.GetProperty().SetColor(self.original_color)
            self.actor.GetProperty().SetEdgeVisibility(False)
            self.actor.GetProperty().SetSpecular(0.0)
            self.is_highlighted = False

    def apply_transform(self, transform_matrix):
        """Apply a 4x4 transformation matrix to the actor"""
        # Convert numpy matrix to vtk transform
        vtk_transform = vtk.vtkTransform()
        vtk_transform.SetMatrix(transform_matrix.flatten())

        # Apply the transform to the actor
        self.actor.SetUserTransform(vtk_transform)

    def set_transparency(self, transparency):
        """Set the transparency of the model (1.0 = opaque, 0.0 = transparent)"""
        self.transparency = transparency
        # Apply transparency to the actor
        self.actor.GetProperty().SetOpacity(transparency)

    def create_axes_actor(self, transform_matrix, axis_length=0.05, cylinder_radius=0.02, axis_text=None):
        """Create coordinate axes for this link with optional text at z-axis
        
        Args:
            transform_matrix: 4x4 transformation matrix
            axis_length: Length of the axes
            cylinder_radius: Radius of the axis cylinders
            axis_text: Text to display at the z-axis (None for no text)
            
        Returns:
            tuple: (axes_actor, text_actor) - text_actor may be None if no text is specified
        """
        # Create axes
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(axis_length, axis_length, axis_length)  # Set the length of the axes
        axes.SetShaftType(0)
        axes.SetAxisLabels(0)
        axes.SetCylinderRadius(cylinder_radius)

        # Create transform
        vtk_transform = vtk.vtkTransform()
        vtk_transform.SetMatrix(transform_matrix.flatten())
        axes.SetUserTransform(vtk_transform)
        
        # Create text actor if text is provided
        text_actor = None
        if axis_text is not None:
            # Create a caption actor for the text
            text_actor = vtk.vtkCaptionActor2D()
            
            # Set the text
            text_actor.SetCaption(axis_text)
            text_actor.GetTextActor().SetTextScaleModeToNone()
            text_actor.GetCaptionTextProperty().SetFontSize(16)
            text_actor.GetCaptionTextProperty().SetColor(0, 0, 1)  # Blue text
            text_actor.GetCaptionTextProperty().SetBold(False)
            
            # Position the text at the end of the z-axis
            # The attachment point is in world coordinates
            # We need to transform the z-axis endpoint using the transform matrix
            z_endpoint = [0, 0, axis_length, 1]  # Homogeneous coordinates for z-axis endpoint
            
            # Apply the transform to get the world coordinates
            transformed_point = vtk_transform.TransformPoint(z_endpoint[0], z_endpoint[1], z_endpoint[2])
            
            # Set the attachment point
            text_actor.SetAttachmentPoint(transformed_point[0], transformed_point[1], transformed_point[2])
            
            # Configure the caption
            text_actor.BorderOff()
            text_actor.LeaderOff()  # leader line
           
            text_actor.ThreeDimensionalLeaderOff()
            
            # Set the position of the text relative to the attachment point
            text_actor.SetPadding(2)
            
        return axes, text_actor
        
    def remove_axes_actor(self):
        """Remove the coordinate axes actor and text actor for this link"""
        removed = False
        
        if hasattr(self, 'axes_actor') and self.axes_actor is not None:
            # If the axes_actor is in a renderer, we need to remove it
            # This assumes the renderer is accessible or the removal is handled elsewhere
            
            # Set the axes_actor to None to indicate it's been removed
            self.axes_actor = None
            removed = True
            
        if hasattr(self, 'text_actor') and self.text_actor is not None:
            # If the text_actor is in a renderer, we need to remove it
            # This assumes the renderer is accessible or the removal is handled elsewhere
            
            # Set the text_actor to None to indicate it's been removed
            self.text_actor = None
            removed = True
            
        return removed
