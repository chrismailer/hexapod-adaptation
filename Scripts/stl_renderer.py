# used to render stl meshes for report
import open3d as o3d
from math import radians

mesh = o3d.io.read_triangle_mesh("./urdf/meshes/simplified/tibia_mesh.stl")
mesh.compute_vertex_normals()

frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

base = mesh.get_rotation_matrix_from_xyz((-radians(70), -radians(0), -radians(90)))
coxa = mesh.get_rotation_matrix_from_xyz((-radians(70), -radians(0), -radians(20)))
femur = mesh.get_rotation_matrix_from_xyz((radians(20), -radians(200), -radians(0)))
tibia = mesh.get_rotation_matrix_from_xyz((radians(-30), radians(-25), radians(-20)))

mesh.rotate(tibia, center=(0,0,0))
# mesh_frame.rotate(femur, center=(0,0,0))
# o3d.visualization.change_field_of_view(0.3)
o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=False)
