import bpy, bmesh
from mathutils import Vector, Matrix
import numpy as np 

def distance_vec(point1: Vector, point2: Vector) -> float: 
    """Calculate distance between two points.""" 
    return (point2 - point1).length

# получить обьект из сцены
foto_obj = bpy.context.scene.objects[0]
bpy.context.view_layer.objects.active = foto_obj

# Выбрать все крайние грани
bpy.ops.object.editmode_toggle()
me = foto_obj.data

x, y, z = [ sum( [v.co[i] for v in me.vertices] ) for i in range(3)]
count = float(len(me.vertices))
center = foto_obj.matrix_world @ (Vector( (x, y, z ) ) / count )
print (center)

bm = bmesh.new()
bm.from_mesh(me)
bm.faces.ensure_lookup_table()

dist_faces = []
faces_id = []
for i in range(len(bm.faces)):
    me.polygons[i].select = False
    face_location = bm.faces[i].calc_center_median()    
    dist  = distance_vec(center, face_location)
    print (face_location, dist)
    dist_faces.append(dist)
    faces_id.append(i)    

max_dist = max(dist_faces)   
idx = dist_faces.index(max_dist) 
print (max(dist_faces), idx)

me.edges[idx].select = True

bpy.ops.mesh.select_mode(use_extend=False, use_expand=False, type='EDGE')
bpy.ops.mesh.select_similar(type='FACE', threshold=0.01)

bpy.ops.mesh.extrude_region_move(MESH_OT_extrude_region={"use_normal_flip":False, 
                                                         "use_dissolve_ortho_edges":False, 
                                                         "mirror":False}, 
                                 TRANSFORM_OT_translate={"value":(0, 0, 3), 
                                                         "orient_axis_ortho":'X', 
                                                         "orient_type":'GLOBAL', 
                                                         "orient_matrix":((1, 0, 0), (0, 1, 0), (0, 0, 1)),
                                                         "orient_matrix_type":'GLOBAL', 
                                                         "constraint_axis":(False, False, False), 
                                                         "mirror":False, 
                                                         "use_proportional_edit":False, 
                                                         "proportional_edit_falloff":'SMOOTH', 
                                                         "proportional_size":1, 
                                                         "use_proportional_connected":False, 
                                                         "use_proportional_projected":False, 
                                                         "snap":False, 
                                                         "snap_target":'CLOSEST', 
                                                         "snap_point":(0, 0, 0), 
                                                         "snap_align":False, 
                                                         "snap_normal":(0, 0, 0), 
                                                         "gpencil_strokes":False, 
                                                         "cursor_transform":False, 
                                                         "texture_space":False, 
                                                         "remove_on_cancel":False, 
                                                         "view2d_edge_pan":False, 
                                                         "release_confirm":False, 
                                                         "use_accurate":False, 
                                                         "use_automerge_and_split":False})




