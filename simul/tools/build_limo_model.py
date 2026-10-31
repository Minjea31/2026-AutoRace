#!/usr/bin/env python3
"""Convert the existing sensor_ws LIMO Collada meshes to compact Panda3D assets.

No extra dependencies: numpy and Panda3D from environment.yml are sufficient.
The source meshes are already in the user's repository. Preserve their material
colours, simplify small details, and normalize to the configured LIMO Pro
envelope. Runtime does not need Collada support or the large original meshes.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from panda3d.core import Geom, GeomNode, GeomTriangles, GeomVertexData, GeomVertexFormat, NodePath

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT/'sensor_ws/src/limo_ros2/limo_description/meshes'
OUT = ROOT/'simul/src/limo_parking_sim/assets'
NS = {'c':'http://www.collada.org/2005/11/COLLADASchema'}


def convert(name, voxel):
    source = SOURCE/(name+'.dae')
    root = ET.parse(source).getroot()
    effects = {e.get('id'):tuple(map(float,e.find('.//c:diffuse/c:color',NS).text.split()))
               for e in root.findall('c:library_effects/c:effect',NS)}
    materials = {m.get('id'):effects[m.find('c:instance_effect',NS).get('url')[1:]]
                 for m in root.findall('c:library_materials/c:material',NS)}
    bindings = {m.get('symbol'):materials[m.get('target')[1:]]
                for m in root.findall('.//c:instance_material',NS)}
    triangles, colours = [],[]
    for geometry in root.findall('c:library_geometries/c:geometry',NS):
        mesh = geometry.find('c:mesh',NS)
        arrays = {s.get('id'):np.fromstring(s.find('c:float_array',NS).text,sep=' ',dtype=np.float32).reshape(-1,3)
                  for s in mesh.findall('c:source',NS)}
        vertices = {v.get('id'):v.find('c:input',NS).get('source')[1:] for v in mesh.findall('c:vertices',NS)}
        for tri in mesh.findall('c:triangles',NS):
            inputs = tri.findall('c:input',NS)
            stride = max(int(i.get('offset')) for i in inputs)+1
            vertex = next(i for i in inputs if i.get('semantic')=='VERTEX')
            indices = np.fromstring(tri.find('c:p',NS).text,sep=' ',dtype=np.int32).reshape(-1,stride)
            points = arrays[vertices[vertex.get('source')[1:]]][indices[:,int(vertex.get('offset'))]].reshape(-1,3,3)
            triangles.append(points)
            colours.extend([bindings[tri.get('material')]]*len(points))
    points = np.concatenate(triangles)
    colours = np.asarray(colours,dtype=np.float32)
    input_triangles = len(points)
    if name=='limo_base':
        # Same +90-degree Z rotation as the local Ackermann Xacro: front is +X.
        points = points @ np.array([[0,1,0],[-1,0,0],[0,0,1]],np.float32)
        lo,hi = points.min(axis=(0,1)),points.max(axis=(0,1))
        points[:,:,0] = (points[:,:,0]-(lo[0]+hi[0])/2)*(.322/(hi[0]-lo[0]))
        points[:,:,1] -= (lo[1]+hi[1])/2
        points[:,:,2] = .024+(points[:,:,2]-lo[2])*(.227/(hi[2]-lo[2]))
    else:
        lo,hi = points.min(axis=(0,1)),points.max(axis=(0,1))
        points[:,:,1] = (points[:,:,1]-(lo[1]+hi[1])/2)*(.045/(hi[1]-lo[1]))
        points[:,:,[0,2]] *= .045/max(abs(lo[0]),abs(hi[0]),abs(lo[2]),abs(hi[2]))
    # Vertex clustering removes tiny fasteners/details while keeping all large
    # panels, screens, sensor shapes and tyre treads from the source model.
    flat = points.reshape(-1,3)
    keys = np.floor(flat/voxel).astype(np.int32)
    _,inverse = np.unique(keys,axis=0,return_inverse=True)
    counts = np.bincount(inverse)
    centres = np.stack([np.bincount(inverse,weights=flat[:,i])/counts for i in range(3)],axis=1).astype(np.float32)
    faces = inverse.reshape(-1,3)
    distinct = (faces[:,0]!=faces[:,1]) & (faces[:,1]!=faces[:,2]) & (faces[:,0]!=faces[:,2])
    faces,colours = faces[distinct],colours[distinct]
    face_keys = np.column_stack((np.sort(faces,axis=1),np.rint(colours*255).astype(np.int32)))
    _,keep = np.unique(face_keys,axis=0,return_index=True)
    faces,colours = faces[keep],colours[keep]
    points = centres[faces]
    normals = np.cross(points[:,1]-points[:,0],points[:,2]-points[:,0])
    area = np.linalg.norm(normals,axis=1)
    good = area>1e-10
    points,colours,normals,area = points[good],colours[good],normals[good],area[good]
    normals /= area[:,None]
    packed = np.empty(len(points)*3,dtype=[('vertex','<f4',(3,)),('normal','<f4',(3,)),('colour','u1',(4,))])
    packed['vertex'] = points.reshape(-1,3)
    packed['normal'] = np.repeat(normals,3,axis=0)
    packed['colour'] = np.repeat(np.rint(colours*255).astype(np.uint8),3,axis=0)
    data = GeomVertexData(name,GeomVertexFormat.getV3n3c4(),Geom.UHStatic)
    data.modifyArray(0).modifyHandle().setData(packed.tobytes())
    primitive = GeomTriangles(Geom.UHStatic)
    primitive.setIndexType(Geom.NTUint32)
    primitive.modifyVertices().modifyHandle().setData(np.arange(len(packed),dtype=np.uint32).tobytes())
    geom = Geom(data)
    geom.addPrimitive(primitive)
    node = GeomNode(name)
    node.addGeom(geom)
    model = NodePath(node)
    model.setTwoSided(True)
    filename = 'limo_pro_body.bam' if name=='limo_base' else 'limo_pro_wheel.bam'
    assert model.writeBamFile(str(OUT/filename))
    result = {'source':str(source.relative_to(ROOT)),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
              'input_triangles':input_triangles,'output_triangles':len(points),'voxel_m':voxel,
              'bounds_m':[points.min(axis=(0,1)).tolist(),points.max(axis=(0,1)).tolist()]}
    return filename,result


if __name__=='__main__':
    OUT.mkdir(parents=True,exist_ok=True)
    info = dict([convert('limo_base',.002),convert('limo_wheel',.002)])
    (OUT/'limo_pro_model.json').write_text(json.dumps(info,indent=2)+'\n')
    print(json.dumps(info,indent=2))
