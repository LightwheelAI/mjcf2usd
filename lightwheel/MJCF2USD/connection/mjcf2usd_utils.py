import os
import copy
import shutil
import numpy as np
import xml.etree.ElementTree as ET
from collections import defaultdict

from scipy.spatial.transform import Rotation as R

import omni.kit.commands
import omni.physx

from pxr import Sdf, Gf, UsdPhysics, PhysxSchema, UsdShade

class XMLHandler:
    def __init__(self, xml_path):
        """
        Initialize the XML processor.

        Args:
            xml_path (str): Path to the XML file.
        """
        self.xml_path = xml_path
        self.tree = ET.parse(xml_path)
        self.root = self.tree.getroot()
        
        self.replicate_name_list = []
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.tree = None
        self.root = None
        return False
    
    def save_xml(self):
        """
        Write the modified XML tree back to the original file path.

        Returns:
            bool: True if the write operation succeeded, False otherwise.
        """
        try:
            self.tree.write(self.xml_path)
            print("Original XML file has been updated.")
            return True
        except Exception as e:
            print(f"Failed to write XML file: {e}")
            return False
            
    def fix_site_placement(self):
        """
        Fix the placement of <site> elements in the XML file by ensuring they are located
        inside the <body name="object"> element if it exists under the same parent body.

        Returns:
            bool: True if any changes were made, False otherwise.
        """
        # Iterate over all <body> elements in the XML
        modified = False
        for parent_body in self.root.findall(".//body"):
            object_body = None
            sites_to_move = []
            
            # Search for a child <body> with name="object"
            for child in parent_body:
                if child.tag == "body" and child.get("name") == "object":
                    object_body = child
            
            # If found, collect all <site> elements directly under parent_body
            if object_body is not None:
                for child in list(parent_body):
                    if child.tag == "site":
                        sites_to_move.append(child)
                
                # Move collected <site> elements into the "object" body
                if sites_to_move:
                    for site in sites_to_move:
                        parent_body.remove(site)
                        object_body.append(site)
                    modified = True
        
        if modified:
            print(f"  Fixed misplaced <site> elements in {self.xml_path}")

        return modified
    
    def get_materials(self):
        """Get material information from the XML file"""
        # Create dict for materials and textures
        textures = {}
        materials = {}
        
        asset = self.root.find('asset')
        
        parent_dir = os.path.dirname(self.xml_path)
        for texture in asset.findall('texture'):
            textures[texture.get('name')] = {
                'file': os.path.join(parent_dir, texture.get('file')),
                'type': texture.get('type')
            }
        
        for material in asset.findall('material'):
            name = material.get('name')
            texture_name = material.get('texture', '')
            if texture_name:
                texture_file = textures.get(texture_name).get('file')
                texture_type = textures.get(texture_name).get('type')
            else:
                texture_file = ''
                texture_type = ''
                
            material_data = {
                'rgba': material.get('rgba'),
                'shininess': material.get('shininess'),
                'specular': material.get('specular'),
                'texture_file': texture_file, 
                'texture_type': texture_type
            }
            material_name = 'material_' + name
            material_name = convert_name_from_mjcf_2_usd(material_name)
            materials[material_name] = material_data

        return materials
    
    def get_geom_material_map(self):
        "Get the mapping between geometries and their associated materials."
        geom_material_dict = {}
        geom_elements = [elem for elem in self.root.iter() if elem.tag == 'geom']
        for geom in geom_elements:
            mesh = geom.get('mesh')
            material = geom.get('material')
            if mesh and material:
                material_name = 'material_'+ material
                material_name = convert_name_from_mjcf_2_usd(material_name)
                geom_material_dict[mesh] = material_name
        return geom_material_dict
    
    def get_joints(self):
        
        # Recursive function to traverse all 'body' and 'joint' nodes
        def find_joints_in_body(body, parent_name=""):
            # Get all 'joint' nodes under the current 'body'
            body_name = body.get("name", "")
            full_parent_name = f"{parent_name}_{body_name}" if parent_name else body_name
            
            for joint in body.findall("joint"):
                name = joint.get("name", "")
                if not name:
                    continue
                    
                # Extract joint attributes
                joint_data = {
                     'damping': joint.get("damping", ""), 
                     'stiffness': joint.get("stiffness", ""), 
                     'frictionloss': joint.get("frictionloss", ""), 
                    # 'type': joint.get("type", "hinge"),  
                    # 'axis': joint.get("axis", "0 0 1"), 
                    # 'pos': joint.get("pos", "0 0 0"),    
                    # 'range': joint.get("range", ""),     
                    # 'armature': joint.get("armature", ""), 
                    # 'limited': joint.get("limited", ""), 
                    # 'parent_body': full_parent_name,     
                }
                
                joints_info[name] = joint_data
            
            for child_body in body.findall("body"):
                find_joints_in_body(child_body, full_parent_name)
        
        """Get joint information from the XML file"""
        joints_info = {}
        
        worldbody = self.root.find(".//worldbody")
        if worldbody is None:
            return joints_info
                
        for body in worldbody.findall("body"):
            find_joints_in_body(body)
        
        return joints_info
    
    def get_density(self):
        class_density_dict = {}
        geom_density_dict = {}
        
        # get class_densitys_dict
        defaultbody = self.root.find(".//default")
        if defaultbody is None:
            return geom_density_dict
        
        for default in defaultbody.findall("default"):
            class_name = default.get("class")
            if not class_name:
                continue
            
            geom = default.find("geom")
            if geom is None:
                continue

            density = geom.get("density") 
            if not density:
                continue 
            
            class_density_dict[class_name] = density
        
        
        #get geom_densitys_dict
        worldbody = self.root.find(".//worldbody")
        for body in worldbody.iter('body'):
            density = self.get_body_density(body,class_density_dict)
            body_name = body.attrib.get("name")
            
            if body_name is not None and density is not None:
                geom_density_dict[body_name] = density
        
        return geom_density_dict
    
    def get_body_density(self,body,class_density_dict):
        """
        The density of all collision geoms under a body is the same, 
        and it is equal to the body's density.
        """
        for geom in body.findall("geom"):
            class_name = geom.attrib.get("class")
            if class_name and "col" in class_name:
                density = class_density_dict.get(class_name)
                if density is not None:
                    return density
        return None
            
    def preprocess_refquat_in_meshes(self):
        """
        Preprocess and correct the 'refquat' attribute from MJCF mesh definitions into corresponding geom elements.

        In MJCF files, if a <mesh> element under <asset> has a 'refquat' attribute, the current importer does not 
        handle it properly. This function collects all mesh names with a 'refquat' attribute and their values, 
        removes the attribute from the original XML, and stores them in a dictionary.

        Later, these 'refquat' values are applied to corresponding <geom> elements to correct their orientation.
        """
        mesh_refquat_dict = defaultdict(list)
        asset_elem = self.root.find(".//asset")
        for mesh in asset_elem.findall("mesh"):
            mesh_name = mesh.get("name")
            mesh_refquat = mesh.get("refquat")
            if mesh_name is not None and mesh_refquat is not None:
                mesh_name = convert_name_from_mjcf_2_usd(mesh_name)
                del mesh.attrib['refquat']
                
                mesh_refquat = list(map(float, mesh_refquat.strip().split()))
                mesh_refquat_dict[mesh_name] = mesh_refquat
        
        self.geom_add_refquat(self.root,mesh_refquat_dict)
            
    def geom_add_refquat(self,parent,mesh_refquat_dict):
        for child in list(parent):
            mesh_name = child.get("mesh")
            if child.tag == 'geom' and mesh_name is not None:
                mesh_name = convert_name_from_mjcf_2_usd(mesh_name)
                if mesh_name in mesh_refquat_dict:
                    mesh_ref_quat = mesh_refquat_dict.get(mesh_name)
                    mesh_quat = self.get_quat(child)
                    self.elem_update_with_ref_quat(child,mesh_quat,mesh_ref_quat)
                    
            self.geom_add_refquat(child,mesh_refquat_dict)

    def get_quat(self, elem, default_eulerseq="xyz"):
        #TODO support xyaxes,zaxis
        """
        Get rotation representation from XML element and convert to quaternion [w, x, y, z]
        
        Parameters:
            elem: XML element
            default_eulerseq: Default Euler angle sequence, defaults to "xyz" (x-y-z)
        
        Returns:
            Quaternion [w, x, y, z]
            
        Exceptions:
            ValueError: Raised when encountering invalid input
            
        Supported attributes (by priority):
        1. quat: [w, x, y, z]
        2. axisangle: [x, y, z, angle]
        3. euler: [e1, e2, e3] + eulerseq
        """
        # Handle angle conversion factor
        use_degree = self.is_angle_in_degrees()
        angle_factor = np.pi / 180.0 if use_degree else 1.0
        
        # 1. Handle quaternion representation
        quat_str = elem.get("quat")
        if quat_str:
            try:
                quat = list(map(float, quat_str.split()))
                if len(quat) != 4:
                    raise ValueError(f"quat requires 4 values, got {len(quat)}: {quat_str}")
                return quat
            except Exception as e:
                raise ValueError(f"Invalid quat value: {quat_str}") from e
        
        # 2. Handle axis-angle representation (x, y, z, angle)
        axisangle_str = elem.get("axisangle")
        if axisangle_str:
            try:
                coords = list(map(float, axisangle_str.split()))
                if len(coords) != 4:
                    raise ValueError(f"axisangle requires 4 values, got {len(coords)}: {axisangle_str}")
                
                x, y, z, angle_val = coords[0], coords[1], coords[2], coords[3]
                angle_rad = angle_val * angle_factor
                
                # Normalize axis vector
                axis = np.array([x, y, z])
                axis_norm = np.linalg.norm(axis)
                if axis_norm < 1e-8:
                    raise ValueError(f"axisangle rotation axis is zero vector: {axisangle_str}")
                
                axis /= axis_norm
                
                # Create rotation
                rot = R.from_rotvec(axis * angle_rad)
                
                # Convert to MuJoCo format quaternion [w, x, y, z]
                scipy_quat = rot.as_quat()
                return [scipy_quat[3], scipy_quat[0], scipy_quat[1], scipy_quat[2]]
                
            except Exception as e:
                raise ValueError(f"Invalid axisangle value: {axisangle_str}") from e

        # 3. Handle euler representation (e1, e2, e3)
        euler_str = elem.get("euler")
        if euler_str:
            try:
                coords = list(map(float, euler_str.split()))
                if len(coords) != 3:
                    raise ValueError(f"euler requires 3 values, got {len(coords)}: {euler_str}")
                
                euler_values = coords[:3]
                eulerseq = elem.get("eulerseq", default_eulerseq)

                # Create rotation
                rot = R.from_euler(eulerseq, euler_values, degrees=use_degree)
                
                # Convert to MuJoCo format quaternion [w, x, y, z]
                scipy_quat = rot.as_quat()
                return [scipy_quat[3], scipy_quat[0], scipy_quat[1], scipy_quat[2]]
                
            except Exception as e:
                raise ValueError(f"Invalid euler value: {euler_str}") from e
        
        return [1, 0, 0, 0]

    def is_angle_in_degrees(self):
        """
        Check whether the angle unit in the XML is set to degrees.

        This function examines the <compiler> tag's 'angle' attribute:
        - Returns True if angle="degree"
        - Returns False if angle="radian"
        - Returns True by default if the attribute is missing (degrees are assumed)


        Returns:
            bool: True if using degrees, False if using radians
        """
        compiler = self.root.find('compiler')
        if compiler is not None:
            angle_unit = compiler.get('angle', 'degree')
            return angle_unit.lower() == 'degree'
        return True

    def add_replicated_item_to_grandparent(self, parent, item, father_pos, father_quat, replicate_depth):
        """
        Insert a replicated XML element under its grandparent node, transforming its position and orientation based on the parent's pose.

        Args:
            parent (Element): The parent XML element of the item to be replicated.
            item (Element): The XML element to be inserted (replicated item).
            father_pos (list[float]): The position of the parent as a list of 3 floats [x, y, z].
            father_quat (scipy.spatial.transform.Rotation): The orientation of the parent as a scipy Rotation object (quaternion).
            replicate_depth (int): The current depth of replication, used to track top-level replication.

        Returns:
            Element or None: The newly inserted element under the grandparent, or None if insertion failed.
        """
        if item is None or parent is None:
            return

        pos = [float(x) for x in item.attrib.get('pos', '0 0 0').split()]
        quat = [float(x) for x in item.attrib.get('quat', '1 0 0 0').split()]

        new_quat = [quat[1], quat[2], quat[3], quat[0]]
        new_quat = R.from_quat(new_quat)

        new_quat = father_quat * new_quat

        pos_diff = np.array([pos[i] for i in range(3)])

        rotation = father_quat
        rotated_pos_local = rotation.apply(pos_diff)
        now_pos = [father_pos[i] + rotated_pos_local[i] for i in range(3)]
        
        def find_grandparent(elem, target):
            for child in elem:
                if child is target:
                    return elem
                result = find_grandparent(child, target)
                if result is not None:
                    return result
            return None

        grandparent = find_grandparent(self.root, parent)
        if grandparent is None:
            return

        item.set('pos', ' '.join(map(str, now_pos)))

        new_quat = R.as_quat(new_quat)
        new_quat = [new_quat[3], new_quat[0], new_quat[1], new_quat[2]]
        item.set('quat', ' '.join(map(str, new_quat)))
        geom_name = item.get('name', None)
        if geom_name is not None and replicate_depth == 1:
            self.replicate_name_list.append(item)

        grandparent.append(item)
        return

    def traverse_and_expand_replicates(self, element, father_element, replicate_depth = 0):
        """
        Recursively traverse the XML tree, expanding <replicate> tags by duplicating their child elements
        according to the specified count, position offset, and rotation. The replicated elements are inserted
        into their grandparent node with updated position and orientation. The original <replicate> tag and its
        children are removed from the tree after expansion.

        Args:
            element (Element): The current XML element being traversed.
            father_element (Element): The parent XML element of the current element.
            replicate_depth (int, optional): The current depth of nested <replicate> tags. Defaults to 0.
        """
        if element is None:
            return

        for item in list(element):  
            self.traverse_and_expand_replicates(item, element, replicate_depth + (1 if element.tag == 'replicate' else 0))

        if element.tag == "replicate":
            count = int(element.attrib.get('count', 1))
            offset = [float(x) for x in element.attrib.get('offset', '0 0 0').split()]
            euler = [float(x) for x in element.attrib.get('euler', '0 0 0').split()]
            is_degree = self.is_angle_in_degrees()
            rot = R.from_euler('xyz', euler, degrees=is_degree)
            new_quat = R.from_euler('xyz', [0, 0, 0], degrees=is_degree)

            for item in list(element):
                if item.tag == 'replicate':
                    continue
                for i in range(count):
                    new_geom = copy.deepcopy(item)

                    new_pos = [
                        offset[0] * i,
                        offset[1] * i,
                        offset[2] * i
                    ]

                    if i > 0:
                        new_quat = rot * new_quat

                    self.add_replicated_item_to_grandparent(element, new_geom, new_pos, new_quat, replicate_depth + 1)
                if item in element:
                    element.remove(item)

            father_element.remove(element)
        return

    def expand_replicates_fields(self):
        """Recursively expand and remove <replicate> tags."""
        
        worldbody = self.root.find(".//worldbody")
        if worldbody is not None:
            for element in worldbody:
                self.traverse_and_expand_replicates(element, worldbody)

        dict_name = {}
        for geom in self.replicate_name_list:
            geom_name = geom.get('name')
            if geom_name not in dict_name:  
                dict_name[geom_name] = 0
            
            geom.set('name', geom_name + '_' + str(dict_name[geom_name] ))
            dict_name[geom_name] += 1
    
    def elem_update_with_ref_quat(self,elem,mesh_quat,mesh_ref_quat):
        if mesh_ref_quat and mesh_quat:
            
            mesh_ref_quat_R = convert_quat_mjcf_2_scipy(mesh_ref_quat)
            mesh_quat_R = convert_quat_mjcf_2_scipy(mesh_quat)
            
            r1 = R.from_quat(mesh_ref_quat_R)
            r2 = R.from_quat(mesh_quat_R)
            r1_inv = r1.inv()
            
            q_list = (r2 * r1_inv).as_quat().tolist()
            quat_mjcf = convert_quat_scipy_2_mjcf(q_list)
            elem.set("quat", ' '.join(map(str, quat_mjcf)))
            
        elif mesh_ref_quat and not mesh_quat:

            mesh_ref_quat_R = convert_quat_mjcf_2_scipy(mesh_ref_quat)
            
            r = R.from_quat(mesh_ref_quat_R)
            r_inv = r.inv()
            
            q_list = r_inv.as_quat().tolist()
            quat_mjcf = convert_quat_scipy_2_mjcf(q_list)
            elem.set("quat", ' '.join(map(str, quat_mjcf)))

def mjcf_to_usd(mjcf_path, usd_path='',need_save_tmp_xml=False):
    if usd_path == '':
        usd_path = mjcf_path[:-4] + ".usd"
        
    tmp_mjcf_path = mjcf_path[:-4] + "_tmp.xml"
    shutil.copy2(mjcf_path, tmp_mjcf_path)
    
    with XMLHandler(tmp_mjcf_path) as xml_handler:
        """preprocess mjcf xml"""
        xml_handler.preprocess_refquat_in_meshes()
        xml_handler.expand_replicates_fields()
        xml_handler.fix_site_placement()
        xml_handler.save_xml()
        
        """Create a new empty USD stage"""
        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()

        "Setting up import configuration"
        status, import_config = omni.kit.commands.execute("MJCFCreateImportConfig")
        import_config.set_fix_base(False)
        import_config.set_make_default_prim(False)
        omni.kit.commands.execute(
            "MJCFCreateAsset",
            mjcf_path=xml_handler.xml_path,
            import_config=import_config,
            prim_path="/root"
        )
         
        "Fix joints"
        joints_info = xml_handler.get_joints()
        fix_joints(stage, joints_info)
        
        "Resolve mesh name conflicts caused by native import"   
        fix_repeat_mesh_name()
        
        "Copy texture assets to the output directory."
        materials = xml_handler.get_materials()
        transfer_texture(usd_path, materials)
        
        "Recreate materials"
        clear_default_error_material(stage)
        geom_meterial_map = xml_handler.get_geom_material_map()
        create_material_to_mesh(stage, materials, geom_meterial_map)
        
        "Fix density"
        density_dict = xml_handler.get_density()
        fix_physics(stage, density_dict)
            
        # # converted by LW rule(Get directory as model name)
        parent_dir = os.path.dirname(mjcf_path)

        # save stage
        save_result = context.save_as_stage(usd_path)
        if not save_result:
            context.close_stage()
            return 
        context.close_stage()
        
        clear_temp_usd(parent_dir)
        if not need_save_tmp_xml:
            os.remove(tmp_mjcf_path)
        return
       
def get_all_sites(prim, sites=[]):
    """Recursively get all sites under the given prim"""
    for child in prim.GetChildren():
        if "sites" in child.GetName():
            for site in child.GetChildren():
                sites.append(site)
        get_all_sites(child, sites)

def fix_physics(stage,density_dict):
    # add physics scence
    scene = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
    scene.CreateBroadphaseTypeAttr().Set("MBP")
    scene.CreateSolverTypeAttr().Set("TGS")

    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and str(prim.GetName()) in density_dict:
            density = float(density_dict.get(prim.GetName()))
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateDensityAttr().Set(density)
                 
def clear_default_error_material(stage):
    """ clear materials under Looks scope """
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        return

    looks_path = default_prim.GetPath().AppendChild("Looks")
    looks_prim = stage.GetPrimAtPath(looks_path)
    for material in looks_prim.GetChildren():
        stage.RemovePrim(material.GetPath())

def create_material_to_mesh(stage,materials,geom_meterial_map):
    
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        return

    #creat material
    material_path_map = {}
    looks_path = default_prim.GetPath().AppendChild("Looks")
    for key in  materials:
        material_name = str(key)
        material_path = looks_path.AppendChild(material_name)
        
        result = omni.kit.commands.execute('CreateAndBindMdlMaterialFromLibrary',
            mdl_name='OmniPBR.mdl',
            mtl_name='OmniPBR',
            mtl_created_list=[material_path],
            bind_selected_prims=['/model/Looks'],
            prim_name=material_name)
        if result[0]:
            material_path_map.setdefault(material_name,material_path)
        
    
    for new_material_path in material_path_map.values():
        material_prim = stage.GetPrimAtPath(new_material_path)
        if not material_prim.IsA(UsdShade.Material):
            continue
        
        preview_surface = UsdShade.Shader(material_prim.GetPrimAtPath(
            material_prim.GetPath().AppendChild("Shader")))
        
        if not preview_surface:
            continue
        
        material_name = material_prim.GetName()
        if material_name not in materials:
            continue
        material_data = materials[material_name]

        # Set rgba
        if material_data.get('rgba'):
            rgba = [float(x) for x in material_data['rgba'].split(' ')]
            if len(rgba) > 3:
                preview_surface.CreateInput('diffuse_color_constant', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(rgba[0], rgba[1], rgba[2]))
            # if len(rgba) == 4:
            #     preview_surface.CreateInput('opacity_constant', Sdf.ValueTypeNames.Float).Set(rgba[3])

        # Set shininess)
        # if material_data.get('shininess'):
        #     shininess = float(material_data['shininess'])
        #     preview_surface.CreateInput('reflection_roughness_constant', Sdf.ValueTypeNames.Float).Set(1.0 - shininess)

        # Set specular)
        if material_data.get('specular'):
            specular = float(material_data['specular'])
            preview_surface.CreateInput('specular_level', Sdf.ValueTypeNames.Float).Set(specular)

        # Set texture_file
        if material_data.get('texture_file'):
            preview_surface.CreateInput('diffuse_texture', Sdf.ValueTypeNames.Asset).Set(material_data['texture_file'])
            
    apply_material_to_geom(material_path_map,geom_meterial_map)

def apply_material_to_geom(material_path_map,geom_meterial_map):
    """Apply materials to geometry prims in the USD stage."""
    stage = omni.usd.get_context().get_stage()
        
    visuals_prim = stage.GetPrimAtPath("/visuals")            
    for references_prim in visuals_prim.GetChildren():
        
        for ref in references_prim.GetChildren():
            ref_name = ref.GetPath()
            geom_name = str(ref_name).split("/")[-1]
            material = geom_meterial_map.get(geom_name)
            material_path = material_path_map.get(material)
            
            if material_path:
                omni.kit.commands.execute('BindMaterial',
                    material_path=material_path,
                    prim_path=[ref_name],
                    strength=['weakerThanDescendants'],
                    material_purpose='')
            
def fix_repeat_mesh_name():
    # Fix the issue of duplicate 'mesh' names created by omni.MJCFCreateAsset
    stage = omni.usd.get_context().get_stage()
    meshes_prim = stage.GetPrimAtPath("/meshes")
    if meshes_prim is not None:
        xformes = meshes_prim.GetChildren()
        for xform in xformes:
            meshes = xform.GetChildren()
            for mesh in meshes:
                if mesh is not None:
                    name = mesh.GetName()
                    if str(name) == "mesh":
                        oldPrimPath = mesh.GetPath()
                        tmpPath = str(oldPrimPath).rsplit('/',1)[0]
                        newPrimPath = tmpPath +"/"+xform.GetName()
                        
                        omni.kit.commands.execute('MovePrim',
                            path_from=Sdf.Path(oldPrimPath),
                            path_to=Sdf.Path(newPrimPath),
                            destructive=False,
                            stage_or_context=omni.usd.get_context().get_stage())

def transfer_texture(usd_path,materials):
    destination_dir = os.path.dirname(usd_path)
    destination_dir = os.path.join(destination_dir,"texture","")

    if not os.path.exists(destination_dir):
        os.makedirs(destination_dir,exist_ok=True)
    
    for material_data in materials.values():
        texture_file = str(material_data['texture_file'])
        if texture_file:
            result = shutil.copy(texture_file,destination_dir)
            if os.path.exists(result):
                material_data['texture_file'] = result
      
def fix_joints(stage, joints_info):

    def get_all_joints(prim, joints=[]):
        """Recursively get all joints under the given prim"""
        for child in prim.GetChildren():
            if "Joint" in child.GetTypeName():
                joints.append(child)
            get_all_joints(child, joints)
    
    default_prim = stage.GetDefaultPrim()
    joints_prim = default_prim.GetChild('joints')
    all_joints = []
    get_all_joints(joints_prim, all_joints)
    
    for joint in all_joints:
        joint_data = joints_info.get(joint.GetName())
        if joint_data:
            set_joint_properties(joint,joint_data)
            
def if_joints_linked(stage,joint_path,target_path)->bool:
    """
    Check whether a joint is connected to a specific target prim.

    Args:
        stage: The USD stage.
        joint_path: The Sdf.Path to the joint prim.
        target_path: The Sdf.Path to the target prim (e.g., a rigid body).

    Returns:
        True if the target prim is linked to the joint via body0 or body1;
        False otherwise.
    """
    joint = UsdPhysics.Joint.Get(stage,joint_path)
    Rel_path_0 = joint.GetBody0Rel().GetTargets()
    Rel_path_1 = joint.GetBody1Rel().GetTargets()
    
    if target_path in Rel_path_0 or target_path in Rel_path_1:
        return True
    else:
        return False
            
def set_joint_properties(joint,joint_data):

    usd_joint_dict = {
        "PhysicsPrismaticJoint": {
            "frictionloss": {"physxJoint:jointFriction"},
            "damping":      {"physxLimit:linear:damping"},
            "stiffness":    {"physxLimit:linear:stiffness"}
        },
        "PhysicsRevoluteJoint": {
            "frictionloss": {"physxJoint:jointFriction"},
            "damping":      {"physxLimit:angular:damping"},
            "stiffness":    {"physxLimit:angular:stiffness"}
        },
        "PhysicsSphericalJoint": {
            "frictionloss": {"physxJoint:jointFriction"},
            "damping": {
                "physxLimit:transX:damping",
                "physxLimit:transY:damping",
                "physxLimit:transZ:damping"
            },
            "stiffness": {
                "physxLimit:transX:stiffness",
                "physxLimit:transY:stiffness",
                "physxLimit:transZ:stiffness"
            }
        }
    }
    
    joint_type = joint.GetTypeName()
    if joint_type not in usd_joint_dict:
        print(f"set unsupported joint properties {joint_type}")
        return
    
    properties = usd_joint_dict[joint_type]
    for prop_name, usd_attrs in properties.items():
        value = joint_data.get(prop_name)
        if not value:
            continue
        for attr in usd_attrs:
            if joint.HasAttribute(attr):
                joint.GetAttribute(attr).Set(float(value))
            else:
                joint.CreateAttribute(attr, Sdf.ValueTypeNames.Float).Set(float(value))
        
def clear_temp_usd(root_dir):
    for dirpath, dirnames, filenames in os.walk(root_dir):
        for filename in filenames:
            file_path = os.path.join(dirpath, filename)
            name1, ext1 = os.path.splitext(filename)
            name2, ext2 = os.path.splitext(name1)
            if ext2 == '.tmp' and ext1 == '.usd':
                os.remove(file_path)

@staticmethod
def convert_name_from_mjcf_2_usd(mjcf_name):
    return mjcf_name.replace('.', '_').replace('-', '_')

@staticmethod
def convert_quat_mjcf_2_scipy(quat_mjcf):
    """Convert quaternion from MJCF (w, x, y, z) to SciPy (x, y, z, w) format."""
    quat_scipy_list = quat_mjcf[1:] + [quat_mjcf[0]]
    return quat_scipy_list

@staticmethod
def convert_quat_scipy_2_mjcf(quat_scipy):
    """Convert quaternion from  SciPy (x, y, z, w) to MJCF (w, x, y, z)format."""
    quat_mjcf = [quat_scipy[3]] + list(quat_scipy[:3])
    return quat_mjcf

def get_xmls(xml_dir):
    """
    Get all XML files
    Args:
        xml_dir: Path to the folder containing XML files
    Returns:
        xmls: List of XML file paths
    """
    xmls = []
    if os.path.isdir(xml_dir):
        for root, dirs, files in os.walk(xml_dir):
            for file in files:
                if file.endswith(".xml"):
                    xmls.append(os.path.join(root, file))
    elif xml_dir.endswith(".xml"):
        xmls.append(xml_dir)
    return xmls     


if __name__ == "__main__":

    xml_path = "/home/lightwheel/Documents/cf.s/mjcf/standard/model.xml"
    mjcf_to_usd(xml_path,'',False)