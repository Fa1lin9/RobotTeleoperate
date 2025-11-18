import numpy as np
import math
import os
from anytree import Node, RenderTree
import transformations as tf



class URDFParser:
    """Class to parse URDF files and extract link and joint information"""

    def __init__(self, urdf_file):
        self.urdf_file = urdf_file
        self.mesh_dir = os.path.dirname(urdf_file)
        self.links = {}
        self.joints = []
        self.parse_urdf()

    def parse_urdf(self):
        """Parse the URDF file to extract links and joints"""
        from xml.etree import ElementTree as ET

        urdf_data = open(self.urdf_file, "r").read()
        root = ET.fromstring(urdf_data)

        # Extract all links
        for link in root.findall("link"):
            visual = link.find("visual")
            if visual is not None:
                geometry = visual.find("geometry")
                if geometry is not None:
                    mesh = geometry.find("mesh")
                    if mesh is not None:
                        filename = mesh.get("filename")
                        # Store the mesh filename and its visual properties
                        self.links[link.get("name")] = {
                            "mesh": filename,
                            "origin": self.parse_origin(visual.find("origin")),
                            "color": self.parse_color(visual.find("material")),
                        }
                    else:
                        self.links[link.get("name")] = {
                            "mesh": None,
                        }
                else:
                    self.links[link.get("name")] = {
                            "mesh": None,
                        }
            else:
                self.links[link.get("name")] = {
                            "mesh": None,
                        }

            collision = link.find("collision")
            if collision is not None:
                geometry = collision.find("geometry")
                if geometry is not None:
                    mesh = geometry.find("mesh")
                    if link.get('name') in self.links:       
                        if mesh is not None:
                            filename = mesh.get("filename")
                            # Store the mesh filename and its visual properties
                            self.links[link.get("name")]['collision_mesh'] = filename
                            self.links[link.get("name")]['collision_origin'] = self.parse_origin(collision.find("origin"))

                        else:
                            self.links[link.get("name")]['collision_mesh'] = None
                            self.links[link.get("name")]['collision_origin'] = None
                else:
                    if link.get('name') in self.links:    
                        self.links[link.get("name")]['collision_mesh'] = None
                        self.links[link.get("name")]['collision_origin'] = None


        # Extract all joints
        for joint in root.findall("joint"):
            joint_data = {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "origin": self.parse_origin(joint.find("origin")),
                "axis": self.parse_axis(joint.find("axis")),
            }
            self.joints.append(joint_data)
            
        self.roots = self.build_multiple_trees()

    def parse_origin(self, origin_element):
        """Parse origin element to get xyz and rpy"""
        if origin_element is None:
            return {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}

        xyz = list(map(float, origin_element.get("xyz", "0 0 0").split()))
        rpy = list(map(float, origin_element.get("rpy", "0 0 0").split()))
        return {"xyz": xyz, "rpy": rpy}

    def parse_color(self, material_element):
        """Parse color from material element"""
        if material_element is None:
            return [0.5, 0.5, 0.5, 1.0]  # Default gray color

        color = material_element.find("color")
        if color is not None:
            return list(map(float, color.get("rgba", "0.5 0.5 0.5 1.0").split()))
        return [0.5, 0.5, 0.5, 1.0]

    def parse_axis(self, axis_element):
        """Parse axis element"""
        if axis_element is None:
            return [0, 0, 1]
        return list(map(float, axis_element.get("xyz", "0 0 1").split()))

    def compute_transformation(self, rpy, xyz):
        """Compute 4x4 transformation matrix from RPY and XYZ"""
        roll, pitch, yaw = rpy

        # Rotation matrices
        Rx = np.array(
            [
                [1, 0, 0],
                [0, math.cos(roll), -math.sin(roll)],
                [0, math.sin(roll), math.cos(roll)],
            ]
        )

        Ry = np.array(
            [
                [math.cos(pitch), 0, math.sin(pitch)],
                [0, 1, 0],
                [-math.sin(pitch), 0, math.cos(pitch)],
            ]
        )

        Rz = np.array(
            [
                [math.cos(yaw), -math.sin(yaw), 0],
                [math.sin(yaw), math.cos(yaw), 0],
                [0, 0, 1],
            ]
        )

        # Combined rotation
        R = Rz @ Ry @ Rx

        # Transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz

        return T

    def forward_kinematics(self, qs=None):
        """Compute the transformations for all links based on the tree structure"""
        
        # Identify all chains in the URDF
        chains, trees = self.identify_chains()
        
        if len(trees) > 1:
            print('Warning: Multiple trees are found, all will be processed')
        
        # Initialize transformations dictionary
        transformations = {}
        
        # Process each tree
        for tree in trees:
            # Root link has identity transformation
            transformations[tree.name] = np.eye(4)
            
            # Find all chains in this tree
            tree_chains = [chain for chain in chains if chain["root"] == tree]
            
            # Process each chain
            for chain in tree_chains:
                # Get the nodes in the chain (alternating links and joints)
                nodes = chain["nodes"]
                
                # Traverse the chain from root to leaf
                for i in range(1, len(nodes)):
                    current_node = nodes[i]
                    parent_node = nodes[i-1]
                    
                    if current_node.node_type == "joint":
                        # Current node is a joint, parent is a link
                        joint_name = current_node.name
                        parent_link = parent_node.name
                        
                        # Find the joint data
                        joint_data = next((j for j in self.joints if j["name"] == joint_name), None)
                        
                        if joint_data:
                            # Get parent transformation
                            if parent_link in transformations:
                                T_parent = transformations[parent_link]
                            else:
                                print(f"Warning: Parent link '{parent_link}' not found in transformations")
                                T_parent = np.eye(4)
                            
                            # Compute joint transformation
                            T_joint = self.compute_transformation(
                                joint_data["origin"]["rpy"], joint_data["origin"]["xyz"]
                            )
                            
                            # If joint angles are provided and this is a revolute joint
                            if qs is not None and joint_data["type"] == "revolute":
                                # Find the index of this joint in the revolute joints list
                                rev_joints = [j for j in self.joints if j['type'] == 'revolute']
                                try:
                                    joint_index = rev_joints.index(joint_data)
                                    if joint_index < len(qs):
                                        q = qs[joint_index]
                                        # Apply rotation based on joint angle and axis
                                        joint_axis = joint_data['axis']
                                        T_update = tf.rotation_matrix(q, joint_axis)
                                        # Store joint transformation (parent * joint * rotation)
                                        transformations[joint_name] = T_parent @ T_joint @ T_update
                                    else:
                                        # Not enough joint angles provided
                                        transformations[joint_name] = T_parent @ T_joint
                                except ValueError:
                                    # Joint not found in revolute joints list
                                    transformations[joint_name] = T_parent @ T_joint
                            else:
                                # Non-revolute joint or no joint angles provided
                                transformations[joint_name] = T_parent @ T_joint
                    
                    elif current_node.node_type == "link":
                        # Current node is a link, parent is a joint
                        link_name = current_node.name
                        parent_joint = parent_node.name
                        
                        # Get parent transformation
                        if parent_joint in transformations:
                            T_parent = transformations[parent_joint]
                        else:
                            print(f"Warning: Parent joint '{parent_joint}' not found in transformations")
                            T_parent = np.eye(4)
                        
                        # Link transformation is the same as its parent joint
                        transformations[link_name] = T_parent
        
        return transformations

    def get_robot_info(self, qs=None):
        """Get information about the robot links and their transformations"""
        transformations = self.forward_kinematics(qs)

        link_names = []
        link_mesh_files = []
        link_mesh_transformations = []
        link_frames = []
        link_colors = []
        
        collision_mesh_files = []
        collision_mesh_transformations = []
        
        # For joint frames
        joint_names = []
        joint_frames = []
        joint_types = []
        joint_axes = []
        joint_parent_links = []
        joint_child_links = []

        # Process each link
        for name, T in transformations.items():
            link_info = self.links.get(name)
            if link_info is None:
                continue
            if link_info.get("mesh") is not None:
                # Get mesh file path
                mesh_file = link_info["mesh"]
                if not os.path.isabs(mesh_file):
                    mesh_file = os.path.join(self.mesh_dir, mesh_file)
                mesh_file = os.path.normpath(mesh_file)

                # Apply the link's visual origin transformation
                T_visual = self.compute_transformation(
                    link_info["origin"]["rpy"], link_info["origin"]["xyz"]
                )

                # Combine with the link's transformation
                T_total = T @ T_visual
                color = link_info["color"]
    
            else: # 虚拟Link
                mesh_file = None
                T_total = T
                color = None
                
            link_names.append(name)
            link_mesh_files.append(mesh_file)
            link_mesh_transformations.append(T_total)
            link_frames.append(T)
            link_colors.append(color)
                
            # Process collision
            if link_info.get("collision_mesh") is not None:
                mesh_file = link_info["collision_mesh"]
                if not os.path.isabs(mesh_file):
                    mesh_file = os.path.join(self.mesh_dir, mesh_file)
                mesh_file = os.path.normpath(mesh_file)
                
                T_collision = self.compute_transformation(
                    link_info["collision_origin"]["rpy"], link_info["collision_origin"]["xyz"]
                )

                # Combine with the link's transformation
                T_total = T @ T_collision
                collision_mesh_files.append(mesh_file)
                collision_mesh_transformations.append(T_total)

            
        # Process each joint
        for joint in self.joints:
            parent = joint["parent"]
            child = joint["child"]
            
            # Get parent transformation
            if parent in transformations:
                T_parent = transformations[parent]
            else:
                print(f"Warning: Parent link '{parent}' not found in transformations")
                T_parent = np.eye(4)
                
            # Compute joint transformation
            T_joint = self.compute_transformation(
                joint["origin"]["rpy"], joint["origin"]["xyz"]
            )
            
            # Joint frame is parent * joint
            joint_frame = T_parent @ T_joint
            
            joint_names.append(joint["name"])
            joint_frames.append(joint_frame)
            joint_types.append(joint["type"])
            joint_axes.append(joint["axis"])
            joint_parent_links.append(parent)
            joint_child_links.append(child)

        return (
            link_names,
            link_mesh_files,
            link_mesh_transformations,
            link_frames,
            link_colors,
            joint_names,
            joint_frames,
            joint_types,
            joint_axes,
            joint_parent_links,
            joint_child_links,
            collision_mesh_files,
            collision_mesh_transformations,

        )
    
    def build_multiple_trees(self):
        """Build multiple kinematic trees if the URDF contains disconnected structures"""
        # Create nodes for all links and joints first
        self.nodes = {}
        
        # Create all link nodes
        for link_name in self.links.keys():
            self.nodes[link_name] = Node(link_name, node_type="link")
            
        # Create joint nodes and establish parent-child relationships
        for joint in self.joints:
            parent_link = joint["parent"]
            child_link = joint["child"]
            joint_name = joint["name"]
            
            # Create joint node with parent being the parent link
            joint_node = Node(
                joint_name,
                parent=self.nodes[parent_link],
                node_type="joint",
                joint_type=joint["type"],
                joint_axis=joint["axis"]
            )
            self.nodes[joint_name] = joint_node
            
            # Set the child link's parent to be this joint
            self.nodes[child_link].parent = joint_node
        
        # Find all root nodes (links with no parent)
        roots = []
        for node in self.nodes.values():
            if node.is_root and node.node_type == "link":
                roots.append(node)
                
        return roots
    
    def identify_chains(self):
        """
        Identify all kinematic chains in the URDF.
        A chain is defined as a path from a root node to a leaf node.
        """
        # Build all trees
        trees = self.build_multiple_trees()
        chains = []
        
        # For each tree, find all paths from root to leaves
        for root in trees:
            # Find all leaf nodes in this tree
            leaves = [node for node in root.descendants if not node.children and node.node_type == "link"]
            
            # For each leaf, create a chain from root to leaf
            for leaf in leaves:
                chain = []
                current = leaf
                
                # Traverse up from leaf to root
                while current:
                    chain.append(current)
                    current = current.parent
                
                # Reverse to get root-to-leaf order
                chain.reverse()
                
                # Create a chain object with relevant information
                chain_info = {
                    "name": f"{root.name}_to_{leaf.name}",
                    "nodes": chain,
                    "links": [node for node in chain if node.node_type == "link"],
                    "joints": [node for node in chain if node.node_type == "joint"],
                    "root": root,
                    "leaf": leaf
                }
                
                chains.append(chain_info)
        
        return chains, trees
    
    def print_chains(self):
        """Print all kinematic chains in the URDF"""
        chains, trees = self.identify_chains()
        
        print(f"Found {len(trees)} kinematic tree(s) and {len(chains)} chain(s)")
        
        for i, tree in enumerate(trees):
            print(f"\nTree {i+1}: Root = {tree.name}")
            
            # Find chains belonging to this tree
            tree_chains = [chain for chain in chains if chain["root"] == tree]
            
            for j, chain in enumerate(tree_chains):
                print(f"  Chain {j+1}: {chain['name']}")
                print(f"    Links: {len(chain['links'])}")
                print(f"    Joints: {len(chain['joints'])}")
                print(f"    Path: ", end="")
                
                # Print the path with alternating links and joints
                for node in chain["nodes"]:
                    if node.node_type == "joint":
                        joint_type = getattr(node, "joint_type", "unknown")
                        print(f"{node.name}({joint_type}) → ", end="")
                    else:
                        if node == chain["nodes"][-1]:  # Last node
                            print(f"{node.name}", end="")
                        else:
                            print(f"{node.name} → ", end="")
                print()
    
    def get_chain_info(self):
        """
        Get detailed information about all kinematic chains.
        Returns a list of dictionaries, each containing information about a chain.
        """
        chains, trees = self.identify_chains()
        chain_info_list = []
        
        # Get transformations for all links
        transformations = self.forward_kinematics()
        
        for chain in chains:
            # Extract link and joint names in order
            link_names = [node.name for node in chain["nodes"] if node.node_type == "link"]
            joint_names = [node.name for node in chain["nodes"] if node.node_type == "joint"]
            
            # Get joint types and axes
            joint_types = []
            joint_axes = []
            
            for joint_name in joint_names:
                joint_node = self.nodes[joint_name]
                joint_types.append(getattr(joint_node, "joint_type", "unknown"))
                joint_axes.append(getattr(joint_node, "joint_axis", [0, 0, 1]))
            
            # Get link transformations
            link_transforms = []
            for link_name in link_names:
                if link_name in transformations:
                    link_transforms.append(transformations[link_name])
                else:
                    link_transforms.append(np.eye(4))
            
            # Create detailed chain info
            detailed_info = {
                "name": chain["name"],
                "root": chain["root"].name,
                "leaf": chain["leaf"].name,
                "link_names": link_names,
                "joint_names": joint_names,
                "joint_types": joint_types,
                "joint_axes": joint_axes,
                "link_transforms": link_transforms,
                "num_links": len(link_names),
                "num_joints": len(joint_names)
            }
            
            chain_info_list.append(detailed_info)
        
        return chain_info_list, trees
    
    def get_joint_axes(self, chain):
        """
        Get the axis of each joint in the chain of the URDF.
        Returns a dictionary with joint names as keys and axis vectors as values.
        """
        link_frames = chain["link_transforms"]
        joint_axes = [[0, 0, 1]] + chain["joint_axes"] # 第一个joint的axis是固定的base，默认为z向上
        joint_types = ['base'] + chain["joint_types"]
        link_names = chain["link_names"]
        
        joint_positions = []
        joint_vectors = []
        joint_xs = []
        
        for name, T, axis, joint_type in zip(link_names, link_frames, joint_axes, joint_types):

            # Extract position and orientation
            pos = T[:3, 3]
            rot = T[:3, :3]@np.array(axis)
            
            if np.allclose(np.abs(np.array(axis)), np.array([0, 0, 1])): # z axis rotation
                joint_x = T[:3, :3]@np.array([1, 0, 0])
            
            if np.allclose(np.abs(np.array(axis)), np.array([0, 1, 0])): # y axis rotation
                joint_x = T[:3, :3]@np.array([1, 0, 0])
                
            if np.allclose(np.abs(np.array(axis)), np.array([1, 0, 0])): # x axis rotation
                joint_x = T[:3, :3]@np.array([0, 1, 0])


            joint_pos = pos
            joint_vector = rot
            
            joint_positions.append(joint_pos)
            joint_vectors.append(joint_vector)
            joint_xs.append(joint_x)

        
        return joint_positions, joint_vectors, joint_xs, joint_types
    
    def calculate_mdh_origin_position(self, joint_pos, joint_vector, joint_pos_next, joint_vector_next):

        """
        计算相邻两个关节坐标系原点oi的位置
        
        参数:
            joint_pos: 第i个关节的位置 (3D向量)
            joint_vector: 第i个关节的向量 (单位向量)
            joint_pos_next: 第i+1个关节的位置 (3D向量)
            joint_vector_next: 第i+1个关节的向量 (单位向量)
        
        返回:
            oi: 坐标系原点位置 (3D向量)
            case: 情况分类 (1-4)
            common_perpendicular: 公垂线信息 (起点, 终点), 若无则为None
        """
        # 确保输入是numpy数组
        zi = np.asarray(joint_vector)
        zi_next = np.asarray(joint_vector_next)
        pi = np.asarray(joint_pos)
        pi_next = np.asarray(joint_pos_next)
        
        common_perpendicular = None
        
        # 情况1: zi和zi+1重合 (向量相同或相反)
        if np.allclose(np.cross(zi, zi_next), np.zeros(3)):
            # 两条线重合或平行
            # 检查是否在同一直线上
            diff_pos = pi_next - pi
            if np.allclose(np.cross(diff_pos, zi), np.zeros(3)):
                # 在同一直线上，返回pi作为原点
                return pi, 'coincident', None

            else:
                # 平行但不重合，进入情况3
                pass
        else:
            # 检查是否相交
            # 计算两条直线的最短距离
            cross_z = np.cross(zi, zi_next)
            diff_pos = pi_next - pi
            distance = np.abs(np.dot(diff_pos, cross_z)) / np.linalg.norm(cross_z)
            
            if np.isclose(distance, 0):
                # 情况2: 相交
                # 解方程组找到交点
                A = np.column_stack((zi, -zi_next))
                b = pi_next - pi
                t, s = np.linalg.lstsq(A, b, rcond=None)[0]
                oi = pi + t * zi
                # The common perpendicular is the intersection point itself
                common_perpendicular = (oi, oi)
                return oi, 'intersect', common_perpendicular

        
        # 情况3: 平行或情况4: 不相交也不平行
        # 计算公垂线
        
        # 向量垂直于zi和zi_next
        n = np.cross(zi, zi_next)
        
        if np.allclose(n, np.zeros(3)):
            # 情况3: 平行
            oi = pi  # 直接使用pi作为原点，即公垂线与zi的交点
            
            # 计算公垂线的两个端点
            point1 = pi
            point2 = pi_next - np.dot(pi_next - pi, zi) * zi
            common_perpendicular = (point1, point2)
            
            return oi, 'parallel', common_perpendicular

        else:
            # 情况4: 不相交也不平行 (异面直线)
            n = n / np.linalg.norm(n)
            
            # 建立方程组
            A = np.column_stack((zi, -zi_next, n))
            b = pi_next - pi
            t, s, _ = np.linalg.lstsq(A, b, rcond=None)[0]
            # 公垂线与zi的交点
            oi = pi + t * zi
            
            # 计算公垂线的两个端点
            point1 = pi + t * zi
            point2 = pi_next + s * zi_next
            common_perpendicular = (point1, point2)
            
            return oi, 'skew', common_perpendicular


    def get_mdh_frames(self, chain):
        mdh_origins, mdh_zs, mdh_xs, mdh_parameters = self.get_mdh_parameters(chain)
        
        mdh_frames = []
        
        for origin, z, x in zip(mdh_origins, mdh_zs, mdh_xs):
            y = np.cross(z, x)
            
            transform = np.eye(4)
            
            transform[:3, 3] = origin
            
            transform[:3, 0] = x
            transform[:3, 1] = y
            transform[:3, 2] = z
            
            mdh_frames.append(transform)
        
        return mdh_frames

    def update_mdh_frames(self,mdh_frames, joint_values):
        T_i_iplus1 = []
        for i, frame in enumerate(mdh_frames): #TO DO update_mdh_frame
            if i >0:
                joint = joint_values[i - 1]
                rot_z = np.eye(4)
                rot_z[0, 0] = math.cos(joint)
                rot_z[1, 0] = math.sin(joint)
                rot_z[1, 1] = math.cos(joint)
                rot_z[0, 1] = -1 * math.sin(joint)
                transform = np.linalg.inv(mdh_frames[i-1]) @ frame @ rot_z
                T_i_iplus1.append(transform)
        for i, frame in enumerate(mdh_frames):
            if i >0:
                mdh_frames[i] = mdh_frames[i - 1] @ T_i_iplus1[i - 1]
        return mdh_frames


        return 0


            
    def get_mdh_parameters(self, chain):
        """
        Get the MDH parameters of the chain.
        Returns a list of dictionaries, each containing the MDH parameters of a joint.
        """
        # https://zhuanlan.zhihu.com/p/285759868
        joint_positions, joint_vectors, joint_xs, joint_types = self.get_joint_axes(chain)


        # skip fixed joints
        joint_positions = [joint_positions[i] for i, t in enumerate(joint_types) if t=='revolute' or t =='base']
        joint_vectors = [joint_vectors[i] for i, t in enumerate(joint_types) if t=='revolute' or t =='base']
        joint_xs = [joint_xs[i] for i, t in enumerate(joint_types) if t=='revolute' or t =='base']

        num_joints = len(joint_positions) - 1 # not base
        
        mdh_origins = []
        mdh_zs = []
        mdh_xs = []
        mdh_cases = []
    
        for i in range(num_joints):

            # zi
            joint_pos = joint_positions[i]
            joint_vector = joint_vectors[i]
            joint_x = joint_xs[i]

            # zi+1
            joint_pos_next = joint_positions[i+1]
            joint_vector_next = joint_vectors[i+1]

            # 建立第i个坐标系的原点oi
            
            oi, case, common_perpendicular = self.calculate_mdh_origin_position(joint_pos, joint_vector, joint_pos_next, joint_vector_next)
            mdh_cases.append(case)

            
            mdh_origins.append(oi)
            mdh_zs.append(joint_vector)
            
            if case == 'coincident':
                xi = joint_x

            if case == 'skew' or case == 'intersect':

                xi = np.cross(joint_vector, joint_vector_next)
                xi = xi / np.linalg.norm(xi)
                
            if case == 'parallel':
                xi = common_perpendicular[1] - common_perpendicular[0]
                xi = xi / np.linalg.norm(xi)

            mdh_xs.append(xi)

        # 最后一个joint
        mdh_origins.append(joint_positions[-1])
        mdh_zs.append(joint_vectors[-1])
        mdh_xs.append(joint_xs[-1])

        mdh_parameters = []

        for i in range(num_joints):
            
            # oi-1
            o_prev = mdh_origins[i]
            # zi-1, unit vector from oi-1
            z_prev = mdh_zs[i]
            # xi-1, unit vector from oi-1
            x_prev = mdh_xs[i]

            # oi
            oi = mdh_origins[i+1]
            # zi, unit vector from oi
            zi = mdh_zs[i+1]
            # xi, unit vector from oi
            xi = mdh_xs[i+1]
            
            case = mdh_cases[i]

            # theta, 绕着zi轴，从xi-1转动到xi的角度
               
            # d, 沿着zi轴，从xi-1到xi的距离
            
            # a，沿着xi-1轴，从zi-1到zi的距离
                
            # alpha, 绕着xi-1轴，从zi-1转动到zi的角度
            
            
            """theta"""          
            # Project xi-1 and xi onto the plane perpendicular to zi
            p_prev = x_prev - np.dot(x_prev, zi) * zi
            pi = xi - np.dot(xi, zi) * zi
                
            # Normalize the projected vectors
            pi_norm = pi / np.linalg.norm(pi)
            p_prev_norm = p_prev / np.linalg.norm(p_prev)
            
            # Calculate cosine and sine of the angle
            cos_theta = np.dot(p_prev_norm, pi_norm)
            sin_theta = np.dot(np.cross(p_prev_norm, pi_norm), zi)
            
            # Compute the directed angle using arctan2
            theta = np.arctan2(sin_theta, cos_theta)
            
            """d"""
            d = (oi - o_prev).dot(zi)
            
            """a"""
            a = (oi - o_prev).dot(x_prev)
        
            """alpha"""
            # Project zi-1 and zi onto the plane perpendicular to xi-1
            p_prev = z_prev - np.dot(z_prev, x_prev) * x_prev
            pi = zi - np.dot(zi, x_prev) * x_prev
            
            # Normalize the projected vectors
            pi_norm = pi / np.linalg.norm(pi)
            p_prev_norm = p_prev / np.linalg.norm(p_prev)
            
            # Calculate cosine and sine of the angle
            cos_theta = np.dot(p_prev_norm, pi_norm)
            sin_theta = np.dot(np.cross(p_prev_norm, pi_norm), x_prev)
            
            # Compute the directed angle using arctan2
            alpha = np.arctan2(sin_theta, cos_theta)            
            
            mdh_parameters.append([theta, d, a, alpha])

        # 最后一个joint的x轴
        return mdh_origins, mdh_zs, mdh_xs, mdh_parameters


if __name__ == "__main__":
    
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import pybullet as p
    
    from codegen.forward_kinematics import FK_SYM
    from codegen.jacobian import JAC_SYM
    
    
    from third_parties import urdf2dh
    
    urdf_path = "descriptions/gx7/urdf/gx7_test.urdf"

    parser = URDFParser(urdf_path)
    
    parser.get_robot_info()
    parser.print_chains()

    chains, _ = parser.get_chain_info()
    
    chain = chains[0]
    
    parser.get_mdh_frames(chain)
    
    mdh_origins, mdh_zs, mdh_xs, mdh_parameters = parser.get_mdh_parameters(chain)
    
    fk = FK_SYM(mdh_parameters)

    q = [0.1] * fk.num_joints
    
    # global pos and rot
    global_pos_rot = fk.calc_fk(*q)
    print("pos=\n", global_pos_rot[:3, -1])
    print("ori=\n", global_pos_rot[:3, :3])

   

    import roboticstoolbox as rtb
    
    mdh_config = []
    for i, params in enumerate(mdh_parameters):
        theta, d, a, alpha = params
        
        mdh_config.append(rtb.RevoluteMDH(d=d, a=a, alpha=alpha, offset=theta))

    robot = rtb.DHRobot(mdh_config, name="gx7")
   
    
    T = robot.fkine(q).data[0]

    pos = T[:3, 3]
    
    ori = T[:3, :3]
    
    print('MDH RTB')
    print("pos=\n", pos)

    print("ori=\n", ori)
    
    p.connect(p.DIRECT)
    
    robot_id = p.loadURDF(urdf_path, useFixedBase=True)
    
    valid_joints = []
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        if info[2] == p.JOINT_REVOLUTE:
            valid_joints.append(i)


    for i, joint_position in zip(valid_joints, q):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_position)
        
    for i in range(200):
        p.stepSimulation()
        
    link_state = p.getLinkState(robot_id, 6)
    position = link_state[4]  # Position of the link
    orientation = link_state[5]  # Orientation of the link (quaternion)

    orientation = p.getMatrixFromQuaternion(orientation)
    
    position = np.array(position)
    ori = np.array(orientation).reshape(3, 3)



    print('bullet:')
    print('pos=\n', position)
    print('ori=\n', ori)


    ##### JAC ######
    
    jac = JAC_SYM(mdh_parameters)
    
    jacobian = jac.calc_jac(*(q+[0]*3)) # the last three are local xyz offset
    print("Jacobian:\n", jacobian)
    
    rtb_jacobian = robot.jacob0(q)
    print("RTB Jacobian:\n", rtb_jacobian)
    
    bullet_jacobian = p.calculateJacobian(robot_id, 6, [0,0,0], list(q), [0]*len(q), [0]*len(q))