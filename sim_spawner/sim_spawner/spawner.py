from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

import rclpy
import os
import sys
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import GetEntityState, GetModelList
from gazebo_msgs.msg import EntityState
rclpy.init()
from rclpy.node import Node
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims.xform_prim import XFormPrim
import numpy as np

class SpawnerNode(Node):
    def __init__(self, args=None):
        super().__init__("spawner", args)
        self.counters = 0
        self.world = World(stage_units_in_meters=0.01)
        add_reference_to_stage(usd_path="/home/ubb/Documents/docker_sim_comp/Isaac/ubb/Isaacdev/src/simple_move_group/urdf/table.usd",
                               prim_path="/World/table")
        prim = XFormPrim(prim_path="/World/table", name="table", position=np.array([70,0,-20]), orientation=np.array([0.707,0,0,0.707])) # w,x,y,z
        self.get_world().scene.add(prim)

        self.entity = self.create_service(
            GetEntityState, 'get_entity_state', self.get_entity_state)
        self.model = self.create_service(
            GetModelList, 'get_model_list', self.get_model_list)
        self.objs = {"table": self.world.scene.get_object("table")}
        for x in range(3, 6):
            for y in range(-3, 4):
                self.spawn_obj("worlds/Cube.wbo", np.array([x*10,y*10,50]))

        self.world.reset()

    def spawn_obj(self, path, position=[0, 0, 0], rotation = [1,0,0,0]):
        name = "Cube"+str(self.counters)
        cube = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/"+name,
                name="name",
                position=np.array([25,10,50]),
                orientation = rotation,
                size=np.array([5,5,5]),
                color=np.array([0, 0, 1.0]),
            )
        )
        self.objs[name] = self.world.scene.get_object(name)

        self.counters+=1

    def get_model_list(self, request: GetModelList.Request, response: GetModelList.Response):
        response.model_names = list(self.objs.keys())
        response.success = True
        return response

    def get_entity_state(self, request: GetEntityState.Request, response: GetEntityState.Response):
        obj = self.objs.get(request.name)
        success = True
        if obj is None:
            response.success = False
            return response
        state = EntityState()
        state.name = request.name
        pose = Pose()
        try:    
            pose.position , pose.orientation = self.get_postion_rotation(obj)
        except: # object got deleted
            success = False
        finally:    
            state.pose = pose
            response.state = state
            response.success = success
        return response

    def get_postion_rotation(self, obj):
        position = Point()
        pose, orientation = self.objs.get(obj).get_world_pose()
        position.x = pose[0]
        position.y = pose[1]
        position.z = pose[2]
        rotation = Quaternion()
        rotation.x = float(orientation[0])
        rotation.y = float(orientation[1])
        rotation.z = float(orientation[2])
        rotation.w = float(orientation[3])
        return position, rotation


def main(args=None):
    rclpy.init(args=args)
    os.environ['WEBOTS_ROBOT_NAME'] = "spawner"

    spawner = SpawnerNode(args=args)

    rclpy.spin(spawner)
    simulation_app.close() # close Isaac Sim

    rclpy.shutdown()


if __name__ == '__main__':
    main()
