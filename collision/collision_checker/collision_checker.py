import itertools
import random
from typing import List
import shapely.geometry
from dataclasses import dataclass

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    PlacedPrimitive,
    Rectangle,
)

from duckietown_world import pose_from_friendly
from duckietown_world.utils import SE2_apply_R2

__all__ = ["CollisionChecker"]


class CollisionChecker:
    params: MapDefinition

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: MapDefinition):
        context.info("initialized")
        self.params = data

    def on_received_query(self, context: Context, data: CollisionCheckQuery):
        collided = check_collision(
            Wcoll=self.params.environment, robot_body=self.params.body, robot_pose=data.pose
        )
        result = CollisionCheckResult(collided)
        context.write("response", result)

def pose_distance(pose1: FriendlyPose, pose2: FriendlyPose):
    x1, y1, x2, y2 = pose1.x, pose1.y, pose2.x, pose2.y
    return ((x1-x2)**2 + (y1-y2)**2)**(1/2)

def circ_circ_collision(pprim_circ1, pprim_circ2):
    pose1 = pprim_circ1.pose
    pose2 = pprim_circ2.pose
    d = pprim_circ1.primitive.radius + pprim_circ2.primitive.radius
    
    return pose_distance(pose1, pose2) < d


def get_shapely_rect(pprim: PlacedPrimitive):
    rect = pprim.primitive

    shapely_rect = shapely.geometry.box(rect.xmin, rect.ymin, rect.xmax, rect.ymax)
    shapely_rect = shapely_apply_fpose(shapely_rect, pprim.pose)

    return shapely_rect

def get_shapely_circle(pprim: PlacedPrimitive):
    circle = pprim.primitive

    shapely_rect = shapely.geometry.Point(0, 0).buffer(circle.radius)
    shapely_rect = shapely_apply_fpose(shapely_rect, pprim.pose)

    return shapely_rect

def rec_rec_collision(pprim1: PlacedPrimitive, pprim2: PlacedPrimitive) -> bool:

    shapely_rect1 = get_shapely_rect(pprim1)
    shapely_rect2 = get_shapely_rect(pprim2)
    
    aoi = shapely_rect1.intersection(shapely_rect2).area
    print(aoi)
    
    return aoi > 0

def shapely_apply_fpose(obj, pose):
    origin = (0, 0)
    obj = shapely.affinity.rotate(obj, pose.theta_deg, origin=origin)
    return shapely.affinity.translate(obj, xoff=pose.x, yoff=pose.y, zoff=0.0)

def check_collision(
    Wcoll: List[PlacedPrimitive], robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose
) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    ##print(robot_body)
    #print(robot_pose)

    # start by rototranslating the robot parts by the robot pose
    rototranslated_robot = []  #

    for pp in robot_body:
        if isinstance(pp.primitive, Rectangle):
            shapely_rect1 = get_shapely_rect(pp)
        elif isinstance(pp.primitive, Circle):
            shapely_rect1 = get_shapely_circle(pp)
        shapely_rect1 = shapely_apply_fpose(shapely_rect1, robot_pose)

        rototranslated_robot.append(shapely_rect1)

    shapely_world = []

    for pp in Wcoll:
        if isinstance(pp.primitive, Rectangle):
            shapely_rect1 = get_shapely_rect(pp)
        elif isinstance(pp.primitive, Circle):
            shapely_rect1 = get_shapely_circle(pp)

        shapely_world.append(shapely_rect1)

    collided = check_collision_list(rototranslated_robot, shapely_world)

    # return a random choice
    return collided


def check_collision_list(A, B) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    for a, b in itertools.product(A, B):
        aoi = a.intersection(b).area
        #print(aoi)
        
        if aoi > 0:
            return  True

    return False


def check_collision_shape(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    if isinstance(a, Circle) and isinstance(b, Circle):
        circ_circ_collision()
    elif isinstance(a, Circle) and isinstance(b, Rectangle):
        ...
    # for now let's return a random guess

    return ...


if __name__ == '__main__':
    from zuper_commons.fs import locate_files, read_ustring_from_utf8_file
    from zuper_ipce import object_from_ipce
    import yaml
    from zuper_nodes_wrapper.wrapper_outside import ComponentInterface
    from zuper_nodes import (
        ExternalProtocolViolation,
        IncompatibleProtocol,
        InteractionProtocol,
    )


    data = read_ustring_from_utf8_file("mooc-exercises/collision/collision_checker/test.yaml")
    ydata = yaml.load(data, Loader=yaml.Loader)
    robotBody = object_from_ipce(ydata['params']['body'], List[PlacedPrimitive])
    Wcoll = object_from_ipce(ydata['params']['environment'], List[PlacedPrimitive])

    for interaction in ydata['interactions']:
        pose = object_from_ipce(interaction['query']['pose'], FriendlyPose)
        print(f"{check_collision(Wcoll, robotBody, pose)} {interaction['gt']['collision']}")

        

       