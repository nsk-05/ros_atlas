#!/usr/bin/env python3
import rospkg
from tf.transformations import euler_from_quaternion
# rspkg=rospkg.RosPack()
# pkg_path=rspkg.get_path("bot_vision")
# file=pkg_path+"/targets/targets.txt"
# target_dict=dict()
# print(file)
# with open(file) as f:
#     targets_list=f.read().splitlines()

# for i in targets_list:
#     j=i.split(" ")
#     target_dict[j[0]]=[int(j[1]),int(j[2])]
# print(target_dict)
print(euler_from_quaternion([0,0,0.99999,0.0047936]))