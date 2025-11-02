from ikpy.chain import Chain
import os
from ament_index_python.packages import get_package_share_directory

# Load chain
package_path = get_package_share_directory('agrirover_description')
urdf_path = os.path.join(package_path, 'urdf', 'mech', 'arm_combined.urdf')

# With active mask
active_mask = [False, True, True, True, True, True]
chain = Chain.from_urdf_file(urdf_path, active_links_mask=active_mask)

print("=" * 60)
print("IK CHAIN INSPECTION")
print("=" * 60)

print(f"\nTotal links: {len(chain.links)}")
print(f"Active links: {sum(active_mask)}")

print("\nLinks in chain:")
for i, link in enumerate(chain.links):
    print(f"  [{i}] {link.name}")

print("\nJoints in chain:")
for i, joint in enumerate(chain.joints):
    print(f"  [{i}] {joint.name} - Type: {joint.joint_type}")
    print(f"       Bounds: {joint.bounds}")

# Test forward kinematics
print("\n" + "=" * 60)
print("FORWARD KINEMATICS TEST")
print("=" * 60)

import numpy as np

# Home position
home_pos = np.array([0.0, 1.57, 1.57, 0.9, 0.8, 0.0])  # With dummy base joint
print(f"\nTesting FK with home position: {home_pos}")

try:
    fk = chain.forward_kinematics(home_pos)
    print(f"End effector position (home):")
    print(f"  X: {fk[0, 3]:.4f}")
    print(f"  Y: {fk[1, 3]:.4f}")
    print(f"  Z: {fk[2, 3]:.4f}")
except Exception as e:
    print(f"FK Error: {e}")