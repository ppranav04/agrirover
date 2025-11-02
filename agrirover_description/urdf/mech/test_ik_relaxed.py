#!/usr/bin/env python3
"""
Fixed IK Test Script
"""
from ikpy.chain import Chain
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory

# Load chain - NO active mask (use all links)
package_path = get_package_share_directory('agrirover_description')
urdf_path = os.path.join(package_path, 'urdf', 'mech', 'arm_combined.urdf')
chain = Chain.from_urdf_file(urdf_path)

print("=" * 60)
print("IK SOLVER TEST (FIXED)")
print("=" * 60)

print(f"\nChain has {len(chain.links)} links:")
for i, link in enumerate(chain.links):
    print(f"  [{i}] {link.name}")

# Test target positions
test_targets = [
    ([0.0, -0.2, 0.2], "Test 1"),
    ([0.2, 0.0, 0.3], "Test 2"),
    ([0.1, -0.1, 0.25], "Test 3"),
    ([0.3, 0.2, 0.1], "Test 4"),
    ([0.0, 0.0, 0.4], "Test 5"),
    ([0.4, -0.2, 0.2], "Test 6"),
]

joint_limits = np.array([
    [0.0, 3.14],  # base_to_link_0
    [0.0, 3.14],  # link_0_to_link_1
    [0.0, 3.14],  # link_1_to_link_2
    [0.0, 3.14],  # link_2_to_gripper
    [0.0, 0.9],   # finger_1_joint
])

for target_pos, label in test_targets:
    print(f"\n{label}: {target_pos}")
    print("-" * 60)
    
    initial_guesses = [
        np.array([1.57, 1.57, 0.9, 0.8, 0.0]),
        np.array([0.785, 1.57, 1.57, 0.4, 0.0]),
        np.array([2.0, 1.0, 1.0, 0.5, 0.0]),
    ]
    
    found_solution = False
    for guess_idx, initial_guess in enumerate(initial_guesses):
        try:
            # Solve IK
            ik_result = chain.inverse_kinematics(
                target_position=target_pos,
                initial_position=initial_guess,
                max_iter=1000,
            )
            
            # ik_result is numpy array [j0, j1, j2, j3, j4]
            joints = ik_result
            
            # Check if within bounds - FIX: use all() for array comparison
            within_bounds = np.all(
                (joints >= joint_limits[:, 0]) & 
                (joints <= joint_limits[:, 1])
            )
            
            if within_bounds:
                # Verify with FK
                fk = chain.forward_kinematics(ik_result)
                achieved = fk[:3, 3]
                error = np.linalg.norm(np.array(target_pos) - achieved)
                
                print(f"  Guess {guess_idx}: ✓ SUCCESS")
                print(f"    Joints: {joints}")
                print(f"    FK error: {error:.6f}m")
                found_solution = True
                break
            else:
                violations = []
                for i, (j, (lb, ub)) in enumerate(zip(joints, joint_limits)):
                    if not (lb <= j <= ub):
                        violations.append(f"J{i}: {j:.3f} (limits: [{lb:.3f}, {ub:.3f}])")
                print(f"  Guess {guess_idx}: Outside bounds")
                for v in violations:
                    print(f"    {v}")
        except Exception as e:
            print(f"  Guess {guess_idx}: ✗ {str(e)[:60]}")
    
    if not found_solution:
        print(f"  ✗ NO SOLUTION FOUND")

