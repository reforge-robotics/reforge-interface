# Standard Bots RO1 meshes

From upstream `standard_bots_description/meshes/sbot`, flattened into this
directory. Visual and collision meshes share filenames upstream, so each carries
a `_visual` / `_collision` suffix here:

    <link>_visual.STL     full-detail shell, used for visualization
    <link>_collision.STL  simplified geometry, required for the collision model

Links: base_link, shoulder_link, upper_arm_link, forearm_link, wrist_1_link,
wrist_2_link, wrist_3_link.

`../modelone.urdf` references these as `meshes/<link>_<kind>.STL` instead of
`package://`, because `CollisionModel` passes the URDF's own directory as
Pinocchio's `package_dirs` and does not consult `ROS_PACKAGE_PATH`.

Not committed: the upstream `package.xml` declares the package Proprietary even
where a mirror carries an MIT root license.
