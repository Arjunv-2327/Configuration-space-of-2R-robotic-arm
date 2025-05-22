This project focuses on the Configuration Space (C-Space) analysis of a 2R (two-revolute-joint) robotic arm using MATLAB. The aim is to identify all possible joint configurations that allow the robotic arm to operate safely—i.e., without colliding with any obstacles in the workspace. The configuration space is a core concept in robotic motion planning, where each point in this abstract space corresponds to a unique pair of joint angles (θ₁, θ₂) that determine the pose of the robotic arm. By analyzing this space, one can visually and computationally distinguish between permissible (collision-free) and forbidden (collision-prone) joint configurations.

This project implements two complementary methods for C-space analysis:

    Binary Matrix-Based C-Space Representation
    In this approach, the joint angles θ₁ and θ₂ are discretized in 2° steps, resulting in a 181×181 grid. Each element of this binary matrix is either marked as 1 (if the configuration results in a collision) or 0 (if it is safe). This yields a comprehensive, data-driven overview of the robot's movement possibilities relative to the obstacle. The matrix serves as a visual and computational tool to identify collision zones within the full range of motion.

    Geometrical C-Space with Circular Arc Modeling
    The second method takes a more analytical, geometry-driven approach by modeling the first link of the robotic arm as a circular arc sweeping around the origin. A static, inclined rectangular obstacle is placed in the workspace. The system then determines collisions by computing the intersections between this circular arc and the edges of the rectangle. The edges are represented as linear equations, and the intersection points are derived by solving quadratic equations formed using the distance formula. Only real roots—which represent physically meaningful intersections—are retained. These points help define the forbidden regions in the configuration space, which are then excluded from the set of valid configurations.

The MATLAB implementation includes:

    Analytical geometry for detecting precise intersection points between links and obstacles,

    Symbolic and numerical computation of intersection roots,

    Filtering of physically valid (real) intersection coordinates, and

    Graphical visualization of both the configuration space and the actual robot structure within the obstacle-filled workspace.

This dual-approach framework offers both discrete and continuous perspectives on motion planning, allowing for cross-validation and deeper understanding of robotic kinematics. It is especially valuable for researchers, robotics students, and enthusiasts working on collision detection, robot design, and safe trajectory planning in constrained environments.
