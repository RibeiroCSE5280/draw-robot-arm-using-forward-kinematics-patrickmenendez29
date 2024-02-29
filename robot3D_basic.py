# coding: utf-8


from vedo import *


def RotationMatrix(theta, axis_name):
    """ calculate single rotation of $theta$ matrix around x,y or z
        code from: https://programming-surgeon.com/en/euler-angle-python-en/
    input
        theta = rotation angle(degrees)
        axis_name = 'x', 'y' or 'z'
    output
        3x3 rotation matrix
    """

    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)

    if axis_name == 'x':
        rotation_matrix = np.array([[1, 0,  0],
                                    [0, c, -s],
                                    [0, s,  c]])
    if axis_name == 'y':
        rotation_matrix = np.array([[c,  0, s],
                                    [0,  1, 0],
                                    [-s,  0, c]])
    elif axis_name == 'z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s,  c, 0],
                                    [0,  0, 1]])
    return rotation_matrix


def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)

    """
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1

    # x-axis as an arrow
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)

    originDot = Sphere(pos=[0, 0, 0],
                       c="black",
                       r=0.10)

    # Combine the axes together to form a frame as a single mesh object
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot

    return F


def getLocalFrameMatrix(R_ij, t_ij):
    """Returns the matrix representing the local frame
    Args:
      R_ij: rotation of Frame j w.r.t. Frame i 
      t_ij: translation of Frame j w.r.t. Frame i 
    Returns:
      T_ij: Matrix of Frame j w.r.t. Frame i. 

    """
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij,                t_ij],
                     [np.zeros((1, 3)),       1]])

    return T_ij

def forward_kinematics(L1, L2, L3, phi1, phi2, phi3):
    """Returns the end-effector position and orientation
    Args:
      L1: Length of link 1
      L2: Length of link 2
      phi1: Rotation angle of part 1 in degrees
      phi2: Rotation angle of part 2 in degrees
      phi3: Rotation angle of the end-effector in degrees
    Returns:
      T_03: Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)

    """
    # Matrix of Frame 1 (written w.r.t. Frame 0, which is the previous frame)
    R_01 = RotationMatrix(phi1, axis_name='z')   # Rotation matrix
    # Frame's origin (w.r.t. previous frame)
    p1 = np.array([[0.0], [0], [0.0]])
    t_01 = p1                                      # Translation vector

    # Matrix of Frame 1 w.r.t. Frame 0 (i.e., the world frame)
    T_01 = getLocalFrameMatrix(R_01, t_01)

    # Matrix of Frame 2 (written w.r.t. Frame 1, which is the previous frame)
    R_12 = RotationMatrix(phi2, axis_name='z')   # Rotation matrix
    # Frame's origin (w.r.t. previous frame)
    p2 = np.array([[0.0], [L1 + 2 * 0.4], [0.0]])
    t_12 = p2                                      # Translation vector

    # Matrix of Frame 2 w.r.t. Frame 1
    T_12 = getLocalFrameMatrix(R_12, t_12)

    # Matrix of Frame 2 w.r.t. Frame 0 (i.e., the world frame)
    T_02 = T_01 @ T_12

    # Matrix of Frame 3 (written w.r.t. Frame 2, which is the previous frame)
    R_23 = RotationMatrix(phi3, axis_name='z')   # Rotation matrix
    # Frame's origin (w.r.t. previous frame)
    p3 = np.array([[0.0], [L2 + 2 * 0.4], [0.0]])
    t_23 = p3                                      # Translation vector

    # Matrix of Frame 3 w.r.t. Frame 2
    T_23 = getLocalFrameMatrix(R_23, t_23)

    # Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)
    T_03 = T_01 @ T_12 @ T_23
    e = T_03[0:3, -1]
    return T_01, T_02, T_03, e


def create_arm(L):
    """Returns the mesh representing a cylinder
    Args:
      L: Length of the cylinder
      r: Radius of the cylinder
      color: Color of the cylinder
      alpha: Transparency of the cylinder
      axis: Axis of the cylinder
    Returns:
      C: vedo.mesh object (cylinder)

    """
    sphere = Sphere(r=0.4).pos(0, 0, 0).color('red')
    arrows = createCoordinateFrameMesh()
    C = Cylinder(height=L, r=0.4, axis=[0, 1, 0]).pos(0, 0.4 + L / 2, 0).color('blue')
    C = C + sphere + arrows
    return C



def main():

    # Set the limits of the graph x, y, and z ranges
    axes = Axes(xrange=(-20, 20), yrange=(-2, 10), zrange=(0, 6))
    plt = Plotter(interactive=False)
    plt += __doc__

    # Lengths of arm parts
    L1 = 5   # Length of link 1
    L2 = 8   # Length of link 2
    L3 = 3
    # Joint angles
    phi1 = 30     # Rotation angle of part 1 in degrees
    phi2 = -10    # Rotation angle of part 2 in degrees
    phi3 = 0      # Rotation angle of the end-effector in degrees



    T_01, T_02, T_03, e = forward_kinematics(L1, L2, L3, phi1, phi2, phi3)

    # Create arms
    Frame1 = create_arm(L1)
    Frame2 = create_arm(L2)
    Frame3 = create_arm(L3)
    end_effector = Sphere(r=0.4).pos(0, L3 + .5, 0).color('green')


    Frame1.apply_transform(T_01)
    Frame2.apply_transform(T_02)
    Frame3.apply_transform(T_03)
    end_effector.apply_transform(T_03)
    plt.show([Frame1, Frame2, Frame3, end_effector], axes, viewup="x, y")

    direction = 1
    
    for i in range(360):
        Frame1.apply_transform(np.linalg.inv(T_01))
        Frame2.apply_transform(np.linalg.inv(T_02))
        Frame3.apply_transform(np.linalg.inv(T_03))
        end_effector.apply_transform(np.linalg.inv(T_03))
        if (min(phi1, phi2, phi3) <= -90 or max(phi1, phi2, phi3) >= 90):
            direction *= -1

        phi1 += 1 * direction
        phi2 += 1 * direction
        phi3 += 1 * direction

        T_01, T_02, T_03, e = forward_kinematics(L1, L2, L3, phi1, phi2, phi3)

        Frame1.apply_transform(T_01)
        Frame2.apply_transform(T_02)
        Frame3.apply_transform(T_03)
        end_effector.apply_transform(T_03)
        # Show everything
        plt.show(screenshot=f"/Users/patrickmenendez/Code/Python/Jupyter/Computer Graphics/draw-robot-arm-using-forward-kinematics-patrickmenendez29/frames/frame-{i}.jpg")

    plt.interactive().close()

if __name__ == '__main__':
    main()
