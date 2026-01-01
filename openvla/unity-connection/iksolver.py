import roboticstoolbox as rtb
from spatialmath import SE3

robot = rtb.models.UR5()
print(robot)

# forward kinematics
print("qz and qr")
print(robot.fkine(robot.qr))
print(robot.fkine(robot.qz))

print("fully extended")
q_extended = [0, 0, 0, 0, 0, 0]
print(robot.fkine(q_extended))

Tep = SE3.Trans(0.6, -0.3, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
print(Tep)
sol = robot.ik_LM(Tep)         # solve IK
print(sol)

q_pickup = sol[0]
print(robot.fkine(q_pickup))

# qt = rtb.jtraj(robot.qr, q_pickup, 50)
# robot.plot(qt.q, backend='pyplot', block=True)
# robot.plot(robot.qn, backend='pyplot', block=True)

for link in robot.links:
    print(f"{link.name}: {link.ets}")  # 'a' is link length
