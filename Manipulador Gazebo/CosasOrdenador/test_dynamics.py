import xacro, pinocchio as pin, numpy as np, tempfile, os

# 1) Expand .xacro → temp .urdf
xacro_path = "/home/chenfu/ros2_ws/src/six_dof_manipulator_description/urdf/6dof_manipulator.xacro"
doc = xacro.process_file(xacro_path)
urdf_xml = doc.toxml()
fd, urdf_tmp = tempfile.mkstemp(suffix=".urdf")
with os.fdopen(fd, "w") as f:
    f.write(urdf_xml)

# 2) Build the Pinocchio model
model = pin.buildModelFromUrdf(urdf_tmp)   # <— Only the filename
data  = model.createData()

# 3) Sample a state
q  = np.zeros(model.nq)
v  = np.zeros(model.nv)

# 4) Compute dynamics
H  = pin.crba(model, data, q)
b  = pin.rnea(model, data, q, v, np.zeros_like(v))

pin.computeJointJacobians(model, data, q)
J  = pin.getFrameJacobian(model, data,
       model.getFrameId("link6"),
       pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

pin.computeJointJacobiansTimeVariation(model, data, q, v)
Jd = pin.getFrameJacobianTimeVariation(model, data,
       model.getFrameId("link6"),
       pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

print("H:\n", H)
print("b:\n", b)
print("J:\n", J)
print("Jdot:\n", Jd)

os.remove(urdf_tmp)

# Hacer un nodo de ros el cual reciba los datos de posición y velocidad del polla y que muestre las matrices en cada iteración
