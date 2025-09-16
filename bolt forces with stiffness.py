import numpy as np

# Mount type: 'A-ARM' or 'SHOCK'
mounttype = 'A-Arm'

inputs = np.array([[6255.272, 3392.495, 0.000],
[-13152.183, 764.162, 207.617],
[6627.199, 2591.850, 0.000],
[-12770.881, 3241.758, -50.796],
[7438.080, -655.969, -324.562],
[7152.713, -2105.017, -517.960],
[10000.000, 0 , 0 ]])

mounttype_list = ['Shock','A-Arm','Shock','A-Arm','A-Arm','A-Arm','A-Arm']

bolt1ARM = np.array([2.0,  .5, 0])
bolt2ARM = np.array([3.0,  0 , 0])
bolt3ARM = np.array([2.0, -2, 0])

bolt1Shock = np.array([2.04,  0.0, 0])
bolt2Shock = np.array([2.41, 1.39, 0])
bolt3Shock = np.array([1.04,  1.76, 0])

# Bolt stiffness (N/m)
k = 2e11  # 1 MN/m in each direction

# Loop through each case
for i in range(len(inputs)):
    print(f"\n--- Case {i+1} ---")
    F_ext = np.array(inputs[i, 0:3])
    M_ext = np.zeros(3)
    W = -np.hstack((F_ext, M_ext))  # right-hand side: negative of applied load

    if mounttype == mounttype_list[i]:
        bolts = [bolt1ARM, bolt2ARM, bolt3ARM]
    else:
        bolts = [bolt1Shock, bolt2Shock, bolt3Shock]

    A_force_moment = np.zeros((6, 15))  # 6 equations: Fx, Fy, Fz, Mx, My, Mz
    A_compatibility = np.zeros((9, 15))  # 3 constraints per bolt: F = k * δ

    for b, r in enumerate(bolts):
        # Indices
        fi = 6 + 3 * b  # start index for bolt reaction force (Fx, Fy, Fz)
        
        # Force balance (rows 0–2)
        A_force_moment[0:3, fi:fi+3] = np.eye(3)
        
        # Moment balance (rows 3–5): r × F
        r_cross = np.array([
            [0, -r[2], r[1]],
            [r[2], 0, -r[0]],
            [-r[1], r[0], 0]
        ])
        A_force_moment[3:6, fi:fi+3] = r_cross

        # Displacement at bolt = flange translation + rotation × r
        # δ = u + θ × r →  δ_i = [I, skew(r)] · [u, θ]
        # F = k * δ → F - k*[I skew(r)]*[u θ] = 0

        # Compatibility constraint: F - k * δ = 0
        A_compatibility[3*b:3*b+3, 0:3] = -k * np.eye(3)         # flange translation u
        A_compatibility[3*b:3*b+3, 3:6] = -k * r_cross           # flange rotation θ
        A_compatibility[3*b:3*b+3, fi:fi+3] = np.eye(3)          # bolt forces

    # Stack both blocks into 15x15 system
    A_total = np.vstack((A_force_moment, A_compatibility))
    RHS = np.hstack((W, np.zeros(9)))  # external force/moment + zero target for constraints

    # Solve
    X = np.linalg.solve(A_total, RHS)

    u = X[0:3]
    theta = X[3:6]
    bolt_forces = X[6:]

    print(f"Flange translation (mm): {u * 1e3}")
    print(f"Flange rotation (rad): {theta}")

    for b in range(3):
        Fx, Fy, Fz = bolt_forces[3*b:3*b+3]
        print(f"Bolt {b+1}: Rx = {Fx:.2f} N, Ry = {Fy:.2f} N, Rz = {Fz:.2f} N")
