Approach #1:
Rot = q.matrix * inversa(q_initial.matrix)
q_theo = rotation_matrix([1, 0, 0] or [0, 1, 0] or [0, 0, 1], -degrees)
distance = norma_L2(Rot - q.matrix)

Approach #2:
q_theo = Quaternion(axis= [1, 0, 0] or [0, 1, 0] or [0, 0, 1], degrees=q_initial.degrees - degrees)
distance = min(distance(q_theo, q), distance(q_theo, -q))

Approach #3:
q_theo = Quaternion(-degree, pitch_initial, roll_initial) or
         Quaternion(yaw_initial, -degree, roll_initial) or
         Quaternion(yaw_initial, pitch_initial, -degree)
distance = min(distance(q_theo, q), distance(q_theo, -q))

Approach #4:(el que hicimos ayer)
q_theo = Quaternion(-degree + yaw_initial, pitch_initial, roll_initial) or
         Quaternion(yaw_initial, -degree + pitch_initial, roll_initial) or
         Quaternion(yaw_initial, pitch_initial, -degree + roll_initial)
distance = min(distance(q_theo, q), distance(q_theo, -q))

Approach #5:
q_theo = Quaternion(axis=q_initial.axis, degrees=-degrees)
distance = min(distance(q_theo, q), distance(q_theo, -q))
