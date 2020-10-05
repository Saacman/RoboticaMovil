# Python3
# Robótica Móvil 
# Saúl Isaac Sánchez Flores

# Actividad 1
# Considere tres marcos de referencia en el plano (2D): {W} es el mundo, {B} es un robot omnidireccional 
# (puede moverse en cualquier dirección y rotar sobre su eje) y {C} es un sensor ultrasónico.

# El origen de {B} coincide con el centro de rotación del robot, y el eje X de {B} apunta hacia el frente del robot. 
# El sensor se encuentra en la parte frontal derecha del robot, de modo que las coordenadas de su origen expresadas en {B} son (0.5, −0.3). 
# El sensor ultrasónico en cuestión apunta hacia la derecha del robot, por lo que el ángulo entre {B} y {C} es θ=−π/2. Para entender mejor esta situación,.

# Escriba un programa en Python que reciba las coordenadas de un punto en el marco de referencia {C}, 
# y que imprima las coordenadas de ese mismo punto en los marcos de referencia {B} y {W}. 
# Especifique en alguna variable interna de su programa la posición y el ángulo del robot {B} con respecto 
# al mundo {W}, por ejemplo, que el robot esté en el punto (2,2) del marco {W}, con ángulo θ=π/4.

import numpy as np

# Funcion de transformacion de coordenadas del marco {B} al {A}
def transfB2A(p_b, rotAngle, B_ACoords):
    T = np.array([[np.cos(rotAngle) , np.sin(rotAngle), float(B_ACoords[0])],
                  [-np.sin(rotAngle), np.cos(rotAngle), float(B_ACoords[1])],
                  [                0,                0,           1]])

    N = np.array([[p_b[0]],
                   [p_b[1]],
                   [  1  ]])
    return np.dot(T, N)



if __name__ == "__main__":
    # Datos de posición:
    sensor_b = (0.5, -0.3)
    angC_b = -np.pi / 2
    robot_pos = (2,2)
    robot_ang = np.pi / 4

    while(True):
        print(f"\nCoordenadas en {{C}}:")
        pos_C = (float(input("x_c: ")), float(input("y_c: ")))
        print(f"Posición en {{C}}: {pos_C[0]:.3f}, {pos_C[1]:.3f}")

        # Transformar de {C} a {B}
        pos_B = transfB2A(pos_C, angC_b, sensor_b)
        pos_B = pos_B.reshape(1,3)[0]
        print(f"Posición en {{B}}: {pos_B[0]:.3f}, {pos_B[1]:.3f}")

        # Transformar de {B} a {W}
        pos_W = transfB2A(pos_B, robot_ang, robot_pos)
        pos_W = pos_W.reshape(1,3)[0]
        print(f"Posición en {{W}}: {pos_W[0]:.3f}, {pos_W[1]:.3f}")

        ans = input("Otra coordenada? (y/n): ") or 'y'
        if not('y' in ans or 'Y' in ans):
            break

        