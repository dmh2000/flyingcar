import sympy
sympy.init_printing(use_unicode=True)

wall,phi,y_dot,y,dt= sympy.symbols('wall,phi,y_dot,y,dt')

x = sympy.Matrix([phi,y_dot,y])

h = sympy.Matrix([
    (wall-y) / sympy.cos(phi),
    y_dot - sympy.sin(phi) * dt,
    y + y_dot * dt
    ])

print(h.jacobian(x))

"""
Matrix([
[0, 0, 0], 
[y*sin(phi)/cos(phi)**2, 0, 1/cos(phi)], 
[0, 1, 1]])
"""