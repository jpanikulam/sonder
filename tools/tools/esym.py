import sympy

def vector(name, length=3):
    names = [name + '_' + str(n) for n in range(length)]
    return sympy.Matrix(names)

def norm(v):
    vv = v.dot(v)
    return sympy.sqrt(vv)
